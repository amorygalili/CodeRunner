import { extname, posix, resolve } from "node:path";
import {
	DASHBOARD_MANIFEST_PATH,
	type DashboardsResponse,
	dashboardManifestSchema,
	type ProjectDashboard,
	type WorkspaceId,
	type WorkspaceSlug,
} from "@frc-coderunner/contracts";
import { getLogger } from "../logging";
import { injectDashboardBridge } from "./dashboard-bridge";
import {
	DASHBOARD_TOKEN_TTL_SECONDS,
	isInsideDashboardScope,
	mintDashboardToken,
	verifyDashboardToken,
} from "./dashboard-token";
import {
	ASSET_CONTENT_TYPES,
	parsePreviewFileRequest,
	previewHeaders,
	readVerifiedProjectFile,
} from "./preview";
import { renderPreviewErrorDocument } from "./preview-markdown";
import { jsonResponse } from "./responses";

const log = getLogger("dashboards");

export const DASHBOARD_FILES_PREFIX = "/api/dashboards/files/";

const MAX_MANIFEST_BYTES = 64 * 1024;
const MAX_DOCUMENT_BYTES = 10 * 1024 * 1024;
const MAX_ASSET_BYTES = 25 * 1024 * 1024;

/** Preview's allowlist plus `.mjs`, which bundlers emit for ES modules. */
const DASHBOARD_ASSET_CONTENT_TYPES: Record<string, string> = {
	...ASSET_CONTENT_TYPES,
	".mjs": "text/javascript; charset=utf-8",
};

type DashboardWorkspace = {
	id: WorkspaceId;
	slug: WorkspaceSlug;
	project_path: string;
};

function encodePath(path: string): string {
	return path.split("/").map(encodeURIComponent).join("/");
}

/** The directory a dashboard may read from: its entry's directory. */
function scopeFor(entry: string): string {
	const directory = posix.dirname(entry);
	return directory === "." ? "" : directory;
}

// --- Listing -----------------------------------------------------------

async function readManifest(
	projectRoot: string,
): Promise<
	| { ok: true; dashboards: { title: string; entry: string }[] }
	| { ok: false; error: string | null }
> {
	const file = await readVerifiedProjectFile(
		projectRoot,
		DASHBOARD_MANIFEST_PATH,
		MAX_MANIFEST_BYTES,
		(_heading, detail, status) => new Response(detail, { status }),
	);
	if (!file.ok) {
		// No manifest is the normal case for most projects, not an error.
		if (file.response.status === 404) return { ok: false, error: null };
		return {
			ok: false,
			error: `${DASHBOARD_MANIFEST_PATH}: ${await file.response.text()}`,
		};
	}

	let json: unknown;
	try {
		json = JSON.parse(new TextDecoder().decode(file.bytes));
	} catch {
		return {
			ok: false,
			error: `${DASHBOARD_MANIFEST_PATH} is not valid JSON.`,
		};
	}
	const parsed = dashboardManifestSchema.safeParse(json);
	if (!parsed.success) {
		const issue = parsed.error.issues[0];
		const where = issue?.path.length ? ` at ${issue.path.join(".")}` : "";
		return {
			ok: false,
			error: `${DASHBOARD_MANIFEST_PATH} is invalid${where}: ${issue?.message ?? "unknown error"}`,
		};
	}
	return { ok: true, dashboards: parsed.data.dashboards };
}

export async function dashboardsResponse(
	sessionSecret: string,
	workspace: DashboardWorkspace,
): Promise<Response> {
	const projectRoot = resolve(workspace.project_path);
	const manifest = await readManifest(projectRoot);
	if (!manifest.ok && manifest.error) {
		log.debug("dashboard manifest rejected", {
			workspaceId: workspace.id,
			error: manifest.error,
		});
	}

	const dashboards: ProjectDashboard[] = [];
	const seen = new Set<string>();
	for (const dashboard of manifest.ok ? manifest.dashboards : []) {
		// Two tabs for one entry would share an id; keep the first.
		if (seen.has(dashboard.entry)) continue;
		seen.add(dashboard.entry);
		const { token } = mintDashboardToken(
			sessionSecret,
			workspace.id,
			scopeFor(dashboard.entry),
		);
		dashboards.push({
			id: dashboard.entry,
			title: dashboard.title,
			entry: dashboard.entry,
			url: `/u/${workspace.slug}${DASHBOARD_FILES_PREFIX}${token}/${encodePath(dashboard.entry)}`,
		});
	}

	return jsonResponse({
		ok: true,
		dashboards,
		error: manifest.ok ? null : manifest.error,
		tokenExpiresIn: DASHBOARD_TOKEN_TTL_SECONDS,
	} satisfies DashboardsResponse);
}

// --- Serving -----------------------------------------------------------

function resourcePrefixFor(
	requestUrl: URL,
	slug: WorkspaceSlug,
	token: string,
): string {
	return `${requestUrl.origin}/u/${slug}${DASHBOARD_FILES_PREFIX}${token}/`;
}

/**
 * Like Preview's document policy — sandboxed, opaque origin, resources pinned
 * to this token's prefix — with two deliberate differences:
 *
 * - `connect-src 'none'` stays, and matters more here: the only way a
 *   dashboard reaches the robot is the `window.coderunner` bridge, so it
 *   cannot open sockets or phone home with telemetry.
 * - `media-src` and `blob:` images are allowed; dashboards draw.
 */
function dashboardCsp(resourcePrefix: string): string {
	return [
		"sandbox allow-scripts",
		"default-src 'none'",
		// The injected bridge is an inline script, as are the inline blocks
		// many build tools emit. The opaque origin is what contains them.
		`script-src ${resourcePrefix} 'unsafe-inline'`,
		`style-src ${resourcePrefix} 'unsafe-inline'`,
		`img-src ${resourcePrefix} data: blob:`,
		`font-src ${resourcePrefix} data:`,
		`media-src ${resourcePrefix} data: blob:`,
		"connect-src 'none'",
		"object-src 'none'",
		"frame-src 'none'",
		"child-src 'none'",
		"worker-src 'none'",
		"form-action 'none'",
		"navigate-to 'none'",
		"base-uri 'none'",
		"frame-ancestors 'self'",
	].join("; ");
}

function dashboardHeaders(contentType: string, csp?: string): Headers {
	const headers = previewHeaders(contentType, csp);
	// The frame's opaque origin makes every subresource request cross-origin
	// (`Origin: null`), and bundlers load ES modules and stylesheets in CORS
	// mode. The capability is the token in the path, not the origin, and no
	// credentials are involved, so a wildcard grants nothing extra.
	headers.set("access-control-allow-origin", "*");
	return headers;
}

function dashboardError(
	heading: string,
	detail: string,
	status: number,
): Response {
	return new Response(renderPreviewErrorDocument(heading, detail), {
		status,
		headers: previewHeaders(
			"text/html; charset=utf-8",
			[
				"sandbox",
				"default-src 'none'",
				"style-src 'unsafe-inline'",
				"frame-ancestors 'self'",
			].join("; "),
		),
	});
}

/** Missing build output is the common authoring mistake; say so plainly. */
function dashboardReadError(
	heading: string,
	detail: string,
	status: number,
): Response {
	if (status === 404) {
		return dashboardError(
			"Dashboard file not found",
			"This dashboard's files are missing from the project. Build the dashboard and copy its output to the path listed in .coderunner/dashboards.json.",
			404,
		);
	}
	return dashboardError(heading, detail, status);
}

/**
 * Serve one dashboard resource. Authenticated by the path token rather than
 * the session cookie, and confined to the directory the token was minted for.
 */
export async function dashboardFileResponse(
	sessionSecret: string,
	workspace: DashboardWorkspace,
	requestUrl: URL,
	encodedSuffix: string,
): Promise<Response> {
	const parsed = parsePreviewFileRequest(encodedSuffix);
	if (!parsed) {
		return dashboardError(
			"Not available",
			"That dashboard path is not valid.",
			400,
		);
	}
	const scope = verifyDashboardToken(sessionSecret, workspace.id, parsed.token);
	if (scope === null) {
		return dashboardError(
			"Dashboard link expired",
			"Reload the dashboard from its tab to reopen it.",
			403,
		);
	}
	if (!isInsideDashboardScope(scope, parsed.path)) {
		return dashboardError(
			"Not available",
			"Dashboards can only load files from their own directory.",
			403,
		);
	}

	const projectRoot = resolve(workspace.project_path);
	const extension = extname(parsed.path).toLowerCase();

	if (extension === ".html" || extension === ".htm") {
		const file = await readVerifiedProjectFile(
			projectRoot,
			parsed.path,
			MAX_DOCUMENT_BYTES,
			dashboardReadError,
		);
		if (!file.ok) return file.response;
		const html = injectDashboardBridge(new TextDecoder().decode(file.bytes));
		return new Response(html, {
			headers: dashboardHeaders(
				"text/html; charset=utf-8",
				dashboardCsp(
					resourcePrefixFor(requestUrl, workspace.slug, parsed.token),
				),
			),
		});
	}

	const contentType = DASHBOARD_ASSET_CONTENT_TYPES[extension];
	if (!contentType) {
		return dashboardError(
			"Not available",
			"Dashboards can only load pages, stylesheets, scripts, images and fonts.",
			415,
		);
	}
	const file = await readVerifiedProjectFile(
		projectRoot,
		parsed.path,
		MAX_ASSET_BYTES,
		dashboardReadError,
	);
	if (!file.ok) return file.response;

	// An SVG can be navigated to directly and can carry script.
	const csp = contentType.startsWith("image/svg")
		? "sandbox; default-src 'none'; style-src 'unsafe-inline'"
		: undefined;
	return new Response(file.bytes, {
		headers: dashboardHeaders(contentType, csp),
	});
}
