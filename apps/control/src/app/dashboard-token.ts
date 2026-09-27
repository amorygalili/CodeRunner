import type { WorkspaceId } from "@frc-coderunner/contracts";
import { pathTokenSignaturesMatch, signPathToken } from "./preview-token";

/**
 * Dashboard frames are sandboxed without `allow-same-origin` for the same
 * reason Preview frames are, so their assets are authorised by a path token
 * rather than the session cookie — see `preview-token.ts` for the full story.
 *
 * Two differences from the Preview token:
 *
 * - It is scoped to one directory (the dashboard's entry directory), signed
 *   into the token, so a dashboard can load its own build output and nothing
 *   else in the project.
 * - It lives much longer. A dashboard stays mounted for a whole session and
 *   may load code-split chunks or reload itself long after the tab opened;
 *   a fifteen-minute grant would break it mid-class. The shell re-mints on
 *   every list fetch (project swap, Reload).
 */

const TOKEN_VERSION = "d1";

export const DASHBOARD_TOKEN_TTL_SECONDS = 8 * 60 * 60;

/** Domain separation from every other HMAC derived from the session secret. */
const SIGNING_LABEL = "coderunner.dashboard.v1";

function sign(
	secret: string,
	workspaceId: string,
	scope: string,
	expiresAt: number,
): string {
	return signPathToken(
		secret,
		`${SIGNING_LABEL}.${workspaceId}.${expiresAt}.${scope}`,
	);
}

/**
 * Mint a token authorising reads under `scope` (a project-relative directory,
 * `""` for the project root) in `workspaceId`'s project.
 */
export function mintDashboardToken(
	secret: string,
	workspaceId: WorkspaceId,
	scope: string,
	nowSeconds: number = Math.floor(Date.now() / 1000),
): { token: string; expiresIn: number } {
	const expiresAt = nowSeconds + DASHBOARD_TOKEN_TTL_SECONDS;
	const encodedScope = Buffer.from(scope, "utf8").toString("base64url");
	const signature = sign(secret, workspaceId, scope, expiresAt);
	return {
		token: `${TOKEN_VERSION}.${expiresAt}.${encodedScope}.${signature}`,
		expiresIn: DASHBOARD_TOKEN_TTL_SECONDS,
	};
}

/**
 * The directory `token` grants for `workspaceId`, or null when the token is
 * malformed, expired, or was minted for a different workspace.
 */
export function verifyDashboardToken(
	secret: string,
	workspaceId: WorkspaceId,
	token: string,
	nowSeconds: number = Math.floor(Date.now() / 1000),
): string | null {
	const parts = token.split(".");
	if (parts.length !== 4) return null;
	const [version, expiresAtRaw, encodedScope, signature] = parts as [
		string,
		string,
		string,
		string,
	];
	if (version !== TOKEN_VERSION) return null;
	if (!/^\d{1,15}$/.test(expiresAtRaw)) return null;
	if (!/^[A-Za-z0-9_-]*$/.test(encodedScope)) return null;

	const expiresAt = Number(expiresAtRaw);
	if (expiresAt <= nowSeconds) return null;

	const scope = Buffer.from(encodedScope, "base64url").toString("utf8");
	return pathTokenSignaturesMatch(
		sign(secret, workspaceId, scope, expiresAt),
		signature,
	)
		? scope
		: null;
}

/** Whether a project-relative `path` lies inside the granted `scope` directory. */
export function isInsideDashboardScope(scope: string, path: string): boolean {
	return scope === "" || path.startsWith(`${scope}/`);
}
