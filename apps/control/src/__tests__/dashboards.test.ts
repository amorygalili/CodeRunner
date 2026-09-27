import { describe, expect, test } from "bun:test";
import { mkdir, symlink, writeFile } from "node:fs/promises";
import { dirname, join } from "node:path";
import type { DashboardsResponse } from "@frc-coderunner/contracts";
import type { ControlApp } from "../app";
import {
	DASHBOARD_BRIDGE_SCRIPT,
	injectDashboardBridge,
} from "../app/dashboard-bridge";
import {
	DASHBOARD_TOKEN_TTL_SECONDS,
	isInsideDashboardScope,
	mintDashboardToken,
	verifyDashboardToken,
} from "../app/dashboard-token";
import { mintPreviewToken } from "../app/preview-token";
import {
	cookieFrom,
	createFakeDocker,
	login,
	withApp,
	workspaceProjectPath,
} from "./helpers";

async function write(
	projectPath: string,
	relativePath: string,
	contents: string,
): Promise<void> {
	const target = join(projectPath, relativePath);
	await mkdir(dirname(target), { recursive: true });
	await writeFile(target, contents, "utf8");
}

const MANIFEST = ".coderunner/dashboards.json";

async function seedDashboard(projectPath: string): Promise<void> {
	await write(
		projectPath,
		MANIFEST,
		JSON.stringify({
			dashboards: [
				{ title: "Shooter", entry: "dashboards/shooter/index.html" },
				{ title: "Drive", entry: "dashboards/drive/index.html" },
			],
		}),
	);
	await write(
		projectPath,
		"dashboards/shooter/index.html",
		'<!doctype html><html><head><title>Shooter</title><script type="module" src="./assets/app.js"></script></head><body></body></html>',
	);
	await write(projectPath, "dashboards/shooter/assets/app.js", "export {};");
	await write(projectPath, "dashboards/shooter/assets/app.mjs", "export {};");
	await write(projectPath, "dashboards/shooter/assets/app.css", "body{}");
	await write(projectPath, "dashboards/shooter/data.json", "{}");
	await write(projectPath, "dashboards/drive/index.html", "<p>drive</p>");
	await write(projectPath, "src/main/java/Robot.java", "class Robot {}");
	await write(projectPath, "README.md", "# Robot\n");
}

async function listFor(
	app: ControlApp,
	slug: string,
	cookie: string,
): Promise<DashboardsResponse> {
	const resp = await app.fetch(
		new Request(`http://localhost/u/${slug}/api/dashboards`, {
			headers: { cookie },
		}),
	);
	expect(resp.status).toBe(200);
	return (await resp.json()) as DashboardsResponse;
}

function get(path: string): Request {
	return new Request(`http://localhost${path}`);
}

/** Swap the file name at the end of a dashboard URL. */
function sibling(url: string, relative: string): string {
	return `${url.slice(0, url.lastIndexOf("/") + 1)}${relative}`;
}

describe("dashboard tokens", () => {
	test("round-trips the scope for the workspace it was minted for", () => {
		const { token, expiresIn } = mintDashboardToken(
			"secret",
			"ws_a",
			"dashboards/shooter",
		);
		expect(expiresIn).toBe(DASHBOARD_TOKEN_TTL_SECONDS);
		expect(verifyDashboardToken("secret", "ws_a", token)).toBe(
			"dashboards/shooter",
		);
	});

	test("supports the project root as a scope", () => {
		const { token } = mintDashboardToken("secret", "ws_a", "");
		expect(verifyDashboardToken("secret", "ws_a", token)).toBe("");
	});

	test("rejects other workspaces, secrets, expiry and tampering", () => {
		const now = Math.floor(Date.now() / 1000);
		const { token } = mintDashboardToken("secret", "ws_a", "dash", now);
		expect(verifyDashboardToken("secret", "ws_b", token)).toBeNull();
		expect(verifyDashboardToken("other", "ws_a", token)).toBeNull();
		expect(
			verifyDashboardToken(
				"secret",
				"ws_a",
				token,
				now + DASHBOARD_TOKEN_TTL_SECONDS + 1,
			),
		).toBeNull();

		// Widening the scope without re-signing must not verify.
		const [version, expiresAt, , signature] = token.split(".");
		const widened = Buffer.from("", "utf8").toString("base64url");
		expect(
			verifyDashboardToken(
				"secret",
				"ws_a",
				`${version}.${expiresAt}.${widened}.${signature}`,
			),
		).toBeNull();
		expect(verifyDashboardToken("secret", "ws_a", "")).toBeNull();
		expect(verifyDashboardToken("secret", "ws_a", "d1.1.2")).toBeNull();
	});

	test("is not interchangeable with a Preview token", () => {
		const { token } = mintPreviewToken("secret", "ws_a");
		expect(verifyDashboardToken("secret", "ws_a", token)).toBeNull();
	});

	test("scope matching is by whole directory", () => {
		expect(isInsideDashboardScope("dash", "dash/index.html")).toBe(true);
		expect(isInsideDashboardScope("dash", "dash/assets/a.js")).toBe(true);
		expect(isInsideDashboardScope("dash", "dashboard/index.html")).toBe(false);
		expect(isInsideDashboardScope("dash", "src/Robot.java")).toBe(false);
		expect(isInsideDashboardScope("", "anything.html")).toBe(true);
	});
});

describe("dashboard bridge injection", () => {
	test("goes straight after <head>, tolerating attributes and case", () => {
		const out = injectDashboardBridge(
			'<!doctype html><HTML lang="en"><HEAD data-x="1"><title>t</title></HEAD></HTML>',
		);
		expect(out.indexOf("data-coderunner-bridge")).toBeGreaterThan(
			out.indexOf('<HEAD data-x="1">'),
		);
		expect(out.indexOf("data-coderunner-bridge")).toBeLessThan(
			out.indexOf("<title>"),
		);
	});

	test("does not mistake <header> for <head>", () => {
		const out = injectDashboardBridge("<html><body><header>x</header></body>");
		expect(out.startsWith("<html><script data-coderunner-bridge>")).toBe(true);
	});

	test("falls back to the start of a fragment", () => {
		expect(injectDashboardBridge("<p>hi</p>")).toMatch(
			/^<script data-coderunner-bridge>.*<\/script><p>hi<\/p>$/s,
		);
	});

	test("the serialised script is self-contained and installs window.coderunner", () => {
		// Runs the exact string we inline, in a minimal fake frame. This is what
		// catches a closure over module scope or a transpiler helper sneaking in.
		const posted: unknown[] = [];
		const listeners: Array<
			(event: { source: unknown; data: unknown }) => void
		> = [];
		const parent = {
			postMessage: (message: unknown) => posted.push(message),
		};
		const fakeWindow: Record<string, unknown> = {
			parent,
			addEventListener: (
				_type: string,
				listener: (event: { source: unknown; data: unknown }) => void,
			) => listeners.push(listener),
		};
		new Function("window", DASHBOARD_BRIDGE_SCRIPT)(fakeWindow);

		const api = fakeWindow.coderunner as {
			version: number;
			embedded: boolean;
			nt: {
				subscribe: (
					topic: string,
					callback: (value: unknown) => void,
				) => () => void;
				setValue: (topic: string, value: unknown) => void;
				getValue: (topic: string) => unknown;
				isConnected: () => boolean;
			};
		};
		expect(api.version).toBe(1);
		expect(api.embedded).toBe(true);
		expect(posted[0]).toEqual({
			channel: "coderunner-dashboard",
			kind: "hello",
		});

		const received: unknown[] = [];
		api.nt.subscribe("/SmartDashboard/Speed", (value) => received.push(value));
		expect(posted[1]).toMatchObject({
			kind: "subscribe",
			subId: 1,
			topics: ["/SmartDashboard/Speed"],
			prefix: false,
		});

		const hostSend = (data: unknown) => {
			for (const listener of listeners) listener({ source: parent, data });
		};
		hostSend({
			channel: "coderunner-host",
			kind: "init",
			connected: true,
			context: { theme: "dark" },
			topics: [],
		});
		expect(api.nt.isConnected()).toBe(true);
		hostSend({
			channel: "coderunner-host",
			kind: "values",
			updates: [
				{
					topic: "/SmartDashboard/Speed",
					type: "double",
					value: 3,
					timestamp: 1,
				},
				{ topic: "/Other", type: "double", value: 9, timestamp: 1 },
			],
		});
		expect(received).toEqual([3]);
		expect(api.nt.getValue("/SmartDashboard/Speed")).toBe(3);

		// Messages from anything but the parent are ignored.
		for (const listener of listeners) {
			listener({
				source: {},
				data: {
					channel: "coderunner-host",
					kind: "connection",
					connected: false,
				},
			});
		}
		expect(api.nt.isConnected()).toBe(true);

		api.nt.setValue("/SmartDashboard/Speed", 5);
		expect(received).toEqual([3, 5]);
		expect(posted.at(-1)).toEqual({
			channel: "coderunner-dashboard",
			kind: "publish",
			topic: "/SmartDashboard/Speed",
			type: "double",
			value: 5,
		});
	});
});

describe("GET /u/:slug/api/dashboards", () => {
	test("lists declared dashboards with scoped, tokenised URLs", async () => {
		const docker = createFakeDocker();
		await withApp(
			async (app) => {
				const cookie = cookieFrom(await login(app, "alice"));
				await seedDashboard(workspaceProjectPath(app, "alice"));

				const body = await listFor(app, "alice", cookie);
				expect(body.error).toBeNull();
				expect(body.tokenExpiresIn).toBe(DASHBOARD_TOKEN_TTL_SECONDS);
				expect(body.dashboards.map((d) => [d.id, d.title])).toEqual([
					["dashboards/shooter/index.html", "Shooter"],
					["dashboards/drive/index.html", "Drive"],
				]);
				const [shooter] = body.dashboards;
				expect(shooter?.url).toMatch(
					/^\/u\/alice\/api\/dashboards\/files\/d1\.[^/]+\/dashboards\/shooter\/index\.html$/,
				);
			},
			{ dockerRunner: docker.runner },
		);
	});

	test("an absent manifest is an empty list, not an error", async () => {
		const docker = createFakeDocker();
		await withApp(
			async (app) => {
				const cookie = cookieFrom(await login(app, "alice"));
				const body = await listFor(app, "alice", cookie);
				expect(body.dashboards).toEqual([]);
				expect(body.error).toBeNull();
			},
			{ dockerRunner: docker.runner },
		);
	});

	test("reports an invalid manifest instead of failing", async () => {
		const docker = createFakeDocker();
		await withApp(
			async (app) => {
				const cookie = cookieFrom(await login(app, "alice"));
				const project = workspaceProjectPath(app, "alice");

				await write(project, MANIFEST, "{ nope");
				let body = await listFor(app, "alice", cookie);
				expect(body.dashboards).toEqual([]);
				expect(body.error).toContain("not valid JSON");

				await write(
					project,
					MANIFEST,
					JSON.stringify({
						dashboards: [{ title: "Escape", entry: "../outside/index.html" }],
					}),
				);
				body = await listFor(app, "alice", cookie);
				expect(body.dashboards).toEqual([]);
				expect(body.error).toContain("dashboards.0.entry");

				await write(
					project,
					MANIFEST,
					JSON.stringify({
						dashboards: [{ title: "Script", entry: "dash/app.js" }],
					}),
				);
				body = await listFor(app, "alice", cookie);
				expect(body.error).toContain(".html");
			},
			{ dockerRunner: docker.runner },
		);
	});

	test("requires the owner's session", async () => {
		const docker = createFakeDocker();
		await withApp(
			async (app) => {
				await login(app, "alice");
				const bob = cookieFrom(await login(app, "bob"));
				const anonymous = await app.fetch(get("/u/alice/api/dashboards"));
				expect(anonymous.status).toBe(401);
				const other = await app.fetch(
					new Request("http://localhost/u/alice/api/dashboards", {
						headers: { cookie: bob },
					}),
				);
				expect(other.status).toBe(403);
			},
			{ dockerRunner: docker.runner },
		);
	});
});

describe("GET /u/:slug/api/dashboards/files/*", () => {
	test("serves the entry with the bridge injected, sandboxed and CORS-readable", async () => {
		const docker = createFakeDocker();
		await withApp(
			async (app) => {
				const cookie = cookieFrom(await login(app, "alice"));
				await seedDashboard(workspaceProjectPath(app, "alice"));
				const [shooter] = (await listFor(app, "alice", cookie)).dashboards;

				// No cookie: the frame's opaque origin never sends one.
				const resp = await app.fetch(get(shooter!.url));
				expect(resp.status).toBe(200);
				expect(resp.headers.get("content-type")).toContain("text/html");
				expect(resp.headers.get("access-control-allow-origin")).toBe("*");
				expect(resp.headers.get("cache-control")).toBe("no-store, private");
				expect(resp.headers.get("referrer-policy")).toBe("no-referrer");
				const csp = resp.headers.get("content-security-policy") ?? "";
				expect(csp).toContain("sandbox allow-scripts");
				expect(csp).not.toContain("allow-same-origin");
				expect(csp).toContain("connect-src 'none'");
				expect(csp).toContain(
					"script-src http://localhost/u/alice/api/dashboards/files/",
				);

				const html = await resp.text();
				expect(html).toContain("<head><script data-coderunner-bridge>");
				expect(html).toContain('src="./assets/app.js"');
			},
			{ dockerRunner: docker.runner },
		);
	});

	test("serves assets inside the dashboard directory only", async () => {
		const docker = createFakeDocker();
		await withApp(
			async (app) => {
				const cookie = cookieFrom(await login(app, "alice"));
				await seedDashboard(workspaceProjectPath(app, "alice"));
				const [shooter] = (await listFor(app, "alice", cookie)).dashboards;
				const url = shooter!.url;

				const js = await app.fetch(get(sibling(url, "assets/app.js")));
				expect(js.status).toBe(200);
				expect(js.headers.get("content-type")).toContain("text/javascript");
				expect(js.headers.get("access-control-allow-origin")).toBe("*");

				const mjs = await app.fetch(get(sibling(url, "assets/app.mjs")));
				expect(mjs.headers.get("content-type")).toContain("text/javascript");

				// Types outside the allowlist are refused even in scope.
				const json = await app.fetch(get(sibling(url, "data.json")));
				expect(json.status).toBe(415);

				// A token scoped to one dashboard cannot read another or the source.
				const base = url.slice(0, url.indexOf("/dashboards/shooter/"));
				const drive = await app.fetch(
					get(`${base}/dashboards/drive/index.html`),
				);
				expect(drive.status).toBe(403);
				const source = await app.fetch(get(`${base}/src/main/java/Robot.java`));
				expect(source.status).toBe(403);
				const traversal = await app.fetch(
					get(sibling(url, "..%2F..%2FREADME.md")),
				);
				expect(traversal.status).toBe(400);
			},
			{ dockerRunner: docker.runner },
		);
	});

	test("explains missing build output", async () => {
		const docker = createFakeDocker();
		await withApp(
			async (app) => {
				const cookie = cookieFrom(await login(app, "alice"));
				const project = workspaceProjectPath(app, "alice");
				await write(
					project,
					MANIFEST,
					JSON.stringify({
						dashboards: [{ title: "Unbuilt", entry: "dash/dist/index.html" }],
					}),
				);
				const [unbuilt] = (await listFor(app, "alice", cookie)).dashboards;
				const resp = await app.fetch(get(unbuilt!.url));
				expect(resp.status).toBe(404);
				expect(await resp.text()).toContain("Build the dashboard");
			},
			{ dockerRunner: docker.runner },
		);
	});

	test("rejects bad tokens, other workspaces and non-GET methods", async () => {
		const docker = createFakeDocker();
		await withApp(
			async (app) => {
				const cookie = cookieFrom(await login(app, "alice"));
				await login(app, "bob");
				await seedDashboard(workspaceProjectPath(app, "alice"));
				await seedDashboard(workspaceProjectPath(app, "bob"));
				const [shooter] = (await listFor(app, "alice", cookie)).dashboards;
				const url = shooter!.url;

				// Alice's token pasted under Bob's slug.
				const crossed = await app.fetch(
					get(url.replace("/u/alice/", "/u/bob/")),
				);
				expect(crossed.status).toBe(403);

				const forged = await app.fetch(
					get(
						"/u/alice/api/dashboards/files/d1.9999999999..AAAA/dashboards/shooter/index.html",
					),
				);
				expect(forged.status).toBe(403);

				const post = await app.fetch(
					new Request(`http://localhost${url}`, { method: "POST" }),
				);
				expect(post.status).toBe(405);
			},
			{ dockerRunner: docker.runner },
		);
	});

	test("refuses symlinks inside the dashboard directory", async () => {
		const docker = createFakeDocker();
		await withApp(
			async (app) => {
				const cookie = cookieFrom(await login(app, "alice"));
				const project = workspaceProjectPath(app, "alice");
				await seedDashboard(project);
				await symlink(
					join(project, "src/main/java/Robot.java"),
					join(project, "dashboards/shooter/assets/leak.js"),
				);
				const [shooter] = (await listFor(app, "alice", cookie)).dashboards;
				const resp = await app.fetch(
					get(sibling(shooter!.url, "assets/leak.js")),
				);
				expect(resp.status).toBe(403);
			},
			{ dockerRunner: docker.runner },
		);
	});
});
