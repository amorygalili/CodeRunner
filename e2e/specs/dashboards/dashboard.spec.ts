/**
 * Custom web dashboards end to end, in a real browser: a project-declared
 * dashboard appears as a sim-pane tab, loads its ES-module bundle from inside
 * a sandboxed opaque-origin frame, gets `window.coderunner` injected, receives
 * NetworkTables values relayed by the shell, and publishes back to the robot.
 */
import { mkdir, writeFile } from "node:fs/promises";
import { dirname, join } from "node:path";
import { expect, test } from "../../fixtures/app";
import { loginAs } from "../../fixtures/auth";
import { startFakeNt4 } from "../../fixtures/fake-nt4";
import { seedWorkspaceProject } from "../../fixtures/runtime";

async function writeProjectFile(
	projectPath: string,
	relativePath: string,
	contents: string,
): Promise<void> {
	const target = join(projectPath, relativePath);
	await mkdir(dirname(target), { recursive: true });
	await writeFile(target, contents, "utf8");
}

/** Minimal MessagePack for one NT4 value frame: [id, timestamp, type, double]. */
function doubleFrame(topicId: number, value: number): Uint8Array {
	const bytes = new Uint8Array(4 + 9);
	bytes.set([0x94, topicId, 0x00, 0x01, 0xcb]);
	new DataView(bytes.buffer).setFloat64(5, value);
	return bytes;
}

async function seedDashboardProject(projectPath: string): Promise<void> {
	await seedWorkspaceProject(projectPath);
	await writeProjectFile(
		projectPath,
		".coderunner/dashboards.json",
		JSON.stringify({
			dashboards: [{ title: "Test Dash", entry: "dashboard/index.html" }],
		}),
	);
	// Shaped like Vite output: a CORS-mode module script and stylesheet.
	await writeProjectFile(
		projectPath,
		"dashboard/index.html",
		`<!doctype html><html><head>
       <script type="module" crossorigin src="./assets/app.js"></script>
       <link rel="stylesheet" crossorigin href="./assets/app.css">
     </head><body>
       <p id="bridge">no bridge</p>
       <p id="value">none</p>
       <p id="status">disconnected</p>
       <p id="isolation">unknown</p>
       <button id="send" type="button">send</button>
     </body></html>`,
	);
	await writeProjectFile(
		projectPath,
		"dashboard/assets/app.css",
		"#value { color: rgb(0, 128, 0); }",
	);
	await writeProjectFile(
		projectPath,
		"dashboard/assets/app.js",
		`const cr = window.coderunner;
     document.getElementById("bridge").textContent = cr ? "bridge v" + cr.version : "no bridge";
     cr.nt.subscribe("/Test/Value", (value) => {
       document.getElementById("value").textContent = String(value);
     });
     const showStatus = (c) => {
       document.getElementById("status").textContent = c ? "connected" : "disconnected";
     };
     showStatus(cr.nt.isConnected());
     cr.nt.onConnectionChange(showStatus);
     try {
       void window.parent.document.title;
       document.getElementById("isolation").textContent = "LEAK";
     } catch {
       document.getElementById("isolation").textContent = "isolated";
     }
     document.getElementById("send").addEventListener("click", () => {
       cr.nt.setValue("/Test/Out", 7);
     });`,
	);
}

test("a project dashboard reads and writes NetworkTables through the bridge", async ({
	page,
	app,
	baseURL,
	runtime,
	fakeVscode,
	fakeHalsim,
}) => {
	const fakeNt4 = await startFakeNt4();
	let pump: ReturnType<typeof setInterval> | null = null;
	try {
		const login = await loginAs(page, app, { name: "dasher" });
		const workspace = app.storage.findWorkspaceBySlug(login.user.slug);
		if (!workspace) throw new Error("workspace missing after login");
		await seedDashboardProject(workspace.project_path);

		runtime.setRuntime({
			workspaceId: workspace.id,
			state: "running",
			image: "coderunner-workspace",
			runtimeName: `frc-${workspace.id.slice(0, 8)}`,
			ports: { nt4: 8080, vscode: 8081, halsim: 8082 },
			endpoints: {
				vscode: {
					httpBaseUrl: fakeVscode.httpBaseUrl,
					wsBaseUrl: fakeVscode.wsBaseUrl,
					basePath: "/",
				},
				nt4: { httpUrl: fakeNt4.httpUrl, wsUrl: fakeNt4.wsUrl },
				halsim: { wsUrl: fakeHalsim.wsUrl },
			},
			lastUsedAt: new Date().toISOString(),
			error: null,
		});

		await page.goto(`${baseURL}/u/${login.user.slug}/`);
		await page.getByRole("tab", { name: "Test Dash" }).click();

		const frame = page.frameLocator('[data-testid="dashboard-frame"]');
		// The bridge exists before the module runs, and the module and its
		// stylesheet load despite the frame's opaque origin.
		await expect(frame.locator("#bridge")).toHaveText("bridge v1");
		await expect(frame.locator("#value")).toHaveCSS("color", "rgb(0, 128, 0)");
		// Author code cannot reach into the shell.
		await expect(frame.locator("#isolation")).toHaveText("isolated");

		// The shell's NT4 client connects through the authenticated proxy.
		await fakeNt4.awaitConnection(1, 15_000);
		await expect(frame.locator("#status")).toHaveText("connected");
		await expect(page.getByTestId("dashboard-nt-status")).toHaveText(
			"Robot connected",
		);

		fakeNt4.pushFrame([
			{
				method: "announce",
				params: { name: "/Test/Value", type: "double", id: 5, properties: {} },
			},
		]);
		pump = setInterval(() => fakeNt4.pushBinary(doubleFrame(5, 3.5)), 100);
		await expect(frame.locator("#value")).toHaveText("3.5");

		await frame.locator("#send").click();
		await expect
			.poll(() =>
				fakeNt4
					.receivedFrames()
					.flatMap((f) => (Array.isArray(f) ? f : []))
					.some(
						(message) =>
							message?.method === "publish" &&
							message.params?.name === "/Test/Out" &&
							message.params?.type === "double",
					),
			)
			.toBe(true);
	} finally {
		if (pump) clearInterval(pump);
		await fakeNt4.stop();
	}
});

test("a project without a manifest shows no dashboard tabs", async ({
	page,
	app,
	baseURL,
}) => {
	const login = await loginAs(page, app, { name: "plain" });
	const workspace = app.storage.findWorkspaceBySlug(login.user.slug);
	await seedWorkspaceProject(workspace?.project_path ?? "");

	const listed = page.waitForResponse((response) =>
		response.url().endsWith("/api/dashboards"),
	);
	await page.goto(`${baseURL}/u/${login.user.slug}/`);
	expect(await (await listed).json()).toMatchObject({ dashboards: [] });
	await expect(page.getByRole("tab", { name: "Preview" })).toBeVisible();
	await expect(page.locator('[data-pane="dashboard"]')).toHaveCount(0);
});
