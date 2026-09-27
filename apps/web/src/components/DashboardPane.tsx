import { LayoutDashboard, Loader2, RefreshCw } from "lucide-react";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Button } from "@/components/ui/button";
import {
	DASHBOARD_BRIDGE_VERSION,
	type DashboardContext,
	type DashboardRobotState,
	dashboardClientMessageSchema,
	type ProjectDashboard,
} from "@/lib/contracts";
import type {
	DashboardNetworkTables,
	DashboardPort,
} from "@/lib/dashboard-host";
import { cn } from "@/lib/utils";

interface DashboardPaneProps {
	dashboard: ProjectDashboard;
	/** Null while no NT4 service exists (no simulator for this workspace). */
	service: DashboardNetworkTables | null;
	connected: boolean;
	/** Whether this dashboard's tab is the visible one. */
	active: boolean;
	moduleId: string | null;
	robot: DashboardRobotState | null;
	/** Re-read the manifest and re-mint frame URLs. */
	onReload: () => void;
}

function readTheme(): "light" | "dark" {
	return document.documentElement.classList.contains("dark") ? "dark" : "light";
}

/** The shell's resolved theme, tracked through the class `ThemeProvider` sets. */
function useResolvedTheme(): "light" | "dark" {
	const [theme, setTheme] = useState(readTheme);
	useEffect(() => {
		const observer = new MutationObserver(() => setTheme(readTheme()));
		observer.observe(document.documentElement, {
			attributes: true,
			attributeFilter: ["class"],
		});
		return () => observer.disconnect();
	}, []);
	return theme;
}

/**
 * Hosts one project-declared dashboard.
 *
 * The frame is sandboxed with `allow-scripts` and no `allow-same-origin`, like
 * Preview: author code runs, but in an opaque origin that cannot reach the
 * shell, the editor, or the session cookie. Everything it may do with the
 * robot goes through `window.coderunner`, which the control plane injects and
 * which talks to this component over postMessage. Messages are accepted only
 * from this frame's own window and are schema-checked before use.
 */
export function DashboardPane({
	dashboard,
	service,
	connected,
	active,
	moduleId,
	robot,
	onReload,
}: DashboardPaneProps) {
	const frameRef = useRef<HTMLIFrameElement>(null);
	const portRef = useRef<DashboardPort | null>(null);
	const frameLoadedRef = useRef(false);
	const [loaded, setLoaded] = useState(false);
	const [reloadCount, setReloadCount] = useState(0);
	// Mount the frame on first view only, so a dashboard nobody opens never
	// subscribes to anything. After that it stays mounted with its state.
	const [opened, setOpened] = useState(active);
	useEffect(() => {
		if (active) setOpened(true);
	}, [active]);

	const theme = useResolvedTheme();
	const context = useMemo<DashboardContext>(
		() => ({
			bridgeVersion: DASHBOARD_BRIDGE_VERSION,
			theme,
			dashboard: { id: dashboard.id, title: dashboard.title },
			moduleId,
			robot,
		}),
		[theme, dashboard.id, dashboard.title, moduleId, robot],
	);
	const contextRef = useRef(context);
	contextRef.current = context;

	useEffect(() => {
		portRef.current?.post({ kind: "context", context });
	}, [context]);

	// A new URL (project swap, re-minted token) or a Reload is a new frame
	// element and a new document. Runs before the port effect below.
	const frameKey = `${dashboard.url}#${reloadCount}`;
	// biome-ignore lint/correctness/useExhaustiveDependencies: `frameKey` is the reset trigger.
	useEffect(() => {
		frameLoadedRef.current = false;
		setLoaded(false);
	}, [frameKey]);

	// biome-ignore lint/correctness/useExhaustiveDependencies: `frameKey` and `opened` swap the frame element this effect binds to.
	useEffect(() => {
		const frame = frameRef.current;
		if (!service || !frame) return;

		let port: DashboardPort | null = null;
		const ensurePort = (): DashboardPort => {
			if (!port) {
				port = service.attach((message) => {
					// The frame's origin is opaque ("null"), so it cannot be named as
					// a target origin; `event.source` checks guard the other way.
					frame.contentWindow?.postMessage(message, "*");
				});
				portRef.current = port;
			}
			return port;
		};

		const onMessage = (event: MessageEvent) => {
			if (event.source === null || event.source !== frame.contentWindow) return;
			const parsed = dashboardClientMessageSchema.safeParse(event.data);
			if (!parsed.success) return;
			const message = parsed.data;
			switch (message.kind) {
				case "hello":
					// A fresh document: drop the previous one's subscriptions.
					ensurePort().sendInit(contextRef.current);
					break;
				case "subscribe":
					ensurePort().subscribe(
						message.subId,
						message.topics,
						message.prefix,
						message.periodic,
					);
					break;
				case "unsubscribe":
					port?.unsubscribe(message.subId);
					break;
				case "publish":
					ensurePort().publish(message.topic, message.value, message.type);
					break;
			}
		};

		window.addEventListener("message", onMessage);
		// A document that loaded before this service existed never gets a reply
		// to its hello; greet it now and it will re-send its subscriptions.
		if (frameLoadedRef.current) ensurePort().sendInit(contextRef.current);

		return () => {
			window.removeEventListener("message", onMessage);
			port?.detach();
			if (portRef.current === port) portRef.current = null;
		};
	}, [service, opened, frameKey]);

	const onFrameLoad = useCallback(() => {
		frameLoadedRef.current = true;
		setLoaded(true);
	}, []);

	const reload = useCallback(() => {
		setReloadCount((n) => n + 1);
		onReload();
	}, [onReload]);

	return (
		<aside
			className="flex h-full min-h-0 min-w-0 flex-col border-l border-border bg-card"
			data-pane="dashboard"
		>
			<div className="flex h-10 shrink-0 items-center gap-2 border-b border-border px-2 text-[12.5px]">
				<LayoutDashboard
					className="size-3.5 shrink-0 text-muted-foreground"
					aria-hidden="true"
				/>
				<span className="min-w-0 flex-1 truncate font-medium">
					{dashboard.title}
				</span>
				<span
					role="status"
					className="flex shrink-0 items-center gap-1.5 text-muted-foreground"
					data-testid="dashboard-nt-status"
				>
					<span
						className={cn(
							"size-2 rounded-full",
							connected ? "bg-emerald-500" : "bg-muted-foreground/40",
						)}
						aria-hidden="true"
					/>
					{connected ? "Robot connected" : "Waiting for robot program"}
				</span>
				<Button
					type="button"
					variant="outline"
					size="sm"
					className="h-7 shrink-0 gap-1.5 px-2 text-[12.5px]"
					onClick={reload}
				>
					<RefreshCw className="size-3.5" aria-hidden="true" />
					Reload
				</Button>
			</div>
			<div className="relative min-h-0 flex-1">
				{opened && (
					<iframe
						key={frameKey}
						ref={frameRef}
						title={`${dashboard.title} dashboard`}
						data-testid="dashboard-frame"
						src={dashboard.url}
						sandbox="allow-scripts"
						onLoad={onFrameLoad}
						className="absolute inset-0 h-full w-full border-0 bg-background"
					/>
				)}
				{opened && !loaded && (
					<div className="absolute inset-0 flex flex-col items-center justify-center gap-3 bg-card">
						<Loader2 className="size-8 animate-spin text-muted-foreground" />
						<span className="font-mono text-sm text-muted-foreground">
							Loading {dashboard.title}…
						</span>
					</div>
				)}
			</div>
		</aside>
	);
}
