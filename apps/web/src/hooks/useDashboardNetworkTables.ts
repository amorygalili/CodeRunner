import { useEffect, useState } from "react";
import { DashboardNetworkTables } from "@/lib/dashboard-host";

export type DashboardNetworkTablesState = {
	service: DashboardNetworkTables | null;
	connected: boolean;
};

/**
 * The page's single NT4 connection for custom dashboards.
 *
 * Only exists while `enabled` (the project declares at least one dashboard
 * and this is a robot layout), so a workspace without dashboards never opens
 * another NT4 socket or polls `/sim/alive`.
 */
export function useDashboardNetworkTables(
	workspaceSlug: string | null,
	enabled: boolean,
): DashboardNetworkTablesState {
	const [service, setService] = useState<DashboardNetworkTables | null>(null);
	const [connected, setConnected] = useState(false);

	useEffect(() => {
		if (!workspaceSlug || !enabled) {
			setService(null);
			setConnected(false);
			return;
		}
		const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
		const next = new DashboardNetworkTables({
			websocketUrl: `${protocol}//${window.location.host}/u/${workspaceSlug}/sim/nt4`,
			aliveUrl: `/u/${workspaceSlug}/sim/alive`,
		});
		const stopListening = next.onConnectionChange(setConnected);
		next.start();
		setService(next);
		return () => {
			stopListening();
			next.stop();
		};
	}, [workspaceSlug, enabled]);

	return { service, connected };
}
