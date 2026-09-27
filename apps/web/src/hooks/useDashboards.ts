import { useCallback, useEffect, useRef, useState } from "react";
import {
	dashboardsResponseSchema,
	type ProjectDashboard,
} from "@/lib/contracts";

export type DashboardsState = {
	dashboards: ProjectDashboard[];
	/** Manifest problem to show the author, or a failed fetch. */
	error: string | null;
	loaded: boolean;
	/** Re-read `.coderunner/dashboards.json` and re-mint the frame tokens. */
	refresh: () => void;
};

/**
 * Loads `GET /api/dashboards` — the dashboards the current project declares.
 *
 * Fetched on mount, on project swap (`reloadNonce`) and on explicit refresh.
 * A lesson load replaces the whole project, so the swap is what matters; an
 * author editing the manifest in place clicks Reload.
 */
export function useDashboards(
	workspaceSlug: string | null,
	reloadNonce: number,
): DashboardsState {
	const [dashboards, setDashboards] = useState<ProjectDashboard[]>([]);
	const [error, setError] = useState<string | null>(null);
	const [loaded, setLoaded] = useState(false);
	const [refreshNonce, setRefreshNonce] = useState(0);
	const generationRef = useRef(0);

	const refresh = useCallback(() => setRefreshNonce((n) => n + 1), []);

	// A swap invalidates the old project's tabs immediately, not after refetch.
	// biome-ignore lint/correctness/useExhaustiveDependencies: `reloadNonce` is the invalidation trigger.
	useEffect(() => {
		setDashboards([]);
		setError(null);
		setLoaded(false);
	}, [reloadNonce]);

	// biome-ignore lint/correctness/useExhaustiveDependencies: `refreshNonce` is a manual refetch trigger.
	useEffect(() => {
		if (!workspaceSlug) {
			setDashboards([]);
			return;
		}
		generationRef.current += 1;
		const generation = generationRef.current;
		const controller = new AbortController();

		void (async () => {
			try {
				const response = await fetch(`/u/${workspaceSlug}/api/dashboards`, {
					credentials: "same-origin",
					signal: controller.signal,
				});
				if (!response.ok) {
					throw new Error(`Dashboard list failed (HTTP ${response.status}).`);
				}
				const parsed = dashboardsResponseSchema.parse(await response.json());
				if (generation !== generationRef.current) return;
				setDashboards(parsed.dashboards);
				setError(parsed.error);
				setLoaded(true);
			} catch (err) {
				if (controller.signal.aborted) return;
				if (generation !== generationRef.current) return;
				setDashboards([]);
				setError(
					err instanceof Error ? err.message : "Unable to load dashboards.",
				);
				setLoaded(true);
			}
		})();

		return () => controller.abort();
	}, [workspaceSlug, reloadNonce, refreshNonce]);

	return { dashboards, error, loaded, refresh };
}
