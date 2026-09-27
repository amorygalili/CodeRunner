import type { TabsTab } from "@base-ui/react/tabs";
import { FileText, LayoutDashboard } from "lucide-react";
import type { ReactNode } from "react";
import {
	createContext,
	useCallback,
	useContext,
	useEffect,
	useState,
} from "react";
import advantagescopeLogo from "@/assets/advantagescope-logo.png";
import pathplannerLogo from "@/assets/pathplanner-logo.png";
import {
	Tabs,
	TabsContent,
	TabsIndicator,
	TabsList,
	TabsTrigger,
} from "@/components/ui/tabs";

type BuiltInTab = "scope" | "pathplanner" | "preview";
/** A project dashboard's tab: `dashboard:<entry path>`. */
type DashboardTab = `dashboard:${string}`;
type SimPaneTab = BuiltInTab | DashboardTab;

const STORAGE_KEY = "coderunner:sim-pane-tab";
const DASHBOARD_TAB_PREFIX = "dashboard:";

export function dashboardTabValue(id: string): DashboardTab {
	return `${DASHBOARD_TAB_PREFIX}${id}`;
}

function parseTab(value: unknown): SimPaneTab {
	if (value === "pathplanner" || value === "preview") return value;
	if (
		typeof value === "string" &&
		value.startsWith(DASHBOARD_TAB_PREFIX) &&
		value.length > DASHBOARD_TAB_PREFIX.length
	) {
		return value as DashboardTab;
	}
	// AdvantageScope stays the default for anything unrecognised, including
	// values written by an older build.
	return "scope";
}

function readStoredTab(): SimPaneTab {
	try {
		return parseTab(sessionStorage.getItem(STORAGE_KEY));
	} catch {
		return "scope";
	}
}

const ActiveTabContext = createContext<SimPaneTab>("scope");

export type SimPaneDashboardTab = {
	id: string;
	title: string;
};

interface SimPaneTabsProps {
	className?: string;
	children: ReactNode;
	/** Notified whenever Preview becomes the active tab, to gate its first fetch. */
	onPreviewActivated?: () => void;
	/**
	 * Ids of the project's dashboards, or null while the list is loading. A
	 * remembered dashboard tab that the project no longer declares falls back
	 * to AdvantageScope; while loading, it waits rather than being forgotten.
	 */
	dashboardIds?: readonly string[] | null;
}

/**
 * Tabs root for the right-hand pane: AdvantageScope (default), PathPlanner,
 * Preview, then one tab per dashboard the project declares. The selector lives
 * in the topbar and the panels live in the pane, so the root has to wrap both
 * — hence a page-level provider. The choice persists for the session.
 */
export function SimPaneTabs({
	className,
	children,
	onPreviewActivated,
	dashboardIds = [],
}: SimPaneTabsProps) {
	const [tab, setTab] = useState<SimPaneTab>(readStoredTab);

	const effectiveTab: SimPaneTab =
		tab.startsWith(DASHBOARD_TAB_PREFIX) &&
		!dashboardIds?.includes(tab.slice(DASHBOARD_TAB_PREFIX.length))
			? "scope"
			: tab;

	// Keyed on the tab rather than on the change handler, so a session that
	// *starts* on Preview (restored from sessionStorage) activates it too.
	useEffect(() => {
		if (effectiveTab === "preview") onPreviewActivated?.();
	}, [effectiveTab, onPreviewActivated]);

	const onValueChange = useCallback((value: TabsTab.Value) => {
		const next = parseTab(value);
		setTab(next);
		try {
			sessionStorage.setItem(STORAGE_KEY, next);
		} catch {
			// Session storage unavailable (private mode); the toggle still works.
		}
	}, []);

	return (
		<ActiveTabContext.Provider value={effectiveTab}>
			<Tabs
				value={effectiveTab}
				onValueChange={onValueChange}
				className={className}
			>
				{children}
			</Tabs>
		</ActiveTabContext.Provider>
	);
}

/** Pill toggle rendered in the topbar. Must sit inside `SimPaneTabs`. */
export function SimPaneTabSelector({
	onReveal,
	dashboards = [],
}: {
	onReveal?: () => void;
	dashboards?: readonly SimPaneDashboardTab[];
} = {}) {
	return (
		<TabsList aria-label="Right pane" variant="pill" className="p-[3px]">
			<TabsIndicator />
			<TabsTrigger
				value="scope"
				onClick={onReveal}
				onFocus={onReveal}
				className="px-3 text-[12.5px]"
			>
				<img src={advantagescopeLogo} alt="" className="size-4 shrink-0" />
				AdvantageScope
			</TabsTrigger>
			<TabsTrigger
				value="pathplanner"
				onClick={onReveal}
				onFocus={onReveal}
				className="px-3 text-[12.5px]"
			>
				<img src={pathplannerLogo} alt="" className="size-4 shrink-0" />
				PathPlanner
			</TabsTrigger>
			<TabsTrigger
				value="preview"
				onClick={onReveal}
				onFocus={onReveal}
				className="px-3 text-[12.5px]"
			>
				<FileText className="size-4 shrink-0" aria-hidden="true" />
				Preview
			</TabsTrigger>
			{dashboards.map((dashboard) => (
				<TabsTrigger
					key={dashboard.id}
					value={dashboardTabValue(dashboard.id)}
					onClick={onReveal}
					onFocus={onReveal}
					className="max-w-48 px-3 text-[12.5px]"
				>
					<LayoutDashboard className="size-4 shrink-0" aria-hidden="true" />
					<span className="truncate">{dashboard.title}</span>
				</TabsTrigger>
			))}
		</TabsList>
	);
}

interface SimPanePanelsProps {
	scope: ReactNode;
	pathplanner: ReactNode;
	preview: ReactNode;
	/** One panel per dashboard; `render` learns whether its tab is showing. */
	dashboards?: readonly {
		id: string;
		render: (active: boolean) => ReactNode;
	}[];
}

/**
 * The right-pane panels. All stay mounted (`keepMounted`) — the hidden
 * iframes hold live state (an AdvantageScope session, PathPlanner's in-memory
 * working copy and save queue, Preview's selected document and scroll
 * position, a dashboard's UI) that unmounting would discard.
 */
export function SimPanePanels({
	scope,
	pathplanner,
	preview,
	dashboards = [],
}: SimPanePanelsProps) {
	const activeTab = useContext(ActiveTabContext);
	return (
		<div className="flex h-full min-h-0 flex-col">
			<TabsContent value="scope" keepMounted className="min-h-0 flex-1">
				{scope}
			</TabsContent>
			<TabsContent value="pathplanner" keepMounted className="min-h-0 flex-1">
				{pathplanner}
			</TabsContent>
			<TabsContent value="preview" keepMounted className="min-h-0 flex-1">
				{preview}
			</TabsContent>
			{dashboards.map((dashboard) => {
				const value = dashboardTabValue(dashboard.id);
				return (
					<TabsContent
						key={dashboard.id}
						value={value}
						keepMounted
						className="min-h-0 flex-1"
					>
						{dashboard.render(activeTab === value)}
					</TabsContent>
				);
			})}
		</div>
	);
}
