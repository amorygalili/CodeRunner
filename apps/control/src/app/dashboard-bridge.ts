/**
 * The `window.coderunner` API every dashboard document gets.
 *
 * The control plane serves a dashboard's HTML with this script inlined as the
 * first thing in `<head>`, so the API exists synchronously before any of the
 * dashboard's own scripts run — whatever framework (or none) it was built
 * with. It is a thin client for the postMessage protocol in
 * `@frc-coderunner/contracts` (`DashboardClientMessage` /
 * `DashboardHostMessage`); the shell owns the one real NT4 connection.
 *
 * IMPORTANT: `installCodeRunnerBridge` is serialised with
 * `Function.prototype.toString` and runs inside the dashboard frame. It must
 * not reference anything outside its own body — no imports, no module-level
 * constants. Types are fine; Bun strips them before `toString` sees the code.
 * `dashboard-bridge.test.ts` executes the serialised script to hold this.
 */
function installCodeRunnerBridge(): void {
	type Unsubscribe = () => void;
	type TopicInfo = {
		name: string;
		type: string;
		properties: Record<string, unknown>;
	};
	type Entry = {
		topic: string;
		type: string;
		value: unknown;
		timestamp: number;
	};
	type Subscription = {
		topics: string[];
		prefix: boolean;
		periodic: number | undefined;
		callback: (value: unknown, entry: Entry) => void;
	};
	type HostMessage = {
		channel?: unknown;
		kind?: unknown;
		context?: unknown;
		connected?: unknown;
		topics?: TopicInfo[];
		announced?: TopicInfo[];
		removed?: string[];
		updates?: Entry[];
		message?: unknown;
	};

	const globalScope = window as unknown as { coderunner?: unknown };
	if (globalScope.coderunner !== undefined) return;

	const CLIENT_CHANNEL = "coderunner-dashboard";
	const HOST_CHANNEL = "coderunner-host";
	const host = window.parent !== window ? window.parent : null;

	let context: unknown = null;
	let connected = false;
	const topics = new Map<string, TopicInfo>();
	const values = new Map<string, Entry>();
	const subscriptions = new Map<number, Subscription>();
	let nextSubId = 1;

	const contextListeners = new Set<(context: unknown) => void>();
	const connectionListeners = new Set<(connected: boolean) => void>();
	const topicListeners = new Set<(topics: TopicInfo[]) => void>();

	let resolveReady: (context: unknown) => void = () => {};
	const ready = new Promise<unknown>((resolve) => {
		resolveReady = resolve;
	});

	function post(message: Record<string, unknown>): void {
		host?.postMessage({ channel: CLIENT_CHANNEL, ...message }, "*");
	}

	function safeCall<T>(listener: (value: T) => void, value: T): void {
		try {
			listener(value);
		} catch (error) {
			console.error("[coderunner] dashboard listener threw", error);
		}
	}

	function matches(subscription: Subscription, name: string): boolean {
		return subscription.prefix
			? subscription.topics.some((pattern) => name.startsWith(pattern))
			: subscription.topics.includes(name);
	}

	function deliver(entry: Entry): void {
		for (const subscription of subscriptions.values()) {
			if (!matches(subscription, entry.topic)) continue;
			try {
				subscription.callback(entry.value, entry);
			} catch (error) {
				console.error("[coderunner] NetworkTables subscriber threw", error);
			}
		}
	}

	function sendSubscribe(subId: number, subscription: Subscription): void {
		post({
			kind: "subscribe",
			subId,
			topics: subscription.topics,
			prefix: subscription.prefix,
			periodic: subscription.periodic,
		});
	}

	function topicList(): TopicInfo[] {
		return Array.from(topics.values());
	}

	function setConnected(next: boolean): void {
		if (next === connected) return;
		connected = next;
		for (const listener of connectionListeners) safeCall(listener, connected);
	}

	function setContext(next: unknown): void {
		context = next;
		for (const listener of contextListeners) safeCall(listener, context);
	}

	function inferType(value: unknown): string | null {
		if (typeof value === "boolean") return "boolean";
		if (typeof value === "number") return "double";
		if (typeof value === "string") return "string";
		if (Array.isArray(value)) {
			if (value.length === 0) return null;
			if (value.every((item) => typeof item === "boolean")) return "boolean[]";
			if (value.every((item) => typeof item === "number")) return "double[]";
			if (value.every((item) => typeof item === "string")) return "string[]";
			return null;
		}
		if (value !== null && typeof value === "object") return "json";
		return null;
	}

	window.addEventListener("message", (event: MessageEvent) => {
		if (host === null || event.source !== host) return;
		const data = event.data as HostMessage | null;
		if (!data || data.channel !== HOST_CHANNEL) return;

		switch (data.kind) {
			case "init": {
				topics.clear();
				for (const topic of data.topics ?? []) topics.set(topic.name, topic);
				setConnected(data.connected === true);
				setContext(data.context ?? null);
				for (const listener of topicListeners) safeCall(listener, topicList());
				// The shell may have restarted its side (a new NT client, a
				// remount); re-establish everything this document asked for.
				for (const [subId, subscription] of subscriptions) {
					sendSubscribe(subId, subscription);
				}
				resolveReady(context);
				break;
			}
			case "context":
				setContext(data.context ?? null);
				break;
			case "connection":
				setConnected(data.connected === true);
				break;
			case "topics": {
				for (const name of data.removed ?? []) topics.delete(name);
				for (const topic of data.announced ?? []) topics.set(topic.name, topic);
				for (const listener of topicListeners) safeCall(listener, topicList());
				break;
			}
			case "values":
				for (const entry of data.updates ?? []) {
					values.set(entry.topic, entry);
					deliver(entry);
				}
				break;
			case "error":
				console.warn("[coderunner]", data.message);
				break;
		}
	});

	const nt = Object.freeze({
		/** Whether the shell currently has a live NT4 connection to the robot program. */
		isConnected(): boolean {
			return connected;
		},
		onConnectionChange(listener: (connected: boolean) => void): Unsubscribe {
			connectionListeners.add(listener);
			return () => {
				connectionListeners.delete(listener);
			};
		},
		/**
		 * Receive values for a topic (or several). With `prefix: true` every
		 * topic starting with one of the given strings matches. The callback
		 * gets the latest cached value straight away unless `immediate` is false.
		 */
		subscribe(
			topicOrTopics: string | string[],
			callback: (value: unknown, entry: Entry) => void,
			options: {
				prefix?: boolean;
				periodic?: number;
				immediate?: boolean;
			} = {},
		): Unsubscribe {
			const list = Array.isArray(topicOrTopics)
				? topicOrTopics.slice()
				: [topicOrTopics];
			if (list.length === 0) return () => {};
			const subId = nextSubId++;
			const subscription: Subscription = {
				topics: list,
				prefix: options.prefix === true,
				periodic: options.periodic,
				callback,
			};
			subscriptions.set(subId, subscription);
			sendSubscribe(subId, subscription);
			if (options.immediate !== false) {
				const cached = Array.from(values.values()).filter((entry) =>
					matches(subscription, entry.topic),
				);
				queueMicrotask(() => {
					if (!subscriptions.has(subId)) return;
					for (const entry of cached) {
						try {
							callback(entry.value, entry);
						} catch (error) {
							console.error(
								"[coderunner] NetworkTables subscriber threw",
								error,
							);
						}
					}
				});
			}
			return () => {
				if (subscriptions.delete(subId)) post({ kind: "unsubscribe", subId });
			};
		},
		/** The latest value seen for `topic`, if any subscription has received one. */
		getValue(topic: string): unknown {
			return values.get(topic)?.value;
		},
		getEntry(topic: string): Entry | undefined {
			return values.get(topic);
		},
		/**
		 * Publish a value. `type` is an NT4 type string ("double", "string[]",
		 * "json", "struct:Pose2d", …); when omitted, the announced type of an
		 * existing topic is used, else one inferred from the value.
		 */
		setValue(topic: string, value: unknown, type?: string): void {
			const resolvedType = type ?? topics.get(topic)?.type ?? inferType(value);
			if (resolvedType === null) {
				console.warn(
					`[coderunner] Cannot infer an NT4 type for "${topic}"; pass one to setValue.`,
				);
				return;
			}
			const entry: Entry = {
				topic,
				type: resolvedType,
				value,
				timestamp: Date.now() * 1000,
			};
			values.set(topic, entry);
			deliver(entry);
			post({ kind: "publish", topic, type: resolvedType, value });
		},
		/** Every topic the robot program has announced. */
		getTopics(): TopicInfo[] {
			return topicList();
		},
		getTopic(name: string): TopicInfo | undefined {
			return topics.get(name);
		},
		onTopicsChange(listener: (topics: TopicInfo[]) => void): Unsubscribe {
			topicListeners.add(listener);
			return () => {
				topicListeners.delete(listener);
			};
		},
	});

	const api = Object.freeze({
		version: 1,
		/** True when running inside a CodeRunner dashboard tab. */
		embedded: host !== null,
		/** Resolves with the first context the shell sends. */
		ready,
		getContext(): unknown {
			return context;
		},
		onContextChange(listener: (context: unknown) => void): Unsubscribe {
			contextListeners.add(listener);
			return () => {
				contextListeners.delete(listener);
			};
		},
		nt,
	});

	Object.defineProperty(window, "coderunner", {
		value: api,
		enumerable: true,
		configurable: false,
		writable: false,
	});

	post({ kind: "hello" });
}

export const DASHBOARD_BRIDGE_SCRIPT = `(${installCodeRunnerBridge.toString()})();`;

/**
 * Insert the bridge as the first script the document runs: straight after
 * `<head>` when there is one, else after `<html>`, else at the very start.
 * Matching is case-insensitive and tolerates attributes on the tag.
 */
export function injectDashboardBridge(html: string): string {
	const tag = `<script data-coderunner-bridge>${DASHBOARD_BRIDGE_SCRIPT}</script>`;
	for (const pattern of [/<head(\s[^>]*)?>/i, /<html(\s[^>]*)?>/i]) {
		const match = pattern.exec(html);
		if (match) {
			const at = match.index + match[0].length;
			return `${html.slice(0, at)}${tag}${html.slice(at)}`;
		}
	}
	return `${tag}${html}`;
}
