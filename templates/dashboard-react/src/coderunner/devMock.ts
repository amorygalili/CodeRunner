import type {
	CodeRunnerApi,
	CodeRunnerContext,
	NTEntry,
	NTTopicInfo,
	Unsubscribe,
} from "./types";

/**
 * A stand-in `window.coderunner` for `bun run dev` outside CodeRunner.
 *
 * It keeps NetworkTables values in memory and animates a few sample topics
 * (the ones the bundled robot-starter lesson publishes), so the UI can be built
 * and styled without a robot. Writes are stored and echoed back like the real
 * thing. Only installed in development, and only when CodeRunner did not
 * already inject the real API.
 */
export function installDevMock(): void {
	if (window.coderunner) return;

	type Sub = {
		topics: string[];
		prefix: boolean;
		callback: (value: unknown, entry: NTEntry) => void;
	};

	const values = new Map<string, NTEntry>();
	const topics = new Map<string, NTTopicInfo>();
	const subs = new Set<Sub>();
	const topicListeners = new Set<(topics: NTTopicInfo[]) => void>();
	const contextListeners = new Set<(context: CodeRunnerContext) => void>();

	const context: CodeRunnerContext = {
		bridgeVersion: 1,
		theme: window.matchMedia("(prefers-color-scheme: dark)").matches
			? "dark"
			: "light",
		dashboard: { id: "dev", title: "Dev dashboard" },
		moduleId: null,
		robot: {
			running: true,
			enabled: false,
			mode: "teleop",
			eStopped: false,
			alliance: "red1",
		},
	};

	const matches = (sub: Sub, name: string) =>
		sub.prefix
			? sub.topics.some((pattern) => name.startsWith(pattern))
			: sub.topics.includes(name);

	function put(topic: string, type: string, value: unknown): void {
		if (!topics.has(topic)) {
			topics.set(topic, { name: topic, type, properties: {} });
			const list = [...topics.values()];
			for (const listener of topicListeners) listener(list);
		}
		const entry: NTEntry = { topic, type, value, timestamp: Date.now() * 1000 };
		values.set(topic, entry);
		for (const sub of subs) if (matches(sub, topic)) sub.callback(value, entry);
	}

	function inferType(value: unknown): string {
		if (typeof value === "boolean") return "boolean";
		if (typeof value === "number") return "double";
		if (typeof value === "string") return "string";
		if (Array.isArray(value)) {
			const first = value[0];
			return typeof first === "string"
				? "string[]"
				: typeof first === "boolean"
					? "boolean[]"
					: "double[]";
		}
		return "json";
	}

	// Sample data, shaped like the robot-starter lesson's AdvantageKit outputs.
	put("/SmartDashboard/Auto Choices/.type", "string", "String Chooser");
	put("/SmartDashboard/Auto Choices/options", "string[]", [
		"Do Nothing",
		"Drive Forward",
		"Score and Leave",
	]);
	put("/SmartDashboard/Auto Choices/default", "string", "Do Nothing");
	put("/SmartDashboard/Auto Choices/active", "string", "Do Nothing");
	put("/SmartDashboard/Auto Choices/selected", "string", "Do Nothing");
	let counter = 0;
	window.setInterval(() => {
		counter += 1;
		const seconds = counter * 0.02;
		put("/AdvantageKit/RealOutputs/Counter", "int", counter);
		put("/AdvantageKit/RealOutputs/RobotPose", "struct:Pose2d", {
			translation: {
				x: 4 + 2 * Math.cos(seconds),
				y: 4 + 2 * Math.sin(seconds),
			},
			rotation: { value: seconds + Math.PI / 2 },
		});
		// Echo a chooser selection back as "active", as a robot would.
		const selected = values.get("/SmartDashboard/Auto Choices/selected");
		const active = values.get("/SmartDashboard/Auto Choices/active");
		if (selected && selected.value !== active?.value) {
			put("/SmartDashboard/Auto Choices/active", "string", selected.value);
		}
	}, 20);

	const on = <T>(set: Set<T>, listener: T): Unsubscribe => {
		set.add(listener);
		return () => {
			set.delete(listener);
		};
	};

	const mock: CodeRunnerApi = {
		version: 1,
		embedded: false,
		ready: Promise.resolve(context),
		getContext: () => context,
		onContextChange: (listener) => on(contextListeners, listener),
		nt: {
			isConnected: () => true,
			onConnectionChange: () => () => {},
			subscribe(topic, callback, options = {}) {
				const sub: Sub = {
					topics: Array.isArray(topic) ? topic : [topic],
					prefix: options.prefix === true,
					callback: callback as Sub["callback"],
				};
				subs.add(sub);
				if (options.immediate !== false) {
					for (const entry of values.values()) {
						if (matches(sub, entry.topic)) sub.callback(entry.value, entry);
					}
				}
				return () => {
					subs.delete(sub);
				};
			},
			getValue: <T>(topic: string) => values.get(topic)?.value as T | undefined,
			getEntry: <T>(topic: string) =>
				values.get(topic) as NTEntry<T> | undefined,
			setValue(topic, value, type) {
				put(topic, type ?? topics.get(topic)?.type ?? inferType(value), value);
			},
			getTopics: () => [...topics.values()],
			getTopic: (name) => topics.get(name),
			onTopicsChange: (listener) => on(topicListeners, listener),
		},
	};

	window.coderunner = mock;
	console.info(
		"[coderunner] Not running inside CodeRunner: using an in-memory mock with sample data.",
	);
}
