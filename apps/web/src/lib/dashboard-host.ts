import {
	DASHBOARD_HOST_CHANNEL,
	type DashboardContext,
	type DashboardHostMessage,
	type DashboardTopicInfo,
	type DashboardValueUpdate,
} from "@frc-coderunner/contracts";
import { NT4Client, type NT4Endpoint, type NT4Topic } from "@/lib/nt4/NT4";
import { StructDecoder } from "@/lib/nt4/StructDecoder";

/**
 * The shell's side of the dashboard bridge.
 *
 * One `DashboardNetworkTables` per workspace page owns the only NT4
 * connection (through the control plane's authenticated `/sim/nt4` proxy).
 * Each dashboard frame gets a `DashboardPort` that holds that frame's
 * subscriptions and batches what it is sent, so a busy AdvantageKit robot
 * does not turn into one postMessage per sample.
 */

type Sink = (message: DashboardHostMessage) => void;

type HostPayload = DashboardHostMessage extends infer M
	? M extends DashboardHostMessage
		? Omit<M, "channel">
		: never
	: never;

const SCHEMA_PREFIX = "/.schema/struct:";
const STRUCT_PREFIX = "struct:";
/** Default NT4 `periodic` (seconds) for dashboard subscriptions. */
const DEFAULT_PERIODIC = 0.05;
/** How long a port coalesces updates before posting them to its frame. */
const FLUSH_MS = 20;

const SCALAR_CHECKS: Record<string, (value: unknown) => boolean> = {
	boolean: (value) => typeof value === "boolean",
	double: (value) => typeof value === "number" && Number.isFinite(value),
	float: (value) => typeof value === "number" && Number.isFinite(value),
	int: (value) => typeof value === "number" && Number.isInteger(value),
	string: (value) => typeof value === "string",
};

/** A best guess at the NT4 type for a value published without one. */
export function inferNtType(value: unknown): string | null {
	if (typeof value === "boolean") return "boolean";
	if (typeof value === "number") return "double";
	if (typeof value === "string") return "string";
	if (Array.isArray(value) && value.length > 0) {
		if (value.every((item) => typeof item === "boolean")) return "boolean[]";
		if (value.every((item) => typeof item === "number")) return "double[]";
		if (value.every((item) => typeof item === "string")) return "string[]";
		return null;
	}
	if (value !== null && typeof value === "object" && !Array.isArray(value)) {
		return "json";
	}
	return null;
}

function topicInfo(topic: NT4Topic): DashboardTopicInfo {
	return {
		name: topic.name,
		type: topic.type,
		properties: { ...topic.properties },
	};
}

function matchesPatterns(
	patterns: string[],
	prefix: boolean,
	name: string,
): boolean {
	return prefix
		? patterns.some((pattern) => name.startsWith(pattern))
		: patterns.includes(name);
}

export class DashboardNetworkTables {
	private readonly client: NT4Client;
	private readonly structs = new StructDecoder();
	private readonly topics = new Map<string, DashboardTopicInfo>();
	private readonly values = new Map<string, DashboardValueUpdate>();
	private readonly ports = new Set<DashboardPort>();
	private readonly connectionListeners = new Set<
		(connected: boolean) => void
	>();
	private connected = false;
	private started = false;

	constructor(endpoint: NT4Endpoint) {
		this.client = new NT4Client(endpoint, {
			onTopicAnnounce: (topic) => this.onAnnounce(topic),
			onTopicUnannounce: (topic) => this.onUnannounce(topic),
			onNewTopicData: (topic, timestamp, value) =>
				this.onData(topic, timestamp, value),
			onConnect: () => this.setConnected(true),
			onDisconnect: () => this.setConnected(false),
		});
	}

	start(): void {
		if (this.started) return;
		this.started = true;
		// Every announcement (not values), so `getTopics()` is complete…
		this.client.subscribe([""], true, { topicsOnly: true });
		// …and every struct schema, so struct topics arrive decoded.
		this.client.subscribe(["/.schema/"], true, { all: true });
		this.client.connect();
	}

	stop(): void {
		this.client.disconnect();
		for (const port of [...this.ports]) port.detach();
		this.setConnected(false);
	}

	isConnected(): boolean {
		return this.connected;
	}

	onConnectionChange(listener: (connected: boolean) => void): () => void {
		this.connectionListeners.add(listener);
		return () => {
			this.connectionListeners.delete(listener);
		};
	}

	topicList(): DashboardTopicInfo[] {
		return [...this.topics.values()];
	}

	/** Connect one dashboard frame. `sink` posts a message into that frame. */
	attach(sink: Sink): DashboardPort {
		const port = new DashboardPort(this, sink);
		this.ports.add(port);
		return port;
	}

	// --- Used by ports -----------------------------------------------------

	/** @internal */
	release(port: DashboardPort): void {
		this.ports.delete(port);
	}

	/** @internal */
	ntSubscribe(topics: string[], prefix: boolean, periodic: number): number {
		return this.client.subscribe(topics, prefix, { periodic });
	}

	/** @internal */
	ntUnsubscribe(uid: number): void {
		this.client.unsubscribe(uid);
	}

	/** @internal */
	cachedValues(): Iterable<DashboardValueUpdate> {
		return this.values.values();
	}

	/**
	 * Publish a value from a dashboard. Returns an error message for the
	 * frame's console, or null on success.
	 */
	publish(
		topic: string,
		value: unknown,
		requestedType?: string,
	): string | null {
		const type =
			requestedType ??
			this.topics.get(topic)?.type ??
			this.client.publishedType(topic) ??
			inferNtType(value);
		if (!type) {
			return `Cannot infer an NT4 type for "${topic}"; pass one to setValue.`;
		}
		const announced = this.topics.get(topic)?.type;
		if (announced && announced !== type) {
			return `"${topic}" is a ${announced} topic; cannot publish a ${type} to it.`;
		}

		let serialized: unknown;
		try {
			serialized = this.serialize(type, value);
		} catch (error) {
			return error instanceof Error ? error.message : String(error);
		}

		this.client.publishTopic(topic, type);
		this.client.addSample(topic, serialized);
		// NT4 servers do not echo a client's own writes back to it, so record
		// the value here for every other frame (and later subscribers).
		this.record({
			topic,
			type,
			value,
			timestamp: this.client.serverTimeUs() ?? Date.now() * 1000,
		});
		return null;
	}

	// --- NT4 callbacks -----------------------------------------------------

	private setConnected(connected: boolean): void {
		if (connected === this.connected) return;
		this.connected = connected;
		for (const listener of this.connectionListeners) listener(connected);
		for (const port of this.ports) port.post({ kind: "connection", connected });
	}

	private onAnnounce(topic: NT4Topic): void {
		const info = topicInfo(topic);
		this.topics.set(topic.name, info);
		for (const port of this.ports) port.queueTopic(info, null);
	}

	private onUnannounce(topic: NT4Topic): void {
		this.topics.delete(topic.name);
		for (const port of this.ports) port.queueTopic(null, topic.name);
	}

	private onData(topic: NT4Topic, timestamp: number, raw: unknown): void {
		if (topic.name.startsWith(SCHEMA_PREFIX) && raw instanceof Uint8Array) {
			this.structs.addSchema(topic.name.slice(SCHEMA_PREFIX.length), raw);
		}
		this.record({
			topic: topic.name,
			type: topic.type,
			value: this.decode(topic.type, raw),
			timestamp,
		});
	}

	private record(update: DashboardValueUpdate): void {
		this.values.set(update.topic, update);
		for (const port of this.ports) port.offer(update);
	}

	private decode(type: string, raw: unknown): unknown {
		if (type === "json" && typeof raw === "string") {
			try {
				return JSON.parse(raw);
			} catch {
				return raw;
			}
		}
		if (type.startsWith(STRUCT_PREFIX) && raw instanceof Uint8Array) {
			const name = type.slice(STRUCT_PREFIX.length);
			try {
				return name.endsWith("[]")
					? this.structs.decodeArray(name.slice(0, -2), raw).data
					: this.structs.decode(name, raw).data;
			} catch {
				// Schema not received yet: hand over the bytes rather than nothing.
				return raw;
			}
		}
		return raw;
	}

	private serialize(type: string, value: unknown): unknown {
		const scalar = SCALAR_CHECKS[type];
		if (scalar) {
			if (!scalar(value)) throw new Error(`Value is not a valid ${type}.`);
			return value;
		}
		if (type.endsWith("[]") && SCALAR_CHECKS[type.slice(0, -2)]) {
			const check = SCALAR_CHECKS[type.slice(0, -2)]!;
			if (!Array.isArray(value) || !value.every(check)) {
				throw new Error(`Value is not a valid ${type}.`);
			}
			return value;
		}
		if (type === "json") return JSON.stringify(value);
		if (type.startsWith(STRUCT_PREFIX)) {
			const name = type.slice(STRUCT_PREFIX.length);
			if (name.endsWith("[]")) {
				if (!Array.isArray(value)) throw new Error(`Value is not a ${type}.`);
				return this.structs.encodeArray(name.slice(0, -2), value);
			}
			return this.structs.encode(name, value);
		}
		if (value instanceof Uint8Array) return value;
		throw new Error(`Publishing ${type} topics is not supported.`);
	}
}

type PortSubscription = { topics: string[]; prefix: boolean; uid: number };

export class DashboardPort {
	private readonly host: DashboardNetworkTables;
	private readonly sink: Sink;
	private readonly subscriptions = new Map<number, PortSubscription>();
	private readonly pendingValues = new Map<string, DashboardValueUpdate>();
	private readonly pendingAnnounced = new Map<string, DashboardTopicInfo>();
	private readonly pendingRemoved = new Set<string>();
	private flushTimer: number | null = null;
	private detached = false;

	constructor(host: DashboardNetworkTables, sink: Sink) {
		this.host = host;
		this.sink = sink;
	}

	post(message: HostPayload): void {
		if (this.detached) return;
		this.sink({
			channel: DASHBOARD_HOST_CHANNEL,
			...message,
		} as DashboardHostMessage);
	}

	/** Greet a (re)loaded frame. The bridge re-sends its subscriptions in reply. */
	sendInit(context: DashboardContext): void {
		this.clearSubscriptions();
		this.pendingAnnounced.clear();
		this.pendingRemoved.clear();
		this.post({
			kind: "init",
			context,
			connected: this.host.isConnected(),
			topics: this.host.topicList(),
		});
	}

	subscribe(
		subId: number,
		topics: string[],
		prefix: boolean,
		periodic: number | undefined,
	): void {
		this.unsubscribe(subId);
		const uid = this.host.ntSubscribe(
			topics,
			prefix,
			periodic ?? DEFAULT_PERIODIC,
		);
		this.subscriptions.set(subId, { topics, prefix, uid });
		// Hand over what is already known rather than waiting a period.
		for (const update of this.host.cachedValues()) {
			if (matchesPatterns(topics, prefix, update.topic)) {
				this.pendingValues.set(update.topic, update);
			}
		}
		this.scheduleFlush();
	}

	unsubscribe(subId: number): void {
		const subscription = this.subscriptions.get(subId);
		if (!subscription) return;
		this.subscriptions.delete(subId);
		this.host.ntUnsubscribe(subscription.uid);
	}

	publish(topic: string, value: unknown, type: string | undefined): void {
		const error = this.host.publish(topic, value, type);
		if (error) this.post({ kind: "error", message: error });
	}

	detach(): void {
		if (this.detached) return;
		this.clearSubscriptions();
		if (this.flushTimer !== null) window.clearTimeout(this.flushTimer);
		this.flushTimer = null;
		this.detached = true;
		this.host.release(this);
	}

	/** @internal */
	offer(update: DashboardValueUpdate): void {
		for (const subscription of this.subscriptions.values()) {
			if (
				matchesPatterns(subscription.topics, subscription.prefix, update.topic)
			) {
				this.pendingValues.set(update.topic, update);
				this.scheduleFlush();
				return;
			}
		}
	}

	/** @internal */
	queueTopic(
		announced: DashboardTopicInfo | null,
		removed: string | null,
	): void {
		if (announced) {
			this.pendingRemoved.delete(announced.name);
			this.pendingAnnounced.set(announced.name, announced);
		}
		if (removed) {
			this.pendingAnnounced.delete(removed);
			this.pendingRemoved.add(removed);
		}
		this.scheduleFlush();
	}

	private clearSubscriptions(): void {
		for (const subId of [...this.subscriptions.keys()]) this.unsubscribe(subId);
		this.pendingValues.clear();
	}

	private scheduleFlush(): void {
		if (this.flushTimer !== null || this.detached) return;
		this.flushTimer = window.setTimeout(() => this.flush(), FLUSH_MS);
	}

	private flush(): void {
		this.flushTimer = null;
		if (this.pendingAnnounced.size > 0 || this.pendingRemoved.size > 0) {
			this.post({
				kind: "topics",
				announced: [...this.pendingAnnounced.values()],
				removed: [...this.pendingRemoved],
			});
			this.pendingAnnounced.clear();
			this.pendingRemoved.clear();
		}
		if (this.pendingValues.size > 0) {
			this.post({ kind: "values", updates: [...this.pendingValues.values()] });
			this.pendingValues.clear();
		}
	}
}
