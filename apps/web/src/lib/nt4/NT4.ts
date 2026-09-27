/**
 * NetworkTables 4 client for the browser.
 *
 * Adapted from AdvantageScope's `NT4.ts` (BSD-3-Clause, see
 * THIRD_PARTY_NOTICES.md), which is itself based on the WPILib reference
 * client. The one functional change mirrors CodeRunner's AdvantageScope patch
 * (`patches/advantagescope/001-lite-nt4-endpoint-injection.patch`): instead of
 * dialling `ws://<host>:5810/nt/<app>` directly, it connects to an injected
 * endpoint — the control plane's cookie-authenticated `/u/:slug/sim/nt4`
 * proxy — and probes readiness through `/u/:slug/sim/alive`.
 */
import { Decoder, Encoder } from "@msgpack/msgpack";

const TYPE_INDEX: Record<string, number> = {
	boolean: 0,
	double: 1,
	int: 2,
	float: 3,
	string: 4,
	json: 4,
	raw: 5,
	rpc: 5,
	msgpack: 5,
	protobuf: 5,
	"boolean[]": 16,
	"double[]": 17,
	"int[]": 18,
	"float[]": 19,
	"string[]": 20,
};

/** Binary (index 5) covers every structured type: struct:*, proto:*, etc. */
function typeIndexFor(type: string): number {
	return TYPE_INDEX[type] ?? 5;
}

export type NT4Endpoint = {
	/** Absolute `ws(s)://` URL of the NT4 WebSocket. */
	websocketUrl: string;
	/** Same-origin URL that answers 2xx once the NT4 server is reachable. */
	aliveUrl: string;
};

export type NT4Topic = {
	/** Server topic id, or our pubuid for a topic this client publishes. */
	uid: number;
	name: string;
	type: string;
	properties: Record<string, unknown>;
};

type Subscription = {
	uid: number;
	topics: string[];
	options: {
		periodic: number;
		all: boolean;
		topicsonly: boolean;
		prefix: boolean;
	};
};

export type NT4ClientCallbacks = {
	onTopicAnnounce: (topic: NT4Topic) => void;
	onTopicUnannounce: (topic: NT4Topic) => void;
	onNewTopicData: (
		topic: NT4Topic,
		timestampUs: number,
		value: unknown,
	) => void;
	onConnect: () => void;
	onDisconnect: () => void;
};

const PROTOCOL_V41 = "v4.1.networktables.first.wpi.edu";
const PROTOCOL_V40 = "networktables.first.wpi.edu";
const PROTOCOL_RTT = "rtt.networktables.first.wpi.edu";

const RTT_PERIOD_MS_V40 = 1000;
const RTT_PERIOD_MS_V41 = 250;
const TIMEOUT_MS_V40 = 5000;
const TIMEOUT_MS_V41 = 1000;
/**
 * How often to re-probe a simulator that is not running yet. Slower than
 * AdvantageScope's 350 ms: every probe is a control-plane request, and a
 * dashboard waiting for the student to press Start is not latency-sensitive.
 */
const ALIVE_RETRY_MS = 1000;

export class NT4Client {
	private readonly endpoint: NT4Endpoint;
	private readonly callbacks: NT4ClientCallbacks;

	private ws: WebSocket | null = null;
	private rttWs: WebSocket | null = null;
	private timestampInterval: number | null = null;
	private rttTimestampInterval: number | null = null;
	private disconnectTimeout: number | null = null;
	private aliveTimeout: number | null = null;
	private connectionActive = false;
	private connectionRequested = false;
	private serverTimeOffsetUs: number | null = null;

	private readonly subscriptions = new Map<number, Subscription>();
	private readonly publishedTopics = new Map<string, NT4Topic>();
	private readonly serverTopics = new Map<string, NT4Topic>();
	private readonly serverTopicsById = new Map<number, NT4Topic>();

	private readonly decoder = new Decoder();
	private readonly encoder = new Encoder();

	constructor(endpoint: NT4Endpoint, callbacks: NT4ClientCallbacks) {
		this.endpoint = endpoint;
		this.callbacks = callbacks;
	}

	// --- Public API ------------------------------------------------------

	/** Start connecting. Reconnects automatically until `disconnect()`. */
	connect(): void {
		if (this.connectionRequested) return;
		this.connectionRequested = true;
		this.timestampInterval = window.setInterval(() => {
			if (this.rttWs === null) this.sendTimestamp();
		}, RTT_PERIOD_MS_V40);
		this.rttTimestampInterval = window.setInterval(() => {
			if (this.rttWs !== null) this.sendTimestamp();
		}, RTT_PERIOD_MS_V41);
		void this.connectOnAlive();
	}

	/** Stop for good. A disconnected client is not reused. */
	disconnect(): void {
		if (!this.connectionRequested) return;
		this.connectionRequested = false;
		for (const handle of [this.timestampInterval, this.rttTimestampInterval]) {
			if (handle !== null) window.clearInterval(handle);
		}
		for (const handle of [this.disconnectTimeout, this.aliveTimeout]) {
			if (handle !== null) window.clearTimeout(handle);
		}
		if (this.ws) this.onClose(this.ws, "");
	}

	isConnected(): boolean {
		return this.connectionActive;
	}

	/**
	 * Subscribe to values. `periodic` is the server-side send interval in
	 * seconds; with `all` false only the latest value per period is sent.
	 */
	subscribe(
		topics: string[],
		prefix: boolean,
		options: { all?: boolean; periodic?: number; topicsOnly?: boolean } = {},
	): number {
		const subscription: Subscription = {
			uid: this.newUid(),
			topics,
			options: {
				periodic: options.periodic ?? 0.1,
				all: options.all ?? false,
				topicsonly: options.topicsOnly ?? false,
				prefix,
			},
		};
		this.subscriptions.set(subscription.uid, subscription);
		if (this.connectionActive) this.sendSubscribe(subscription);
		return subscription.uid;
	}

	unsubscribe(uid: number): void {
		if (!this.subscriptions.delete(uid)) return;
		if (this.connectionActive) {
			this.sendJson("unsubscribe", { subuid: uid });
		}
	}

	/** Announce a topic this client will publish. Idempotent per name. */
	publishTopic(name: string, type: string): void {
		if (this.publishedTopics.has(name)) return;
		const topic: NT4Topic = {
			uid: this.newUid(),
			name,
			type,
			properties: {},
		};
		this.publishedTopics.set(name, topic);
		if (this.connectionActive) this.sendPublish(topic);
	}

	/** The type a name was published with, if this client publishes it. */
	publishedType(name: string): string | undefined {
		return this.publishedTopics.get(name)?.type;
	}

	/** Send a value for a published topic, stamped with the current server time. */
	addSample(name: string, value: unknown): void {
		const topic = this.publishedTopics.get(name);
		if (!topic) throw new Error(`Topic "${name}" is not published.`);
		const timestamp = this.serverTimeUs() ?? 0;
		this.sendBinary(
			this.encoder.encode([
				topic.uid,
				timestamp,
				typeIndexFor(topic.type),
				value,
			]),
		);
	}

	/** Current server time in microseconds, or null before the first sync. */
	serverTimeUs(): number | null {
		return this.serverTimeOffsetUs === null
			? null
			: Date.now() * 1000 + this.serverTimeOffsetUs;
	}

	// --- Connection maintenance ------------------------------------------

	private async connectOnAlive(): Promise<void> {
		this.aliveTimeout = null;
		if (!this.connectionRequested) return;
		let alive = false;
		try {
			const response = await fetch(this.endpoint.aliveUrl, {
				credentials: "same-origin",
				signal: AbortSignal.timeout(1000),
			});
			alive = response.ok;
		} catch {
			alive = false;
		}
		if (!this.connectionRequested) return;
		if (alive) {
			this.openSocket(false);
		} else {
			this.aliveTimeout = window.setTimeout(
				() => void this.connectOnAlive(),
				ALIVE_RETRY_MS,
			);
		}
	}

	private openSocket(rtt: boolean): void {
		const ws = new WebSocket(
			this.endpoint.websocketUrl,
			rtt ? [PROTOCOL_RTT] : [PROTOCOL_V41, PROTOCOL_V40],
		);
		ws.binaryType = "arraybuffer";
		if (rtt) {
			this.rttWs = ws;
		} else {
			this.ws = ws;
		}
		ws.addEventListener("open", () => this.onOpen(ws));
		ws.addEventListener("message", (event: MessageEvent) =>
			this.onMessage(event, rtt),
		);
		if (!rtt) {
			// The RTT socket's lifetime follows the main one.
			ws.addEventListener("error", () => this.onError(ws));
			ws.addEventListener("close", (event: CloseEvent) =>
				this.onClose(ws, event.reason),
			);
		}
	}

	private onOpen(ws: WebSocket): void {
		if (ws.protocol === PROTOCOL_V41) {
			this.openSocket(true);
		} else {
			// v4.0 and RTT-only sockets send timestamps immediately.
			this.sendTimestamp();
		}
		if (ws.protocol === PROTOCOL_RTT) return;

		this.connectionActive = true;
		for (const topic of this.publishedTopics.values()) this.sendPublish(topic);
		for (const subscription of this.subscriptions.values()) {
			this.sendSubscribe(subscription);
		}
		this.callbacks.onConnect();
	}

	private onClose(source: WebSocket, reason: string): void {
		if (source !== this.ws) return;
		this.ws?.close();
		this.rttWs?.close();
		this.ws = null;
		this.rttWs = null;
		if (this.disconnectTimeout !== null) {
			window.clearTimeout(this.disconnectTimeout);
			this.disconnectTimeout = null;
		}
		if (this.connectionActive) {
			this.connectionActive = false;
			this.callbacks.onDisconnect();
		}
		for (const topic of this.serverTopics.values()) {
			this.callbacks.onTopicUnannounce(topic);
		}
		this.serverTopics.clear();
		this.serverTopicsById.clear();
		if (reason !== "") console.debug("[NT4] Socket closed:", reason);
		if (this.connectionRequested) void this.connectOnAlive();
	}

	private onError(source: WebSocket): void {
		if (source !== this.ws) return;
		this.ws?.close();
		this.rttWs?.close();
	}

	private resetTimeout(): void {
		if (this.disconnectTimeout !== null) {
			window.clearTimeout(this.disconnectTimeout);
		}
		const timeout = this.rttWs === null ? TIMEOUT_MS_V40 : TIMEOUT_MS_V41;
		this.disconnectTimeout = window.setTimeout(() => {
			if (this.ws) this.onClose(this.ws, `No data for ${timeout} ms`);
		}, timeout);
	}

	// --- Incoming --------------------------------------------------------

	private onMessage(event: MessageEvent, rttOnly: boolean): void {
		this.resetTimeout();
		if (typeof event.data === "string") {
			if (!rttOnly) this.onTextMessage(event.data);
			return;
		}
		for (const frame of this.decoder.decodeMulti(
			new Uint8Array(event.data as ArrayBuffer),
		)) {
			if (!Array.isArray(frame)) continue;
			const [topicId, timestampUs, , value] = frame as [
				unknown,
				unknown,
				unknown,
				unknown,
			];
			if (typeof topicId !== "number" || typeof timestampUs !== "number") {
				continue;
			}
			if (topicId === -1) {
				if (typeof value === "number") {
					this.onReceiveTimestamp(timestampUs, value);
				}
				continue;
			}
			if (rttOnly || topicId < 0) continue;
			const topic = this.serverTopicsById.get(topicId);
			if (topic) this.callbacks.onNewTopicData(topic, timestampUs, value);
		}
	}

	private onTextMessage(text: string): void {
		let messages: unknown;
		try {
			messages = JSON.parse(text);
		} catch {
			return;
		}
		if (!Array.isArray(messages)) return;
		for (const message of messages) {
			if (typeof message !== "object" || message === null) continue;
			const { method, params } = message as {
				method?: unknown;
				params?: Record<string, unknown>;
			};
			if (typeof method !== "string" || typeof params !== "object") continue;

			if (method === "announce") {
				const topic: NT4Topic = {
					uid: Number(params.id),
					name: String(params.name),
					type: String(params.type),
					properties: (params.properties as Record<string, unknown>) ?? {},
				};
				this.serverTopics.set(topic.name, topic);
				this.serverTopicsById.set(topic.uid, topic);
				this.callbacks.onTopicAnnounce(topic);
			} else if (method === "unannounce") {
				const topic = this.serverTopics.get(String(params.name));
				if (!topic) continue;
				this.serverTopics.delete(topic.name);
				this.serverTopicsById.delete(topic.uid);
				this.callbacks.onTopicUnannounce(topic);
			} else if (method === "properties") {
				const topic = this.serverTopics.get(String(params.name));
				const update = params.update as Record<string, unknown> | undefined;
				if (!topic || !update) continue;
				for (const [key, value] of Object.entries(update)) {
					if (value === null) {
						delete topic.properties[key];
					} else {
						topic.properties[key] = value;
					}
				}
			}
		}
	}

	private onReceiveTimestamp(serverUs: number, clientSentUs: number): void {
		const receivedUs = Date.now() * 1000;
		const latencyUs = (receivedUs - clientSentUs) / 2;
		this.serverTimeOffsetUs = serverUs + latencyUs - receivedUs;
	}

	// --- Outgoing --------------------------------------------------------

	private sendTimestamp(): void {
		this.sendBinary(
			this.encoder.encode([-1, 0, TYPE_INDEX.int, Date.now() * 1000]),
			this.rttWs !== null,
		);
	}

	private sendSubscribe(subscription: Subscription): void {
		this.sendJson("subscribe", {
			topics: subscription.topics,
			subuid: subscription.uid,
			options: subscription.options,
		});
	}

	private sendPublish(topic: NT4Topic): void {
		this.sendJson("publish", {
			name: topic.name,
			type: topic.type,
			pubuid: topic.uid,
			properties: topic.properties,
		});
	}

	private sendJson(method: string, params: Record<string, unknown>): void {
		if (this.ws?.readyState === WebSocket.OPEN) {
			this.ws.send(JSON.stringify([{ method, params }]));
		}
	}

	private sendBinary(data: Uint8Array, rtt = false): void {
		const ws = rtt ? this.rttWs : this.ws;
		if (ws?.readyState === WebSocket.OPEN) ws.send(data);
	}

	private newUid(): number {
		return Math.floor(Math.random() * 99_999_999);
	}
}
