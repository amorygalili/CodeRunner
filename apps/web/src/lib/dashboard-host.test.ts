import type { DashboardHostMessage } from "@frc-coderunner/contracts";
import { afterEach, beforeEach, describe, expect, test, vi } from "vitest";
import type { NT4ClientCallbacks, NT4Topic } from "@/lib/nt4/NT4";

// A stand-in NT4 client that records calls and lets tests drive callbacks.
const clients: FakeClient[] = [];
class FakeClient {
	callbacks: NT4ClientCallbacks;
	subscriptions = new Map<
		number,
		{ topics: string[]; prefix: boolean; options: unknown }
	>();
	published = new Map<string, string>();
	samples: Array<[string, unknown]> = [];
	connected = false;
	private nextUid = 1;

	constructor(_endpoint: unknown, callbacks: NT4ClientCallbacks) {
		this.callbacks = callbacks;
		clients.push(this);
	}
	connect() {
		this.connected = true;
	}
	disconnect() {
		this.connected = false;
	}
	subscribe(topics: string[], prefix: boolean, options: unknown = {}) {
		const uid = this.nextUid++;
		this.subscriptions.set(uid, { topics, prefix, options });
		return uid;
	}
	unsubscribe(uid: number) {
		this.subscriptions.delete(uid);
	}
	publishTopic(name: string, type: string) {
		if (!this.published.has(name)) this.published.set(name, type);
	}
	publishedType(name: string) {
		return this.published.get(name);
	}
	addSample(name: string, value: unknown) {
		this.samples.push([name, value]);
	}
	serverTimeUs() {
		return 42;
	}
}

vi.mock("@/lib/nt4/NT4", () => ({ NT4Client: FakeClient }));

const { DashboardNetworkTables, inferNtType } = await import(
	"@/lib/dashboard-host"
);

function topic(name: string, type: string, uid = 1): NT4Topic {
	return { uid, name, type, properties: {} };
}

function setup() {
	const service = new DashboardNetworkTables({
		websocketUrl: "ws://x/u/alice/sim/nt4",
		aliveUrl: "/u/alice/sim/alive",
	});
	service.start();
	const client = clients.at(-1)!;
	const sent: DashboardHostMessage[] = [];
	const port = service.attach((message) => sent.push(message));
	return { service, client, port, sent };
}

describe("DashboardNetworkTables", () => {
	beforeEach(() => {
		vi.useFakeTimers();
		clients.length = 0;
	});
	afterEach(() => {
		vi.useRealTimers();
	});

	test("subscribes to every announcement and every struct schema on start", () => {
		const { client } = setup();
		const subs = [...client.subscriptions.values()];
		expect(subs).toContainEqual({
			topics: [""],
			prefix: true,
			options: { topicsOnly: true },
		});
		expect(subs).toContainEqual({
			topics: ["/.schema/"],
			prefix: true,
			options: { all: true },
		});
		expect(client.connected).toBe(true);
	});

	test("forwards only subscribed topics, coalesced to the latest value", () => {
		const { client, port, sent } = setup();
		port.subscribe(1, ["/SmartDashboard/"], true, undefined);
		const speed = topic("/SmartDashboard/Speed", "double");
		const other = topic("/Other", "double", 2);
		client.callbacks.onTopicAnnounce(speed);
		client.callbacks.onNewTopicData(speed, 1, 1.5);
		client.callbacks.onNewTopicData(speed, 2, 2.5);
		client.callbacks.onNewTopicData(other, 2, 9);
		vi.advanceTimersByTime(50);

		const values = sent.filter((m) => m.kind === "values");
		expect(values).toHaveLength(1);
		expect(values[0]).toMatchObject({
			channel: "coderunner-host",
			updates: [{ topic: "/SmartDashboard/Speed", type: "double", value: 2.5 }],
		});
		expect(sent.find((m) => m.kind === "topics")).toMatchObject({
			announced: [{ name: "/SmartDashboard/Speed", type: "double" }],
			removed: [],
		});
	});

	test("replays cached values to a new subscription", () => {
		const { client, port, sent } = setup();
		const speed = topic("/SmartDashboard/Speed", "double");
		client.callbacks.onNewTopicData(speed, 1, 3);
		port.subscribe(7, ["/SmartDashboard/Speed"], false, 0.02);
		vi.advanceTimersByTime(50);
		expect(sent.at(-1)).toMatchObject({
			kind: "values",
			updates: [{ topic: "/SmartDashboard/Speed", value: 3 }],
		});
		// The NT4 subscription carries the requested period.
		const sub = [...client.subscriptions.values()].at(-1);
		expect(sub).toEqual({
			topics: ["/SmartDashboard/Speed"],
			prefix: false,
			options: { periodic: 0.02 },
		});
	});

	test("unsubscribe and detach release the NT4 subscriptions", () => {
		const { client, port } = setup();
		const baseline = client.subscriptions.size;
		port.subscribe(1, ["/a"], false, undefined);
		port.subscribe(2, ["/b"], false, undefined);
		expect(client.subscriptions.size).toBe(baseline + 2);
		port.unsubscribe(1);
		expect(client.subscriptions.size).toBe(baseline + 1);
		port.detach();
		expect(client.subscriptions.size).toBe(baseline);
	});

	test("init clears the previous document's subscriptions", () => {
		const { client, port, sent } = setup();
		const baseline = client.subscriptions.size;
		port.subscribe(1, ["/a"], false, undefined);
		port.sendInit({
			bridgeVersion: 1,
			theme: "dark",
			dashboard: { id: "d/index.html", title: "D" },
			moduleId: null,
			robot: null,
		});
		expect(client.subscriptions.size).toBe(baseline);
		expect(sent.at(-1)).toMatchObject({ kind: "init", connected: false });
	});

	test("publishes with the announced type and echoes the value locally", () => {
		const { client, port, sent } = setup();
		client.callbacks.onTopicAnnounce(topic("/Tuning/kP", "double"));
		port.subscribe(1, ["/Tuning/kP"], false, undefined);
		port.publish("/Tuning/kP", 0.5, undefined);
		expect(client.published.get("/Tuning/kP")).toBe("double");
		expect(client.samples).toEqual([["/Tuning/kP", 0.5]]);
		vi.advanceTimersByTime(50);
		expect(sent.at(-1)).toMatchObject({
			kind: "values",
			updates: [{ topic: "/Tuning/kP", value: 0.5, timestamp: 42 }],
		});
	});

	test("rejects values that do not fit the topic's type", () => {
		const { client, port, sent } = setup();
		client.callbacks.onTopicAnnounce(topic("/Tuning/kP", "double"));
		port.publish("/Tuning/kP", "fast", undefined);
		port.publish("/Tuning/kP", true, "boolean");
		port.publish("/Empty", [], undefined);
		expect(client.samples).toEqual([]);
		const errors = sent.filter((m) => m.kind === "error");
		expect(errors).toHaveLength(3);
	});

	test("serialises json topics", () => {
		const { client, port } = setup();
		port.publish("/Config", { a: 1 }, undefined);
		expect(client.published.get("/Config")).toBe("json");
		expect(client.samples).toEqual([["/Config", '{"a":1}']]);
	});

	test("reports connection changes to every port", () => {
		const { service, client, sent } = setup();
		const second: DashboardHostMessage[] = [];
		service.attach((message) => second.push(message));
		client.callbacks.onConnect();
		expect(service.isConnected()).toBe(true);
		expect(sent.at(-1)).toMatchObject({ kind: "connection", connected: true });
		expect(second.at(-1)).toMatchObject({
			kind: "connection",
			connected: true,
		});
	});
});

describe("inferNtType", () => {
	test("maps JavaScript values to NT4 types", () => {
		expect(inferNtType(true)).toBe("boolean");
		expect(inferNtType(1.5)).toBe("double");
		expect(inferNtType("x")).toBe("string");
		expect(inferNtType([1, 2])).toBe("double[]");
		expect(inferNtType(["a"])).toBe("string[]");
		expect(inferNtType([true])).toBe("boolean[]");
		expect(inferNtType({ a: 1 })).toBe("json");
		expect(inferNtType([])).toBeNull();
		expect(inferNtType([1, "a"])).toBeNull();
		expect(inferNtType(null)).toBeNull();
	});
});
