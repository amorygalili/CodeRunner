import { useEffect } from "react";
import {
	useCodeRunnerContext,
	useNTConnection,
	useNTTopics,
	useNTValue,
	useRobotState,
} from "./coderunner/hooks";
import { NumberSetting } from "./components/NumberSetting";
import { SendableChooser } from "./components/SendableChooser";

// Topics the bundled robot-starter lesson publishes through AdvantageKit.
// Change these to match your own robot code.
const COUNTER_TOPIC = "/AdvantageKit/RealOutputs/Counter";
const POSE_TOPIC = "/AdvantageKit/RealOutputs/RobotPose";

type Pose2d = {
	translation: { x: number; y: number };
	rotation: { value: number };
};

export default function App() {
	const context = useCodeRunnerContext();
	const connected = useNTConnection();
	const robot = useRobotState();
	const [counter] = useNTValue<number>(COUNTER_TOPIC);
	const [pose] = useNTValue<Pose2d>(POSE_TOPIC);
	const topics = useNTTopics();

	// Follow CodeRunner's light/dark theme.
	useEffect(() => {
		document.documentElement.dataset.theme = context?.theme ?? "dark";
	}, [context?.theme]);

	return (
		<main className="dashboard">
			<header className="status-bar">
				<span className={connected ? "dot dot-on" : "dot"} />
				<span>{connected ? "Connected to robot" : "Waiting for robot…"}</span>
				{robot && (
					<span className="robot-state">
						{robot.eStopped
							? "E-stopped"
							: robot.enabled
								? `Enabled · ${robot.mode}`
								: "Disabled"}
					</span>
				)}
			</header>

			<section className="cards">
				<article className="card">
					<h2>Loop counter</h2>
					<p className="big-number">{counter ?? "—"}</p>
				</article>

				<article className="card">
					<h2>Robot pose</h2>
					{pose ? (
						<dl className="pose">
							<dt>x</dt>
							<dd>{pose.translation.x.toFixed(2)} m</dd>
							<dt>y</dt>
							<dd>{pose.translation.y.toFixed(2)} m</dd>
							<dt>θ</dt>
							<dd>{((pose.rotation.value * 180) / Math.PI).toFixed(0)}°</dd>
						</dl>
					) : (
						<p className="muted">No pose yet — start the robot.</p>
					)}
				</article>

				<article className="card">
					<h2>Controls</h2>
					<SendableChooser
						ntKey="/SmartDashboard/Auto Choices"
						label="Autonomous"
					/>
					<NumberSetting
						topic="/SmartDashboard/SpeedScale"
						label="Speed scale"
						min={0}
						max={1}
						step={0.05}
						defaultValue={1}
					/>
				</article>
			</section>

			<details className="card topics">
				<summary>{topics.length} NetworkTables topics</summary>
				<ul>
					{topics.map((topic) => (
						<li key={topic.name}>
							<code>{topic.name}</code> <span className="muted">{topic.type}</span>
						</li>
					))}
				</ul>
			</details>
		</main>
	);
}
