// A dashboard with no build step: plain HTML, CSS and JavaScript.
// CodeRunner injects `window.coderunner` into this page; see
// https://mathewdunne.github.io/CodeRunner/lessons/custom-dashboards
const nt = window.coderunner.nt;

// These are the topics RobotContainer.java logs with Logger.recordOutput(...).
const COUNTER = "/AdvantageKit/RealOutputs/Counter";
const POSE = "/AdvantageKit/RealOutputs/RobotPose";

// The field drawn below is 16.5 m x 8.2 m (a standard FRC field).
const FIELD_LENGTH = 16.54;
const FIELD_WIDTH = 8.21;

function showConnection(connected) {
	document.getElementById("nt-dot").classList.toggle("on", connected);
	document.getElementById("nt-status").textContent = connected
		? "Connected to robot"
		: "Waiting for robot…";
}
showConnection(nt.isConnected());
nt.onConnectionChange(showConnection);

window.coderunner.onContextChange((context) => {
	document.documentElement.dataset.theme = context.theme;
	const robot = context.robot;
	document.getElementById("robot-state").textContent = !robot
		? ""
		: robot.eStopped
			? "E-stopped"
			: robot.enabled
				? `Enabled · ${robot.mode}`
				: "Disabled";
});

nt.subscribe(COUNTER, (value) => {
	document.getElementById("counter").textContent = String(value);
});

// struct:Pose2d values arrive already decoded:
// { translation: { x, y }, rotation: { value } }  (metres, radians)
nt.subscribe(POSE, (pose) => {
	const { x, y } = pose.translation;
	const degrees = (pose.rotation.value * 180) / Math.PI;
	document.getElementById("pose").textContent =
		`x ${x.toFixed(2)} m · y ${y.toFixed(2)} m · ${degrees.toFixed(0)}°`;
	drawField(x, y, pose.rotation.value);
});

function drawField(x, y, heading) {
	const canvas = document.getElementById("field");
	const ctx = canvas.getContext("2d");
	const scale = canvas.width / FIELD_LENGTH;
	ctx.clearRect(0, 0, canvas.width, canvas.height);
	ctx.strokeStyle = getComputedStyle(document.body).getPropertyValue(
		"--border",
	);
	ctx.strokeRect(0.5, 0.5, canvas.width - 1, FIELD_WIDTH * scale - 1);

	// Field coordinates have +y pointing up; the canvas has +y pointing down.
	ctx.save();
	ctx.translate(x * scale, (FIELD_WIDTH - y) * scale);
	ctx.rotate(-heading);
	ctx.fillStyle = getComputedStyle(document.body).getPropertyValue("--accent");
	ctx.fillRect(-8, -8, 16, 16);
	ctx.fillStyle = "white";
	ctx.fillRect(4, -2, 6, 4); // front of the robot
	ctx.restore();
}
