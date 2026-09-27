---
sidebar_position: 4
title: Custom Dashboards
---

# Custom Dashboards

A lesson can ship its own web dashboard: a page that reads and writes the
running robot program's NetworkTables. When the project declares one, it
appears as a tab beside **AdvantageScope**, **PathPlanner** and **Preview**.

Use a dashboard to give a lesson a purpose-built view: a shooter-tuning panel
with sliders, a pose readout with a field drawing, a checklist that lights up as
the student's code publishes the right values.

Dashboards can be written with any framework, or none. CodeRunner provides a
React + Vite + TypeScript starter in
[`templates/dashboard-react/`](https://github.com/mathewdunne/CodeRunner/tree/main/templates/dashboard-react),
and the bundled `robot-starter` lesson includes a plain HTML/JavaScript example
in its `dashboard/` folder.

## How it fits together

```text
modules/robot-starter/
  .coderunner/
    dashboards.json      ← declares the dashboard tab(s)
  dashboard/             ← the built dashboard: index.html + assets
    index.html
    assets/…
  src/main/java/…        ← the robot program it talks to
```

A dashboard lives **inside the project**, next to the robot code it talks to. It
is copied into the student's workspace with the rest of the module, so it works
the same for lessons from the bundled catalog, a remote lessons repository, or a
team repository imported from GitHub.

CodeRunner does not build dashboards. Commit the **built output** (for a Vite
project, the contents of `dist/`) into the module. Keep the dashboard's source
somewhere else in your lessons repository if you don't want students to see it.

## Declaring dashboards

Add `.coderunner/dashboards.json` to the module:

```json
{
  "dashboards": [
    { "title": "Robot Dashboard", "entry": "dashboard/index.html" }
  ]
}
```

| Field | Notes |
| --- | --- |
| `title` | Tab label, 1–40 characters. |
| `entry` | Project-relative path to the dashboard's `.html` page. No leading `/`, no `..`. |

Up to 8 dashboards per project. If the file is missing, the project simply has no
dashboard tabs. If it is invalid, the student sees an error notice naming the
problem and no tabs.

A dashboard may only load files from **its entry's directory and below**, so
keep each dashboard's page and assets together in one folder. Allowed file
types are HTML, CSS, JavaScript (`.js`, `.mjs`), images (`.png`, `.jpg`, `.gif`,
`.webp`, `.svg`, `.ico`) and fonts. Asset URLs must be **relative**
(`./assets/app.js`, not `/assets/app.js`).

Dashboard tabs appear only for robot lessons and imported robot projects, not
for `plain-java` lessons, because there is no robot program to talk to.

## The `window.coderunner` API

CodeRunner adds a `window.coderunner` object to every dashboard page before any
of the page's own scripts run. It is the dashboard's only connection to the
robot. CodeRunner keeps one NetworkTables connection for the whole page and
passes values to each dashboard.

```js
const nt = window.coderunner.nt;

// Read: called with the latest value, then again on every change.
const stop = nt.subscribe("/SmartDashboard/Speed", (value) => {
  document.getElementById("speed").textContent = value.toFixed(2);
});

// Read a whole table: every topic under a prefix.
nt.subscribe("/SmartDashboard/", (value, entry) => {
  console.log(entry.topic, value);
}, { prefix: true });

// Write.
nt.setValue("/SmartDashboard/SpeedScale", 0.5);
nt.setValue("/SmartDashboard/Counter", 3, "int"); // explicit NT4 type
```

### NetworkTables: `coderunner.nt`

| Method | Description |
| --- | --- |
| `subscribe(topic, callback, options?)` | Calls `callback(value, entry)` for each update. `topic` can be a string or an array of strings. Options: `prefix` (match every topic starting with the string), `periodic` (seconds between updates, default `0.05`), `immediate` (send the cached value straight away, default `true`). Returns an unsubscribe function. |
| `getValue(topic)` | The latest value received for a subscribed topic, or `undefined`. |
| `getEntry(topic)` | `{ topic, type, value, timestamp }` for the latest value. `timestamp` is robot time in microseconds. |
| `setValue(topic, value, type?)` | Publish a value. Without `type`, CodeRunner uses the topic's existing type, or infers one: `boolean`, `double`, `string`, their arrays, or `json` for objects. |
| `isConnected()` / `onConnectionChange(cb)` | Whether a robot program is running and connected. |
| `getTopics()` / `getTopic(name)` / `onTopicsChange(cb)` | Every topic the robot program has announced, as `{ name, type, properties }`. |

Value formats:

- Numbers, strings, booleans and their arrays arrive as-is.
- `json` topics are parsed into objects.
- `struct:` topics (what AdvantageKit and WPILib's `StructPublisher` produce)
  are decoded into plain objects. A `Pose2d` is
  `{ translation: { x, y }, rotation: { value } }`, in metres and radians.
- Anything else (raw bytes, protobuf) arrives as a `Uint8Array`.

Writing a `struct:` topic works too: pass the object and its type, for example
`nt.setValue("/Target", pose, "struct:Pose2d")`, once the robot has published
that struct's schema.

### Context: theme, lesson and robot state

| Member | Description |
| --- | --- |
| `getContext()` / `onContextChange(cb)` | `{ theme, dashboard, moduleId, robot }`. `theme` is `"light"` or `"dark"`, matching CodeRunner. `robot` is the Driver Station state: `{ running, enabled, mode, eStopped, alliance }`, or `null` until the simulator reports it. |
| `ready` | A promise that resolves with the first context. |
| `embedded` | `true` inside CodeRunner. |
| `version` | API version, currently `1`. |

## The React starter template

The template gives you typed hooks over the API:

```tsx
import { useNTValue, useNTConnection, useRobotState } from "./coderunner/hooks";

function SpeedPanel() {
  const connected = useNTConnection();
  const [speed] = useNTValue<number>("/SmartDashboard/Speed", 0);
  const [scale, setScale] = useNTValue<number>("/SmartDashboard/SpeedScale", 1);
  // …
}
```

To use it:

1. Copy `templates/dashboard-react/` into your lessons repository, for example
   as `dashboards-src/robot-starter/`.
2. `bun install` (or `npm install`), then `bun run dev`. Outside CodeRunner the
   template installs an in-memory mock of `window.coderunner` with sample data,
   so you can build the UI without a robot.
3. Build straight into the module:
   `bun run build -- --outDir ../../modules/robot-starter/dashboard --emptyOutDir`.
4. Add `.coderunner/dashboards.json` to the module pointing at
   `dashboard/index.html`, commit, and push.

The template's `vite.config.ts` already sets `base: "./"` so asset URLs are
relative. If you set up Vite yourself, do the same.

## Plain JavaScript

No build step is needed. The `robot-starter` lesson's `dashboard/` folder is a
complete example: an `index.html`, a stylesheet, and a script that subscribes to
the counter and pose the robot logs.

## What a dashboard cannot do

Dashboards run in a sandboxed frame, like Preview's HTML reports. A dashboard's
scripts run, but it cannot read the CodeRunner page, the student's session,
the editor, or files outside its own folder. It also cannot make network
requests: `fetch`, WebSockets, and loading scripts from a CDN are blocked. Bundle
every dependency into the build output. Robot data only reaches a dashboard
through `window.coderunner`.

## Troubleshooting

- **The tab says "Dashboard file not found".** The `entry` path does not exist.
  Check that the built files were committed to the module at that path.
- **Blank page, and the browser console shows 404s or blocked scripts for
  `/assets/…`.** Asset URLs are absolute. Set `base: "./"` (Vite) or the
  equivalent in your tool.
- **"Waiting for robot program".** Values arrive only while the robot is
  running. Click **Start** in the Driver Station.
- **Edited `dashboards.json` or rebuilt the dashboard.** Click **Reload** in the
  dashboard's toolbar.
