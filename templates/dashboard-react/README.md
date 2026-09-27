# CodeRunner dashboard template (React + Vite + TypeScript)

A starting point for a custom lesson dashboard: a web page that shows up as a
tab in CodeRunner and reads/writes the running robot program's NetworkTables.

Full guide: [Custom Dashboards](https://mathewdunne.github.io/CodeRunner/lessons/custom-dashboards)
(`docs/lessons/custom-dashboards.md` in the CodeRunner repo).

## Quick start

```sh
bun install          # or: npm install
bun run dev          # UI development with sample data, no robot needed
```

Outside CodeRunner, `src/coderunner/devMock.ts` installs an in-memory
`window.coderunner` that animates the topics the bundled `robot-starter` lesson
publishes, so you can build and style the dashboard on the Vite dev server.

## Ship it with a lesson

1. Build into the lesson module (adjust the path to your lessons repo):

   ```sh
   bun run build -- --outDir ../../modules/robot-starter/dashboard --emptyOutDir
   ```

2. Declare it in the module's `.coderunner/dashboards.json`:

   ```json
   {
     "dashboards": [
       { "title": "Robot Dashboard", "entry": "dashboard/index.html" }
     ]
   }
   ```

3. Commit the built files and the manifest, then push.

`vite.config.ts` sets `base: "./"`: CodeRunner serves the dashboard from a path
inside the student's project, so asset URLs must be relative.

## What's here

| Path | Purpose |
| --- | --- |
| `src/coderunner/hooks.ts` | `useNTValue`, `useNTConnection`, `useNTTopics`, `useCodeRunnerContext`, `useRobotState` |
| `src/coderunner/types.ts` | Types for `window.coderunner` |
| `src/coderunner/devMock.ts` | Sample-data stand-in for `vite dev` |
| `src/components/SendableChooser.tsx` | A WPILib `SendableChooser` picker (e.g. autonomous) |
| `src/components/NumberSetting.tsx` | A slider that publishes a number for robot code to read |
| `src/App.tsx` | Example layout: connection, robot state, counter, pose, controls, topic list |

```tsx
const [speed, setSpeed] = useNTValue<number>("/SmartDashboard/Speed", 0);
setSpeed(0.5);           // type inferred ("double"), or pass one: setSpeed(3, "int")
```

## Limits

Dashboards run in a sandbox: no network requests (`fetch`, WebSockets, CDN
scripts), no workers, and they can only load files from their own folder. Bundle
every dependency; talk to the robot only through `window.coderunner`.
