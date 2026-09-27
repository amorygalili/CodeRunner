# 043 — Custom web dashboards

Status: **Accepted** — 2026-09-27

## Context

Lesson authors want to give a robot lesson its own purpose-built view (a
tuning panel, a pose readout, a checklist) that reads and writes the running
robot program's NetworkTables. AdvantageScope is a general-purpose tool;
authors need a page of their own, written in whatever framework they like, shown
as a tab beside AdvantageScope, PathPlanner and Preview.

Two constraints shape the design:

1. **Dashboard code is untrusted.** It comes from a lessons repository or a
   student's imported team repository. It must not be able to read the shell,
   the editor, the session cookie, or other students' data. This is the same
   problem as Preview's project HTML (decision 041).
2. **NT4 is behind the session cookie.** The browser reaches the simulator's
   NT4 server only through the control plane's authenticated
   `/u/:slug/sim/nt4` proxy (decision 013). A frame without same-origin
   privileges does not send that cookie.

## Decision

### Declared by the project, served like Preview

A project declares dashboards in `.coderunner/dashboards.json`
(`{ dashboards: [{ title, entry }] }`, max 8). The file and the built dashboard
live in the project itself, not in `modules.json`, so:

- a dashboard is copied into the workspace with the files it was built for, and
  lesson modules, remote catalogs and team imports all get it the same way;
- nothing new is persisted per workspace, and there is no catalog lookup at
  session time;
- the control plane reads it straight from the project directory it already
  serves Preview from.

`GET /u/:slug/api/dashboards` (cookie-authenticated) returns the list with one
iframe URL per dashboard. The files route
`/u/:slug/api/dashboards/files/<token>/<path>` reuses Preview's hardened reader
(`readVerifiedProjectFile`: lexical checks, `O_NOFOLLOW`, descriptor-inside-root
check, size caps) and its MIME allowlist (+ `.mjs`).

### Scoped, longer-lived path token

The frame is sandboxed `allow-scripts` without `allow-same-origin` (opaque
origin), so, as with Preview, its subresource requests carry no cookie, and the
grant travels in the URL path. The dashboard token differs from Preview's:

- **Scoped to the entry's directory**, which is signed into the token, so a
  dashboard can read its own build output and nothing else in the project, not
  the Java source and not another dashboard.
- **8-hour TTL** instead of 15 minutes. A dashboard stays mounted for a whole
  class session and may lazy-load chunks or reload itself long after its tab
  opened. The shell re-mints on every list fetch (project swap, Reload).
- **Separate HMAC label** (`coderunner.dashboard.v1`), so the two token kinds
  are not interchangeable.

The files route is dispatched ahead of the cookie ownership check, like
Preview's, and GET-only. Its URLs are templated in metrics and redacted from
request logs.

### CORS on dashboard files

Bundlers emit `<script type="module" crossorigin>` and
`<link rel="stylesheet" crossorigin>`. From an opaque origin those are CORS
requests with `Origin: null`, and they fail without
`Access-Control-Allow-Origin`. Dashboard responses send `*`. The capability is
the path token, not the origin, and no credentials are involved, so this grants
nothing a direct navigation to the URL would not.

### The shell owns NT4; the frame gets a postMessage bridge

The shell runs **one** NT4 client per workspace page (adapted from
AdvantageScope's, with the same endpoint injection as CodeRunner's AS patch,
connecting to `/u/:slug/sim/nt4` and probing `/u/:slug/sim/alive`), created only
when the project declares at least one dashboard. Each dashboard frame gets a
*port* that holds its subscriptions, coalesces updates to the latest value per
topic, and flushes every 20 ms.

The control plane injects a small script as the first child of `<head>` in the
dashboard's HTML. It defines `window.coderunner` synchronously, before any
author script runs, so authors need no SDK package and any framework (or none)
works. The script speaks the protocol typed in `@frc-coderunner/contracts`
(`DashboardClientMessage` / `DashboardHostMessage`).

The injected script is authored as a TypeScript function and serialised with
`Function.prototype.toString`. That keeps it type-checked and testable. The
cost is that it must be fully self-contained, and a test executes the exact
serialised string to hold that.

Alternatives rejected:

- **Let the frame open NT4 itself with a token-authorised socket.** That
  adds a second, cookie-less auth path to the NT4 proxy and gives author code
  a raw socket to the simulator. The bridge keeps the frame at
  `connect-src 'none'`.
- **Ship an npm SDK instead of injecting.** Authors would have to install and
  import it, which rules out plain-HTML dashboards, and version skew with the
  shell would become the author's problem.
- **Subscribe the shell to all topics and broadcast.** AdvantageKit robots
  publish hundreds of topics at high rates. Per-frame subscriptions with NT4
  `periodic` and coalescing keep postMessage traffic proportional to what a
  dashboard shows. The shell does keep a topics-only subscription to everything,
  so `getTopics()` is complete, plus a value subscription to `/.schema/` so
  `struct:` values arrive decoded.

### Trust boundary in the shell

The shell accepts a message only if `event.source` is that dashboard frame's own
`contentWindow`, and only if it parses against `dashboardClientMessageSchema`.
Publishes are type-checked against the topic's announced type before they reach
NT4. Messages to the frame use target origin `*`, because an opaque origin
cannot be named. The frame's CSP (`navigate-to 'none'`, sandbox without
top-navigation) makes it hard for a dashboard to navigate itself elsewhere. If
one did, the data exposed would be the student's own simulator telemetry.

## Consequences

- Dashboards appear only in the robot layout. `plain-java` lessons have no
  robot program, so there is no NT4 connection and no tabs.
- A workspace without a manifest opens no extra socket and makes no extra
  `/sim/alive` probes.
- Authors must commit built output with relative asset URLs (`base: "./"` in
  Vite). The React template (`templates/dashboard-react/`) is set up this way
  and installs an in-memory mock of the API under `vite dev`.
- Dashboards cannot fetch the network, load CDN scripts, or use workers.
  Everything must be bundled.
- The bundled `robot-starter` lesson ships a plain-JavaScript dashboard as the
  zero-config example, so the bundled catalog in the workspace image changes.
