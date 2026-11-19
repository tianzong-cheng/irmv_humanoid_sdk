# IRMV Studio

The workbench is a local web page and a local service for looking at the
experiments the SDK already records. It never commands the robot, never starts
or stops an SDK process, and never rewrites a run. All data comes from the
reader's committed files; the live tail is a temporary, bounded view.

## Start

1. Enable the recorder in the SDK config. The workbench needs the runtime's
   `run_*` directory; the application's `app_run_*` stream is optional:

   ```json
   "telemetry": {
     "recorder": {
       "enabled": true,
       "sample_hz": 100,
       "queue_capacity": 256,
       "directory": "/data0/<user>/records"
     }
   }
   ```

2. Start the runtime and application in the usual order. To feed the live tail,
   export one socket path for both processes. The launcher option does exactly
   this and does not start the service:

   ```bash
   python3 tools/sdk_launch.py --config config/sdk.json --domain 3 \
     --web-telemetry-socket /data0/<user>/web/live.sock
   ```

   Start the two processes by hand instead:

   ```bash
   export IRMV_WEB_TELEMETRY_SOCKET=/data0/<user>/web/live.sock
   build/robot_runtime --config config/sdk.json --domain 3
   build/application  --config config/sdk.json --domain 3
   ```

   The environment is read once at startup. Without it the recorder makes no
   live envelope; committed recording is unchanged.

3. Start the service on loopback only:

   ```bash
   python3 -m tools.experiment_web \
     --records-root /data0/<user>/records \
     --data-dir /data0/<user>/web-data \
     --reader build/web_record_query \
     --port 8765
   ```

   It prints `http://127.0.0.1:8765/`. Open that URL. The first page sets an
   HttpOnly, SameSite cookie; API calls without it are refused. Layouts are
   written under `--data-dir/layouts/` with atomic replacement. The service's
   live datagram defaults to `<data-dir>/live.sock`.

4. Stop the service whenever you like. It owns no SDK process, holds no device
   lock and has no endpoint that starts, stops or signals one.

## Installing

The default build is unchanged. To install the workbench with the SDK:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DIRMV_BUILD_EXPERIMENT_WEB=ON
cmake --build build -j2
cmake --install build --prefix /tmp/irmv-sdk
```

The prefix then holds:

|Path|Content|
|---|---|
|`bin/irmv_experiment_web`|Installed entry point|
|`bin/web_record_query`|Read-only JSONL bridge|
|`share/irmv/experiment-web/experiment_web/`|Python service package|
|`share/irmv/experiment-web/web/`|Static workbench|

`irmv_experiment_web` resolves its reader, package and assets relative to its
own prefix; it does not need the source tree, and it does not link or need ONNX
Runtime. A missing reader or asset is named explicitly.

## Choosing an experiment

One host controls one robot. **View live** opens the current runtime experiment
immediately. While following live, the window shows the latest ten seconds;
before ten seconds are available it grows with the experiment. **Follow latest**
restores that ten-second window after reviewing or zooming. It never opens a
source picker. The adjacent status describes telemetry availability, not robot
health. Without a readable live runtime the button is disabled. Application-only
telemetry does not count as a live experiment. Conflicting runtime streams are
reported instead of selecting one arbitrarily.

**History** opens a recording browser without leaving the charts. It lists
runtime experiments, not individual application streams. Search by name or
start date, or use **Refresh** to rescan. The newest known start comes first;
start times come from the recorder directory's host-clock timestamp, never from
monotonic sample times. Renamed directories may show an unavailable start time.
Rows include recorded duration and recording/completion status; unreadable rows
show the reason and cannot be opened. **Replay** opens saved data; the current
replay is marked **Viewing**. Escape or the close button dismisses the browser
and restores focus to History. No recording is opened automatically.

Opening a different experiment clears charts, cursor and diagnostic details.
Reopening the same experiment retains charts. Saved layouts remain available
to load explicitly; missing signals remain visible as placeholders.

Runtime and application are data sources within an experiment. On opening a
runtime, the workbench checks application streams using the service's
boot/runtime/session association. A single verified match is included
automatically. **Experiment data sources** shows the selected sources and lets
you select from verified matches or use runtime data alone. Multiple matches
require selection; unrelated application streams are never offered. An
association check failure cannot replace the current view with a mismatched
stream. If a candidate cannot be checked, runtime data remains available and the
source details explain that the check was incomplete.

Pausing, seeking or zooming a live experiment keeps it selected while you
review. **Follow latest** returns to the tail. A disconnected stream shows
**Offline** and retains chart data. Catalog updates never switch the selected
experiment. These controls only change displayed data; they never start, stop
or command the robot.

## Chart interaction

- Drag horizontally in a plot to select a shared time range for every chart.
  Value ranges stay fixed. Hold **Ctrl** before dragging vertically to zoom
  only that chart's value axes; the shared time range stays unchanged. The
  selection band fills the unused direction, and Escape cancels either mode.
- Drag a signal into a chart to overlay it, or onto an edge to split the view.
  Drag the legend area to move the chart. Removing the last curve closes it.
  Live data refreshes redraw plots without replacing drag targets or legend
  buttons, so dropping and removing curves remain available while following.
- Legends identify each curve by joint/source, signal and unit, using the same
  color and line style as the trace. Hover a legend entry to
  emphasize its trace; use its close button to remove that signal.
- Value axes group signals automatically by physical quantity and unit. The
  unit beside each legend entry identifies its axis. Ctrl-drag adjusts all
  value axes in that chart using the same selected vertical band.
- Use the mouse wheel over a plot to zoom about the time under the pointer.
  Time zoom retains the displayed value ranges across all charts. Zooming out
  stops at the full recording; neither edge can move beyond its time coverage.
- A completed time selection pauses playback/live following. Ctrl-drag keeps
  the shared time window and live-follow setting. Press Esc to cancel, or
  simply click to leave the view unchanged.
- All displayed timestamps use seconds since the experiment's first sample,
  including chart ticks, cursor readouts and event times. Timestamp-valued
  signals use relative microseconds on their existing axes; diagnostic JSON
  timestamp fields also remain in microseconds. Queries and recordings retain
  their absolute timestamps. Live buffer changes do not move the time origin.
- Playback moves the cursor through the current window without moving the
  chart. The window follows only after the cursor passes its right edge, and
  playback stops at the end of the experiment.
- Range queries include neighboring samples beyond the visible edges, so
  clipping shows complete line segments even between samples. Gaps and invalid
  samples still break the trace.
- The time cursor follows the pointer on the browser's animation frames using
  an independent overlay; moving it does not redraw the traces. Exact value
  lookups run at most every 100 ms with one request in flight. The readout names
  its queried time, and value dots appear only when they match the current
  cursor. Stopping the pointer still queries its final position.
- **Reset Y** at the right of a chart's legend, or a double-click in its plot,
  restores only that chart's automatic value ranges. **Reset time** in the
  playback toolbar restores the shared full time range while preserving every
  chart's displayed value ranges. View changes never alter a recording.

## Semantics

- **Two sources, never stitched by themselves.** A runtime run records the
  lower layer. The application's velocity stream is a separate directory with
  its own sampling phase. The UI includes a stream only after the service
  verifies boot/runtime/session association; it never merges by file name or
  array index.
- **Committed history is authoritative.** Range queries read the chunk index and
  verify each chunk's header and payload CRC. A corrupt chunk contributes no
  rows and is named; a truncated run says so. A live envelope is a temporary
  copy until a committed row exists, then the committed value wins.
- **Gaps are data.** Queue/write losses, an open run, a missing fault sidecar
  and an empty complete fault stream are different facts. `complete: false` is
  not a query failure. A missing `faults.jsonl` is an error, because the
  recorder creates it before the run starts.
- **Invalid is not zero.** A value behind a false validity flag stays the raw
  value in the record and is not drawn as a valid zero. Non-finite doubles are
  `"NaN"`, `"+Inf"`, `"-Inf"` strings and are not drawn.
- **IMU conventions are what the run states.** The IMU axis frame, order and
  gravity convention are `unknown` when the driver contract does not state
  them; the workbench displays the metadata verbatim and never transforms the
  axes.
- **Replay never commands.** The history and live views publish no DDS message.
- **Live tail is bounded.** Per run: at most 15 s and 32 MiB; globally at most
  128 MiB and 8 active runs. A refused subscription or an evicted stretch is
  reported; history is never deleted to make room. A transport sequence gap is
  counted, not invented into a recorded gap.
- **No maintenance writes.** The maintenance page is intentionally empty: the
  first maintenance task is not part of this release, the maintenance tool
  still requires its device lock and defaults to dry-run, and the workbench does
  not stop the runtime for it.

## Tests

```bash
# deterministic C++ catalog/range and live-envelope tests
ctest --test-dir build -L core --output-on-failure

# local HTTP, layouts, live store, two real Fake processes
ctest --test-dir build -R '^experiment_web_server_test$' --output-on-failure
ctest --test-dir build -R '^experiment_web_test$' --output-on-failure

# model and browser acceptance
npm --prefix web test
npm --prefix web run test:e2e

# installed prefix (option IRMV_BUILD_EXPERIMENT_WEB=ON)
ctest --test-dir build -R '^experiment_web_package_test$' --output-on-failure
```

The browser test is opt-in in CTest:

```bash
cmake -S . -B build -DIRMV_WEB_BROWSER_TESTS=ON
ctest --test-dir build -R '^experiment_web_browser_test$' --output-on-failure
```

If it is enabled but npm, the pinned Playwright package or its browser is
missing, the test fails rather than silently skipping.

## Performance Evidence

Measured on the development host (12th Gen Intel Core i7-12700H, 20 threads,
Linux, Release build), with a generated Fake record of 64 joints, 100 Hz,
180,000 rows (30 minutes, 875 MB) and 24 curves from six joints:

```bash
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release \
  -DIRMV_BUILD_APPLICATION=OFF \
  -DIRMV_NLOHMANN_JSON_SOURCE_DIR=$PWD/build/_deps/nlohmann_json-src
cmake --build build-bench --target web_record_query -j2
python3 tools/benchmark_experiment_web.py --iterations 5
```

|Measurement|Result|
|---|---|
|First window, bridge p50 / p95|0.78 s / 0.78 s|
|First window, HTTP p50 / p95|0.89 s / 0.89 s|
|Bridge RSS before / after the window|0.4 MB / 46 MB|
|Live source sample to service snapshot p50 / p95|1.2 ms / 3.9 ms|

The first-window target is 2 s and is met. The live target is p95 <= 250 ms;
the service side is far inside it. Browser paint is not measured by that script
because its clock origin differs; the Playwright acceptance measures the live
window's pause/follow behaviour and the visible canvas, not a paint timestamp.
The report is archived at `build/web-benchmark/report.json` (gitignored), which
records the exact machine and file sizes.

These numbers are Fake-backend evidence. They say nothing about physical motor
execution, and `send_accepted` never becomes a device acknowledgement.
