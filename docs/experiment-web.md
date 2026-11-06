# Experiment Web Workbench

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

## Semantics

- **Two sources, never stitched by themselves.** A runtime run records the
  lower layer. The application's velocity stream is a separate directory with
  its own sampling phase. It is offered as a candidate; the UI shows the
  boot/runtime/session association and does not merge on a file name or an
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
  -DIRMV_CYCLONEDDS_PREFIX=$PWD/build/cyclonedds-prefix \
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
