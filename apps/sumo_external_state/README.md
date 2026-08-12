# SUMO `setExternalState` build

This directory contains the source patch for a dedicated SUMO 1.23.1 build.
It adds a vehicle TraCI setter that assimilates external pose and motion state
without running planning, movement, output, or advancing SUMO time.

The source is pinned to:

- tag: `v1_23_1`
- commit: `676720d13f6f42d8c79d156e9d67001f8c22f6f6`
- experimental vehicle variable ID: `0xf8`

## API

```python
traci.vehicle.setExternalState(
    vehicle_id,
    edge_id,
    lane_index,
    x,
    y,
    angle,
    speed,
    acceleration,
    keepRoute=1,
    matchThreshold=100,
    strictLaneHint=False,
)
```

With the default `strictLaneHint=False`, the setter retains the original
`moveToXY` matching path. With `strictLaneHint=True`, it projects lane position
and lateral offset only against the supplied current lane, rejects invalid or
stale route lanes and threshold violations, and never rematches an adjacent,
predecessor, or different internal lane. Both paths immediately apply the
remote state, remove it from the global post-move queue, retire the
same-timestamp remote-control latch, clear stale lane-change state, and release
any older TraCI `setSpeed` timeline before setting speed and acceleration.
Normal `moveToXY` behavior is unchanged.

Before releasing the remote-control latch, the dedicated completion path
separates the lane-change model's lane-index-relative lateral coordinate from
the sign needed to project that coordinate onto the actual lane geometry. It
inspects the center geometry of an adjacent lane to infer the physical
direction of increasing lane indices, normalizes the internal lateral state,
and stores a per-vehicle projection sign. This handles both ordinary left-hand
networks and Odaiba edges whose declared lane order is reversed in space. The
service still uses raw SUMO x/y to disambiguate drive-side-dependent lateral
signs when exporting lane-relative geometry.

The intended cycle is:

1. Phase A calls `setExternalState`; SUMO time does not change.
2. TeraSim/NADE reads the assimilated state and selects behavior.
3. Phase B calls `simulationStep()` exactly once for 0.05 seconds.
4. CARLA ticks exactly once for 0.05 seconds.

## Build

Build the existing co-simulation image first, then build the dedicated image:

```bash
docker build -t terasim-service:ackermann-feedback-gui -f Dockerfile.cosim .
docker build \
  -t terasim-service:sumo-external-state-v1.23.1 \
  -f Dockerfile.sumo-external-state .
```
The dedicated Dockerfile copies the matching `terasim_service` source into the
final image. The image therefore contains both the SUMO-side normalization and
the service-side lane geometry disambiguation without a working-tree mount.

The Dockerfile checks the upstream commit before applying the patch. The final
image contains patched `sumo`, `sumo-gui`, TraCI, and libsumo implementations.

## Minimal validation

Run both the raw API gate and the TeraSim priority-order gate before any Odaiba
integration run:

```bash
docker run --rm \
  -v "$PWD:/workspace:ro" \
  -w /workspace \
  --entrypoint /bin/bash \
  terasim-service:sumo-external-state-v1.23.1 \
  -c 'pip install --no-cache-dir pytest==8.3.5 >/dev/null && python3 -m pytest \
    -o addopts= -q tests/test_integration/test_sumo_external_state.py'
```

The TeraSim gate executes Phase A at priority `-90`, observes and plans at
priority `0` (including `executeMove`), and calls the ordinary 0.05-second
`simulationStep` once at priority `10`. It repeats 12 cycles and rejects large
position corrections or speed/yaw oscillation. The same test file also runs the
legacy 80-cycle Odaiba lane-boundary regression plus strict AV/BV gates for a
20-cycle predecessor/internal boundary, lane-1-side poses constrained to
`edge_426_0` with lane-0 lookahead, and the `edge_0_0 -> edge_3_0` route.

Do not proceed to Odaiba until this gate passes.

## TeraSim service mode

The shared Redis and direct-gRPC command handlers select the dedicated Phase A
path with:

```bash
export CARLA_COSIM_ACKERMANN_FEEDBACK_ASSIMILATION_MODE=external_state
```

The repository default is `legacy`, which preserves the existing
`moveTo` plus `setPreviousSpeed` behavior. In `external_state` mode only the
lane selected by the preceding SUMO Phase B is projected; CARLA lateral offset
or speed never triggers current-edge all-lane selection. The handler then calls
`setExternalState(..., strictLaneHint=True)` with raw CARLA x/y/yaw/speed.
A legitimate Phase B lane or route transition automatically becomes the next
Phase A hint.

Immediate time, position, angle, speed, and primary-lane validation is enabled
by default.
Position readback allows 1 millimetre by default to cover CARLA float32 and
SUMO lane-geometry reconstruction precision. Override it with
`CARLA_COSIM_ACKERMANN_FEEDBACK_EXTERNAL_STATE_POSITION_TOLERANCE` when needed.
It can be disabled only after validation with
`CARLA_COSIM_ACKERMANN_FEEDBACK_VALIDATE_EXTERNAL_STATE=0`. Selecting
`external_state` with an unpatched SUMO build fails closed before the normal
SUMO step.

Selecting `external_state` also enables lane-relative SUMO state export and
CARLA consumption by default. This keeps the next longitudinal target tied to
the live lane ID and lane position after Phase A. The legacy mode retains its
previous opt-in defaults for these lane-relative fields.
