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
)
```

The setter reuses SUMO's `moveToXY` mapping, immediately applies the queued
remote state for that vehicle, removes it from the global post-move queue, and
retires the same-timestamp remote-control latch. It also clears any older
TraCI `setSpeed` timeline before setting the assimilated speed and acceleration,
so Phase B starts from the external state. TeraSim/NADE may install a new speed
action after Phase A. Normal `moveToXY` behavior is unchanged.

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
position corrections or speed/yaw oscillation.

Do not proceed to Odaiba until this gate passes.

## TeraSim service mode

The shared Redis and direct-gRPC command handlers select the dedicated Phase A
path with:

```bash
export CARLA_COSIM_ACKERMANN_FEEDBACK_ASSIMILATION_MODE=external_state
```

The repository default is `legacy`, which preserves the existing
`moveTo` plus `setPreviousSpeed` behavior. In `external_state` mode the lane
projection remains a route/elevation/distance safety check, but it does not
move the vehicle. The handler then calls `setExternalState` once with the raw
CARLA x/y/yaw/speed and the validated edge/lane hint.

Immediate time, position, angle, and speed validation is enabled by default.
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
