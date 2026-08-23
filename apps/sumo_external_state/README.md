# Physical co-simulation SUMO external-state build

This directory contains the pinned SUMO 1.23.1 patch used by the optional
Autoware x CARLA x TeraSim physical co-simulation mode.

The patch adds the low-level API used by the service's `setExternalState`
operation:

```python
traci.vehicle.moveToXYImmediate(
    vehicle_id,
    edge_id,
    lane_index,
    x,
    y,
    angle,
    keepRoute=1,
    matchThreshold=8.0,
    strictLaneHint=True,
)
```

Unlike normal `moveToXY`, this call updates pose immediately without advancing
SUMO time. It also completes the remote-control operation immediately, rebases
the lane-change model onto the realized CARLA lateral state, releases stale
TraCI speed overrides, and lets the in-process service apply CARLA measured
speed and acceleration with `setPreviousSpeed`.

The cycle is intentionally serial:

1. CARLA advances frame N under the co-sim clock master.
2. Phase A assimilates each selected background vehicle into its current SUMO
   lane without changing SUMO time.
3. TeraSim/NADE plans and the existing priority-10 `simulationStep()` advances
   SUMO exactly 0.05 seconds.
4. Phase B exports SUMO desired speed, acceleration, and a route/lane-change
   lookahead target.
5. CARLA applies Ackermann control for frame N+1.

The Autoware ego is never selected for Ackermann ownership. It keeps the normal
CARLA-to-SUMO AV observation path.

Build from the repository root:

```bash
docker build \
  -t terasim-service:physics-cosim-20260817 \
  -f Dockerfile.sumo-external-state .
```

Enable the mode only with the dedicated image and master tick:

```bash
export CARLA_COSIM_VEHICLE_CONTROL_MODE=ackermann_physics
export CARLA_COSIM_ACKERMANN_FEEDBACK_MODE=apply
export CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS='*'
export CARLA_COSIM_ACKERMANN_FEEDBACK_ASSIMILATION_MODE=external_state
export CARLA_COSIM_ACKERMANN_FEEDBACK_EXTERNAL_STATE_STRICT_LANE_HINT=1
export CARLA_COSIM_STEP_LENGTH=0.05
export USE_LIBSUMO=0
python -m terasim_service.run_cosim --tick_mode master ...
```

The dedicated physical launcher sets `USE_LIBSUMO=0`, matching the 3-way
workflow. This runs the patched SUMO engine as a TraCI child process instead
of loading libsumo into TeraSim's simulation thread. Both generated Python
APIs expose `moveToXYImmediate`; focused integration tests cover TraCI and
libsumo, while the Odaiba physical 3-way run is validated through TraCI.

`examples/scripts/check_physics_motion.py` requires a non-ego CARLA vehicle to
show both displacement and non-zero speed. The personal one-command launcher
uses it after the normal 3-way health check, so actor existence alone is not
reported as physical co-sim success.

All flags default to the legacy teleport behavior. Missing immediate-move API,
current-lane projection failure, frame mismatch, invalid lookahead, or excessive
position error fails closed instead of rematching another lane. In strict-lane
mode, SUMO never rematches an adjacent, predecessor, or unrelated internal lane.
