<div align="center">
<p align="center">

<img src="docs/figure/logo.png" height="100px">

</p>
</div>

<p align="center">
<strong>Naturalistic and adversarial traffic simulation for discovering unknown unsafe events</strong>
</p>

---

## Overview

TeraSim is an open-source platform for automated autonomous-vehicle simulation.
Its objective is to **efficiently uncover real-world unknown unsafe events** by
automatically creating diverse and statistically realistic traffic environments:

- **Naturalistic driving environment (NDE)** — background traffic derived from
  large-scale naturalistic driving data, statistically realistic rather than
  hand-scripted.
- **Adversarial scenario synthesis (NADE)** — rare, high-risk interactions
  (aggressive cut-ins, unexpected crossings, ...) injected into that traffic to
  reach failures a nominal drive never encounters.
- Built on [SUMO](https://www.eclipse.org/sumo/), and able to drive third-party
  simulators such as [CARLA](https://carla.org/) and
  [Autoware](https://github.com/autowarefoundation/autoware).

## About this fork

This is a reduced fork of [mcity/TeraSim](https://github.com/mcity/TeraSim),
reworked for **three-way co-simulation: Autoware × CARLA × TeraSim**. Autoware
drives the ego vehicle through
[`autoware_carla_interface`](https://github.com/autowarefoundation/autoware_universe/tree/main/simulator/autoware_carla_interface),
and TeraSim supplies the background traffic around it.

- **Single-process CARLA link** (`terasim-service`): the TeraSim loop and the
  CARLA client run as two threads of one process, exchanging states and commands
  as plain Python objects. TeraSim traffic is mirrored into a running CARLA
  server; the externally driven ego is fed back into SUMO so the background
  traffic reacts to it. TeraSim can follow the ego side's clock or own it.
- **Optional physics-based background vehicles**: selected CARLA vehicles are
  driven by Ackermann control instead of being teleported, and their measured
  pose is written back into SUMO.
- **Everything outside that scope was removed**: TeraSim-World/Cosmos generative
  sensor simulation, environment generation, dataset tooling, and the
  Redis/FastAPI service with its clients. See the upstream repository for those.

Detailed documentation of the co-simulation setup will follow.

## Running

Everything map-specific (SUMO net, routes, adversity setup, run time) lives in a
scenario YAML under `examples/scenarios/`, which you pass to the runner. Three
maps ship with the repository: **Town01** (CARLA's stock map), **Kashiwanoha**
(a public Japanese campus map) and **Mcity** (standalone runs only, since CARLA
has no matching world).

```bash
# Install (Python 3.10-3.12, gcc/g++ for the Cython extensions)
conda create -n terasim python=3.10 -y && conda activate terasim
./setup_environment.sh

# Standalone NADE run, no CARLA needed
python scripts/run_experiments_debug.py --config examples/scenarios/Mcity_safety_assessment.yaml

# Co-simulation against a running CARLA server
python -m terasim_service.run_cosim --config examples/scenarios/cosim_town01_dt005.yaml
```

For the full three-way setup, `docker-compose.cosim-inprocess.yml` builds on
`Dockerfile.cosim` and runs `examples/scripts/run_3cosim_inprocess.sh` against a
CARLA server that an Autoware bridge is already attached to.

## Publications

TeraSim builds on the following research:

* **NDE** – Learning naturalistic driving environment with statistical realism
  [Paper](https://doi.org/10.1038/s41467-023-37677-5) | [Code](https://github.com/michigan-traffic-lab/Learning-Naturalistic-Driving-Environment)

* **NADE** – Intelligent driving intelligence test with naturalistic and adversarial environment
  [Paper](https://doi.org/10.1038/s41467-021-21007-8) | [Code](https://github.com/michigan-traffic-lab/Naturalistic-and-Adversarial-Driving-Environment)

* **D2RL** – Dense deep reinforcement learning for AV safety validation
  [Paper](https://doi.org/10.1038/s41586-023-05732-2) | [Code](https://github.com/michigan-traffic-lab/Dense-Deep-Reinforcement-Learning)

## **📄 License**

- **TeraSim Core and other packages**: Apache 2.0 License
- **Visualization Tools**: MIT License

This project includes modified code from [SumoNetVis](https://github.com/patmalcolm91/SumoNetVis) licensed under the MIT License.
