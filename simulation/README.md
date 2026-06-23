# Simulation

Unity-based simulation environment for training and testing.

- `unity_build/` — built simulator binary (worked_version.x86_64)
- `gym-donkeycar/` — OpenAI Gym wrapper (git submodule)

The RL training scripts in `rl_lane_keeping/` connect to this simulator.

```bash
# make sure submodule is initialized
git submodule update --init --recursive
```
