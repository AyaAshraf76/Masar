# RL Lane Keeping

PPO lane-keeping agent trained in the DonkeyCar simulator (Unity). The car drives on a two-lane oval track (inner lane at CTE = -1.6, outer at CTE = +1.6, walls at ±3.2) and has to track a target that switches lanes every 1500 steps.

Went through six iterations (v1 → v3.3) to get from 30 crashes per 5000 steps to zero crashes with tight tracking.

## Development history

### v1 (baseline) — 500k steps
First attempt. 30 crashes in 5000 steps, all the same pattern: the car corrects with full steering lock, recovers, then holds full lock for 12-15 more steps and sweeps into the opposite wall. 72% of steps were at steering saturation (±1.00). Never reached the left lane at all.

Two problems: (1) no lateral velocity in the observation, so the policy couldn't tell "still drifting" from "already recovered", and (2) raw steering output clips to [-1,+1] which makes a Gaussian policy collapse into bang-bang control.

### v2 — added cte_rate to observation
Added per-step CTE change to the obs. Crashes continued — having the info helped but the bang-bang steering made it impossible to act on it. Necessary but not sufficient.

### v3 — structural fixes (the big one)
Two changes that fixed everything:

1. **Positive reward while alive**: the old reward made crashing rational — sitting in the wrong lane cost ~-2/step forever, crashing cost -5 once. The policy correctly learned to crash. Fix: reward is always ≥ 0 while alive, crash = -10. Now surviving always beats dying.

2. **Steering-rate actions**: instead of outputting a raw angle, the policy outputs a steering *delta* (max ±0.15/step), integrated by the wrapper. Sweeping across the track now takes many deliberate actions instead of one held saturation. Current steering angle added to obs.

Also fixed: cte_rate spike on first step (stale last_cte), cte_rate scaled ×5.

Result: **zero crashes** for the first time. Outer lane median error 0.22. But the inner lane had a ~200-step limit cycle (sweeping ±2.2 through the target), and the throttle head went dead.

### v3.1 — damping bonus
Added `r_damp`: bonus for arriving at the target slowly (low lateral velocity when close). Always-on speed term to wake up the throttle head.

Result: right lane improved to 0.10 median error. But 5 crashes appeared at one specific track location — car held full-left lock drifting into the wall for 14 steps. The damping only fired below error 0.5, and the near-wall reward was too weak to pull it back.

### v3.2 — wall drift arrest
Added a near-wall bonus (when CTE > 2.0) rewarding low lateral velocity regardless of lane error.

Result: **zero crashes** again. Max CTE dropped from 3.26 to 2.43. Left lane mean error down to 1.30, oscillation range contracted to [-2.3, +1.5]. But the limit cycle still survived at reduced amplitude.

### v3.3 — glide-slope damping + lane-change speed cap
Two targeted fixes:

1. Widened damping window to error < 0.9 with allowed lateral speed scaling with distance (allowed = 0.10 + 0.35 × lane_error). Rewards decelerating approach at every distance, not just the last 0.5.

2. Cut throttle to 0 during lane changes when speed > 0.45. The old code let the car enter the new lane at 0.8 speed with lateral rate ~1.2, seeding the oscillation every time.

## Results summary

Eval is a deterministic 5000-step rollout. "Right err" is the median tracking error on the outer lane (+1.6), "Left err" is the mean error on the inner lane (-1.6).

| Version | What changed                   | Crashes | Right err | Left err | Max CTE |
|---------|--------------------------------|---------|-----------|----------|---------|
| v1      | baseline                       | 30      | —         | —        | >3.2    |
| v2      | + cte_rate obs                 | ~30     | —         | —        | >3.2    |
| v3      | rate actions + positive reward | 0       | 0.22      | 1.93     | 2.63    |
| v3.1    | + damping + speed term         | 5       | 0.10      | 1.60     | 3.26    |
| v3.2    | + wall drift arrest            | 0       | 0.09      | 1.30     | 2.43    |
| v3.3    | + glide-slope + entry cap      | 0       | 0.09      | 1.17     | 2.37    |

The biggest jump was v3: replacing the reward (so crashing is never better than surviving) and switching to steering-rate actions (so bang-bang is impossible). Everything after that was reward tuning to tighten tracking.

## Files

- `train_lane_scratch_v3.py` — v3: steering-rate actions + positive reward
- `train_lane_scratch_v3_2.py` — v3.2: damping + always-on speed + wall drift arrest
- `train_lane_scratch_v3_3.py` — v3.3: glide-slope damping + lane-change speed cap
- `ppo_lane_scratch_v3.zip` — trained model weights (place in same dir to resume)
- `measure_cte.py` — logs CTE values while driving in the sim, for checking tracking accuracy

## Running

```bash
# train (resumes from ppo_lane_scratch_v3.zip if present)
python train_lane_scratch_v3_3.py

# measure CTE while manually driving
python measure_cte.py
```

Needs `gymnasium`, `gym_donkeycar`, `stable-baselines3`, `torch`, and the Unity sim binary from `simulation/`.
