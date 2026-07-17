# RL Lane Keeping

PPO lane-keeping agent trained in the DonkeyCar simulator. Went through several iterations (v3 → v3.2 → v3.3) to fix issues like bang-bang steering, reward shaping problems, and left-lane oscillation.

## Files

- `train_lane_scratch_v3.py` — first version with steering-rate actions and positive-only reward
- `train_lane_scratch_v3_2.py` — added damping bonus near the target + always-on speed term
- `train_lane_scratch_v3_3.py` — widened damping window (glide-slope) + capped speed during lane changes
- `ppo_lane_scratch_v3.zip` — trained model weights (place in same dir as training script to resume)
- `measure_cte.py` — connects to the sim and logs CTE values while driving, useful for checking tracking accuracy

## Running

```bash
# train (or resume from existing weights)
python train_lane_scratch_v3_3.py

# measure CTE while manually driving
python measure_cte.py
```

Needs `gymnasium`, `gym_donkeycar`, `stable-baselines3`, `torch`, and the Unity sim binary.
