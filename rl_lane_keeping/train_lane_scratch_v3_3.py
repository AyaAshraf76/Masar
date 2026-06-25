import os
import gymnasium as gym
import gym_donkeycar
import numpy as np
import torch

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback
from gymnasium import spaces

# =========================================================
# v3.3 — fixes on top of v3.2
#
# v3.2 ran 5000 steps with zero crashes and the right lane
# tracks well (median err 0.09), but left lane still
# oscillates. It's smaller than before (~1.3 mean, range
# about -2.3 to +1.5) but it won't settle.
#
# What I changed:
#
# 1) Glide-slope damping — the v3.2 damping bonus only
#    kicked in below lane_error 0.5, but the left lane
#    oscillation lives mostly around 0.5-2.0 so nothing
#    was penalizing fast lateral movement there. Now the
#    allowed lateral speed scales with distance from target
#    (allowed = 0.10 + 0.35*lane_error), and the reward
#    window is wider (0.9 instead of 0.5). This should
#    reward the car for decelerating as it approaches.
#
# 2) Lane-change speed cap — during lane changes the drift
#    protections were disabled, so the car entered the new
#    lane at ~0.8 speed with lateral rate ~1.2, basically
#    shoving itself into an oscillation every time it
#    arrived. Now if changing and speed > 0.45, throttle
#    goes to 0.
#
# Same obs/action space as v3. Resumes from the existing
# ppo_lane_scratch_v3.zip — make sure it's in the same dir.
# =========================================================

exe_path = "/home/lenovo76/projects/sdsandbox/sdsim/worked_version.x86_64"

conf = {
    "exe_path": exe_path,
    "port": 9091,
    "max_cte": 10.0
}

LEFT_LANE_CTE  = -1.6
RIGHT_LANE_CTE =  1.6
MAX_CTE        =  3.2

SAVE_NAME   = "ppo_lane_scratch_v3"
TRAIN_STEPS = 300_000

MAX_SPEED       = 0.80
MAX_STEER_DELTA = 0.15   # max steering change per step
CTE_RATE_SCALE  = 5.0    # makes the rate magnitude comparable to cte in obs

device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")


class LaneTargetWrapper(gym.Wrapper):

    def __init__(self, env):
        super().__init__(env)

        self.lane_target  = RIGHT_LANE_CTE
        self.desired_lane = RIGHT_LANE_CTE

        self.prev_steering   = 0.0
        self.prev_throttle   = 0.0
        self.step_count      = 0
        self.changing        = True
        self._last_cte       = 0.0
        self._last_speed     = 0.0
        self._cte_rate       = 0.0
        self._first_step     = True

        # integrated steering — policy outputs a delta, not absolute
        self.current_steering = 0.0

        self.observation_space = spaces.Dict({
            **env.observation_space.spaces,
            "lane_target": spaces.Box(low=-5.0, high=5.0, shape=(1,), dtype=np.float32),
            "cte":         spaces.Box(low=-5.0, high=5.0, shape=(1,), dtype=np.float32),
            "speed":       spaces.Box(low=0.0,  high=5.0, shape=(1,), dtype=np.float32),
            "cte_rate":    spaces.Box(low=-10.0, high=10.0, shape=(1,), dtype=np.float32),
            "steering":    spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32),
        })

        self.action_space = spaces.Box(
            low=np.array([-1.0, 0.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)

        self.desired_lane     = np.random.choice([LEFT_LANE_CTE, RIGHT_LANE_CTE])
        self.lane_target      = self.desired_lane
        self.prev_steering    = 0.0
        self.prev_throttle    = 0.0
        self.step_count       = 0
        self.changing         = True
        self._last_cte        = 0.0
        self._last_speed      = 0.0
        self._cte_rate        = 0.0
        self._first_step      = True
        self.current_steering = 0.0

        return self._get_obs(obs), info

    def step(self, action):
        self.step_count += 1

        # switch lanes every 1500 steps
        if self.step_count % 1500 == 0:
            self.desired_lane = (
                RIGHT_LANE_CTE if self.desired_lane == LEFT_LANE_CTE
                else LEFT_LANE_CTE
            )
            self.changing = True

        # smooth transition toward desired lane
        self.lane_target += 0.01 * (self.desired_lane - self.lane_target)

        # steering-rate integration
        steer_delta = float(np.clip(action[0], -1.0, 1.0)) * MAX_STEER_DELTA
        self.current_steering = float(
            np.clip(self.current_steering + steer_delta, -1.0, 1.0)
        )
        steering = self.current_steering

        throttle = float(action[1])
        throttle = np.clip(throttle, 0.10, 0.55)

        # speed interventions
        prev_cte_estimate   = self._last_cte
        last_speed          = self._last_speed
        lane_error_estimate = abs(prev_cte_estimate - self.desired_lane)

        if self.changing and lane_error_estimate < 0.4:
            self.changing = False

        # v3.3: cap speed during lane changes to reduce entry oscillation
        if self.changing and last_speed > 0.45:
            throttle = 0.0

        if last_speed > MAX_SPEED:
            throttle = 0.0
        elif abs(prev_cte_estimate) > 1.85 and throttle > 0.10:
            throttle = 0.10

        if (not self.changing) and lane_error_estimate > 0.35:
            if last_speed > 0.30:
                throttle = 0.0
            else:
                throttle = 0.25

        real_action = np.array([steering, throttle], dtype=np.float32)

        obs, _, done, truncated, info = self.env.step(real_action)

        info["applied_steering"] = steering
        info["applied_throttle"] = throttle
        info["changing"]         = self.changing
        info["steer_delta"]      = steer_delta

        cte   = info["cte"]
        speed = info["speed"]

        # lateral velocity (skip first step — stale _last_cte causes a spike)
        if self._first_step:
            self._cte_rate   = 0.0
            self._first_step = False
        else:
            self._cte_rate = float(np.clip(cte - self._last_cte, -2.0, 2.0))
        self._last_cte   = cte
        self._last_speed = speed

        lane_error = abs(cte - self.lane_target)

        # === Reward (always >= 0 while alive, crash = -10) ===

        # long-range pull toward target
        r_far = 0.3 * max(0.0, 1.0 - lane_error / 5.0)

        # mid/near-range shaping
        r_track  = 1.0 * np.exp(-(lane_error / 0.60) ** 2)
        r_track += 0.6 * np.exp(-(lane_error / 0.15) ** 2)

        # v3.3: glide-slope damping — reward arriving slowly at any
        # distance, not just when already close. The allowed lateral
        # speed grows with distance so far-away corrections aren't
        # penalized, but near the target the car must be decelerating.
        if lane_error < 0.9:
            allowed_rate = 0.10 + 0.35 * lane_error
            r_damp = 0.4 * max(0.0, 1.0 - abs(self._cte_rate) / allowed_rate)
        else:
            r_damp = 0.0

        # near-wall: reward low lateral speed to prevent drifting out
        if abs(cte) > 2.0:
            r_damp += 0.3 * max(0.0, 1.0 - abs(self._cte_rate) / 0.08)

        # speed bonus — small always-on term + bigger when tracking well
        r_speed = 0.05 * speed
        if lane_error < 0.30:
            r_speed += 0.12 * speed

        # smoothness costs (small enough to keep total positive)
        r_smooth  = -0.10 * abs(steer_delta) / MAX_STEER_DELTA
        r_smooth += -0.03 * abs(throttle - self.prev_throttle)
        r_stall   = -0.20 if speed < 0.10 else 0.0

        reward = r_far + r_track + r_speed + r_damp + r_smooth + r_stall

        # near wall: scale down reward (still better than dying)
        if abs(cte) > 2.4:
            reward *= 0.3

        reward = max(reward, 0.0)

        if abs(cte) > MAX_CTE:
            reward = -10.0
            done = True

        self.prev_steering = steering
        self.prev_throttle = throttle

        return self._get_obs(obs), reward, done, truncated, info

    def _get_obs(self, obs):
        obs["lane_target"] = np.array([self.lane_target], dtype=np.float32)
        obs["cte"]         = np.array([self._last_cte],   dtype=np.float32)
        obs["speed"]       = np.array([self._last_speed], dtype=np.float32)
        obs["cte_rate"]    = np.array(
            [self._cte_rate * CTE_RATE_SCALE], dtype=np.float32
        )
        obs["steering"]    = np.array([self.current_steering], dtype=np.float32)
        return obs


def make_env():
    env = gym.make("donkey-generated-track-v0", conf=conf)
    env = LaneTargetWrapper(env)
    return env


env = DummyVecEnv([make_env])

checkpoint_callback = CheckpointCallback(
    save_freq=10_000,
    save_path="./checkpoints/",
    name_prefix=SAVE_NAME
)

save_zip = f"./{SAVE_NAME}.zip"

if TRAIN_STEPS > 0:
    if os.path.exists(save_zip):
        print(f"Resuming from {save_zip}")
        model = PPO.load(save_zip, env=env, device=device)
    else:
        print("No checkpoint found, starting fresh")
        model = PPO(
            "MultiInputPolicy", env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,
            target_kl=0.05,
            verbose=1,
            device=device,
        )

    print(f"\nTraining for {TRAIN_STEPS} steps on {device}...\n")
    try:
        model.learn(
            total_timesteps=TRAIN_STEPS,
            callback=checkpoint_callback,
            reset_num_timesteps=False,
        )
    except KeyboardInterrupt:
        print("\nTraining interrupted!")
    finally:
        model.save(SAVE_NAME)
        print(f"Saved as {save_zip}")
        model = PPO.load(save_zip, env=env, device=device)
else:
    print(f"Evaluation only — loading {save_zip}")
    model = PPO.load(save_zip, env=env, device=device)

# =========================================================
# EVALUATION
# =========================================================

print("\nEvaluating...\n")
obs = env.reset()

for step in range(5000):

    action, _ = model.predict(obs, deterministic=True)

    raw_delta = float(action[0][0])
    raw_thr   = float(action[0][1])

    obs, reward, done, info = env.step(action)

    lane_target = obs["lane_target"][0][0]
    cte_rate    = obs["cte_rate"][0][0]
    steer_now   = obs["steering"][0][0]
    cte         = info[0]["cte"]
    lane_error  = abs(cte - lane_target)
    speed       = info[0]["speed"]
    app_thr     = info[0]["applied_throttle"]
    changing    = info[0]["changing"]

    if changing:
        mode = "CHANGE"
    elif lane_error > 0.35:
        mode = "DRIFT "
    elif speed > MAX_SPEED:
        mode = "SPDCAP"
    else:
        mode = "      "

    print(
        f"STEP={step:4d} "
        f"[{mode}] "
        f"TARGET={lane_target:+.2f} "
        f"CTE={cte:+.2f} "
        f"RATE={cte_rate:+.2f} "
        f"ERR={lane_error:.2f} "
        f"SPD={speed:.2f} "
        f"D_S={raw_delta:+.2f} "
        f"STEER={steer_now:+.2f} "
        f"RAW_T={raw_thr:.2f} "
        f"APP_T={app_thr:.2f}"
    )

    if done[0]:
        obs = env.reset()
