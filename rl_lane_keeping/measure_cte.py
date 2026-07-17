import gymnasium as gym
import gym_donkeycar
import time

exe_path = "/home/lenovo76/sdsandbox/sdsim/worked_version.x86_64"

conf = {
    "exe_path": exe_path,
    "port": 9091,
    "max_cte": 20.0
}

# Create env (this launches simulator)
env = gym.make("donkey-generated-track-v0", conf=conf)

obs, info = env.reset()

print("📡 Connected. Drive manually in the simulator window...")

step = 0

while True:
    # Send neutral action (important: don't override manual driving too much)
    action = [0.0, 0.0]

    obs, reward, done, truncated, info = env.step(action)

    cte = info["cte"]

    # print every 10 steps (reduce spam)
    if step % 10 == 0:
        print(f"CTE: {cte:.3f}")

    step += 1

    time.sleep(0.05)

    if done:
        print("RESET\n")
        obs, info = env.reset()
