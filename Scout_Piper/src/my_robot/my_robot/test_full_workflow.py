import time
import numpy as np

from scout_env import ScoutEnv


env = ScoutEnv(
    num_bottles=4,
    max_carry=4
)

print("\n========== FULL WORKFLOW TEST ==========\n")

obs, info = env.reset()

for step in range(1000):

    distance = float(obs[20])
    angle = float(obs[21])

    # Simple target-following controller
    #
    # Keep moving slowly while turning so the robot does not
    # get trapped rotating in place.

    linear = min(
        0.22,
        max(0.08, distance * 0.15)
    )

    angular = float(
        np.clip(angle * 1.2, -0.5, 0.5)
    )

    if distance < 1.0:
        linear = 0.10

    action = np.array(
        [linear, angular],
        dtype=np.float32
    )

    obs, reward, terminated, truncated, info = env.step(action)

    if step % 10 == 0:
        print(
            f"[{step:03d}] "
            f"phase={info['phase']} "
            f"distance={info['goal_distance']:.2f} "
            f"angle={angle:.2f} "
            f"carried={info['carried_bottles']} "
            f"collected={info['collected_bottles']} "
            f"collision={info['collision']}"
        )

    if terminated or truncated:
        print("\n========== WORKFLOW END ==========")
        print(f"Terminated        : {terminated}")
        print(f"Truncated         : {truncated}")
        print(f"Collision         : {info['collision']}")
        print(f"Workflow complete : {info['workflow_complete']}")
        print(f"Collected         : {info['collected_bottles']}")
        print(f"Carried           : {info['carried_bottles']}")
        break

else:
    print("\n[TEST] Maximum steps reached.")

env.close()
