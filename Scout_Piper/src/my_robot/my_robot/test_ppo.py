from stable_baselines3 import PPO
from my_robot.scout_env import ScoutEnv

MODEL_PATH = (
    "/home/kowshik0714/ros2_ws/"
    "models/ppo_scout/ppo_scout_bottle.zip"
)

def main():
    print("============================================")
    print("PPO GAZEBO TEST")
    print("============================================")

    env = ScoutEnv()

    try:
        print("[TEST] Loading PPO model...")
        model = PPO.load(MODEL_PATH, device="cpu")

        print("[TEST] Resetting environment...")
        observation, info = env.reset()

        print(
            f"[TEST] Initial distance = {float(observation[20]):.3f} m, "
            f"angle = {float(observation[21]):.3f} rad"
        )

        step = 0

        while True:
            step += 1

            action, _ = model.predict(
                observation,
                deterministic=True
            )

            print(
                f"[STEP {step}] "
                f"distance={float(observation[20]):.3f} "
                f"angle={float(observation[21]):.3f} "
                f"v={float(action[0]):.3f} "
                f"w={float(action[1]):.3f}"
            )

            (
                observation,
                reward,
                terminated,
                truncated,
                info
            ) = env.step(action)

            if terminated:
                print("\n[TEST] Goal reached.")
                break

            if truncated:
                print("\n[TEST] Episode truncated / max steps reached.")
                break

        print("============================================")
        print("TEST FINISHED")
        print("============================================")

    finally:
        env.ros_node.stop_robot()
        env.close()


if __name__ == "__main__":
    main()
