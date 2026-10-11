import random
import subprocess
import time
import math

from stable_baselines3 import PPO

from my_robot.scout_env import ScoutEnv
from my_robot.pick_bottle import pick_bottle


MODEL_PATH = "/home/kowshik0714/ros2_ws/models/ppo_scout/ppo_scout_bottle.zip"

GOAL_DISTANCE = 0.70
MAX_BOTTLES = 4

WORLD = "/world/scout_world"
BOTTLE_MODEL = "/home/kowshik0714/ros2_ws/src/my_robot/models/bottle/model.sdf"


DUSTBIN_X = 4.0
DUSTBIN_Y = 0.0


def get_dustbin_target(env):
    """Return distance and heading from Scout to the dustbin."""
    x = env.ros_node.robot_x
    y = env.ros_node.robot_y
    yaw = env.ros_node.robot_yaw

    dx = DUSTBIN_X - x
    dy = DUSTBIN_Y - y

    distance = math.hypot(dx, dy)

    angle = math.atan2(dy, dx) - yaw
    angle = math.atan2(math.sin(angle), math.cos(angle))

    return distance, angle


def navigate_to_dustbin(env, model):
    print("[TASK] Navigating to dustbin.")

    while True:
        distance, angle = get_dustbin_target(env)

        print(
            f"\r[TASK] Dustbin: "
            f"distance={distance:.2f} m "
            f"angle={angle:.2f} rad",
            end="",
            flush=True
        )

        # Stop well before the physical dustbin.
        if distance <= 1.50:
            env.ros_node.stop_robot()
            print("\n[TASK] Reached dustbin drop zone.")
            return True

        # Turn toward dustbin first.
        if abs(angle) > 0.15:
            linear = 0.0
            angular = max(
                -0.35,
                min(0.35, 1.2 * angle)
            )
        else:
            # Slow down significantly before the drop zone.
            if distance < 2.0:
                linear = 0.12
            else:
                linear = 0.25

            angular = max(
                -0.20,
                min(0.20, angle)
            )

        env.ros_node.publish_cmd_vel(
            linear,
            angular
        )

        time.sleep(0.05)

def unload_bottles():
    print("[TASK] Unloading bottles...")
    return True


def spawn_next_bottle(env):
    """Spawn the next bottle relative to Scout's current world pose."""

    env.ros_node.update_ground_truth_pose()

    x = env.ros_node.robot_x
    y = env.ros_node.robot_y
    yaw = env.ros_node.robot_yaw

    distance = random.uniform(0.8, 2.2)

    # Keep the bottle inside the practical camera view.
    max_lateral = min(
        0.75,
        distance * math.tan(math.radians(27.0))
    )

    local_y = random.uniform(
        -max_lateral,
        max_lateral
    )

    local_x = math.sqrt(
        max(0.01, distance * distance - local_y * local_y)
    )

    world_x = (
        x
        + math.cos(yaw) * local_x
        - math.sin(yaw) * local_y
    )

    world_y = (
        y
        + math.sin(yaw) * local_x
        + math.cos(yaw) * local_y
    )

    command = [
        "gz",
        "service",
        "-s",
        f"{WORLD}/create",
        "--reqtype",
        "gz.msgs.EntityFactory",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "5000",
        "--req",
        (
            f'sdf_filename: "{BOTTLE_MODEL}", '
            f'pose: {{position: {{'
            f'x: {world_x}, '
            f'y: {world_y}, '
            f'z: 0.001'
            f'}}}}'
        )
    ]

    result = subprocess.run(
        command,
        capture_output=True,
        text=True
    )

    if "data: true" in result.stdout:
        print(
            f"[TASK] Spawned bottle at "
            f"x={world_x:.2f}, y={world_y:.2f}"
        )
        return True

    print("[TASK] Failed to spawn bottle.")
    print(result.stdout)
    print(result.stderr)

    return False


def main():
    env = ScoutEnv()

    model = PPO.load(
        MODEL_PATH,
        env=env,
        device="cuda"
    )

    carried_bottles = 0

    try:
        obs, _ = env.reset()

        while True:

            # -------------------------------------------------
            # Search when bottle is not visible
            # -------------------------------------------------
            if obs[24] < 0.5:

                obs, _, terminated, truncated, _ = env.step(
                    [-1.0, 0.30]
                )

                if terminated or truncated:
                    obs = env._get_observation()

                continue

            # -------------------------------------------------
            # PPO navigation
            # -------------------------------------------------
            action, _ = model.predict(
                obs,
                deterministic=True
            )

            obs, _, terminated, truncated, _ = env.step(action)

            # -------------------------------------------------
            # Bottle reached
            # -------------------------------------------------
            if (
                obs[24] > 0.5
                and obs[20] <= GOAL_DISTANCE
            ):

                env.ros_node.stop_robot()

                print("[TASK] Bottle reached.")

                # -------------------------------------------------
                # Piper pick interface
                # -------------------------------------------------
                if pick_bottle():

                    carried_bottles += 1

                    print(
                        f"[TASK] Pick successful: "
                        f"{carried_bottles}/{MAX_BOTTLES}"
                    )

                    # -------------------------------------------------
                    # Four bottles collected
                    # -------------------------------------------------
                    if carried_bottles >= MAX_BOTTLES:

                        env.ros_node.stop_robot()

                        if not navigate_to_dustbin(env, model):
                            print(
                                "[TASK] Failed to reach dustbin."
                            )
                            break

                        if not unload_bottles():
                            print(
                                "[TASK] Failed to unload bottles."
                            )
                            break

                        carried_bottles = 0

                        print(
                            "[TASK] Unload successful. "
                            "Carried: 0/4"
                        )

                        # Do NOT call env.reset().
                        # Scout remains at the dustbin.

                        if not spawn_next_bottle(env):
                            print(
                                "[TASK] Failed to spawn next bottle."
                            )
                            break

                        obs = env._get_observation()
                        continue

                    # -------------------------------------------------
                    # Fewer than four bottles:
                    # spawn next bottle from current Scout position
                    # -------------------------------------------------
                    if not spawn_next_bottle(env):
                        print(
                            "[TASK] Failed to spawn next bottle."
                        )
                        break

                    obs = env._get_observation()

                else:
                    print(
                        "[TASK] Pick failed. Retrying."
                    )

                    obs = env._get_observation()

            elif terminated or truncated:

                obs = env._get_observation()

    except KeyboardInterrupt:

        print("[TASK] Stopped.")

    finally:

        env.ros_node.stop_robot()
        env.close()


if __name__ == "__main__":
    main()
