import math
import time
import threading
import subprocess
import sys

sys.path.append('/usr/lib/python3/dist-packages')

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode

import gymnasium as gym
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

from my_robot.pick_bottle import pick_bottle


WORLD = "/world/scout_world"

BOTTLE_SDF = (
    "/home/kowshik0714/ros2_ws/src/my_robot/"
    "models/bottle/model.sdf"
)

BOTTLE_NAME = "bottle"

DUSTBIN_X = 4.0
DUSTBIN_Y = 0.0


# ============================================================
# ROS INTERFACE
# ============================================================

class ScoutROSNode(Node):

    def __init__(self):

        super().__init__("scout_rl_interface")

        self.depth_sectors = None
        self.bottle_target = None

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None

        self.create_subscription(
            Float32MultiArray,
            "/depth_sectors",
            self.depth_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            "/bottle_target",
            self.target_callback,
            10
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.get_logger().info(
            "Scout RL interface started"
        )

    def depth_callback(self, msg):

        if len(msg.data) != 20:
            return

        self.depth_sectors = np.array(
            msg.data,
            dtype=np.float32
        )

    def target_callback(self, msg):

        if len(msg.data) < 2:
            return

        self.bottle_target = np.array(
            msg.data[:2],
            dtype=np.float32
        )

    def odom_callback(self, msg):

        self.linear_velocity = float(
            msg.twist.twist.linear.x
        )

        self.angular_velocity = float(
            msg.twist.twist.angular.z
        )

        self.odom_x = float(
            msg.pose.pose.position.x
        )

        self.odom_y = float(
            msg.pose.pose.position.y
        )

        q = msg.pose.pose.orientation

        sin_yaw = 2.0 * (
            q.w * q.z + q.x * q.y
        )

        cos_yaw = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        self.odom_yaw = math.atan2(
            sin_yaw,
            cos_yaw
        )

    def send_action(
        self,
        linear_velocity,
        angular_velocity
    ):

        cmd = Twist()

        cmd.linear.x = float(
            linear_velocity
        )

        cmd.angular.z = float(
            angular_velocity
        )

        self.cmd_pub.publish(cmd)

    def stop_robot(self):

        self.send_action(
            0.0,
            0.0
        )


# ============================================================
# GYMNASIUM ENVIRONMENT
# ============================================================

class ScoutEnv(gym.Env):

    metadata = {
        "render_modes": []
    }

    def __init__(
        self,
        num_bottles=4,
        max_carry=4
    ):

        super().__init__()

        # ----------------------------------------------------
        # ROS
        # ----------------------------------------------------

        if not rclpy.ok():
            rclpy.init()

        self.ros_node = ScoutROSNode()

        self.gz_node = GazeboNode()
        self.gz_pose = None

        self.gz_node.subscribe(
            Pose_V,
            f"{WORLD}/dynamic_pose/info",
            self._gazebo_pose_callback
        )

        self.executor = SingleThreadedExecutor()

        self.executor.add_node(
            self.ros_node
        )


        self.executor_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True
        )

        self.executor_thread.start()

        # ====================================================
        # OBSERVATION
        #
        # 0-19  : depth sectors
        # 20    : target distance
        # 21    : target angle
        # 22    : linear velocity
        # 23    : angular velocity
        #
        # The target is generic:
        # bottle OR dustbin.
        # ====================================================

        low = np.array(
            [0.0] * 20
            + [
                0.0,
                -np.pi,
                -1.0,
                -3.0
            ],
            dtype=np.float32
        )

        high = np.array(
            [8.0] * 20
            + [
                8.0,
                np.pi,
                1.0,
                3.0
            ],
            dtype=np.float32
        )

        self.observation_space = gym.spaces.Box(
            low=low,
            high=high,
            dtype=np.float32
        )

        # ====================================================
        # ACTION
        # ====================================================

        self.action_space = gym.spaces.Box(
            low=np.array(
                [0.0, -1.0],
                dtype=np.float32
            ),
            high=np.array(
                [0.5, 1.0],
                dtype=np.float32
            ),
            dtype=np.float32
        )

        # ====================================================
        # NAVIGATION
        # ====================================================

        self.goal_threshold = 0.75
        self.dustbin_goal_threshold = 1.00
        self.collision_threshold = 0.25

        # ====================================================
        # REWARD
        # ====================================================

        self.progress_weight = 20.0

        self.goal_reward = 100.0

        self.pick_reward = 150.0

        self.unload_reward = 200.0

        self.completion_reward = 500.0

        self.collision_penalty = 100.0

        self.pick_failure_penalty = 50.0

        self.time_penalty = 0.02

        self.linear_energy_weight = 0.01

        self.angular_energy_weight = 0.005

        self.smoothness_weight = 0.005

        # ====================================================
        # WORKFLOW STATE
        # ====================================================

        self.num_bottles = num_bottles

        self.max_carry = max_carry

        self.remaining_to_spawn = 0

        self.collected_bottles = 0

        self.carried_bottles = 0

        self.phase = "bottle"

        self.workflow_complete = False

        # ====================================================
        # ODOM BASELINE
        # ====================================================

        self.episode_odom_x = None
        self.episode_odom_y = None
        self.episode_odom_yaw = None

        # ====================================================
        # EPISODE STATE
        # ====================================================

        self.previous_goal_distance = None

        self.previous_action = np.zeros(
            2,
            dtype=np.float32
        )

        self.current_step = 0

        self.max_steps = 1500

        self.sensor_timeout = 10.0

        self.closed = False

    # ========================================================
    # HELPERS
    # ========================================================

    @staticmethod
    def _wrap_angle(angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def _wait_for_odom(self):

        start = time.time()

        while time.time() - start < 5.0:

            if (
                self.ros_node.odom_x is not None
                and self.ros_node.odom_y is not None
                and self.ros_node.odom_yaw is not None
            ):

                return True

            time.sleep(0.05)

        return False

    def _set_odom_baseline(self):

        if not self._wait_for_odom():

            raise RuntimeError(
                "No /odom data available."
            )

        self.episode_odom_x = (
            self.ros_node.odom_x
        )

        self.episode_odom_y = (
            self.ros_node.odom_y
        )

        self.episode_odom_yaw = (
            self.ros_node.odom_yaw
        )

    def _gazebo_pose_callback(self, msg):

        for pose in msg.pose:

            if pose.name == "scout_piper":

                self.gz_pose = (
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w)
                )

                return

    def _get_gazebo_robot_pose(self):

        if self.gz_pose is None:
            return None

        x, y, qx, qy, qz, qw = self.gz_pose

        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        return x, y, yaw

    def _relative_robot_pose(self):

        if self.episode_odom_x is None:

            return 0.0, 0.0, 0.0

        x = (
            self.ros_node.odom_x
            - self.episode_odom_x
        )

        y = (
            self.ros_node.odom_y
            - self.episode_odom_y
        )

        yaw = self._wrap_angle(
            self.ros_node.odom_yaw
            - self.episode_odom_yaw
        )

        return x, y, yaw

    # ========================================================
    # DUSTBIN TARGET
    # ========================================================

    def _get_dustbin_target(self):

        robot_pose = self._get_gazebo_robot_pose()

        if robot_pose is None:
            return np.array(
                [0.0, 0.0],
                dtype=np.float32
            )

        robot_x, robot_y, robot_yaw = robot_pose

        dx = DUSTBIN_X - robot_x
        dy = DUSTBIN_Y - robot_y

        distance = math.sqrt(
            dx * dx + dy * dy
        )

        world_angle = math.atan2(
            dy,
            dx
        )

        relative_angle = self._wrap_angle(
            world_angle - robot_yaw
        )

        return np.array(
            [
                distance,
                relative_angle
            ],
            dtype=np.float32
        )

    # ========================================================
    # CURRENT TARGET
    # ========================================================

    def _get_current_target(self):

        if self.phase == "bottle":

            target = self.ros_node.bottle_target

            if target is None:

                return np.array(
                    [8.0, 0.0],
                    dtype=np.float32
                )

            return target.copy()

        return self._get_dustbin_target()

    # ========================================================
    # OBSERVATION
    # ========================================================

    def _get_observation(self):

        depth = self.ros_node.depth_sectors

        if depth is None:

            depth = np.full(
                20,
                8.0,
                dtype=np.float32
            )

        target = self._get_current_target()

        observation = np.concatenate(
            [
                depth,

                target,

                np.array(
                    [
                        self.ros_node.linear_velocity,
                        self.ros_node.angular_velocity
                    ],
                    dtype=np.float32
                )
            ]
        )

        observation = np.clip(
            observation,
            self.observation_space.low,
            self.observation_space.high
        )

        return observation.astype(
            np.float32
        )

    # ========================================================
    # GAZEBO SERVICE
    # ========================================================

    def _run_gz_service(self, command):

        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=6
        )

        if result.returncode != 0:

            raise RuntimeError(
                "Gazebo service failed:\n"
                + result.stderr
            )

        return result

    # ========================================================
    # REMOVE BOTTLE
    # ========================================================

    def _remove_bottle(self):

        command = [
            "gz",
            "service",
            "-s",
            f"{WORLD}/remove",
            "--reqtype",
            "gz.msgs.Entity",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            f'name: "{BOTTLE_NAME}", type: MODEL'
        ]

        try:

            self._run_gz_service(
                command
            )

        except Exception:

            pass

    # ========================================================
    # SPAWN BOTTLE
    # ========================================================

    def _spawn_bottle(self):
        """Spawn the next bottle 1.5 m directly in front of Scout."""

        distance = 1.5

        # Use the robot's current Gazebo world pose.
        # /odom is accumulated and must not be used as the
        # absolute world position.

        command = [
            "gz",
            "service",
            "-s",
            f"{WORLD}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            'name: "bottle", position: {x: 0.0, y: 0.0, z: 0.001}'
        ]

        subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=3.0
        )

        # Use the actual Gazebo world pose of scout_piper.
        robot_pose = self._get_gazebo_robot_pose()

        if robot_pose is None:
            raise RuntimeError(
                "Gazebo robot pose for scout_piper is not available."
            )

        robot_x, robot_y, robot_yaw = robot_pose

        bottle_x = (
            robot_x +
            distance * math.cos(robot_yaw)
        )

        bottle_y = (
            robot_y +
            distance * math.sin(robot_yaw)
        )

        print(
            f"[SPAWN DEBUG] Robot Gazebo world pose: "
            f"x={robot_x:.3f}, "
            f"y={robot_y:.3f}, "
            f"yaw={robot_yaw:.3f}"
        )

        print(
            f"[SPAWN DEBUG] Bottle position: "
            f"x={bottle_x:.3f}, "
            f"y={bottle_y:.3f}"
        )

        # Remove old bottle.
        subprocess.run(
            [
                "gz",
                "service",
                "-s",
                f"{WORLD}/remove",
                "--reqtype",
                "gz.msgs.Entity",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "2000",
                "--req",
                'name: "bottle", type: MODEL'
            ],
            capture_output=True,
            text=True,
            timeout=3.0
        )

        time.sleep(0.2)

        # Spawn bottle 1.5 m in front.
        spawn_command = [
            "gz",
            "service",
            "-s",
            f"{WORLD}/create",
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            (
                f'sdf_filename: "{BOTTLE_SDF}", '
                f'pose: {{'
                f'position: {{'
                f'x: {bottle_x}, '
                f'y: {bottle_y}, '
                f'z: 0.001'
                f'}}'
                f'}}'
            )
        ]

        result = subprocess.run(
            spawn_command,
            capture_output=True,
            text=True,
            timeout=4.0
        )

        if "data: true" not in result.stdout:
            print("[SPAWN] Failed:")
            print(result.stdout)
            print(result.stderr)
            raise RuntimeError("Bottle spawn failed.")

        print(
            f"[WORKFLOW] Spawned bottle at "
            f"({bottle_x:.2f}, {bottle_y:.2f})"
        )

    def _wait_for_bottle_target(self):

        self.ros_node.bottle_target = None

        start = time.time()

        while time.time() - start < self.sensor_timeout:

            depth_ready = (
                self.ros_node.depth_sectors
                is not None
            )

            target_ready = (
                self.ros_node.bottle_target
                is not None
            )

            if depth_ready and target_ready:

                return True

            time.sleep(0.05)

        return False

    # ========================================================
    # START NEXT BOTTLE
    # ========================================================

    def _start_next_bottle(self):

        self.phase = "bottle"

        self._spawn_bottle()

        # Give Gazebo, camera, YOLO and depth processing
        # time to detect the newly spawned bottle.
        for attempt in range(3):

            if self._wait_for_bottle_target():
                observation = self._get_observation()

                self.previous_goal_distance = float(
                    observation[20]
                )

                print(
                    f"[WORKFLOW] Bottle target ready "
                    f"(attempt {attempt + 1}/3)."
                )

                return

            print(
                f"[WORKFLOW] Waiting for bottle target "
                f"(attempt {attempt + 1}/3)..."
            )

            time.sleep(1.0)

        raise RuntimeError(
            "Bottle spawned but /bottle_target "
            "was not received after retries."
        )

    # ========================================================
    # START DUSTBIN PHASE
    # ========================================================

    def _start_dustbin_phase(self):

        self.phase = "dustbin"

        self.ros_node.bottle_target = None

        observation = self._get_observation()

        self.previous_goal_distance = float(
            observation[20]
        )

        print(
            "[WORKFLOW] Carry limit reached "
            "or no bottles remain."
        )

        print(
            "[WORKFLOW] Navigating to dustbin."
        )

    # ========================================================
    # UNLOAD
    # ========================================================

    def _unload_bottles(self):

        # Simulation interface.
        #
        # Later this can be replaced by the validated
        # real Piper unload controller without changing
        # PPO navigation.

        print(
            f"[WORKFLOW] Unloading "
            f"{self.carried_bottles} bottles."
        )

        time.sleep(0.5)

        self.carried_bottles = 0

        return True

    # ========================================================
    # WORKFLOW TRANSITION
    # ========================================================

    def _handle_target_reached(self):

        # ----------------------------------------------------
        # Bottle reached
        # ----------------------------------------------------

        if self.phase == "bottle":

            self.ros_node.stop_robot()

            print(
                "[WORKFLOW] Bottle reached."
            )

            picked = pick_bottle()

            if not picked:

                print(
                    "[WORKFLOW] Pick failed."
                )

                return self.pick_failure_penalty, False

            self.collected_bottles += 1

            self.carried_bottles += 1

            print(
                f"[WORKFLOW] Pick successful. "
                f"Collected={self.collected_bottles}/"
                f"{self.num_bottles}, "
                f"carried={self.carried_bottles}/"
                f"{self.max_carry}"
            )

            # ------------------------------------------------
            # Need dustbin?
            # ------------------------------------------------

            if (
                self.carried_bottles >= self.max_carry
                or self.remaining_to_spawn <= 0
            ):

                self._start_dustbin_phase()

            else:

                self.remaining_to_spawn -= 1

                self._start_next_bottle()

            return self.pick_reward, False

        # ----------------------------------------------------
        # Dustbin reached
        # ----------------------------------------------------

        self.ros_node.stop_robot()

        print(
            "[WORKFLOW] Dustbin reached."
        )

        unloaded = self._unload_bottles()

        if not unloaded:

            return -100.0, False

        # ----------------------------------------------------
        # More bottles?
        # ----------------------------------------------------

        if self.remaining_to_spawn > 0:

            self.remaining_to_spawn -= 1

            self._start_next_bottle()

            return self.unload_reward, False

        # ----------------------------------------------------
        # Entire workflow complete
        # ----------------------------------------------------

        self.workflow_complete = True

        print(
            "[WORKFLOW] ALL BOTTLES COLLECTED."
        )

        return (
            self.unload_reward
            + self.completion_reward,
            True
        )

    # ========================================================
    # RESET
    # ========================================================

    def reset(
        self,
        seed=None,
        options=None
    ):

        super().reset(seed=seed)

        self.ros_node.stop_robot()

        time.sleep(0.2)

        # ----------------------------------------------------
        # Reset Scout
        # ----------------------------------------------------

        scout_command = [
            "gz",
            "service",
            "-s",
            f"{WORLD}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            (
                'name: "scout_piper", '
                'position: {x: 0.0, y: 0.0, z: 0.18}, '
                'orientation: {'
                'w: 1.0, x: 0.0, y: 0.0, z: 0.0'
                '}'
            )
        ]

        self._run_gz_service(
            scout_command
        )

        time.sleep(1.0)

        # ----------------------------------------------------
        # Reset sensor state
        # ----------------------------------------------------

        self.ros_node.depth_sectors = None

        self.ros_node.bottle_target = None

        # ----------------------------------------------------
        # Establish odom baseline AFTER teleport.
        # ----------------------------------------------------

        self._set_odom_baseline()

        # ----------------------------------------------------
        # Workflow reset
        # ----------------------------------------------------

        self.collected_bottles = 0

        self.carried_bottles = 0

        self.remaining_to_spawn = (
            self.num_bottles - 1
        )

        self.phase = "bottle"

        self.workflow_complete = False

        self.current_step = 0

        self.previous_action = np.zeros(
            2,
            dtype=np.float32
        )

        self.previous_goal_distance = None

        # ----------------------------------------------------
        # Spawn first bottle
        # ----------------------------------------------------

        self._spawn_bottle()

        # Wait for the perception pipeline to detect
        # the newly spawned bottle.
        target_ready = False

        for attempt in range(3):

            if self._wait_for_bottle_target():
                target_ready = True

                print(
                    f"[WORKFLOW] Initial bottle target ready "
                    f"(attempt {attempt + 1}/3)."
                )

                break

            print(
                f"[WORKFLOW] Waiting for initial bottle target "
                f"(attempt {attempt + 1}/3)..."
            )

            time.sleep(1.0)

        if not target_ready:
            raise RuntimeError(
                "Reset failed: no fresh /bottle_target "
                "after retries."
            )

        self.ros_node.stop_robot()

        observation = self._get_observation()

        self.previous_goal_distance = float(
            observation[20]
        )

        info = {
            "goal_distance":
                self.previous_goal_distance,

            "phase":
                self.phase,

            "carried_bottles":
                self.carried_bottles,

            "collected_bottles":
                self.collected_bottles,

            "remaining_bottles":
                self.remaining_to_spawn,

            "workflow_complete":
                False,

            "sensor_ready":
                True
        }

        print(
            "[WORKFLOW] Episode reset: "
            f"{self.num_bottles} bottles"
        )

        return observation, info

    # ========================================================
    # STEP
    # ========================================================

    def step(self, action):

        action = np.clip(
            action,
            self.action_space.low,
            self.action_space.high
        )

        linear_velocity = float(
            action[0]
        )

        angular_velocity = float(
            action[1]
        )

        self.ros_node.send_action(
            linear_velocity,
            angular_velocity
        )

        time.sleep(0.1)

        observation = self._get_observation()

        goal_distance = float(
            observation[20]
        )

        min_obstacle = float(
            np.min(observation[:20])
        )

        # ====================================================
        # REWARD
        # ====================================================

        if self.previous_goal_distance is None:

            progress = 0.0

        else:

            progress = (
                self.previous_goal_distance
                - goal_distance
            )

        reward = (
            self.progress_weight
            * progress
        )

        reward -= self.time_penalty

        energy_penalty = (
            self.linear_energy_weight
            * linear_velocity ** 2
            +
            self.angular_energy_weight
            * angular_velocity ** 2
        )

        reward -= energy_penalty

        action_change = (
            action
            - self.previous_action
        )

        smoothness_penalty = (
            self.smoothness_weight
            * float(
                np.sum(
                    action_change ** 2
                )
            )
        )

        reward -= smoothness_penalty

        terminated = False

        truncated = False

        goal_reached = False

        collision = False

        # ====================================================
        # COLLISION
        # ====================================================

        if min_obstacle <= self.collision_threshold:

            reward -= self.collision_penalty

            collision = True

            terminated = True

            self.ros_node.stop_robot()

        # ====================================================
        # TARGET REACHED
        # ====================================================

        elif goal_distance <= (
            self.dustbin_goal_threshold
            if self.phase == "dustbin"
            else self.goal_threshold
        ):

            goal_reached = True

            reward += self.goal_reward

            transition_reward, workflow_done = (
                self._handle_target_reached()
            )

            reward += transition_reward

            terminated = workflow_done

            observation = self._get_observation()

            goal_distance = float(
                observation[20]
            )

        # ====================================================
        # STATE UPDATE
        # ====================================================

        self.previous_goal_distance = (
            goal_distance
        )

        self.previous_action = np.array(
            action,
            dtype=np.float32
        )

        self.current_step += 1

        if self.current_step >= self.max_steps:

            truncated = True

            self.ros_node.stop_robot()

        # ====================================================
        # INFO
        # ====================================================

        info = {

            "goal_distance":
                goal_distance,

            "min_obstacle":
                min_obstacle,

            "progress":
                progress,

            "energy_penalty":
                energy_penalty,

            "smoothness_penalty":
                smoothness_penalty,

            "goal_reached":
                goal_reached,

            "collision":
                collision,

            "phase":
                self.phase,

            "carried_bottles":
                self.carried_bottles,

            "collected_bottles":
                self.collected_bottles,

            "remaining_bottles":
                self.remaining_to_spawn,

            "workflow_complete":
                self.workflow_complete,

            "step":
                self.current_step
        }

        return (
            observation,
            float(reward),
            terminated,
            truncated,
            info
        )

    # ========================================================
    # CLOSE
    # ========================================================

    def close(self):

        if self.closed:

            return

        self.closed = True

        try:

            self.ros_node.stop_robot()

            time.sleep(0.1)

        except Exception:

            pass

        try:

            self.executor.shutdown(
                timeout_sec=1.0
            )

        except Exception:

            pass

        try:

            self.executor_thread.join(
                timeout=1.0
            )

        except Exception:

            pass

        try:

            self.ros_node.destroy_node()

        except Exception:

            pass
