import os

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor

from my_robot.scout_env import ScoutEnv


def main():

    # ---------------------------------------------------------
    # Paths
    # ---------------------------------------------------------

    model_dir = os.path.expanduser(
        "~/ros2_ws/models/ppo_scout"
    )

    log_dir = os.path.expanduser(
        "~/ros2_ws/logs/ppo_scout"
    )

    os.makedirs(model_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    # ---------------------------------------------------------
    # Environment
    # ---------------------------------------------------------

    print("Creating Scout environment...")

    env = ScoutEnv()
    env = Monitor(env)

    # ---------------------------------------------------------
    # PPO
    # ---------------------------------------------------------

    print("Creating PPO model...")

    model = PPO(
        policy="MlpPolicy",
        env=env,

        learning_rate=3e-4,

        n_steps=1024,
        batch_size=128,

        gamma=0.99,
        gae_lambda=0.95,

        ent_coef=0.02,

        policy_kwargs=dict(
            net_arch=dict(
                pi=[128, 128],
                vf=[128, 128]
            )
        ),

        verbose=1,

        tensorboard_log=None,

        device="cpu"
    )

    # ---------------------------------------------------------
    # Training
    # ---------------------------------------------------------

    total_timesteps = 50_000

    print()
    print("=" * 60)
    print("Starting PPO training")
    print("Total timesteps:", total_timesteps)
    print("=" * 60)

    try:

        model.learn(
            total_timesteps=total_timesteps,
            progress_bar=False
        )

        # -----------------------------------------------------
        # Save trained model
        # -----------------------------------------------------

        model_path = os.path.join(
            model_dir,
            "ppo_scout_bottle"
        )

        model.save(model_path)

        print()
        print("=" * 60)
        print("TRAINING COMPLETE")
        print("Model saved to:")
        print(model_path + ".zip")
        print("=" * 60)

    except KeyboardInterrupt:

        print("\nTraining interrupted.")

        emergency_path = os.path.join(
            model_dir,
            "ppo_scout_interrupted"
        )

        model.save(emergency_path)

        print(
            "Current model saved to:",
            emergency_path + ".zip"
        )

    finally:

        env.close()
        print("Environment closed.")


if __name__ == "__main__":
    main()
