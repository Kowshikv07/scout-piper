import subprocess


WORLD = "/world/scout_world"
BOTTLE_NAME = "bottle"


def pick_bottle():
    """
    Simulation Piper pick interface.

    Removes the selected bottle model from Gazebo and
    returns True when Gazebo confirms the removal.
    """

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
        "2000",
        "--req",
        f'name: "{BOTTLE_NAME}", type: MODEL'
    ]

    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=3.0
        )
    except subprocess.TimeoutExpired:
        print("[PICKER] Remove service timed out.")
        return False

    if "data: true" in result.stdout:
        print("[PICKER] Simulated pick successful.")
        return True

    print("[PICKER] Failed to remove bottle.")
    print(result.stdout)
    print(result.stderr)

    return False
