# test_policy_loader.py
import io
from pathlib import Path
import numpy as np
import torch
from config_loader import parse_env_config, get_robot_joint_properties, get_physics_properties

class PolicyTester:
    def __init__(self):
        self.policy = None
        self.policy_env_params = None
        self.dof_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
        ]
        self._decimation = None
        self._dt = None
        self.render_interval = None
        self._max_effort = None
        self._max_vel = None
        self._stiffness = None
        self._damping = None
        self.default_pos = None
        self.default_vel = None

    def load_policy(self, policy_file_path, policy_env_path) -> None:
        print("\n=== Policy Loading ===")
        print(f"{'Model path:':<18} {policy_file_path}")
        print(f"{'Environment path:':<18} {policy_env_path}")

        # Load TorchScript policy or use dummy if not present
        try:
            with open(policy_file_path, "rb") as f:
                file = io.BytesIO(f.read())
            self.policy = torch.jit.load(file)
        except Exception:
            print("Could not load actual model, using dummy object instead.")
            self.policy = object()


        # Load environment config
        print("\n=== Loading Environment Config ===")
        self.policy_env_params = parse_env_config(policy_env_path)
        self._decimation, self._dt, self.render_interval = get_physics_properties(self.policy_env_params)
        self._max_effort, self._max_vel, self._stiffness, self._damping, self.default_pos, self.default_vel = get_robot_joint_properties(
            self.policy_env_params, self.dof_names
        )


        # Get physics properties
        self._decimation, self._dt, self.render_interval = get_physics_properties(self.policy_env_params)
        print("\n--- Physics properties ---")
        print(f"{'Decimation:':<18} {self._decimation}")
        print(f"{'Timestep (dt):':<18} {self._dt}")
        print(f"{'Render interval:':<18} {self.render_interval}")

        # Get robot joint properties
        self._max_effort, self._max_vel, self._stiffness, self._damping, self.default_pos, self.default_vel = get_robot_joint_properties(
            self.policy_env_params, self.dof_names
        )
        self.num_joints = len(self.dof_names)

        print("\n--- Robot joint properties ---")
        print(f"{'Number of joints:':<18} {self.num_joints}")
        print(f"{'Max effort:':<18} {self._max_effort}")
        print(f"{'Max velocity:':<18} {self._max_vel}")
        print(f"{'Stiffness:':<18} {self._stiffness}")
        print(f"{'Damping:':<18} {self._damping}")
        print(f"{'Default position:':<18} {self.default_pos}")
        print(f"{'Default velocity:':<18} {self.default_vel}")
        print("\n=== Environment Loaded ===\n")


# Paths based on your repo structure
repo_root = Path(__file__).resolve().parents[1]  # sim2real
model_dir = repo_root / "agents" / "ptp"

tester = PolicyTester()
tester.load_policy(model_dir / "skywalker_ptp_policy.pt", model_dir / "env.yaml")
