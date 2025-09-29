#!/usr/bin/env python3
"""
gen3.py
--------------------

Thin wrapper around a pre-trained reach policy for the Kinova Gen3 arm.
Extends `PolicyController` with:

* State update via `update_joint_state()`
* Forward pass (`forward`) that returns a target joint-position command
  every call, computing a new action every ``decimation`` steps.

Change the default paths below or pass them explicitly in the constructor.

Author: Louis Le Lay
"""

from pathlib import Path

import numpy as np

from rob10_robot_policy.controllers.policy_controller import PolicyController


class Gen3ReachPolicy(PolicyController):
    """Policy controller for Gen3 Reach using a pre-trained policy model."""

    def __init__(self) -> None:
        """Initialize the URReachPolicy instance."""
        super().__init__()
        self.dof_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        # Load the pre-trained policy model and environment configuration
        # Current file is sim2real/robots/gen3.py
        repo_root = Path(__file__).resolve().parents[1]  # go up two levels to sim2real
        model_dir = repo_root / "agents" / "move"

        self.load_policy(
            model_dir / "base_move_policy_1.pt",  # your new policy filename
            model_dir / "env_3.yaml",  # the corresponding env.yaml in the same folder
        )

        self._action_scale = 0.5
        self._previous_action = np.zeros(7)
        self._policy_counter = 0
        self.target_command = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])  # Initialize target command
        ##self.curret_ee_position = np.zeros(3)

        self.has_joint_data = False
        self.current_joint_positions = np.zeros(7)
        self.current_joint_velocities = np.zeros(7)

    def update_joint_state(self, position, velocity) -> None:
        """
        Update the current joint state.

        Args:
            position: A list or array of joint positions.
            velocity: A list or array of joint velocities.
        """
        self.current_joint_positions = np.array(position[:self.num_joints], dtype=np.float32)
        self.current_joint_velocities = np.array(velocity[:self.num_joints], dtype=np.float32)
        self.has_joint_data = True

    def _compute_observation(self, command: np.ndarray) -> np.ndarray:
        """
        Compute the observation vector for the policy network.

        Args:
            command: The target command vector.

        Returns:
            An observation vector if joint data is available, otherwise None.
        """
        if not self.has_joint_data:
            return None
        obs = np.zeros(35)  # Observation size matches `num_observations` in env.yaml
        obs[:7] = self.current_joint_positions - self.default_pos  # Joint position deltas
        obs[7:14] = self.current_joint_velocities  # Joint velocities
        obs[14:21] = self.current_base_position  # End-effector position (to be computed)
        obs[21:28] = command  # Target position in 6D space
        obs[28:35] = self._previous_action  ##np.linalg.norm(obs[14:17] - obs[17:20])  # Distance to target
        return obs

    def forward(self, dt: float, command: np.ndarray) -> np.ndarray:
        """
        Compute the next joint positions based on the policy.

        Args:
            dt: Time step for the forward pass.
            command: The target command vector.

        Returns:
            The computed joint positions if joint data is available, otherwise None.
        """
        if not self.has_joint_data:
            return None

        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            if obs is None:
                return None
            full_action = self._compute_action(obs)
            self.action = full_action[:7]  # Only use the first 7 dimensions for joint positions
            self._previous_action = self.action.copy()

            # # Debug Logging (commented out)
            print("\n=== Policy Step ===")
            print(f"{'Command:':<20} {np.round(command, 4)}\n")
            print("--- Observation ---")
            print(f"{'Δ Joint Positions:':<20} {np.round(obs[:6], 4)}")
            print(f"{'Joint Velocities:':<20} {np.round(obs[6:12], 4)}")
            print(f"{'Command:':<20} {np.round(obs[12:19], 4)}")
            print(f"{'Previous Action:':<20} {np.round(obs[19:25], 4)}\n")
            print("--- Action ---")
            print(f"{'Raw Action:':<20} {np.round(self.action, 4)}")
            processed_action = self.default_pos + (self.action * self._action_scale)
            print(f"{'Processed Action:':<20} {np.round(processed_action, 4)}")
            print(f"{'Current Joint Pos:':<20} {np.round(self.current_joint_positions, 4)}")
            print(f"{'Current Joint Vel:':<20} {np.round(self.current_joint_velocities, 4)}")

            # debug print of the obs
            # obs = np.zeros(35)  # Observation size matches `num_observations` in env.yaml
            # obs[:7] = self.current_joint_positions - self.default_pos  # Joint position deltas
            # obs[7:14] = self.current_joint_velocities  # Joint velocities
            # obs[14:21] = self.current_ee_position  # End-effector position (to be computed)
            # obs[21:28] = command  # Target position in 6D space
            # obs[28:35] = self._previous_action
            print(f"current joint pos:  {np.round(self.current_joint_velocities, 4)}")
            print(f"current joint vel:  {np.round(self.current_joint_velocities, 4)}")
            print(f"current ee pos:  {np.round(self.current_base_position, 4)}")
            print(f"target pos:  {np.round(command, 4)}")
            print(f"previous action: {np.round(self._previous_action, 4)}")

        joint_positions = self.default_pos + (self.action * self._action_scale)
        self._policy_counter += 1
        return joint_positions
