# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Target-accumulating differential IK action.

Isaac Lab's relative-mode DifferentialInverseKinematicsAction adds each delta to the end-effector pose *measured* at
that moment. Whatever part of the previous delta the arm had not yet achieved is silently dropped, so with any tracking
lag the arm travels less than the policy commanded, step after step.

The real WidowX stack Bridge data was collected on, and SIMPLER's reproduction of it ("arm_pd_ee_target_delta_pose"),
add each delta to the previous *target* instead, so commanded motion is never lost. This term does the same for the
position part of the command. The orientation delta is still applied to the measured orientation (Octo's rotation
deltas are small and a drifting orientation target is not worth the risk). To avoid wind-up when the arm is blocked,
e.g. pressing on the table, the target is kept within ``max_target_offset`` metres of the measured position.
"""

from __future__ import annotations

from collections.abc import Sequence

import torch

from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.envs.mdp.actions.task_space_actions import DifferentialInverseKinematicsAction
from isaaclab.utils import configclass


class TargetDeltaIKAction(DifferentialInverseKinematicsAction):
    cfg: "TargetDeltaIKActionCfg"

    def __init__(self, cfg: "TargetDeltaIKActionCfg", env):
        super().__init__(cfg, env)
        self._target_pos = torch.zeros(self.num_envs, 3, device=self.device)
        self._has_target = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    def process_actions(self, actions: torch.Tensor):
        # Base class: scale / clip the action and set the IK command from the measured pose
        super().process_actions(actions)

        ee_pos_curr, ee_quat_curr = self._compute_frame_pose()

        # Where the previous target exists, apply the delta to it instead of to the measured position
        base_pos = torch.where(self._has_target.unsqueeze(-1), self._target_pos, ee_pos_curr)

        # Anti-wind-up: never let the base of the new target be further than max_target_offset from the arm
        offset = base_pos - ee_pos_curr
        dist = torch.linalg.norm(offset, dim=-1, keepdim=True)
        scale = torch.clamp(self.cfg.max_target_offset / dist.clamp_min(1e-9), max=1.0)
        base_pos = ee_pos_curr + offset * scale

        self._ik_controller.set_command(self._processed_actions, base_pos, ee_quat_curr)

        # Remember the resulting target for the next step
        self._target_pos[:] = self._ik_controller.ee_pos_des
        self._has_target[:] = True

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        if env_ids is None:
            self._has_target[:] = False
        else:
            self._has_target[env_ids] = False


@configclass
class TargetDeltaIKActionCfg(DifferentialInverseKinematicsActionCfg):
    """Relative differential IK whose position deltas accumulate on the previous target (see module docstring)."""

    class_type: type = TargetDeltaIKAction

    max_target_offset: float = 0.05
    """Maximum distance (m) the accumulated target may lead the measured end-effector position."""
