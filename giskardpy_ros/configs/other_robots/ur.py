from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import TYPE_CHECKING, Optional

from giskardpy.model.world_config import WorldWithFixedRobot
from giskardpy_ros.configs.robot_interface_config import (
    RobotInterfaceConfig,
    StandAloneRobotInterfaceConfig,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.abstract_robot import AbstractRobot
from semantic_digital_twin.robots.ur5 import UR5

if TYPE_CHECKING:
    from semantic_digital_twin.world import World

@dataclass
class WorldWithUR5Config(WorldWithFixedRobot):
    urdf_view: AbstractRobot = field(kw_only=True, default=UR5)
    root_name: PrefixedName = field(default=PrefixedName("map2"))

    def setup_collision_config(self) -> None:
        pass

    def setup_world(self, robot_name: Optional[str] = None) -> None:
        """
        Set up the world and locate the UR10Bolt semantic annotation.

        :param robot_name: Unused, kept for interface compatibility.
        """
        super().setup_world()
        self.robot = self.world.get_semantic_annotations_by_type(UR5)[0]


class UR5VelocityInterface(RobotInterfaceConfig):
    """
    Robot interface for the UR10 in velocity control mode.

    Subscribes to ``/joint_states`` and publishes velocity commands on
    ``/forward_velocity_controller/commands``.
    """

    def setup(self) -> None:
        self.sync_joint_state_topic("/joint_states")
        joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.add_joint_velocity_group_controller(
            cmd_topic="/realtime_body_controller_real/command",
            connections=joints,
        )


class UR5StandAloneRobotInterfaceConfig(StandAloneRobotInterfaceConfig):
    """Standalone robot interface for the UR10 (simulation, no hardware)."""

    def __init__(self) -> None:
        super().__init__(
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ]
        )