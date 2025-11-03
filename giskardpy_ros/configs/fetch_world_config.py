from dataclasses import dataclass, field
from giskardpy.model.world_config import WorldConfig
from semantic_digital_twin.robots.abstract_robot import AbstractRobot
from semantic_digital_twin.world import World

@dataclass
class WorldWithFetchedRobot(WorldConfig):
    robot_world: World = None
    robot_class: type[AbstractRobot] = None  # 👈 Type hint: expects a class, not instance
    robot: AbstractRobot = field(init=False, default=None)

    def setup_collision_config(self):
        pass

    def setup_world(self):
        super().setup_world()
        self.world.merge_world(self.robot_world)
        self.robot = self.robot_class.from_world(self.world)
