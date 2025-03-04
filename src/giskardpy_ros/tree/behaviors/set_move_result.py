from line_profiler import profile
from py_trees import Status

from giskard_msgs.msg import MoveResult, GiskardError
from giskardpy.data_types.exceptions import *
from giskardpy.god_map import god_map
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy.middleware import get_middleware
from giskardpy_ros.tree.behaviors.publish_feedback import giskard_state_to_execution_state
from giskardpy_ros.tree.blackboard_utils import GiskardBlackboard
from giskardpy.utils.decorators import record_time
import giskardpy_ros.ros1.msg_converter as msg_converter
from collections import OrderedDict
from giskardpy.model.trajectory import Trajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header


class SetMoveResult(GiskardBehavior):

    @profile
    def __init__(self, name, context, print=True):
        self.print = print
        self.context = context
        super().__init__(name)

    @record_time
    @profile
    def update(self):
        e = self.get_blackboard_exception()
        if e is None:
            move_result = MoveResult()
        else:
            move_result = MoveResult(error=msg_converter.exception_to_error_msg(e))

        trajectory = god_map.trajectory
        joints = [god_map.world.joints[joint_name] for joint_name in god_map.world.movable_joint_names]
        sample_period = god_map.qp_controller.mpc_dt
        debug_trajectory = god_map.debug_expression_manager.raw_traj_to_traj(god_map.qp_controller.control_dt)
        try:
            move_result.trajectory = self.trajectory_to_ros_message(debug_trajectory)
        except:
            move_result.trajectory = msg_converter.trajectory_to_ros_trajectory(trajectory,
                                                                                sample_period=sample_period,
                                                                                start_time=0,
                                                                                joints=joints)

        if isinstance(e, PreemptedException):
            get_middleware().logwarn(f'Goal preempted: \'{move_result.error.msg}\'.')
        else:
            if self.print:
                if move_result.error.type == GiskardError.SUCCESS:
                    get_middleware().loginfo(f'{self.context} succeeded.')
                else:
                    get_middleware().logwarn(f'{self.context} failed: {move_result.error.msg}.')
        GiskardBlackboard().move_action_server.result_msg = move_result
        move_result.execution_state = giskard_state_to_execution_state()
        return Status.SUCCESS

    def trajectory_to_ros_message(self, traj: Trajectory) -> JointTrajectory:
        ros_traj = JointTrajectory()

        # Set header (optional: add timestamp or frame_id)
        ros_traj.header = Header()
        # ros_traj.header.stamp = rospy.Time.now()  # Assuming ROS is running
        ros_traj.header.frame_id = "map"  # Change if needed

        # Get joint names
        try:
            ros_traj.joint_names = traj.get_joint_names()
        except IndexError:
            raise ValueError("Cannot convert empty Trajectory to JointTrajectory")

        # Iterate through trajectory points
        for time, joint_states in traj.items():
            point = JointTrajectoryPoint()
            # point.time_from_start = rospy.Duration(time)  # Convert time to ROS duration

            # Extract position, velocity, acceleration for each joint
            positions, velocities, accelerations = [], [], []

            for joint_name in ros_traj.joint_names:
                joint_state = joint_states.get(joint_name)  # Get JointState data for joint

                if joint_state:
                    positions.append(joint_state.state[0])  # Position
                    if len(joint_state.state) > 1:
                        velocities.append(joint_state.state[1])  # Velocity
                    if len(joint_state.state) > 2:
                        accelerations.append(joint_state.state[2])  # Acceleration
                else:
                    positions.append(0.0)
                    velocities.append(0.0)
                    accelerations.append(0.0)

            # Assign extracted values to ROS JointTrajectoryPoint
            point.positions = positions
            if velocities:
                point.velocities = velocities
            if accelerations:
                point.accelerations = accelerations

            ros_traj.points.append(point)

        return ros_traj
