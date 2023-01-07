#!/usr/bin/env python

import tf2_ros
import tf
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, WrenchStamped, TransformStamped
import rospy
import numpy as np
from copy import deepcopy
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from std_msgs.msg import Float64
from math import fabs
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest
from moveit_msgs.msg import ExecuteTrajectoryActionResult
from copy import deepcopy
from trac_ik_python.trac_ik import IK
from moveit_msgs.msg import OrientationConstraint, Constraints, JointConstraint, PositionConstraint


def sample_from_func(func, start, stop, number_of_points):
    func_input = np.linspace(start=start, stop=stop,
                             num=number_of_points).tolist()
    func_output = [func(i) for i in func_input]
    return func_input, func_output


def filter_plan(plan):
    last_time_step = plan.joint_trajectory.points[0].time_from_start.to_sec()
    new_plan = RobotTrajectory()
    new_plan.joint_trajectory.header = plan.joint_trajectory.header
    new_plan.joint_trajectory.joint_names = plan.joint_trajectory.joint_names
    new_plan.joint_trajectory.points.append(
        plan.joint_trajectory.points[0])
    for i in range(1, len(plan.joint_trajectory.points)):
        point = plan.joint_trajectory.points[i]
        if point.time_from_start.to_sec() > last_time_step:
            new_plan.joint_trajectory.points.append(point)
        last_time_step = point.time_from_start.to_sec()
    return new_plan


class TransformServices():
    def __init__(self):
        self.transformer_listener = tf.TransformListener()
        self.transformer_broadcaster = tf2_ros.StaticTransformBroadcaster()

    def transform_poses(self, target_frame, source_frame, pose_arr):
        """
        Transform poses from source_frame to target_frame
        """
        trans_pose_arr = PoseArray()
        for i in range(len(pose_arr.poses)):
            trans_pose = PoseStamped()
            pose = PoseStamped()
            pose.header.frame_id = source_frame
            pose.pose = pose_arr.poses[i]
            self.transformer_listener.waitForTransform(
                target_frame, source_frame, rospy.Time(), rospy.Duration(1))
            trans_pose = self.transformer_listener.transformPose(
                target_frame, pose)
            trans_pose_arr.poses.append(trans_pose.pose)

        trans_pose_arr.header.frame_id = target_frame
        trans_pose_arr.header.stamp = rospy.Time()
        return trans_pose_arr

    def lookup_transform(self, target_frame, source_frame):
        self.transformer_listener.waitForTransform(
            target_frame, source_frame, rospy.Time(), rospy.Duration(1))
        t, r = self.transformer_listener.lookupTransform(
            target_frame, source_frame, rospy.Time())
        pose = Pose()
        pose.position.x = t[0]
        pose.position.y = t[1]
        pose.position.z = t[2]
        pose.orientation.x = r[0]
        pose.orientation.y = r[1]
        pose.orientation.z = r[2]
        pose.orientation.w = r[3]
        return pose

    def create_frame(self, ref_frame, moving_frame, new_frame):
        pose = self.lookup_transform(ref_frame, moving_frame)
        return self.create_frame_at_pose(pose, ref_frame, new_frame)
    
    def create_frame_at_pose(self, pose, ref_frame, new_frame):
        translation = [pose.position.x, pose.position.y, pose.position.z]
        orientation = [pose.orientation.x, pose.orientation.y,
                       pose.orientation.z, pose.orientation.w]
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = ref_frame
        static_transformStamped.child_frame_id = new_frame

        static_transformStamped.transform.translation.x = translation[0]
        static_transformStamped.transform.translation.y = translation[1]
        static_transformStamped.transform.translation.z = translation[2]

        static_transformStamped.transform.rotation.x = orientation[0]
        static_transformStamped.transform.rotation.y = orientation[1]
        static_transformStamped.transform.rotation.z = orientation[2]
        static_transformStamped.transform.rotation.w = orientation[3]
        self.transformer_broadcaster.sendTransform(static_transformStamped)
        return pose


class MotionServices():
    def __init__(self, tool_group="cutting_tool", wrench_topic="ft_sensor_wrench/wrench/filtered", transform_force=False, from_frame=None, to_frame=None):
        self.move_group = MoveGroupCommander(tool_group)
        self.ts = TransformServices()
        # TODO: Adjust quaternions of links
        self.tool_quat_base_link = [0, 1, 0, 0]
        self.tool_quat_table_link = [0.707, -0.707, 0.000, -0.000]
        self.current_force = {'z': 0, 'x': 0, 'y': 0, 'xy': 0, 'yz': 0, 'xz': 0}
        self.current_torque = {'z': 0, 'xy': 0, 'x': 0, 'y': 0}
        self.current_arm = {'x' : 0, 'y': 0, 'z': 0}
        self.transformed_current_arm = {'x' : 0, 'y': 0, 'z': 0}
        self.transformed_force_pub = rospy.Publisher("/wrench/transformed", WrenchStamped, queue_size=1)
        self.transformed_force = None
        self.do_transform_force = transform_force
        self.to_frame = to_frame
        self.from_frame = from_frame
        self.init_force = 0
        self.trigger_stop = False
        self.force_thresh = 0
        self.axis = 'z' if self.do_transform_force else 'xy'
        self.constraints_dict = {}
        self.get_constraints_dict()
        # rospy.Subscriber("ft_sensor_wrench/resultant/filtered", Float64, self.force_xy_cb)
        # rospy.Subscriber("ft_sensor_wrench/filtered_z", Float64, self.forces_cb)
        rospy.Subscriber("ft_sensor_wrench/resultant/filtered/xy",
                         Float64, self.force_xy_cb)
        rospy.Subscriber("ft_sensor_wrench/resultant/filtered/yz",
                         Float64, self.force_yz_cb)
        rospy.Subscriber("ft_sensor_wrench/resultant/filtered/xz",
                         Float64, self.force_xz_cb)
        rospy.Subscriber(wrench_topic,
                         WrenchStamped, self.forces_cb)
        rospy.Subscriber("/execute_trajectory/result", ExecuteTrajectoryActionResult, self.goal_status_cb)
        self.goal_status = None
    
    def get_constraints_dict(self):
        constraints = rospy.get_param(
            "/generate_state_database/constraints/constraints")
        constraints_name = rospy.get_param(
            "/generate_state_database/constraints/name")
        if len(constraints) > 0:
            all_constr = Constraints()
            all_constr.name = constraints_name
            self.constraints_dict[constraints_name] = all_constr
            for constr_str in constraints:
                if constr_str['type'] == 'orientation':
                    constr = OrientationConstraint()
                    quat = quaternion_from_euler(constr_str['orientation'][0],
                                                 constr_str['orientation'][1],
                                                 constr_str['orientation'][2], axes='sxyz')
                    constr.orientation.x = quat[0]
                    constr.orientation.y = quat[1]
                    constr.orientation.z = quat[2]
                    constr.orientation.w = quat[3]

                if constr_str['type'] == 'position':
                    constr = PositionConstraint()
                    constr.position.x = constr_str['position'][0]
                    constr.position.y = constr_str['position'][1]
                    constr.position.z = constr_str['position'][2]

                if constr_str['type'] in ['position', 'orientation']:
                    constr.absolute_x_axis_tolerance = constr_str['tolerances'][0]
                    constr.absolute_y_axis_tolerance = constr_str['tolerances'][1]
                    constr.absolute_z_axis_tolerance = constr_str['tolerances'][2]

                if constr_str['type'] == 'joint':
                    constr = JointConstraint()
                    constr.position = constr_str['position']
                    constr.tolerance_above = constr_str['tolerance_above']
                    constr.tolerance_below = constr_str['tolerance_below']

                constr.link_name = constr_str['link_name']
                constr.header.frame_id = constr_str['frame_id']
                constr.weight = constr_str['weight']
                constr_type_list = getattr(
                    all_constr, constr_str['type'] + '_constraints')
                constr_type_list.append(constr)
        
    def pose_goal(self, pose,
                  ref_frame="base_link",
                  tool_frame="gripper_tip_link",
                  position_shift=(0,0,0),
                  tol_pos=(0.01, 0.01, 0.01),
                  tol_ori=(0.1, 0.1, 6.28),
                  joint_goal=False,
                  approximated=False,
                  constraints_name=None):
        success = False
        while not success and not rospy.is_shutdown():
            self.move_group.set_pose_reference_frame(ref_frame)
            self.ts.create_frame_at_pose(pose, ref_frame, "rotate_to_frame")
            if constraints_name is not None:
                constr = self.constraints_dict[constraints_name]
                constr.orientation_constraints[0].orientation = pose.orientation
                constr.orientation_constraints[0].link_name = tool_frame
                self.move_group.set_path_constraints(constr)
                print("Setting constraints: {}".format(constraints_name))
            if joint_goal:
                ik_solver = IK("base_link",
                            "gripper_tip_link")

                seed_state = [0.0] * ik_solver.number_of_joints
                result = ik_solver.get_ik(seed_state,
                                        pose.position.x, pose.position.y, pose.position.z,  # X, Y, Z
                                        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                                          bx=tol_pos[0], by=tol_pos[1], bz=tol_pos[2],
                                          brx=tol_ori[0], bry=tol_ori[1], brz=tol_ori[2])  # QX, QY, QZ, QW
                self.move_group.set_joint_value_target(result)
            else:
                self.move_group.set_pose_target(pose)
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            self.move_group.clear_path_constraints()
            if not success:
                rospy.sleep(1)
        return success

    def move_straight(self, poses, ref_frame="base_link",
                      vel_scale=0.005, acc_scale=0.005,
                      avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0,
                      wait_execution=True, retry_with_large_step=False):
        # set the pose_arr
        self.move_group.set_pose_reference_frame(ref_frame)
        fraction = 0.0
        while fraction < 0.99:
            plan, fraction = self.move_group.compute_cartesian_path(
                poses.poses, eef_step, jump_threshold, avoid_collisions)
            if fraction < 0.99 and retry_with_large_step:
                eef_step += 0.01
                print("retrying with eef_step: ", eef_step)
            else:
                break
            print(fraction)

        # filter the output plan 
        filtered_plan = filter_plan(plan)

        # execute the filtered plan
        final_traj = self.move_group.retime_trajectory(
            self.move_group.get_current_state(), filtered_plan, vel_scale, acc_scale)
        result = self.move_group.execute(final_traj, wait_execution)

        return result
    
    def transform_force(self, msg):
        force_pose = Pose()
        force_pose.position.x = msg.wrench.force.x
        force_pose.position.y = msg.wrench.force.y
        force_pose.position.z = msg.wrench.force.z
        force_pose.orientation.x = 0
        force_pose.orientation.y = 0
        force_pose.orientation.z = 0
        force_pose.orientation.w = 1
        pose_arr = PoseArray()
        pose_arr.header.frame_id = self.from_frame
        pose_arr.poses.append(force_pose)
        transformed_force = self.ts.transform_poses(self.to_frame, self.from_frame, pose_arr).poses[0]
        # torque_pose = Pose()
        # torque_pose.position.x = msg.wrench.torque.x
        # torque_pose.position.y = msg.wrench.torque.y
        # torque_pose.position.z = msg.wrench.torque.z
        # torque_pose.orientation.x = 0
        # torque_pose.orientation.y = 0
        # torque_pose.orientation.z = 0
        # torque_pose.orientation.w = 1
        # pose_arr = PoseArray()
        # pose_arr.header.frame_id = self.from_frame
        # pose_arr.poses.append(torque_pose)
        # transformed_torque = self.ts.transform_poses(self.to_frame, self.from_frame, pose_arr).poses[0]
        transformed_force_dict = {'x':0,'y':0,'z':0}
        transformed_torque_dict = {'x':0,'y':0,'z':0}
        transformed_current_arm_dict = {'x':0,'y':0,'z':0}
        transformed_force_dict['x'] = transformed_force.position.x
        transformed_force_dict['y'] = transformed_force.position.y
        transformed_force_dict['z'] = transformed_force.position.z
        # transformed_torque_dict['x'] = transformed_torque.position.x
        # transformed_torque_dict['y'] = transformed_torque.position.y
        # transformed_torque_dict['z'] = transformed_torque.position.z
        # transformed_current_arm_dict['y'] = transformed_torque_dict['y'] / (transformed_force_dict['z'] + 1e-6)
        return transformed_force_dict, transformed_torque_dict, transformed_current_arm_dict

    def forces_cb(self, msg):
        if self.do_transform_force:
            self.current_force, self.current_torque, self.current_arm = self.transform_force(msg)
            transformed_force_msg = WrenchStamped()
            transformed_force_msg.header.frame_id = self.to_frame
            transformed_force_msg.wrench.force.x = self.current_force['x']
            transformed_force_msg.wrench.force.y = self.current_force['y']
            transformed_force_msg.wrench.force.z = self.current_force['z']
            transformed_force_msg.wrench.torque.x = self.current_torque['x']
            transformed_force_msg.wrench.torque.y = self.current_torque['y']
            transformed_force_msg.wrench.torque.z = self.current_torque['z']
            # Monitor the force until it reaches force_thresh.
            change_force = fabs(self.current_force[self.axis] - self.init_force)
            if change_force >= self.force_thresh and self.goal_status != 3 and self.trigger_stop:
                self.move_group.stop()
                self.trigger_stop = False
                print("change_force: ", change_force)
            self.transformed_force_pub.publish(transformed_force_msg)
        else:
            self.current_force['x'] = msg.wrench.force.x
            self.current_force['y'] = msg.wrench.force.y
            self.current_force['z'] = msg.wrench.force.z
            self.current_torque['x'] = msg.wrench.torque.x
            self.current_torque['y'] = msg.wrench.torque.y
            self.current_torque['z'] = msg.wrench.torque.z
            self.current_arm['y'] = self.current_torque['y'] / (self.current_force['z'] + 1e-6)
            self.current_force['xy'] = np.sqrt(self.current_force['x']**2 + self.current_force['y']**2)
            # print(self.current_force['xy'])

    def force_xy_cb(self, msg):
        self.current_force['xy'] = msg.data
        
    def force_yz_cb(self, msg):
        self.current_force['yz'] = msg.data
        
    def force_xz_cb(self, msg):
        self.current_force['xz'] = msg.data
    
    def goal_status_cb(self, msg):
        # status = 2 means "Preempted" i.e. We stopped in the middle of motion.
        # status = 3 means "Solution was found and executed." i.e. Motion completed successfully.
        self.goal_status = msg.status.status

    def move_to_touch(self, poses, axis, force_thresh=0.5, ref_frame="base_link",
                      vel_scale=0.005, acc_scale=0.005,
                      avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0, dist_thresh=None):

        # get the initial force
        init_force = deepcopy(self.current_force[axis])
        self.init_force = init_force
        self.axis = axis
        self.force_thresh = force_thresh
        self.trigger_stop = True
        current_force = init_force
        change_force = 0
        rospy.loginfo("initial_force = {}".format(init_force))

        # Reset goal status
        self.goal_status = -1
        
        # Move fast at first.
        result = self.move_straight(
            poses, vel_scale=vel_scale, acc_scale=acc_scale, wait_execution=False, ref_frame=ref_frame)

        # Monitor the force until it reaches force_thresh.
        while change_force < force_thresh and self.goal_status != 3 and not rospy.is_shutdown():
            current_force = self.current_force[axis]
            change_force = fabs(current_force - init_force)
            # rospy.loginfo("change in force = {}".format(change_force))

        # rospy.loginfo("Initial Time")
        if self.goal_status != 3:
            self.move_group.stop()
        # self.trigger_stop = False
        # wait a bit for goal status to be updated in case of preempted.
        rospy.sleep(0.1)
        
        # goal_status = -1 means it wasn't updated.
        # goal_status = 2 means "Preempted" i.e. We stopped in the middle of motion.
        # goal_status = 3 means "Solution was found and executed." i.e. Motion completed successfully.
        return self.goal_status

    
    def hole_search(self, tf_services, init_z, pose_array, z_thresh=0.003, z_upper=0.002, z_lower=0.05,
                    force_thresh=3, axis='xy', ref_frame="base_link", vel_scale=1, acc_scale=1):
        moving_frame = pose_array.header.frame_id
        current_pose = tf_services.lookup_transform(ref_frame, moving_frame)
        pose_idx = 0
        array_sz = len(pose_array.poses)
        print("current_z = ", current_pose.position.z)
        print("init_z = ", init_z)
        while fabs(current_pose.position.z - init_z) <= z_thresh and pose_idx < array_sz:
            
            # move above next spiral position
            spiral_array = PoseArray()
            spiral_array.poses.append(deepcopy(pose_array.poses[pose_idx]))
            spiral_array.poses[0].position.z += z_upper
            self.move_straight(spiral_array, vel_scale=vel_scale, acc_scale=acc_scale, ref_frame=ref_frame)
            
            print("moved up")
            
            # move to touch at next spiral position
            current_pose = tf_services.lookup_transform(
                ref_frame, moving_frame)
            current_pose.position.z += z_lower
            ethernet_array = PoseArray()
            ethernet_array.poses.append(current_pose)
            self.move_to_touch(
                ethernet_array, axis=axis, force_thresh=force_thresh, ref_frame=ref_frame)
            
            print("touched down")


            current_pose = tf_services.lookup_transform(
                ref_frame, moving_frame)
            pose_idx += 1

        if pose_idx >= array_sz:
            print("couldn't insert the Ethernet")
            return
    
    def shift_pose_by(self, tf_services, ref_frame, moving_frame, new_frame="new_frame", trans=(0, 0, 0), rot=(0, 0, 0)):
        pose_array = PoseArray()
        tf_services.create_frame(ref_frame, moving_frame, new_frame)
        rospy.sleep(1)
        current_pose = Pose()
        current_pose.position.x = trans[0]
        current_pose.position.y = trans[1]
        current_pose.position.z = trans[2]

        angles = list(rot)
        q = quaternion_from_euler(angles[0], angles[1], angles[2])
        current_pose.orientation.x = q[0]
        current_pose.orientation.y = q[1]
        current_pose.orientation.z = q[2]
        current_pose.orientation.w = q[3]

        pose_array.poses.append(current_pose)
        pose_array.header.frame_id = new_frame
        return pose_array

    def change_tool_status(self, signal, status=0):
        self.tool.wait_for_service()
        tool_status = SetIOSignalRequest()
        tool_status.signal = signal
        tool_status.value = str(status)
        response = self.tool(tool_status)
        return response

if __name__ == '__main__':
    rospy.init_node('robot_helpers')
    ms = MotionServices(tool_group='uji_ur5', wrench_topic="/wrench/filtered", transform_force=True, from_frame="tool0_controller", to_frame="gripper_tip_link")
    rospy.spin()
