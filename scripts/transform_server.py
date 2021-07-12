#!/usr/bin/env python
import rospy
from robot_helpers.srv import TransformPoses, TransformPosesResponse,\
    CreateFrame, CreateFrameResponse, LookupTransform, LookupTransformResponse,\
        CreateFrameAtPose, CreateFrameAtPoseResponse
from robot_helpers.robot_helpers import TransformServices
from std_msgs.msg import String


class TransformServer(TransformServices):
    def __init__(self):
        TransformServices.__init__(self)
        rospy.Service("transform_poses", TransformPoses, self.handle_transform_poses)
        rospy.Service("create_frame", CreateFrame, self.handle_create_frame)
        rospy.Service("create_frame_at_pose", CreateFrameAtPose, self.handle_create_frame_at_pose)
        rospy.Service("lookup_transform", LookupTransform, self.handle_lookup_transform)

    def handle_transform_poses(self, req):
        transformed_poses = self.transform_poses(
            req.target_frame.data, req.ref_frame.data, req.poses_to_transform)
        return TransformPosesResponse(transformed_poses)
    
    def handle_create_frame(self, req):
        _ = self.create_frame(req.ref_frame.data, req.moving_frame.data, req.new_frame.data)
        return CreateFrameResponse(String())

    def handle_create_frame_at_pose(self, req):
        _ = self.create_frame_at_pose(req.frame_pose, req.ref_frame.data, req.new_frame.data)
        return CreateFrameAtPoseResponse(String())

    def handle_lookup_transform(self, req):
        pose = self.lookup_transform(req.target_frame.data, req.source_frame.data)
        return LookupTransformResponse(pose) 

if __name__ == "__main__":
    rospy.init_node("transform_server")
    transform_server = TransformServer()
    rospy.spin()
