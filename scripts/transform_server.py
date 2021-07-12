#!/usr/bin/env python

import rospy
from robot_helpers.srv import TransformPoses, TransformPosesResponse
from robot_helpers.robot_helpers import TransformServices


class TransformServer():
    def __init__(self):
        self.tf_services = TransformServices()
        transform_poses_server = rospy.Service(
            "transform_poses", TransformPoses, self.handle_transfrom_poses)
        
    def handle_transform_poses(self, req):
        target_frame = req.target_frame
        source_frame = req.ref_frame
        poses_to_transform = req.poses_to_transform
        transformed_poses = self.tf_services.transform_poses(
            target_frame, source_frame, poses_to_transform)
        return TransformPosesResponse(transformed_poses)


if __name__ == "__main__":
    rospy.init_node("transform_server")
    transform_server = TransformServer()
    rospy.spin()
