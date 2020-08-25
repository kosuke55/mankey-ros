#! /usr/bin/env python

import argparse
import os

import cv2
import rospy
from sensor_msgs.msg import RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

from mankey_ros.srv import *


def main(visualize):
    rospy.wait_for_service('detect_keypoints')
    detect_keypoint = rospy.ServiceProxy(
        'detect_keypoints', MankeyKeypointDetection)

    # Get the test data path
    project_path = os.path.join(os.path.dirname(__file__), os.path.pardir)
    project_path = os.path.abspath(project_path)
    test_data_path = os.path.join(project_path, 'test_data')
    cv_rbg_path = os.path.join(test_data_path, '000000_rgb.png')
    cv_depth_path = os.path.join(test_data_path, '000000_depth.png')

    # Read the image
    cv_rgb = cv2.imread(cv_rbg_path, cv2.IMREAD_COLOR)
    cv_depth = cv2.imread(cv_depth_path, cv2.IMREAD_ANYDEPTH)

    # The bounding box
    roi = RegionOfInterest()
    roi.x_offset = 261
    roi.y_offset = 194
    roi.width = 327 - 261
    roi.height = 260 - 194

    # Build the request
    request = MankeyKeypointDetectionRequest()
    bridge = CvBridge()
    request.rgb_image = bridge.cv2_to_imgmsg(cv_rgb, 'bgr8')
    request.depth_image = bridge.cv2_to_imgmsg(cv_depth)
    request.bounding_box = roi
    response = detect_keypoint(request)
    print response

    if visualize:
        import numpy as np
        import open3d as o3d
        import skrobot
        import trimesh

        color = o3d.geometry.Image(cv_rgb)
        depth = o3d.geometry.Image(cv_depth)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth)

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd, o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

        viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
        trimesh_pc = trimesh.PointCloud(
            np.asarray(
                pcd.points), np.asarray(
                pcd.colors))
        viewer.scene.add_geometry(trimesh_pc)
        viewer.show()

        for keypoint in response.keypoints_camera_frame:
            keypoint_sphere = skrobot.models.Sphere(0.01, color=[255, 0, 0])
            keypoint_sphere.newcoords(skrobot.coordinates.Coordinates(
                pos=[keypoint.x, keypoint.y, keypoint.z]))
            viewer.add(keypoint_sphere)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--visualize', '-v', type=int,
        default=0)
    args = parser.parse_args()
    visualize = args.visualize
    main(visualize)
