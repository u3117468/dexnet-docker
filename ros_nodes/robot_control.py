# -*- coding: utf-8 -*-
"""
Copyright ©2017. The Regents of the University of California (Regents). All Rights Reserved.
Permission to use, copy, modify, and distribute this software and its documentation for educational,
research, and not-for-profit purposes, without fee and without a signed licensing agreement, is
hereby granted, provided that the above copyright notice, this paragraph and the following two
paragraphs appear in all copies, modifications, and distributions. Contact The Office of Technology
Licensing, UC Berkeley, 2150 Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-
7201, otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial licensing opportunities.

IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL,
INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF
THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS BEEN
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED
HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
"""
#!/usr/bin/env python
"""
Example node for planning grasps from point clouds using the gqcnn module and
executing the grasps with an ABB YuMi.
Additionally depends on the dex-net, meshpy, and yumipy modules.

This file is intended as an example, not as code that will run with standard installation.

Author: Vishal Satish
"""
import rospy
import logging
import numpy as np
import signal
import time

from autolab_core import RigidTransform
from autolab_core import YamlConfig
from dexnet.grasping import RobotGripper
import perception as perception
from perception import RgbdDetectorFactory, RgbdSensorFactory
from perception import VirtualSensor
from gqcnn import Visualizer as vis

from gqcnn.msg import GQCNNGrasp, BoundingBox
from sensor_msgs.msg import Image, CameraInfo
from gqcnn.srv import GQCNNGraspPlanner

from cv_bridge import CvBridge, CvBridgeError


def process_GQCNNGrasp(grasp, robot, left_arm, right_arm, subscriber, home_pose, config):
    """ Processes a ROS GQCNNGrasp message and executes the resulting grasp on the ABB Yumi """

    grasp = grasp.grasp
    rospy.loginfo('Processing Grasp')

    rotation_quaternion = np.asarray([grasp.pose.orientation.w, grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z])
    translation = np.asarray([grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z])
    T_grasp_world = RigidTransform(rotation_quaternion, translation, 'grasp', T_camera_world.from_frame)

    T_gripper_world = T_camera_world * T_grasp_world * gripper.T_grasp_gripper
    rospy.loginfo(T_gripper_world);





def run_experiment():
    """ Run the experiment """

    # create ROS CVBridge
    cv_bridge = CvBridge()

    # wait for Grasp Planning Service and create Service Proxy
    rospy.wait_for_service('plan_gqcnn_grasp')
    plan_grasp = rospy.ServiceProxy('plan_gqcnn_grasp', GQCNNGraspPlanner)

    # get camera intrinsics
    camera_intrinsics = sensor.ir_intrinsics

    # setup experiment logger

    logging.info('Saving experiment to %s' )
    object_keys = config['test_object_keys']
    trial_number = 1
    re_try = False

    logging.info('Beginning experiment')

    while True:
    #    rospy.loginfo('Please place object: ' + obj + ' on the workspace.')
        # start the next trial
        rospy.loginfo('Trial %d' % (trial_number))

        # get the images from the sensor
        color_image, depth_image, _ = sensor.frames()

        # log some trial info

        # inpaint to remove holes
        inpainted_color_image = color_image.inpaint(rescale_factor=config['inpaint_rescale_factor'])
        inpainted_depth_image = depth_image.inpaint(rescale_factor=config['inpaint_rescale_factor'])

        detector = RgbdDetectorFactory.detector('point_cloud_box')
        detection = detector.detect(inpainted_color_image, inpainted_depth_image, detector_cfg, camera_intrinsics, T_camera_world, vis_foreground=False, vis_segmentation=False
            )[0]

        boundingBox = BoundingBox()
        boundingBox.minY = detection.bounding_box.min_pt[0]
        boundingBox.minX = detection.bounding_box.min_pt[1]
        boundingBox.maxY = detection.bounding_box.max_pt[0]
        boundingBox.maxX = detection.bounding_box.max_pt[1]


if __name__ == '__main__':

    print('test')
    # initialize the ROS node
    rospy.init_node('Yumi_Control_Node')

    config = YamlConfig('./control_node.yaml')

    #rospy.loginfo('Loading Gripper')
    #gripper = RobotGripper.load('yumi_metal_spline')

    rospy.loginfo('Loading T_camera_world')
#    T_camera_world = RigidTransform.load('/home/autolab/Public/alan/calib/primesense_overhead/primesense_overhead_to_world.tf')

    detector_cfg = config['detector']

    # create rgbd sensor
    rospy.loginfo('Creating RGBD Sensor')
    sensor_cfg = config['sensor_cfg']
    sensor_type = sensor_cfg['type']
    sensor = RgbdSensorFactory.sensor(sensor_type, sensor_cfg)
    #sensor = VirtualSensor('/apps/test/ros_nodes/images/')
    sensor.start()
    rospy.loginfo('Sensor Running')

    # setup safe termination

    # run experiment
    run_experiment()

    rospy.spin()
