#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from copy import deepcopy
from sensor_msgs.msg import CameraInfo
from tf2_geometry_msgs import PoseStamped
from gazebo_msgs.msg import ModelStates, ModelState
from humanoid_league_msgs.msg import PoseWithCertainty, PoseWithCertaintyArray

rospy.init_node("fake_vis_in_sim")

tf_buffer = tf2_ros.Buffer(rospy.Duration(30))
tf_listener = tf2_ros.TransformListener(tf_buffer)


ball_pose = None
goal_post_1_pose = None
goal_post_2_pose = None
cam_info = None
ball_pub = None
goal_posts_pub = None

def state_update(state_msg):
    global ball_pose
    global goal_post_1_pose
    global goal_post_2_pose
    global tf_buffer
    global cam_info
    global ball_pub
    global goal_posts_pub

    if not cam_info:
        return

    ball_indices = []
    for i, name in enumerate(state_msg.name):
        if name == "teensize_ball":
            ball_indices.append(i)


    post_index1 = 0
    post_index2 = 0
    for name in state_msg.name:
        if name == "teensize_goal":
            if post_index1 == 0:
                post_index1 = post_index2
            else:
                break
        post_index2 += 1

    ball_msgs = []
    for ball_index in ball_indices:
        ball_pose = state_msg.pose[ball_index]

        ball_pose_stamped = PoseStamped()
        ball_pose_stamped.header.stamp = rospy.Time.now()
        ball_pose_stamped.header.frame_id = "map"
        ball_pose_stamped.pose = ball_pose
        ball_in_camera_optical_frame = tf_buffer.transform(ball_pose_stamped, cam_info.header.frame_id, timeout=rospy.Duration(0.5))

        # we only have to compute if the ball is inside the image, if the ball is in front of the camera
        if ball_in_camera_optical_frame.pose.position.z >= 0:
            p = [ball_in_camera_optical_frame.pose.position.x, ball_in_camera_optical_frame.pose.position.y, ball_in_camera_optical_frame.pose.position.z]
            k = np.reshape(cam_info.K, (3,3))
            p_pixel = np.matmul(k, p)
            p_pixel = p_pixel * (1/p_pixel[2])

            # make sure that the transformed pixel is inside the resolution and positive.
            # also z has to be positive to make sure that the ball is in front of the camera and not in the back
            if 0 < p_pixel[0] <= cam_info.width and 0 < p_pixel[1] <= cam_info.height:
                ball_in_footprint_frame = tf_buffer.transform(
                    ball_in_camera_optical_frame,
                    "base_footprint",
                    timeout=rospy.Duration(0.5))

                ball_relative = PoseWithCertainty()
                ball_relative.pose.pose = ball_in_footprint_frame.pose.position
                ball_relative.confidence = 1.0
                ball_msgs.append(ball_relative)

            balls_relative_msg = PoseWithCertaintyArray()
            balls_relative_msg.header = ball_in_footprint_frame.header
            balls_relative_msg.poses = ball_msgs
            ball_pub.publish(balls_relative_msg)

    goal = PoseWithCertaintyArray()

    goal_post_1_pose = state_msg.pose[post_index1]
    goal_post_2_pose = state_msg.pose[post_index2]

    for gp in (goal_post_1_pose, goal_post_2_pose):
        goal_post_stamped = PoseWithCertainty()
        goal_post_stamped.pose.pose = gp

        left_post = deepcopy(goal_post_stamped)
        left_post.pose.pose.position.y += 1.35

        left_post_in_camera_optical_frame = tf_buffer.transform(left_post.pose, cam_info.header.frame_id, timeout=rospy.Duration(0.5))

        if left_post_in_camera_optical_frame.pose.position.z >= 0:
            p = [left_post_in_camera_optical_frame.pose.position.x, left_post_in_camera_optical_frame.pose.position.y, left_post_in_camera_optical_frame.pose.position.z]
            k = np.reshape(cam_info.K, (3, 3))
            p_pixel = np.matmul(k, p)
            p_pixel = p_pixel * (1 / p_pixel[2])

            if 0 < p_pixel[0] <= cam_info.width and 0 < p_pixel[1] <= cam_info.height:
                left_post = PoseWithCertainty()
                left_post.confidence = 1
                left_post.pose = tf_buffer.transform(left_post.pose, "base_footprint",
                                                    timeout=rospy.Duration(0.5)).pose.position
                goal.poses.append(left_post)

        right_post = goal_post_stamped
        right_post.pose.position.y -= 1.35
        right_post_in_camera_optical_frame = tf_buffer.transform(right_post, cam_info.header.frame_id, timeout=rospy.Duration(0.5))
        if right_post_in_camera_optical_frame.pose.position.z >= 0:
            p = [right_post_in_camera_optical_frame.pose.position.x, right_post_in_camera_optical_frame.pose.position.y, right_post_in_camera_optical_frame.pose.position.z]
            k = np.reshape(cam_info.K, (3, 3))
            p_pixel = np.matmul(k, p)
            p_pixel = p_pixel * (1 / p_pixel[2])

            if 0 < p_pixel[0] <= cam_info.width and 0 < p_pixel[1] <= cam_info.height:
                right_post = PoseWithCertainty()
                right_post.confidence = 1
                right_post.pose = tf_buffer.transform(right_post_in_camera_optical_frame, "base_footprint",
                                                    timeout=rospy.Duration(0.5)).pose.position
                goal.poses.append(right_post)

        if len(goal.poses) > 0:
            goal_posts_pub.publish(goal)


def cam_info_cb(msg):
    global cam_info
    cam_info = msg


if __name__ == "__main__":
    # wait for transforms to become available
    tf_buffer.can_transform("base_footprint", "camera_optical_frame", rospy.Time(0), timeout=rospy.Duration(30))
    tf_buffer.can_transform("map", "camera_optical_frame", rospy.Time(0), timeout=rospy.Duration(30))

    balls_pub = rospy.Publisher("balls_relative", PoseWithCertaintyArray, queue_size=1)
    goal_posts_pub = rospy.Publisher("goal_posts_in_image", PoseWithCertaintyArray, queue_size=1)
    model_subscriber = rospy.Subscriber("gazebo/model_states", ModelStates, state_update, tcp_nodelay=True, queue_size=1)
    cam_info_sub = rospy.Subscriber("camera_info", CameraInfo, cam_info_cb)

    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn(
            "We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
        except rospy.exceptions.ROSInterruptException:
            exit()
