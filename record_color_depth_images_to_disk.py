#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils.lib_ros_rgbd_pub_and_sub import ColorImageSubscriber, DepthImageSubscriber

import numpy as np
import rospy
import numpy as np
import sys
import os
import cv2
import datetime
import argparse

ROOT = os.path.join(os.path.dirname(__file__))
KEY_RECORD_SINGLE_IMAGE = 'a'
KEY_START_RECORDING_VIDEO = 's'
KEY_STOP_RECORDING_VIDEO = 'd'
KEY_QUIT_PROGRAM = 'q'
DST_FOLDER = ROOT + "output/"
COLOR_NAME = "color_"
DEPTH_NAME = "depth_"


def parse_command_line_arguments():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Subscribe color/depth images and save them to disk based on key press.")

    parser.add_argument("-c", "--color_topic", required=True, type=str,
                        help="")
    parser.add_argument("-d", "--depth_topic", required=True, type=str,
                        help="")
    parser.add_argument("-f", "--dst_folder", required=False, type=str,
                        default=DST_FOLDER,
                        help="Dst folder to save images.")

    args = parser.parse_args(rospy.myargv()[1:])
    return args


def get_time():
    s = str(datetime.datetime.now())[5:].replace(
        ' ', '-').replace(":", '-').replace('.', '-')[:-3]
    return s  # day, hour, seconds: 02-26-15-51-12-556


def int2str(num, blank):
    return ("{:0"+str(blank)+"d}").format(num)


def write_image(dst_folder, color, depth, index, blank=5, img_suffix=".png"):
    suffix = int2str(index, blank) + img_suffix
    filename_color = dst_folder + "/" + COLOR_NAME + suffix
    filename_depth = dst_folder + "/" + DEPTH_NAME + suffix
    cv2.imwrite(filename_color, color)
    cv2.imwrite(filename_depth, depth)
    print("Write color image to: " + filename_color)
    print("Write depth image to: " + filename_depth)


class KeyProcessorAndImageRecorder(object):

    def __init__(self, dst_folder):

        # Folder for saving single image.
        self._dst_folder = dst_folder + "/"
        self._cnt_single_img = 0
        if not os.path.exists(self._dst_folder):
            os.makedirs(self._dst_folder)

        # Folder for saving video frames.
        self._dst_subfolder = None
        self._is_recording = False
        self._cnt_img_in_video_clip = 0

    def check_key_and_save_image(self, key, color, depth):

        key = chr(key).lower()

        if key == KEY_RECORD_SINGLE_IMAGE:
            self._save_a_single_image(color, depth)

        if self._is_recording or key == KEY_START_RECORDING_VIDEO:
            self._save_image_to_the_recording_folder(key, color, depth)

    def _save_a_single_image(self, color, depth):
        self._cnt_single_img += 1
        write_image(self._dst_folder, color, depth,
                    self._cnt_single_img)

    def _save_image_to_the_recording_folder(self, key, color, depth):

        # Set recording state to `True` and create dst folder.
        if key == KEY_START_RECORDING_VIDEO and self._is_recording == False:
            self._is_recording = True
            self._cnt_img_in_video_clip = 0
            self._dst_subfolder = self._dst_folder + "_" + get_time() + "/"

            if not os.path.exists(self._dst_subfolder):
                os.makedirs(self._dst_subfolder)

            print("\n==============================================")
            print("Start recording video ...")

        # Set recording state to `False`.
        if key == KEY_STOP_RECORDING_VIDEO and self._is_recording == True:
            self._is_recording = False
            print("Stop recording video ...")
            print("==============================================\n")

        # Write color/depth image to `self._dst_subfolder` if recording state is `True`.
        if self._is_recording:
            self._cnt_img_in_video_clip += 1
            write_image(self._dst_subfolder, color, depth,
                        self._cnt_img_in_video_clip)


def main(args):

    # -- Set subscribers.
    sub_color = ColorImageSubscriber(args.color_topic)
    sub_depth = DepthImageSubscriber(args.depth_topic)
    key_proc = KeyProcessorAndImageRecorder(args.dst_folder)

    # -- Loop, subscribe images, process key events, and save images.
    while not rospy.is_shutdown():

        if sub_color.has_image() and sub_depth.has_image():
            color = sub_color.get_image()
            depth = sub_depth.get_image()
            # print(color.shape, depth.shape)

            img_disp = np.hstack((
                color,
                cv2.cvtColor((depth/10.0).astype(np.uint8), cv2.COLOR_GRAY2BGR)))
            cv2.imshow("Color/Depth image", img_disp)

            key = np.uint8(cv2.waitKey(10))
            if chr(key).lower() == KEY_QUIT_PROGRAM:
                break 

            key_proc.check_key_and_save_image(key, color, depth)

        rospy.sleep(0.005)


if __name__ == '__main__':
    node_name = "record_images_to_disk"
    rospy.init_node(node_name)
    args = parse_command_line_arguments()
    main(args)
    rospy.logwarn("Node `{}` stops.".format(node_name))
