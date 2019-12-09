# Press key to record color/depth images from ROS Topics

Keys:
* `a`: Save a single pair of color/depth images to folder.
* `s`: Start continuous recording. (Continuously saving color/depth images to folder.)
* `d`: Stop continuous recording.

Example of usage:
```
rosrun ros_record_rgbd_images record_color_depth_images_to_disk.py \
    --depth_topic test_data/depth \
    --color_topic test_data/color
    --dst_folder output
```

Unit test with local data:
```
bash run_unit_test.py
```

TODO: Add a test case of recording images directly from Realsense camera.