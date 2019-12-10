# Press key to record color/depth images from ROS topics

Keys:
* `a`: Save a single pair of color/depth images to folder.
* `s`: Start continuous recording. (Continuously saving color/depth images to folder.)
* `d`: Stop continuous recording.

Example of usage:
```
rosrun ros_record_rgbd_images record_color_depth_images_to_disk.py \
    --depth_topic camera/aligned_depth_to_color/image_raw \
    --color_topic camera/color/image_raw \
    --dst_folder output \
    --max_frame_rate 10.0 \
    --save_to_separate_folder true
```

Test with local data:
```
bash run_unit_test.py
```

Test on Realsense:
```
roslaunch ros_record_rgbd_images run_realsense.launch 
roslaunch ros_record_rgbd_images run_record_images.launch 
```