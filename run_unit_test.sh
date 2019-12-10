set -e

# 1. Publish color and depth images.

# Download color/depth images publisher.
ROOT=$(rospack find ros_record_rgbd_images)
cd ${ROOT}/.. # cd to catkin src/ folder.
if [ ! -d "ros_pub_and_sub_rgbd_and_cloud" ]; then
    repo="https://github.com/felixchenfy/ros_pub_and_sub_rgbd_and_cloud"
    echo "Installing rgbd images publisher: ${repo}"
    git clone ${repo}
    cd ros_pub_and_sub_rgbd_and_cloud
    chmod a+x pub_rgbd_and_cloud.py
fi
cd ${ROOT}

publish_images(){
    ROOT=$(rospack find ros_record_rgbd_images)
    rosrun ros_pub_and_sub_rgbd_and_cloud pub_rgbd_and_cloud.py \
        --base_dir $ROOT \
        --config_file $ROOT/config/pub_rgbd_config.yaml
    # Data folder path == `base_dir` + relative path in `config_file`
}

# 2. Start images recorder.
record_images(){
    ROOT=$(rospack find ros_record_rgbd_images)
    FILE="record_color_depth_images_to_disk.py"
    cd $ROOT
    chmod a+x $FILE 
    rosrun ros_record_rgbd_images $FILE \
        --depth_topic camera/aligned_depth_to_color/image_raw \
        --color_topic camera/color/image_raw \
        --dst_folder output \
        --max_frame_rate 10.0 \
        --save_to_separate_folder true
}


# 3. Run all the above scripts in parallel.
# https://stackoverflow.com/questions/3004811/how-do-you-run-multiple-programs-in-parallel-from-a-bash-script
# https://unix.stackexchange.com/questions/204480/run-multiple-commands-and-kill-them-as-one-in-bash
trap 'kill %1' SIGINT
record_images & publish_images 