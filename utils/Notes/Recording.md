ros2 bag record \
    /camera/image_raw \
    /camera/camera_info \
    -o camera_test
ros2 bag play camera_test

