mkdir -p test_bins

# build motor test
g++ ./nodes/test/test_motor_driver.cpp ./nodes/motor_controller/src/motor_driver.cpp -Inodes/motor_controller/include -lwiringPi -o ./test_bins/motor_test

# build camera test
g++ ./nodes/test/test_opencl.cpp -o ./test_bins/test_opencl `pkg-config --cflags --libs opencv4`
g++ ./nodes/test/test_opengl.cpp -o ./test_bins/test_opengl `pkg-config --cflags --libs opencv4`
g++ ./nodes/test/test_gstreamer.cpp -o ./test_bins/test_gstreamer `pkg-config --cflags --libs opencv4`
g++ ./nodes/test/test_dual_camera.cpp -o ./test_bins/test_dual_camera `pkg-config --cflags --libs opencv4`
