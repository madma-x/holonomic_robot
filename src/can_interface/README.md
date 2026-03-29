
// # Build
// colcon build --packages-select can_interface

// # Terminal 1: Setup CAN and run tester
// sudo ip link set can0 up type can bitrate 1000000
// ros2 run can_interface can_test_node

// # Terminal 2: Test motors
// ros2 service call /can_test/enable_motors std_srvs/srv/Trigger
// ros2 service call /can_test/spin_motors std_srvs/srv/Trigger
// ros2 service call /can_test/stop_motors std_srvs/srv/Trigger

// # Terminal 3: Test with cmd_vel (holonomic movement)
// ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
// # Enable motors
// ros2 service call /can_test/enable_motors std_srvs/srv/Trigger

// # Spin and gradual stop
// ros2 service call /can_test/spin_motors std_srvs/srv/Trigger
// # Manual gradual stop
// ros2 service call /can_test/stop_motors std_srvs/srv/Trigger

// # Emergency immediate stop (use with caution!)
// ros2 service call /can_test/emergency_stop std_srvs/srv/Trigger