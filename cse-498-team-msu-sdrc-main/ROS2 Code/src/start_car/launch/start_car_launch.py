from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """ Function to launch all of the ROS nodes on the car """
    
    # Declare a new launch argument
    # Allows the user the set the 
    speed_max_arg = DeclareLaunchArgument(
        "speed_max",
        default_value="6000.0",
        description="Maximum motor speed"
    )

    return LaunchDescription([
        # Set the max speed
        speed_max_arg,
        # Start the IMU
        Node(
            package="imu_visualizer",
            executable="imu_publisher",
            name="imu_publisher",
            output="screen"
        ),
        # Start the LiDAR
        Node(
            package="lidar_visualizer",
            executable="lidar_publisher",
            name="lidar_publisher",
            output="screen"
        ),
        # Start the GNSS
        Node(
            package="gnss_data",
            executable="gnss_publisher",
            name="gnss_publisher",
            output="screen"
        ),
        # Start the controller subsciber. Pass the speed parameter
        Node(
            package="controller_receiver",
            executable="control_receive",
            name="control_receive",
            parameters=[
                {"speed_max": LaunchConfiguration("speed_max")}
            ],
            output="screen"
        ),
        # Start the Camera publisher
        Node(
            package="my_camera_sensor",
            executable="camera_publisher",
            name="camera_publisher",
            output="screen"
        ),
        # Start the IMU
        Node(
            package="vesc_imu",
            executable="imu_publisher",
            name="imu_publisher",
            output="screen"
        ),
        # Start the Loopback node
        Node(
            package="loopback_test",
            executable="loopback_test_car",
            name="loopback_test",
            output="screen"
        ),
        # Start the VESC. Pass the parameters
        Node(
            package="vesc_driver",
            executable="vesc_driver_node",
            name="vesc_driver_node",
            parameters=[
                {"port": "/dev/vesc_device"},
                {"baud": 115200},
                {"speed_max": LaunchConfiguration("speed_max")},
                {"servo_max": 1.0},
                {"servo_min": 0.0},
                {"brake_min": -20000.0},
                {"brake_max": 20000.0},
                {"enable_imu": True},
                {"imu_rate": 50.0}
            ],
            output="screen"
        ),
    ])
