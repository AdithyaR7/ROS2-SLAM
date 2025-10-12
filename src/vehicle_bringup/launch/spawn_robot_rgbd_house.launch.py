import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('vehicle_bringup'),
        'urdfs',
        'urdf',
        # 'basic_robot.urdf'    # robot with lidar only
        'robot_rgbd.urdf'       # robot with lidar + rgb-d realsense camera
    )
    
    # Read URDF content
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()


    turtlebot3_world_path = os.path.join(
        '/opt/ros',
        os.environ.get('ROS_DISTRO', 'iron'),  # fallback to iron
        'share/turtlebot3_gazebo/worlds/turtlebot3_house.world'
    )

    return LaunchDescription([
        # Set Gazebo model path to include our custom models
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(get_package_share_directory('vehicle_bringup'), 'urdfs', 'models') +
            ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
        ),
        
        # Start Gazebo with turtlebot3_house world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'),
                '/launch/gazebo.launch.py'
            ]),
            ## Launch included default turtlebot3 house world - testing
            # launch_arguments={'world': turtlebot3_world_path}.items()
            
            ## Launch modified world
            launch_arguments={'world': os.path.join(
                get_package_share_directory('vehicle_bringup'),
                'urdfs',
                'worlds',
                'turtlebot3_closed_house_simple.world'
            )}.items()

            ## Spawn empty world for testing
            # launch_arguments={'world': '/opt/ros/iron/share/gazebo_ros/worlds/empty.world'}.items()
            
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # Spawn robot in Gazebo (assumes Gazebo is already running)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            # arguments=['-topic', '/robot_description', '-entity', 'my_robot', 
            #           '-x', '-2.0', '-y', '1.0', '-z', '0.1'],
            arguments=['-topic', '/robot_description', '-entity', 'autonomous_vehicle',
                      '-x', '-4.0', '-y', '1.0', '-z', '0.1'],
            parameters=[{'start_delay': 2.0}]  # Wait 2 seconds
        )
    ])