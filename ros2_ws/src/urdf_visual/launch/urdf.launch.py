import os
from sys import executable

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():

    #Lidar#
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB1')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('urdf_visual'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    rviz_file_name = 'new_view_slam.rviz'
    rviz_config = os.path.join(get_package_share_directory('urdf_visual'),'config', rviz_file_name)

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Teleop Launch #
    teleop_path = os.path.join(get_package_share_directory('teleop_twist_joy'))

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                teleop_path + '/launch/teleop-launch.py'))
    
    # Launch inverse kinematics #
    kin_path = os.path.join(get_package_share_directory('jetson_sub_node'))

    kin_node = Node(
        package='jetson_sub_node',
        executable='inv_kin.py'
    )

    # Launch Kalman Filter #
    ekf_path = os.path.join(get_package_share_directory('robot_localization'))

    ekf_launch = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           ekf_path + '/launch/ekf.launch.py'
       )
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),


        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_node',
        #     name='rplidar_node',
        #     parameters=[{'channel_type':channel_type,
        #                  'serial_port': serial_port,
        #                  'serial_baudrate': serial_baudrate,
        #                  'frame_id': frame_id,
        #                  'inverted': inverted,
        #                  'angle_compensate': angle_compensate,
        #                   'scan_mode': scan_mode}],
        #     output='screen'),

        teleop_launch, 

        kin_node,

       Node(
           package='jetson_sub_node',
           executable='wheel_odom_publish.py'
       ),

        node_robot_state_publisher,

        #Node(
        ##    package='joint_state_publisher',
        #    executable='joint_state_publisher',
        #    name='joint_state_publisher',
        #),

       ekf_launch, 

         Node(
             package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz_config],
         ),
     
    ])
