# Importing necessary libraries
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
import os


def generate_launch_description():
    namespace = 'gnss'

    ld = LaunchDescription([
        # COMMON PARAMS TO ALL MRPT_SENSOR NODES:
        # --------------------------------------------
        # Declare an argument for the config file
        DeclareLaunchArgument(
            'process_rate',
            default_value='"50"',
            description='Rate (Hz) for the process() main sensor loop.'
        ),

        DeclareLaunchArgument(
            'out_rawlog_prefix',
            default_value='',
            description='If not empty, a .rawlog file will be created with all recorded data, apart of publishing it as ROS messages.'
        ),

        DeclareLaunchArgument(
            'publish_mrpt_obs_topic',
            default_value='',
            description='If not empty, mrpt_msgs/GenericObservation messages will be published to this topic name with the binary serialization of mrtp::obs::CObservation objects from the sensor.'
        ),

        DeclareLaunchArgument(
            'publish_topic',
            default_value='sensor',
            description='If not empty, messages of the appropriate type will be published to this topic for each sensor observation.'
        ),

        DeclareLaunchArgument(
            'sensor_frame_id',
            default_value='sensor',
            description='The sensor frame_id name. Used to populate msg header and to publish to /tf too.'
        ),

        DeclareLaunchArgument(
            'robot_frame_id',
            default_value='base_link',
            description='The robot frame_id name. Used to publish the sensor pose to /tf.'
        ),

        DeclareLaunchArgument(
            'sensor_label',
            default_value='sensor',
            description='The sensorLabel field of mrpt::obs::CObservation: a "name" for the sensor.'
        ),


        # PARAMS FOR THIS NODE:
        # --------------------------------------------
        DeclareLaunchArgument(
            'serial_port',
            default_value='',
            description='Serial port to open'
        ),

        DeclareLaunchArgument(
            'serial_baud_rate',
            default_value='"4800"',
            description='Serial port baud rate (typ: 4800, 9600, etc.)'
        ),

        DeclareLaunchArgument(
            'sensor_pose_x',
            default_value='"0.0"',
            description='Sensor pose coordinate on the vehicle frame.'
        ),
        DeclareLaunchArgument(
            'sensor_pose_y',
            default_value='"0.0"',
            description='Sensor pose coordinate on the vehicle frame.'
        ),
        DeclareLaunchArgument(
            'sensor_pose_z',
            default_value='"0.0"',
            description='Sensor pose coordinate on the vehicle frame.'
        ),

        DeclareLaunchArgument(
            "log_level",
            default_value=TextSubstitution(text=str("INFO")),
            description="Logging level"
        ),

        # Node to launch the mrpt_generic_sensor_node
        Node(
            package='mrpt_sensor_gnss_nmea',
            executable='mrpt_sensor_gnss_nmea_node',
            name='mrpt_sensor_gnss_nmea',
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            parameters=[
                # ------------------------------------------------
                # common params:
                # ------------------------------------------------
                {'process_rate': LaunchConfiguration('process_rate')},
                {'out_rawlog_prefix': LaunchConfiguration(
                    'out_rawlog_prefix')},
                {'publish_mrpt_obs_topic': LaunchConfiguration(
                    'publish_mrpt_obs_topic')},
                {'publish_topic': LaunchConfiguration('publish_topic')},
                {'sensor_frame_id': LaunchConfiguration('sensor_frame_id')},
                {'robot_frame_id': LaunchConfiguration('robot_frame_id')},
                {'sensor_label': LaunchConfiguration('sensor_label')},

                # ------------------------------------------------
                # node params:
                # ------------------------------------------------
                {'serial_port': LaunchConfiguration('serial_port')},
                {'serial_baud_rate': LaunchConfiguration('serial_baud_rate')},

                {'sensor_pose_x': LaunchConfiguration('sensor_pose_x')},
                {'sensor_pose_y': LaunchConfiguration('sensor_pose_y')},
                {'sensor_pose_z': LaunchConfiguration('sensor_pose_z')},
            ]
        )
    ])

    # Namespace to avoid clash launch argument names with the parent scope:
    return LaunchDescription([GroupAction(
        actions=[
            PushRosNamespace(
                # condition=IfCondition(use_namespace),
                namespace=namespace),
            ld
        ])])
