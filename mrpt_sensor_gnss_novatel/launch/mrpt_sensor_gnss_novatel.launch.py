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
            'novatel_main_serial_port',
            default_value='',
            description='Main Novatel comms port'
        ),
        DeclareLaunchArgument(
            'novatel_ntrip_serial_port',
            default_value='',
            description='Novatel comms port that expects NTRIP messages'
        ),
        DeclareLaunchArgument(
            'serial_baud_rate',
            default_value='"4800"',
            description='Serial port baud rate (typ: 4800, 9600, etc.)'
        ),

        DeclareLaunchArgument(
            'raw_dump_file',
            default_value='""',
            description='If not empty, raw GNSS data will be dumped to this file.'
        ),

        DeclareLaunchArgument(
            'novatel_imu_orientation',
            default_value='"6"',
            description='See Novatel docs for SETIMUORIENTATION.'
        ),
        DeclareLaunchArgument(
            'novatel_veh_body_rotation',
            default_value='"0.000000 0.000000 90.000000 0.000000 0.000000 0.000000"',
            description='See Novatel docs for VEHICLEBODYROTATION.'
        ),
        DeclareLaunchArgument(
            'novatel_imu_to_ant_offset',
            default_value='"-0.28 -0.08 -0.01 0.000000 0.000000 0.000000"',
            description='See Novatel docs for SETIMUTOANTOFFSET.'
        ),
        DeclareLaunchArgument(
            'novatel_ins_offset',
            default_value='"0.000000 0.000000 0.000000"',
            description='See Novatel docs for SETINSOFFSET.'
        ),
        DeclareLaunchArgument(
            'novatel_init_azimuth',
            default_value='"0.000000 25.000000"',
            description='See Novatel docs for SETINITAZIMUTH.'
        ),
        DeclareLaunchArgument(
            'ntrip_server',
            default_value='"www.euref-ip.net"',
            description='DNS or IP of the NTRIP server.'
        ),
        DeclareLaunchArgument(
            'ntrip_port',
            default_value='"2101"',
            description='TCP port for connecting to the NTRIP server.'
        ),
        DeclareLaunchArgument(
            'ntrip_mount_point',
            default_value='"ALME00ESP0"',
            description='Mount point to connect inside the NTRIP server.'
        ),
        DeclareLaunchArgument(
            'ntrip_user',
            default_value='""',
            description='NTRIP server username.'
        ),
        DeclareLaunchArgument(
            'ntrip_password',
            default_value='""',
            description='NTRIP server password.'
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
            package='mrpt_sensor_gnss_novatel',
            executable='mrpt_sensor_gnss_novatel_node',
            name='mrpt_sensor_gnss_novatel',
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
                {'novatel_main_serial_port': LaunchConfiguration(
                    'novatel_main_serial_port')},
                {'serial_baud_rate': LaunchConfiguration('serial_baud_rate')},
                {'novatel_ntrip_serial_port': LaunchConfiguration(
                    'novatel_ntrip_serial_port')},
                {'raw_dump_file': LaunchConfiguration('raw_dump_file')},
                {'novatel_imu_orientation': LaunchConfiguration(
                    'novatel_imu_orientation')},
                {'novatel_veh_body_rotation': LaunchConfiguration(
                    'novatel_veh_body_rotation')},
                {'novatel_imu_to_ant_offset': LaunchConfiguration(
                    'novatel_imu_to_ant_offset')},
                {'novatel_ins_offset': LaunchConfiguration(
                    'novatel_ins_offset')},
                {'novatel_init_azimuth': LaunchConfiguration(
                    'novatel_init_azimuth')},
                {'ntrip_server': LaunchConfiguration('ntrip_server')},
                {'ntrip_port': LaunchConfiguration('ntrip_port')},
                {'ntrip_mount_point': LaunchConfiguration(
                    'ntrip_mount_point')},
                {'ntrip_user': LaunchConfiguration('ntrip_user')},
                {'ntrip_password': LaunchConfiguration('ntrip_password')},

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
