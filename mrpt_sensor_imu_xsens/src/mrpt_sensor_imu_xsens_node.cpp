/* +------------------------------------------------------------------------+
   |                             mrpt_sensors                               |
   |                                                                        |
   | Copyright (c) 2017-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_sensors                      |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/bits_math.h>  // square()
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/ros2bridge/image.h>

#include <rclcpp/rclcpp.hpp>

#include "mrpt_sensorlib/mrpt_sensorlib.h"

const char* node_name = "mrpt_sensor_imu_xsens";
const char* sensorConfig = R""""(
# Configuration INI file block for mrpt-hwdrivers sensor.
# Read more online:
# https://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/
#
[SENSOR]
driver			= CIMUXSens_MT4
process_rate	= ${PROCESS_RATE} // Hz

sensorLabel		= ${SENSOR_LABEL}

portname_LIN	= ${PORT_NAME}
baudRate		= 115200     // 4098

#deviceId     = xxxxx    // Device ID to open, or first one if empty.
#logFile      = xsens.log     // If provided, will enable XSens SDK's own log

sampleFreq    = 100    // Sensor sample rate

pose_x       = ${SENSOR_POSE_X}      // (meters)
pose_y       = ${SENSOR_POSE_Y}
pose_z       = ${SENSOR_POSE_Z}
pose_yaw     = ${SENSOR_POSE_YAW}      // (deg)
pose_pitch   = ${SENSOR_POSE_PITCH}
pose_z       = ${SENSOR_POSE_ROLL}

)"""";

int main(int argc, char** argv)
{
    try
    {
        // Init ROS:
        rclcpp::init(argc, argv);

        auto node =
            std::make_shared<mrpt_sensors::GenericSensorNode>(node_name);

        node->init(
            sensorConfig,
            {
                {"process_rate", "PROCESS_RATE", "80", false},
                {"sensor_label", "SENSOR_LABEL", "stereo", false},
                {"port_name", "PORT_NAME", "", false},

                {"sensor_pose_x", "SENSOR_POSE_X", "0", false},
                {"sensor_pose_y", "SENSOR_POSE_Y", "0", false},
                {"sensor_pose_z", "SENSOR_POSE_Z", "0", false},
                {"sensor_pose_yaw", "SENSOR_POSE_YAW", "0", false},
                {"sensor_pose_pitch", "SENSOR_POSE_PITCH", "0", false},
                {"sensor_pose_roll", "SENSOR_POSE_ROLL", "0", false},
            },
            "SENSOR");

        node->run();

        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger(""),
            "Exception in " << node_name << " main(): " << e.what());
        return 1;
    }
}
