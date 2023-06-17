#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "integrated_navigation_driver/NMEA_GPFPD.h"
#include "integrated_navigation_driver/NMEA_GTIMU.h"
#include "integrated_navigation_driver/NMEA_GPCHC.h"
#include "integrated_navigation_driver/NMEA_NVSTD.h"
#include "utility.h"

#include <cstring>
#include <vector>
#include <sstream>

class Imu_Generator
{

public:

    Imu_Generator()
    {
        ros::NodeHandle nh_local("~");
        nh_local.getParam("Imu_generate_from", generate_from);
        nh_local.getParam("use_gnss_time", use_gnss_time);
        nh_.getParam("time_set/leap_second", leap_second);

        imu_pub = nh_.advertise<sensor_msgs::Imu>("/integrated_nav/Imu", 10);
        ROS_WARN("If no Imu message, maybe you source %s message is empty", generate_from.c_str());

        if (generate_from == "GPCHC")
        {
            nmea_sub = nh_.subscribe("/nmea/gpchc", 1, &Imu_Generator::parseGPCHCmsgCallback, this);
        } else if (generate_from == "GPFPD-GTIMU")
        {
            gpfpd_sub.subscribe(nh_, "/nmea/gpfpd", 1);
            gtimu_sub.subscribe(nh_, "/nmea/gtimu", 1);
            fpd_imu_sync.reset(new GPFPD_GTIMU_Sync (GPFPD_GTIMU_Policy (10), gpfpd_sub, gtimu_sub));
            fpd_imu_sync->registerCallback(boost::bind(&Imu_Generator::parseGPFPDmsgGTIMUmsgCallback, this, _1, _2));
        } else
        {
            ROS_ERROR("Uncorrect Imu source. It should be 'GPCHC' / 'GPFPD-GTIMU'.");
            return;
        }


    }

    void parseGPCHCmsgCallback(const integrated_navigation_driver::NMEA_GPCHC::ConstPtr msg_in)
    {
        auto msg_out = new sensor_msgs::Imu();

        msg_out->header.frame_id = "imu_link";
        uint64_t nanosecond = 0;
        if (use_gnss_time)
        {
//     conside GNSS UTC time from 1980, we need to mines leap second from 1970 to 1980
            nanosecond = GNSSUTCWeekAndTime2Nanocecond(msg_in->gnss_week, msg_in->gnss_time, leap_second - 9);
//            nanosecond = GNSSUTCWeekAndTime2Nanocecond(msg_in->gnss_week, msg_in->gnss_time, 0);
        } else
        {
            nanosecond = msg_in->header.stamp.toNSec();
        }

        auto yaw = DEG2RAD(msg_in->heading);
        auto pitch = DEG2RAD(msg_in->pitch);
        auto roll = DEG2RAD(msg_in->roll);
        auto angular_velocity_x = DEG2RAD(msg_in->gyroscope_x);
        auto angular_velocity_y = DEG2RAD(msg_in->gyroscope_y);
        auto angular_velocity_z = DEG2RAD(msg_in->gyroscope_z);
        auto linear_acceleration_x = msg_in->acceleration_x * 9.8;
        auto linear_acceleration_y = msg_in->acceleration_y * 9.8;
        auto linear_acceleration_z = msg_in->acceleration_z * 9.8;
        fillBasicImumsg(*msg_out, nanosecond, yaw, pitch, roll, angular_velocity_x, angular_velocity_y, angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);

        pubImu(msg_out);

        delete(msg_out);
    }

    void parseGPFPDmsgGTIMUmsgCallback(const integrated_navigation_driver::NMEA_GPFPD::ConstPtr& msgGPFPD_in, const integrated_navigation_driver::NMEA_GTIMU::ConstPtr& msgGTIMU_in)
    {
        auto msg_out = new sensor_msgs::Imu();

        msg_out->header.frame_id = "imu";
        uint64_t nanosecond = 0;
        if (use_gnss_time)
        {
//     conside GNSS UTC time from 1980, we need to mines leap second from 1970 to 1980
            nanosecond = GNSSUTCWeekAndTime2Nanocecond(msgGPFPD_in->gnss_week, msgGPFPD_in->gnss_time, leap_second - 9);
        } else
        {
            nanosecond = msgGPFPD_in->header.stamp.toNSec();
        }

        auto yaw = DEG2RAD(msgGPFPD_in->heading);
        auto pitch = DEG2RAD(msgGPFPD_in->pitch);
        auto roll = DEG2RAD(msgGPFPD_in->roll);
        auto angular_velocity_x = DEG2RAD(msgGTIMU_in->gyroscope_x);
        auto angular_velocity_y = DEG2RAD(msgGTIMU_in->gyroscope_y);
        auto angular_velocity_z = DEG2RAD(msgGTIMU_in->gyroscope_z);
        auto linear_acceleration_x = msgGTIMU_in->acceleration_x * 9.8;
        auto linear_acceleration_y = msgGTIMU_in->acceleration_y * 9.8;
        auto linear_acceleration_z = msgGTIMU_in->acceleration_z * 9.8;

        fillBasicImumsg(*msg_out, nanosecond, yaw, pitch, roll, angular_velocity_x, angular_velocity_y, angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);

        pubImu(msg_out);

        delete(msg_out);
    }


    void pubImu(const sensor_msgs::Imu * msg)
    {
        imu_pub.publish(*msg);
    }

    static void fillBasicImumsg(sensor_msgs::Imu & msg, const uint64_t nanosecond, double yaw, double pitch, double roll, double angular_velocity_x, double angular_velocity_y, double angular_velocity_z, double linear_acceleration_x, double linear_acceleration_y, double linear_acceleration_z)
    {
        msg.header.frame_id = "imu";

        ros::Time tROSTime;
        msg.header.stamp = tROSTime.fromNSec(nanosecond);

        tf2::Quaternion orientation_quat;
        orientation_quat.setRPY(roll, pitch, yaw);
        orientation_quat.normalize();

        msg.orientation = tf2::toMsg(orientation_quat);

        msg.angular_velocity.x = angular_velocity_x;
        msg.angular_velocity.y = angular_velocity_y;
        msg.angular_velocity.z = angular_velocity_z;
        msg.angular_velocity_covariance = {0};

        msg.linear_acceleration.x = linear_acceleration_x;
        msg.linear_acceleration.y = linear_acceleration_y;
        msg.linear_acceleration.z = linear_acceleration_z;
        msg.linear_acceleration_covariance={0};
    }


private:

    ros::NodeHandle nh_;
    ros::Publisher imu_pub;
    ros::Subscriber nmea_sub;

    std::string generate_from;
    bool use_gnss_time = false;
    int leap_second = 0;

    message_filters::Subscriber<integrated_navigation_driver::NMEA_GPFPD> gpfpd_sub;
    message_filters::Subscriber<integrated_navigation_driver::NMEA_GTIMU> gtimu_sub;
    typedef message_filters::sync_policies::ApproximateTime<integrated_navigation_driver::NMEA_GPFPD, integrated_navigation_driver::NMEA_GTIMU> GPFPD_GTIMU_Policy;
    typedef message_filters::Synchronizer<GPFPD_GTIMU_Policy> GPFPD_GTIMU_Sync;
    boost::shared_ptr<GPFPD_GTIMU_Sync> fpd_imu_sync;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_generator_node");

    Imu_Generator generator;
    ros::spin();
    return 0;
}