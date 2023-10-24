#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "integrated_navigation_reader/NMEA_GPFPD.h"
#include "integrated_navigation_reader/NMEA_GTIMU.h"
#include "integrated_navigation_reader/NMEA_GPCHC.h"
#include "integrated_navigation_reader/NMEA_NVSTD.h"
#include "integrated_navigation_reader/SPANLog_INSPVAXB.h"
#include "integrated_navigation_reader/SPANLog_RAWIMUB.h"
#include "integrated_navigation_reader/Log_DUALANTENNAHEADINGB.h"
#include "integrated_navigation_reader/SPANLog_CORRIMUDATAB.h"
#include "utility.h"

#include <cstring>
#include <vector>
#include <sstream>

int leap_second = 0;
bool use_gnss_time = false;

double gravity = 9.806;
std::string frame_id;

void fillBasicImumsg(sensor_msgs::Imu &msg, const uint64_t nanosecond, double yaw, double pitch, double roll,
                     double angular_velocity_x, double angular_velocity_y, double angular_velocity_z,
                     double linear_acceleration_x, double linear_acceleration_y, double linear_acceleration_z) {
    msg.header.frame_id = frame_id;

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
    msg.linear_acceleration_covariance = {0};
}

class Imu_Generator {
public:

    Imu_Generator(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : nh_(nh), nh_local(private_nh) {
        nh_local.getParam("Imu_generate_from", generate_from);
        nh_local.getParam("frame_id", frame_id);

        imu_pub = nh_.advertise<sensor_msgs::Imu>("/integrated_nav/Imu", 10);
        timereference_pub = nh_.advertise<sensor_msgs::TimeReference>("/integrated_nav/IMU_Time", 10);

        ROS_WARN("If no Imu message, maybe you source %s message is empty", generate_from.c_str());

        if (generate_from == "GPCHC") {
            nmea_sub = nh_.subscribe("/nmea/gpchc", 1, &Imu_Generator::parseGPCHCmsgCallback, this,
                                     ros::TransportHints().tcpNoDelay());
        } else if (generate_from == "GPFPD-GTIMU") {
            gpfpd_sub.subscribe(nh_, "/nmea/gpfpd", 10);
            gtimu_sub.subscribe(nh_, "/nmea/gtimu", 10);
            fpd_imu_sync.reset(new GPFPD_GTIMU_Sync(GPFPD_GTIMU_Policy(10), gpfpd_sub, gtimu_sub));
            fpd_imu_sync->registerCallback(boost::bind(&Imu_Generator::parseGPFPDmsgGTIMUmsgCallback, this, _1, _2));
        } else if (generate_from == "GTIMU") {
            nmea_sub = nh_.subscribe("/nmea/gtimu", 10, &Imu_Generator::parseGTIMUmsgCallback, this,
                                     ros::TransportHints().tcpNoDelay());
        } else if (generate_from == "INSPVAXB-GTIMU") {
            inspvaxb_sub.subscribe(nh_, "/spanlog/inspvaxb", 10);
            gtimu_sub.subscribe(nh_, "/nmea/gtimu", 10);
            inspvaxb_imu_sync.reset(new INSPVAXB_GTIMU_Sync(INSPVAXB_GTIMU_Policy(10), inspvaxb_sub, gtimu_sub));
            inspvaxb_imu_sync->registerCallback(
                    boost::bind(&Imu_Generator::parseINSPVAXBmsgGTIMUmsgCallback, this, _1, _2));
        } else if (generate_from == "RAWIMUB-DUALANTENNAHEADINGB") {
            rawimub_sub.subscribe(nh_, "/spanlog/rawimub", 10);
            dualantennaheadingb_sub.subscribe(nh_, "/spanlog/dualantennaheadingb", 10);
            rawimu_dualantennaheading_sync.reset(new RAWIMUB_DUALANTENNAHEADINGB_Sync (RAWIMUB_DUALANTENNAHEADINGB_Policy (10), rawimub_sub, dualantennaheadingb_sub));
            rawimu_dualantennaheading_sync->registerCallback(
                    boost::bind(&Imu_Generator::parseRAWIMUBmsgDUALANTENNAHEADINGBmsgCallback, this, _1, _2));
        } else if (generate_from == "NVSTD-GPFPD-GTIMU") {
            gpfpd_sub.subscribe(nh_, "/nmea/gpfpd", 10);
            nvstd_sub.subscribe(nh_, "/nmea/nvstd", 10);
            gtimu_sub.subscribe(nh_, "/nmea/gtimu", 10);
            nvstd_fpd_imu_sync.reset(
                    new NVSTD_GPFPD_GTIMU_Sync(NVSTD_GPFPD_GTIMU_Policy(10), nvstd_sub, gpfpd_sub, gtimu_sub));
            nvstd_fpd_imu_sync->registerCallback(
                    boost::bind(&Imu_Generator::parseNVSTDmsgGPFPDmsgGTIMUmsgCallback, this, _1, _2, _3));
        } else if (generate_from == "CORRIMUDATAB") {
            corrimudatab_sub = nh_.subscribe("/spanlog/corrimudatab", 10, &Imu_Generator::parseCORRIMUDATABmsgCallback, this,
                                             ros::TransportHints().tcpNoDelay());
        } else {
            ROS_ERROR(
                    "Uncorrect Imu source. It should be 'GPCHC' / 'GPFPD-GTIMU' / 'GTIMU' / 'INSPVAXB-GTIMU' / 'NVSTD-GPFPD-GTIMU' / 'RAWIMUB-DUALANTENNAHEADINGB' / 'CORRIMUDATAB'.");
            return;
        }


    }

    void parseGPCHCmsgCallback(const integrated_navigation_reader::NMEA_GPCHC::ConstPtr msg_in) {
        auto msg_out = new sensor_msgs::Imu();

        uint64_t nanosecond = 0;
        // conside GNSS UTC time from 1980, we need to mines leap second from 1970 to 1980
        auto gnss_nanosecond = GNSSUTCWeekAndTime2Nanocecond(msg_in->gnss_week, msg_in->gnss_time, leap_second - 9);
        auto local_nanosecond = msg_in->header.stamp.toNSec();
        generateTimeReference(local_nanosecond, gnss_nanosecond, "NMEA_GPCHC");
        if (use_gnss_time) {
            nanosecond = gnss_nanosecond;
        } else {
            nanosecond = local_nanosecond;
        }

        auto yaw = DEG2RAD(-msg_in->heading + 90.0);
        auto pitch = DEG2RAD(msg_in->pitch);
        auto roll = DEG2RAD(msg_in->roll);
        auto angular_velocity_x = DEG2RAD(msg_in->gyroscope_x);
        auto angular_velocity_y = DEG2RAD(msg_in->gyroscope_y);
        auto angular_velocity_z = DEG2RAD(msg_in->gyroscope_z);
        auto linear_acceleration_x = msg_in->acceleration_x * gravity;
        auto linear_acceleration_y = msg_in->acceleration_y * gravity;
        auto linear_acceleration_z = msg_in->acceleration_z * gravity;
        fillBasicImumsg(*msg_out, nanosecond, yaw, pitch, roll, angular_velocity_x, angular_velocity_y,
                        angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);

        msg_out->orientation_covariance[0] = -1;
        msg_out->angular_velocity_covariance[0] = -1;
        msg_out->linear_acceleration_covariance[0] = -1;

        pubImu(msg_out);
//        ROS_INFO("IMU frame id: %s",msg_out->header.frame_id.c_str());

        delete (msg_out);
    }

    void parseGTIMUmsgCallback(const integrated_navigation_reader::NMEA_GTIMU::ConstPtr msg_in) {
        auto msg_out = new sensor_msgs::Imu();

        uint64_t nanosecond = 0;
        // conside GNSS UTC time from 1980, we need to mines leap second from 1970 to 1980
        auto gnss_nanosecond = GNSSUTCWeekAndTime2Nanocecond(msg_in->gnss_week, msg_in->gnss_time, leap_second - 9);
        auto local_nanosecond = msg_in->header.stamp.toNSec();
        generateTimeReference(local_nanosecond, gnss_nanosecond, "NMEA_GPCHC");
        if (use_gnss_time) {
            nanosecond = gnss_nanosecond;
        } else {
            nanosecond = local_nanosecond;
        }

        auto yaw = DEG2RAD(0);
        auto pitch = DEG2RAD(0);
        auto roll = DEG2RAD(0);
        auto angular_velocity_x = DEG2RAD(msg_in->gyroscope_x);
        auto angular_velocity_y = DEG2RAD(msg_in->gyroscope_y);
        auto angular_velocity_z = DEG2RAD(msg_in->gyroscope_z);
        auto linear_acceleration_x = msg_in->acceleration_x * gravity;
        auto linear_acceleration_y = msg_in->acceleration_y * gravity;
        auto linear_acceleration_z = msg_in->acceleration_z * gravity;
        fillBasicImumsg(*msg_out, nanosecond, yaw, pitch, roll, angular_velocity_x, angular_velocity_y,
                        angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);

        msg_out->orientation_covariance[0] = -1;
        msg_out->angular_velocity_covariance[0] = -1;
        msg_out->linear_acceleration_covariance[0] = -1;

        pubImu(msg_out);
//        ROS_INFO("IMU frame id: %s",msg_out->header.frame_id.c_str());

        delete (msg_out);
    }

    void parseGPFPDmsgGTIMUmsgCallback(const integrated_navigation_reader::NMEA_GPFPD::ConstPtr &msgGPFPD_in,
                                       const integrated_navigation_reader::NMEA_GTIMU::ConstPtr &msgGTIMU_in) {
        auto msg_out = new sensor_msgs::Imu();

        uint64_t nanosecond = 0;
        // conside GNSS UTC time from 1980, we need to mines leap second from 1970 to 1980
        auto gnss_nanosecond = GNSSUTCWeekAndTime2Nanocecond(msgGPFPD_in->gnss_week, msgGPFPD_in->gnss_time,
                                                             leap_second - 9);
        auto local_nanosecond = msgGPFPD_in->header.stamp.toNSec();
        generateTimeReference(local_nanosecond, gnss_nanosecond, "NMEA_GPFPD");
        if (use_gnss_time) {
            nanosecond = gnss_nanosecond;
        } else {
            nanosecond = local_nanosecond;
        }

        auto yaw = DEG2RAD(-msgGPFPD_in->heading + 90.0);
        auto pitch = DEG2RAD(msgGPFPD_in->pitch);
        auto roll = DEG2RAD(msgGPFPD_in->roll);
        auto angular_velocity_x = DEG2RAD(msgGTIMU_in->gyroscope_x);
        auto angular_velocity_y = DEG2RAD(msgGTIMU_in->gyroscope_y);
        auto angular_velocity_z = DEG2RAD(msgGTIMU_in->gyroscope_z);
        auto linear_acceleration_x = msgGTIMU_in->acceleration_x * gravity;
        auto linear_acceleration_y = msgGTIMU_in->acceleration_y * gravity;
        auto linear_acceleration_z = msgGTIMU_in->acceleration_z * gravity;

        fillBasicImumsg(*msg_out, nanosecond, yaw, pitch, roll, angular_velocity_x, angular_velocity_y,
                        angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);

        msg_out->orientation_covariance[0] = -1;
        msg_out->angular_velocity_covariance[0] = -1;
        msg_out->linear_acceleration_covariance[0] = -1;

        pubImu(msg_out);
//        ROS_INFO("IMU frame id: %s",msg_out->header.frame_id.c_str());

        delete (msg_out);
    }

    void
    parseINSPVAXBmsgGTIMUmsgCallback(const integrated_navigation_reader::SPANLog_INSPVAXB::ConstPtr &msgINSPVAXB_in,
                                     const integrated_navigation_reader::NMEA_GTIMU::ConstPtr &msgGTIMU_in) {
        auto msg_out = new sensor_msgs::Imu();

        uint64_t nanosecond = 0;
        // conside GNSS UTC time from 1980, we need to mines leap second from 1970 to 1980
        auto gnss_nanosecond = GNSSUTCWeekAndTime2Nanocecond(msgINSPVAXB_in->log_header.week,
                                                             msgINSPVAXB_in->log_header.milliseconds * 1e-3,
                                                             leap_second - 9);
        auto local_nanosecond = msgINSPVAXB_in->header.stamp.toNSec();
        generateTimeReference(local_nanosecond, gnss_nanosecond, "Spanlog_INSPVAXB");
        if (use_gnss_time) {
            nanosecond = gnss_nanosecond;
        } else {
            nanosecond = local_nanosecond;
        }

        // follow REP-103 right-handed and east zero, https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSPVA.htm
        auto yaw = DEG2RAD(-msgINSPVAXB_in->azimuth + 90.0);
        auto pitch = DEG2RAD(msgINSPVAXB_in->pitch);
        auto roll = DEG2RAD(msgINSPVAXB_in->roll);
        auto angular_velocity_x = DEG2RAD(msgGTIMU_in->gyroscope_x);
        auto angular_velocity_y = DEG2RAD(msgGTIMU_in->gyroscope_y);
        auto angular_velocity_z = DEG2RAD(msgGTIMU_in->gyroscope_z);
        auto linear_acceleration_x = msgGTIMU_in->acceleration_x * gravity;
        auto linear_acceleration_y = msgGTIMU_in->acceleration_y * gravity;
        auto linear_acceleration_z = msgGTIMU_in->acceleration_z * gravity;

        fillBasicImumsg(*msg_out, nanosecond, yaw, pitch, roll, angular_velocity_x, angular_velocity_y,
                        angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);

        msg_out->orientation_covariance[0] = msgINSPVAXB_in->pitch_std;
        msg_out->orientation_covariance[4] = msgINSPVAXB_in->roll_std;
        msg_out->orientation_covariance[8] = msgINSPVAXB_in->azimuth_std;

        msg_out->angular_velocity_covariance[0] = -1;
        msg_out->linear_acceleration_covariance[0] = -1;

        pubImu(msg_out);
//        ROS_INFO("IMU frame id: %s",msg_out->header.frame_id.c_str());

        delete (msg_out);
    }

    void parseNVSTDmsgGPFPDmsgGTIMUmsgCallback(const integrated_navigation_reader::NMEA_NVSTD::ConstPtr &msgNVSTD_in,
                                               const integrated_navigation_reader::NMEA_GPFPD::ConstPtr &msgGPFPD_in,
                                               const integrated_navigation_reader::NMEA_GTIMU::ConstPtr &msgGTIMU_in) {
        auto msg_out = new sensor_msgs::Imu();

        uint64_t nanosecond = 0;
        // conside GNSS UTC time from 1980, we need to mines leap second from 1970 to 1980
        auto gnss_nanosecond = GNSSUTCWeekAndTime2Nanocecond(msgGPFPD_in->gnss_week, msgGPFPD_in->gnss_time,
                                                             leap_second - 9);
        auto local_nanosecond = msgGPFPD_in->header.stamp.toNSec();
        generateTimeReference(local_nanosecond, gnss_nanosecond, "NMEA_GPFPD");
        if (use_gnss_time) {
            nanosecond = gnss_nanosecond;
        } else {
            nanosecond = local_nanosecond;
        }

        auto yaw = DEG2RAD(-msgGPFPD_in->heading + 90.0);
        auto pitch = DEG2RAD(msgGPFPD_in->pitch);
        auto roll = DEG2RAD(msgGPFPD_in->roll);
        auto angular_velocity_x = DEG2RAD(msgGTIMU_in->gyroscope_x);
        auto angular_velocity_y = DEG2RAD(msgGTIMU_in->gyroscope_y);
        auto angular_velocity_z = DEG2RAD(msgGTIMU_in->gyroscope_z);
        auto linear_acceleration_x = msgGTIMU_in->acceleration_x * gravity;
        auto linear_acceleration_y = msgGTIMU_in->acceleration_y * gravity;
        auto linear_acceleration_z = msgGTIMU_in->acceleration_z * gravity;

        fillBasicImumsg(*msg_out, nanosecond, yaw, pitch, roll, angular_velocity_x, angular_velocity_y,
                        angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);

        msg_out->orientation_covariance[0] = msgNVSTD_in->pitch_std;
        msg_out->orientation_covariance[4] = msgNVSTD_in->roll_std;
        msg_out->orientation_covariance[8] = msgNVSTD_in->heading_std;

        msg_out->angular_velocity_covariance[0] = -1;
        msg_out->linear_acceleration_covariance[0] = -1;

        pubImu(msg_out);
//        ROS_INFO("IMU frame id: %s",msg_out->header.frame_id.c_str());

        delete (msg_out);
    }

    void parseRAWIMUBmsgDUALANTENNAHEADINGBmsgCallback(const integrated_navigation_reader::SPANLog_RAWIMUB::ConstPtr &msgRAWIMUB_in, const integrated_navigation_reader::Log_DUALANTENNAHEADINGB::ConstPtr &msgDUALANTENNAHEADINGB_in) {
        auto msg_out = new sensor_msgs::Imu();

        uint64_t nanosecond = 0;
        // conside GNSS UTC time from 1980, we need to mines leap second from 1970 to 1980
        auto gnss_nanosecond = GNSSUTCWeekAndTime2Nanocecond(msgRAWIMUB_in->week, msgRAWIMUB_in->seconds, leap_second - 9);

        auto local_nanosecond = msgRAWIMUB_in->header.stamp.toNSec();
        generateTimeReference(local_nanosecond, gnss_nanosecond, "SPANLog_RAWIMUB");
        if (use_gnss_time) {
            nanosecond = gnss_nanosecond;
        } else {
            nanosecond = local_nanosecond;
        }
//        std::cout<< "frequence: " << frequence<< "     last_nanosecond: " << last_nanosecond << std::endl;

        if (gyro_factor < 0 || accel_factor < 0) {
            if (last_nanosecond > 0) {
                frequence = 1.0 / ((gnss_nanosecond-last_nanosecond) * 1e-9);
                gyro_factor = frequence / 160849.543863;
                accel_factor = frequence / gravity / 655360.0;
            } else {
                last_nanosecond = gnss_nanosecond;
                return;
            }
        }

        auto yaw = DEG2RAD(-msgDUALANTENNAHEADINGB_in->heading + 90.0);
        auto pitch = DEG2RAD(msgDUALANTENNAHEADINGB_in->pitch);
        auto roll = 0.0;

        auto angular_velocity_x = DEG2RAD(msgRAWIMUB_in->x_gyroscope * gyro_factor);
        auto angular_velocity_y = DEG2RAD(-msgRAWIMUB_in->negative_y_gyroscope * gyro_factor);
        auto angular_velocity_z = DEG2RAD(msgRAWIMUB_in->z_gyroscope * gyro_factor);
        auto linear_acceleration_x = msgRAWIMUB_in->x_acceleration * accel_factor;
        auto linear_acceleration_y = -msgRAWIMUB_in->negative_y_acceleration * accel_factor;
        auto linear_acceleration_z = msgRAWIMUB_in->z_acceleration * accel_factor * gravity;

        fillBasicImumsg(*msg_out, nanosecond, yaw, pitch, roll, angular_velocity_x, angular_velocity_y,
                        angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);

        msg_out->orientation_covariance[0] = msgDUALANTENNAHEADINGB_in->pitch_std;
        msg_out->orientation_covariance[4] = 0;
        msg_out->orientation_covariance[8] = msgDUALANTENNAHEADINGB_in->heading_std;

        msg_out->angular_velocity_covariance[0] = -1;
        msg_out->linear_acceleration_covariance[0] = -1;

        pubImu(msg_out);
//        ROS_INFO("IMU frame id: %s",msg_out->header.frame_id.c_str());

        delete (msg_out);
    }

    void parseCORRIMUDATABmsgCallback(const integrated_navigation_reader::SPANLog_CORRIMUDATAB::ConstPtr &msg_in) {
        auto msg_out = new sensor_msgs::Imu();

        uint64_t nanosecond = 0;
        // conside GNSS UTC time from 1980, we need to mines leap second from 1970 to 1980
        auto gnss_nanosecond = GNSSUTCWeekAndTime2Nanocecond(msg_in->week, msg_in->seconds, leap_second - 9);

        auto local_nanosecond = msg_in->header.stamp.toNSec();
        generateTimeReference(local_nanosecond, gnss_nanosecond, "SPANLog_CORRIMUDATAB");
        if (use_gnss_time) {

            nanosecond = gnss_nanosecond;
        } else {
            nanosecond = local_nanosecond;
        }
//        std::cout<< "frequence: " << frequence<< "     last_nanosecond: " << last_nanosecond << std::endl;

        if (last_nanosecond > 0) {
            frequence = 1.0 / ((gnss_nanosecond-last_nanosecond) * 1e-9);
            gyro_factor = frequence / 160849.543863;
            accel_factor = frequence / gravity / 655360.0;
        } else {
            last_nanosecond = gnss_nanosecond;
            return;
        }

        auto yaw = 0;
        auto pitch = 0;
        auto roll = 0;

        auto angular_velocity_x = msg_in->pitch_rate * frequence;
        auto angular_velocity_y = msg_in->roll_rate * frequence;
        auto angular_velocity_z = msg_in->yaw_rate * frequence;
        auto linear_acceleration_x = msg_in->lateral_acceleration * frequence;
        auto linear_acceleration_y = msg_in->longitudinal_acceleration * frequence;
        auto linear_acceleration_z = msg_in->vertical_acceleration * frequence;

        fillBasicImumsg(*msg_out, nanosecond, yaw, pitch, roll, angular_velocity_x, angular_velocity_y,
                        angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);

        msg_out->orientation_covariance[0] = -1;
        msg_out->angular_velocity_covariance[0] = -1;
        msg_out->linear_acceleration_covariance[0] = -1;

        pubImu(msg_out);
//        ROS_INFO("IMU frame id: %s",msg_out->header.frame_id.c_str());

        delete (msg_out);
    }


    void pubImu(const sensor_msgs::Imu *msg) {
        imu_pub.publish(*msg);
    }


private:

    void generateTimeReference(const uint64_t stamp_nanosecond, const uint64_t gnss_nanosecond, const char *source) {
        sensor_msgs::TimeReference msg;
        msg.header.frame_id = frame_id + "_time";

        ros::Time tROSTime;
        msg.header.stamp = tROSTime.fromNSec(stamp_nanosecond);
        msg.time_ref = tROSTime.fromNSec(gnss_nanosecond);
        msg.source = source;

        timereference_pub.publish(msg);
    }

    ros::NodeHandle &nh_, &nh_local;

    ros::Publisher imu_pub;
    ros::Publisher timereference_pub;
    ros::Subscriber nmea_sub;

    ros::Subscriber corrimudatab_sub;

    std::string generate_from;

    message_filters::Subscriber<integrated_navigation_reader::NMEA_GPFPD> gpfpd_sub;
    message_filters::Subscriber<integrated_navigation_reader::NMEA_GTIMU> gtimu_sub;
    typedef message_filters::sync_policies::ApproximateTime<integrated_navigation_reader::NMEA_GPFPD, integrated_navigation_reader::NMEA_GTIMU> GPFPD_GTIMU_Policy;
    typedef message_filters::Synchronizer<GPFPD_GTIMU_Policy> GPFPD_GTIMU_Sync;
    boost::shared_ptr<GPFPD_GTIMU_Sync> fpd_imu_sync;

    message_filters::Subscriber<integrated_navigation_reader::SPANLog_INSPVAXB> inspvaxb_sub;
    typedef message_filters::sync_policies::ApproximateTime<integrated_navigation_reader::SPANLog_INSPVAXB, integrated_navigation_reader::NMEA_GTIMU> INSPVAXB_GTIMU_Policy;
    typedef message_filters::Synchronizer<INSPVAXB_GTIMU_Policy> INSPVAXB_GTIMU_Sync;
    boost::shared_ptr<INSPVAXB_GTIMU_Sync> inspvaxb_imu_sync;

    message_filters::Subscriber<integrated_navigation_reader::SPANLog_RAWIMUB> rawimub_sub;
    message_filters::Subscriber<integrated_navigation_reader::Log_DUALANTENNAHEADINGB> dualantennaheadingb_sub;
    typedef message_filters::sync_policies::ApproximateTime<integrated_navigation_reader::SPANLog_RAWIMUB, integrated_navigation_reader::Log_DUALANTENNAHEADINGB> RAWIMUB_DUALANTENNAHEADINGB_Policy;
    typedef message_filters::Synchronizer<RAWIMUB_DUALANTENNAHEADINGB_Policy> RAWIMUB_DUALANTENNAHEADINGB_Sync;
    boost::shared_ptr<RAWIMUB_DUALANTENNAHEADINGB_Sync> rawimu_dualantennaheading_sync;

    double frequence = 0;
    uint64_t last_nanosecond = 0;
    double gyro_factor = -1;
    double accel_factor = -1;

    message_filters::Subscriber<integrated_navigation_reader::NMEA_NVSTD> nvstd_sub;
    typedef message_filters::sync_policies::ApproximateTime<integrated_navigation_reader::NMEA_NVSTD, integrated_navigation_reader::NMEA_GPFPD, integrated_navigation_reader::NMEA_GTIMU> NVSTD_GPFPD_GTIMU_Policy;
    typedef message_filters::Synchronizer<NVSTD_GPFPD_GTIMU_Policy> NVSTD_GPFPD_GTIMU_Sync;
    boost::shared_ptr<NVSTD_GPFPD_GTIMU_Sync> nvstd_fpd_imu_sync;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_generator_node");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local("~");
    nh_.getParam("time_set/leap_second", leap_second);
    nh_local.getParam("use_gnss_time", use_gnss_time);

    Imu_Generator generator(nh_, nh_local);

    ros::spin();
    return 0;
}