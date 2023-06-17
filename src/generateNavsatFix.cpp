#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include "integrated_navigation_driver/NMEA_GPFPD.h"
#include "integrated_navigation_driver/NMEA_GPGGA.h"
#include "integrated_navigation_driver/NMEA_GPCHC.h"
#include "utility.h"

#include <cmath>
#include <cstring>
#include <vector>
#include <sstream>

ros::Publisher navsatfix_pub;

std::string generate_from;
std::string frame_id;
bool use_gnss_time = false;
int leap_second = 27;

void pubNavsatFix(const sensor_msgs::NavSatFix * msg)
{
    navsatfix_pub.publish(*msg);
}

void fillBasicNavsatFixmsg(sensor_msgs::NavSatFix& msg, const uint64_t nanosecond, const double latitude, const double longitude, const double altitude)
{
    msg.header.frame_id = frame_id;

    ros::Time tROSTime;
    msg.header.stamp = tROSTime.fromNSec(nanosecond);

    msg.latitude = latitude;
    msg.longitude = longitude;
    msg.altitude = altitude;
}

void parseGPGGAmsgCallback(const integrated_navigation_driver::NMEA_GPGGA::ConstPtr msg_in)
{
    char *ptr_t;
    auto msg_out = new sensor_msgs::NavSatFix();

    if (use_gnss_time)
    {
        ros::NodeHandle nh_;
        ros::Time tROSTime;

        auto utc_time = boost::numeric_cast<double>(strtof64(msg_in->utc_time.c_str(), &ptr_t));

        int time_zone = nh_.param("time_set/time_zone", 0);

        unsigned long utc_data_sec = getUTCDate2second();
        unsigned int hour = int(utc_time)/10000;
        unsigned int minute = (int(utc_time)-hour*10000)/100;
        unsigned int second = int(utc_time-hour*10000-minute*100);
        unsigned long nanosecond = int((utc_time-hour*10000-minute*100-second)*1e9);

        msg_out->header.stamp = tROSTime.fromNSec((utc_data_sec+(hour+time_zone)*60*60+minute*60+second)*1e9+nanosecond);

    } else
    {
        msg_out->header.stamp = msg_in->header.stamp;
    }

    // refer: https://github.com/ros-drivers/nmea_navsat_driver/blob/indigo-devel/src/libnmea_navsat_driver/driver.py#L110-L114
    if (msg_in->gnss_quality == integrated_navigation_driver::NMEA_GPGGA::GNSS_FIX_INVALID)
    {
        msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    } else if (msg_in->gnss_quality == integrated_navigation_driver::NMEA_GPGGA::GNSS_SINGLE_POINT || msg_in->gnss_quality == integrated_navigation_driver::NMEA_GPGGA::GNSS_MANUAL_FIXED)
    {
        msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    } else if (msg_in->gnss_quality == integrated_navigation_driver::NMEA_GPGGA::GNSS_PSEUDORANGE_DIFFERENTIAL)
    {
        msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
    } else if (msg_in->gnss_quality == integrated_navigation_driver::NMEA_GPGGA::GNSS_FIX_SOLUTION || msg_in->gnss_quality == integrated_navigation_driver::NMEA_GPGGA::GNSS_FLOATING_SOLUTION)
    {
        msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    } else
    {
        msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    }

    msg_out->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

    auto latitude = boost::numeric_cast<double>(strtof64(msg_in->latitude.substr(0,2).c_str(), &ptr_t)) + boost::numeric_cast<double>(strtof64(msg_in->latitude.substr(2).c_str(), &ptr_t))/60;
    msg_out->latitude = latitude;
    if (msg_in->latitude_direction == integrated_navigation_driver::NMEA_GPGGA::LATITUDE_SOUTH) {
        msg_out->latitude = - msg_out->latitude;
    }
    auto longitude = boost::numeric_cast<double>(strtof64(msg_in->longitude.substr(0,3).c_str(), &ptr_t)) + boost::numeric_cast<double>(strtof64(msg_in->longitude.substr(3).c_str(), &ptr_t))/60;
    msg_out->longitude = longitude;
    if (msg_in->longitude_direction == integrated_navigation_driver::NMEA_GPGGA::LONGITUDE_WEST) {
        msg_out->longitude = - msg_out->longitude;
    }

    msg_out->altitude = transfer2MeterUnit(msg_in->altitude, msg_in->altitude_units);

    msg_out->position_covariance[0] = pow(msg_in->horizontal_dilution_of_precision, 2);
    msg_out->position_covariance[4] = pow(msg_in->horizontal_dilution_of_precision, 2);
    msg_out->position_covariance[8] = pow(msg_in->horizontal_dilution_of_precision * 2, 2);
    msg_out->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    pubNavsatFix(msg_out);

    delete(msg_out);
}

void parseGPCHCmsgCallback(const integrated_navigation_driver::NMEA_GPCHC::ConstPtr msg_in)
{
    auto msg_out = new sensor_msgs::NavSatFix();

    uint64_t nanosecond = 0;
    if (use_gnss_time)
    {
//     conside GNSS UTC time from 1980, we need to mines leap second from 1970 to 1980
        nanosecond = GNSSUTCWeekAndTime2Nanocecond(msg_in->gnss_week, msg_in->gnss_time, leap_second - 9);
    } else
    {
        nanosecond = msg_in->header.stamp.toNSec();
    }

    auto satellite_status = msg_in->status[0];
    switch (satellite_status) {
        case 49:
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            break;
        case 50:
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
            break;
        case 51:
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
            break;
        case 52:
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            break;
        case 53:
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            break;
        case 54:
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            break;
        case 55:
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
            break;
        case 56:
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            break;
        case 57:
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            break;
        default:
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    }

    auto latitude = msg_in->latitude;
    auto longitude = msg_in->longitude;
//    GPFPD altitude refer to earth geoid, not WGS84 ellipsoid which defined in NavsatFix.
    auto altitude = msg_in->altitude;
    fillBasicNavsatFixmsg(*msg_out, nanosecond, latitude, longitude, altitude);

    msg_out->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    pubNavsatFix(msg_out);

    delete(msg_out);
}

void parseGPFPDmsgCallback(const integrated_navigation_driver::NMEA_GPFPD::ConstPtr msg_in)
{
    auto msg_out = new sensor_msgs::NavSatFix();

    uint64_t nanosecond = 0;
    if (use_gnss_time)
    {
//     conside GNSS time from 1980, we need to mines leap second from 1970 to 1980
        nanosecond = GNSSUTCWeekAndTime2Nanocecond(msg_in->gnss_week, msg_in->gnss_time, leap_second - 9);
    } else
    {
        nanosecond = msg_in->header.stamp.toNSec();
    }

    if (msg_in->status[1] > 48) {
        if (msg_in->status[0] == 50) {
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        } else if (msg_in->status[0] == 52 || msg_in->status[0] == 53) {
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
        } else {
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        }
    } else {
        msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    }

    auto latitude = msg_in->latitude;
    auto longitude = msg_in->longitude;
//    GPFPD altitude refer to earth geoid, not WGS84 ellipsoid which defined in NavsatFix.
    auto altitude = msg_in->altitude;
    fillBasicNavsatFixmsg(*msg_out, nanosecond, latitude, longitude, altitude);

    msg_out->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    pubNavsatFix(msg_out);

    delete(msg_out);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "navsatfix_generator_node");
    ros::NodeHandle nh_;
    navsatfix_pub = nh_.advertise<sensor_msgs::NavSatFix>("/integrated_nav/NavsatFix", 10);
    ros::Subscriber nmea_sub;

    ros::NodeHandle nh_local("~");
    nh_local.getParam("NavsatFix_generate_from", generate_from);
    nh_local.getParam("frame_id", frame_id);
    nh_local.getParam("use_gnss_time", use_gnss_time);
    nh_.getParam("time_set/leap_second", leap_second);

    ROS_WARN("If no NavsatFix message, maybe you source %s message is empty", generate_from.c_str());
    if (generate_from == "GPGGA")
    {
        nmea_sub = nh_.subscribe("/nmea/gpgga", 10, parseGPGGAmsgCallback);
    } else if (generate_from == "GPCHC")
    {
        ROS_WARN("GPCHC altitude refer to earth geoid, not WGS84 ellipsoid which defined in NavsatFix");
        nmea_sub = nh_.subscribe("/nmea/gpchc", 10, parseGPCHCmsgCallback);
    } else if (generate_from == "GPFPD")
    {
        ROS_WARN("GPFPD altitude refer to earth geoid, not WGS84 ellipsoid which defined in NavsatFix");
        nmea_sub = nh_.subscribe("/nmea/gppfd", 10, parseGPFPDmsgCallback);
    } else
    {
        ROS_ERROR("Uncorrect NavsatFix source. It should be 'GPGGA' / 'GPFPD' / 'GPCHC'.");
        return 0;
    }

    ros::spin();
    return 0;
}