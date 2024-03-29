#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include "integrated_navigation_reader/NMEA_GPFPD.h"
#include "integrated_navigation_reader/NMEA_GPGGA.h"
#include "integrated_navigation_reader/NMEA_GPCHC.h"
#include "integrated_navigation_reader/SPANLog_INSPVAXB.h"
#include "integrated_navigation_reader/SPANLog_BESTGNSSPOSB.h"
#include "utility.h"

#include <cmath>
#include <cstring>
#include <vector>
#include <sstream>

std::string frame_id;
bool use_gnss_time = false;
int leap_second = 27;

void fillBasicNavsatFixmsg(sensor_msgs::NavSatFix &msg, const uint64_t nanosecond, const double latitude,
                           const double longitude, const double altitude) {
    msg.header.frame_id = frame_id;

    ros::Time tROSTime;
    msg.header.stamp = tROSTime.fromNSec(nanosecond);

    msg.latitude = latitude;
    msg.longitude = longitude;
    msg.altitude = altitude;
}

class NavsatFix_Generator {
public:
    NavsatFix_Generator(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : nh_(nh), nh_local(private_nh) {
//        ROS_WARN(" ################   NavsatFix_Generator   ################");
        navsatfix_pub = nh_.advertise<sensor_msgs::NavSatFix>("/integrated_nav/NavsatFix", 10);
        timereference_pub = nh_.advertise<sensor_msgs::TimeReference>("/integrated_nav/INS_Time", 10);

        nh_local.getParam("NavsatFix_generate_from", generate_from);

        ROS_WARN("If no NavsatFix message, maybe you source %s message is empty", generate_from.c_str());
        if (generate_from == "GPGGA")
        {
            nmea_sub = nh_.subscribe("/nmea/gpgga", 10, &NavsatFix_Generator::parseGPGGAmsgCallback, this, ros::TransportHints().tcpNoDelay());
        } else if (generate_from == "GPCHC")
        {
            ROS_WARN("GPCHC altitude refer to earth geoid, not WGS84 ellipsoid which defined in NavsatFix");
            nmea_sub = nh_.subscribe("/nmea/gpchc", 10, &NavsatFix_Generator::parseGPCHCmsgCallback, this, ros::TransportHints().tcpNoDelay());
        } else if (generate_from == "GPFPD")
        {
            ROS_WARN("GPFPD altitude refer to earth geoid, not WGS84 ellipsoid which defined in NavsatFix");
            nmea_sub = nh_.subscribe("/nmea/gpfpd", 10, &NavsatFix_Generator::parseGPFPDmsgCallback, this, ros::TransportHints().tcpNoDelay());
        } else if (generate_from == "INSPVAXB")
        {
            nmea_sub = nh_.subscribe("/spanlog/inspvaxb", 10, &NavsatFix_Generator::parseINSPVAXBmsgCallback, this, ros::TransportHints().tcpNoDelay());
        } else if (generate_from == "BESTGNSSPOSB")
        {
            nmea_sub = nh_.subscribe("/spanlog/bestgnssposb", 10, &NavsatFix_Generator::parseBESTGNSSPOSBmsgCallback, this, ros::TransportHints().tcpNoDelay());
        } else
        {
            ROS_ERROR("Uncorrect NavsatFix source: %s. It should be 'GPGGA' / 'GPFPD' / 'GPCHC' / 'INSPVAXB' / 'BESTGNSSPOSB'.", generate_from.c_str());
            return;
        }
    }

    void pubNavsatFix(const sensor_msgs::NavSatFix *msg) {
        navsatfix_pub.publish(*msg);
    }

    void generateTimeReference(const uint64_t stamp_nanosecond, const uint64_t gnss_nanosecond, const char *source) {
        sensor_msgs::TimeReference msg;
        msg.header.frame_id = frame_id + "_time";

        ros::Time tROSTime;
        msg.header.stamp = tROSTime.fromNSec(stamp_nanosecond);
        msg.time_ref = tROSTime.fromNSec(gnss_nanosecond);
        msg.source = source;

        timereference_pub.publish(msg);
    }

    void parseGPGGAmsgCallback(const integrated_navigation_reader::NMEA_GPGGA::ConstPtr msg_in) {
        char *ptr_t;
        auto msg_out = new sensor_msgs::NavSatFix();
        ros::NodeHandle nh_;
        int time_zone = nh_.param("time_set/time_zone", 0);

        auto utc_time = boost::numeric_cast<double>(strtof64(msg_in->utc_time.c_str(), &ptr_t));
        unsigned long utc_data_sec = getUTCDate2second();
        unsigned int utc_hour = int(utc_time) / 10000;
        unsigned int utc_minute = (int(utc_time) - utc_hour * 10000) / 100;
        unsigned int utc_second = int(utc_time - utc_hour * 10000 - utc_minute * 100);
        unsigned long utc_nanosecond = int((utc_time - utc_hour * 10000 - utc_minute * 100 - utc_second) * 1e9);

        uint64_t nanosecond = 0;
        auto gnss_nanosecond =
                (utc_data_sec + (utc_hour + time_zone) * 60 * 60 + utc_minute * 60 + utc_second) * 1e9 + utc_nanosecond;
        auto local_nanosecond = msg_in->header.stamp.toNSec();
        generateTimeReference(local_nanosecond, gnss_nanosecond, "NMEA_GPGGA");
        if (use_gnss_time) {
            nanosecond = gnss_nanosecond;
        } else {
            nanosecond = local_nanosecond;
        }

        // refer: https://github.com/ros-drivers/nmea_navsat_driver/blob/indigo-devel/src/libnmea_navsat_driver/driver.py#L110-L114
        if (msg_in->gnss_quality == integrated_navigation_reader::NMEA_GPGGA::GNSS_FIX_INVALID) {
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        } else if (msg_in->gnss_quality == integrated_navigation_reader::NMEA_GPGGA::GNSS_SINGLE_POINT ||
                   msg_in->gnss_quality == integrated_navigation_reader::NMEA_GPGGA::GNSS_MANUAL_FIXED) {
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        } else if (msg_in->gnss_quality == integrated_navigation_reader::NMEA_GPGGA::GNSS_PSEUDORANGE_DIFFERENTIAL) {
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        } else if (msg_in->gnss_quality == integrated_navigation_reader::NMEA_GPGGA::GNSS_FIX_SOLUTION ||
                   msg_in->gnss_quality == integrated_navigation_reader::NMEA_GPGGA::GNSS_FLOATING_SOLUTION) {
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
        } else {
            msg_out->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        }

        msg_out->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        auto latitude = boost::numeric_cast<double>(strtof64(msg_in->latitude.substr(0, 2).c_str(), &ptr_t)) +
                        boost::numeric_cast<double>(strtof64(msg_in->latitude.substr(2).c_str(), &ptr_t)) / 60;
        if (msg_in->latitude_direction == integrated_navigation_reader::NMEA_GPGGA::LATITUDE_SOUTH) {
            latitude = -latitude;
        }
        auto longitude = boost::numeric_cast<double>(strtof64(msg_in->longitude.substr(0, 3).c_str(), &ptr_t)) +
                         boost::numeric_cast<double>(strtof64(msg_in->longitude.substr(3).c_str(), &ptr_t)) / 60;
        if (msg_in->longitude_direction == integrated_navigation_reader::NMEA_GPGGA::LONGITUDE_WEST) {
            longitude = -longitude;
        }

        auto altitude = transfer2MeterUnit(msg_in->altitude, msg_in->altitude_units) + transfer2MeterUnit(msg_in->undulation, msg_in->undulation_units) ;

        fillBasicNavsatFixmsg(*msg_out, nanosecond, latitude, longitude, altitude);

        msg_out->position_covariance[0] = pow(msg_in->horizontal_dilution_of_precision, 2);
        msg_out->position_covariance[4] = pow(msg_in->horizontal_dilution_of_precision, 2);
        msg_out->position_covariance[8] = pow(msg_in->horizontal_dilution_of_precision * 2, 2);
        msg_out->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

        pubNavsatFix(msg_out);

        delete (msg_out);
    }

    void parseGPCHCmsgCallback(const integrated_navigation_reader::NMEA_GPCHC::ConstPtr msg_in) {
        auto msg_out = new sensor_msgs::NavSatFix();

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

        msg_out->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        auto latitude = msg_in->latitude;
        auto longitude = msg_in->longitude;
        //    GPFPD altitude refer to earth geoid, not WGS84 ellipsoid which defined in NavsatFix.
        auto altitude = msg_in->altitude;
        fillBasicNavsatFixmsg(*msg_out, nanosecond, latitude, longitude, altitude);

        msg_out->position_covariance[0] = -1;
        msg_out->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        pubNavsatFix(msg_out);
//        ROS_INFO("NavsatFix frame id: %s",msg_out->header.frame_id.c_str());


        delete (msg_out);
    }

    void parseGPFPDmsgCallback(const integrated_navigation_reader::NMEA_GPFPD::ConstPtr msg_in) {
        auto msg_out = new sensor_msgs::NavSatFix();

        uint64_t nanosecond = 0;
        // conside GNSS time from 1980, we need to mines leap second from 1970 to 1980
        auto gnss_nanosecond = GNSSUTCWeekAndTime2Nanocecond(msg_in->gnss_week, msg_in->gnss_time, leap_second - 9);
        auto local_nanosecond = msg_in->header.stamp.toNSec();
        generateTimeReference(local_nanosecond, gnss_nanosecond, "NMEA_GPFPD");
        if (use_gnss_time) {
            //     conside GNSS time from 1980, we need to mines leap second from 1970 to 1980
            nanosecond = gnss_nanosecond;
        } else {
            nanosecond = local_nanosecond;
        }

        if (msg_in->status[1] > 50 && msg_in->status[1] < 55) {
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

        msg_out->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        //    if (msg_in->status[0] != 52 || msg_in->status[1] != 66) {
        //        ROS_ERROR("GNSS STATUS != 4B");
        //    }

        auto latitude = msg_in->latitude;
        auto longitude = msg_in->longitude;
        //    GPFPD altitude refer to earth geoid, not WGS84 ellipsoid which defined in NavsatFix.
        auto altitude = msg_in->altitude;
        fillBasicNavsatFixmsg(*msg_out, nanosecond, latitude, longitude, altitude);

        msg_out->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        pubNavsatFix(msg_out);

        //    ROS_INFO("NavsatFix frame id: %s",msg_out->header.frame_id.c_str());

        delete (msg_out);
    }

    void parseINSPVAXBmsgCallback(const integrated_navigation_reader::SPANLog_INSPVAXB::ConstPtr msg_in) {
        auto msg_out = new sensor_msgs::NavSatFix();

        uint64_t nanosecond = 0;
        // conside GNSS time from 1980, we need to mines leap second from 1970 to 1980
        auto gnss_nanosecond = GNSSUTCWeekAndTime2Nanocecond(msg_in->log_header.week,
                                                             msg_in->log_header.milliseconds * 1e-3, leap_second - 9);
        auto local_nanosecond = msg_in->header.stamp.toNSec();
        generateTimeReference(local_nanosecond, gnss_nanosecond, "Spanlog_INSPVAXB");
        if (use_gnss_time) {
            //     conside GNSS time from 1980, we need to mines leap second from 1970 to 1980
            nanosecond = gnss_nanosecond;
        } else {
            nanosecond = local_nanosecond;
        }

        msg_out->status = parseNavSatStatus(msg_in->ins_status, msg_in->position_type);

        msg_out->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        auto latitude = msg_in->latitude;
        auto longitude = msg_in->longitude;
        auto altitude = msg_in->height + msg_in->undulation;
        fillBasicNavsatFixmsg(*msg_out, nanosecond, latitude, longitude, altitude);

        msg_out->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        msg_out->position_covariance[0] = pow(msg_in->longitude_std, 2);
        msg_out->position_covariance[4] = pow(msg_in->latitude_std, 2);
        msg_out->position_covariance[8] = pow(msg_in->height_std, 2);

        pubNavsatFix(msg_out);

        delete (msg_out);
    }

    void parseBESTGNSSPOSBmsgCallback(const integrated_navigation_reader::SPANLog_BESTGNSSPOSB::ConstPtr msg_in) {
        auto msg_out = new sensor_msgs::NavSatFix();

        uint64_t nanosecond = 0;
        // conside GNSS time from 1980, we need to mines leap second from 1970 to 1980
        auto gnss_nanosecond = GNSSUTCWeekAndTime2Nanocecond(msg_in->log_header.week,
                                                             msg_in->log_header.milliseconds * 1e-3, leap_second - 9);
        auto local_nanosecond = msg_in->header.stamp.toNSec();
        generateTimeReference(local_nanosecond, gnss_nanosecond, "Spanlog_BESTGNSSPOSB");
        if (use_gnss_time) {
            //     conside GNSS time from 1980, we need to mines leap second from 1970 to 1980
            nanosecond = gnss_nanosecond;
        } else {
            nanosecond = local_nanosecond;
        }

        msg_out->status = parseNavSatStatus(msg_in->solution_status, msg_in->position_type);

        msg_out->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        auto latitude = msg_in->latitude;
        auto longitude = msg_in->longitude;
        auto altitude = msg_in->height + msg_in->undulation;
        fillBasicNavsatFixmsg(*msg_out, nanosecond, latitude, longitude, altitude);

        msg_out->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        msg_out->position_covariance[0] = pow(msg_in->longitude_std, 2);
        msg_out->position_covariance[4] = pow(msg_in->latitude_std, 2);
        msg_out->position_covariance[8] = pow(msg_in->height_std, 2);

        pubNavsatFix(msg_out);

        delete (msg_out);
    }

    sensor_msgs::NavSatStatus parseNavSatStatus(const integrated_navigation_reader::Log_SolutionStatus msg_status,
                                   const integrated_navigation_reader::Log_PositionVelocityType msg_in) {
        sensor_msgs::NavSatStatus status;
        if (msg_status.status == integrated_navigation_reader::Log_SolutionStatus::SOL_COMPUTED) {
            status = parseNavSatStatus(msg_in);
        } else {
            status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        }

        return status;
    }

    sensor_msgs::NavSatStatus parseNavSatStatus(const integrated_navigation_reader::SPANLog_INSStatus msg_status,
                                                const integrated_navigation_reader::Log_PositionVelocityType msg_in) {
        sensor_msgs::NavSatStatus status;
        if (msg_status.status == integrated_navigation_reader::SPANLog_INSStatus::INS_SOLUTION_GOOD) {
            status = parseNavSatStatus(msg_in);
        } else {
            status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        }

        return status;
    }

    sensor_msgs::NavSatStatus parseNavSatStatus(const integrated_navigation_reader::Log_PositionVelocityType msg_in) {
        sensor_msgs::NavSatStatus status;
        if (msg_in.type <= 0 || (msg_in.type >= 3 && msg_in.type <= 16) ||
            (msg_in.type >= 19 && msg_in.type <= 47) || (msg_in.type >= 52 && msg_in.type <= 68) ||
            (msg_in.type >= 71 && msg_in.type <= 77) || msg_in.type == 79 || msg_in.type == 80) {
            status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        } else {
            if (msg_in.type == 18 || msg_in.type == 69 || msg_in.type == 78) {
                status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
            } else if (msg_in.type == 17) {
                status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            } else {
                status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            }
        }

        return status;
    }


private:
    ros::NodeHandle &nh_, &nh_local;

    ros::Subscriber nmea_sub;

    ros::Publisher navsatfix_pub;
    ros::Publisher timereference_pub;

    std::string generate_from;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navsatfix_generator_node");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local("~");

    nh_local.getParam("frame_id", frame_id);
    nh_local.getParam("use_gnss_time", use_gnss_time);
    nh_.getParam("time_set/leap_second", leap_second);

    NavsatFix_Generator generator(nh_, nh_local);

    ros::spin();
    return 0;
}