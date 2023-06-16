#ifndef _ROSTEST_HPP_
#define _ROSTEST_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nmea_msgs/Sentence.h>

#include "integrated_navigation_driver/NMEA_GPFPD.h"
#include "integrated_navigation_driver/NMEA_GTIMU.h"
#include "integrated_navigation_driver/NMEA_GPGGA.h"
#include "integrated_navigation_driver/NMEA_NVSTD.h"
#include "integrated_navigation_driver/NMEA_GPCHC.h"

#include <cmath>
#include <cstring>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <vector>
#include <sstream>

class NMEA_Parser
{
public:
    NMEA_Parser()
    {
        ros::NodeHandle nh_local("~");

        nh_local.getParam("GPFPD", use_GPFPD);
        nh_local.getParam("GTIMU", use_GTIMU);
        nh_local.getParam("GPGGA", use_GPGGA);
        nh_local.getParam("NVSTD", use_NVSTD);
        nh_local.getParam("GPCHC", use_GPCHC);

        nmea_sentense_sub = nh_.subscribe("/nmea_sentence", 1, &NMEA_Parser::NMEAReader, this);

        if (use_GPFPD)
        {
            GPFPD_pub = nh_.advertise<integrated_navigation_driver::NMEA_GPFPD>("/nmea/gpfpd", 10);
        }
        if (use_GTIMU)
        {
            GTIMU_pub = nh_.advertise<integrated_navigation_driver::NMEA_GTIMU>("/nmea/gtimu", 10);
        }
        if (use_GPGGA)
        {
            GPGGA_pub = nh_.advertise<integrated_navigation_driver::NMEA_GPGGA>("/nmea/gpgga", 10);
        }
        if (use_NVSTD)
        {
            NVSTD_pub = nh_.advertise<integrated_navigation_driver::NMEA_NVSTD>("/nmea/nvstd", 10);
        }
        if (use_GPCHC)
        {
            GPCHC_pub = nh_.advertise<integrated_navigation_driver::NMEA_GPCHC>("/nmea/gpchc", 10);
        }

    }

    void NMEAReader(const nmea_msgs::Sentence::ConstPtr& nmea_msg)
    {

        bool detected_GPFPD = false;
        bool detected_GTIMU = false;
        bool detected_GPGGA = false;
        bool detected_NVSTD = false;
        bool detected_GPCHC = false;
        integrated_navigation_driver::NMEA_GPFPD* pGPFPD = nullptr;
        integrated_navigation_driver::NMEA_GTIMU* pGTIMU = nullptr;
        integrated_navigation_driver::NMEA_GPGGA* pGPGGA = nullptr;
        integrated_navigation_driver::NMEA_NVSTD* pNVSTD = nullptr;
        integrated_navigation_driver::NMEA_GPCHC* pGPCHC = nullptr;

        const std::string nmea_str = nmea_msg->sentence;
        if (nmea_str.find("$GPFPD") != std::string::npos || nmea_str.find("$gpfpd") != std::string::npos)
        {
            detected_GPFPD = true;
        }
        if (nmea_str.find("$GTIMU") != std::string::npos || nmea_str.find("$gtimu") != std::string::npos)
        {
            detected_GTIMU = true;
        }
        if (nmea_str.find("$GPGGA") != std::string::npos || nmea_str.find("$gpgga") != std::string::npos)
        {
            detected_GPGGA = true;
        }
        if (nmea_str.find("$NVSTD") != std::string::npos || nmea_str.find("$nvstd") != std::string::npos)
        {
            detected_NVSTD = true;
        }
        if (nmea_str.find("$GPCHC") != std::string::npos || nmea_str.find("$gpchc") != std::string::npos)
        {
            detected_GPCHC = true;
        }


        if (use_GPFPD && detected_GPFPD)
        {
            pGPFPD = new integrated_navigation_driver::NMEA_GPFPD();
            parseGPFPD2msg(nmea_str, *pGPFPD);
            if (!pGPFPD)
            {
                ROS_WARN("%s", nmea_msg->sentence.c_str());
            }
            else
            {
                pGPFPD->header.stamp = nmea_msg->header.stamp;
                pubGPFPD(pGPFPD);
            }
        }

        if (use_GTIMU && detected_GTIMU)
        {
            pGTIMU = new integrated_navigation_driver::NMEA_GTIMU();
            parseGTIMU2msg(nmea_str, *pGTIMU);
            if (!pGTIMU)
            {
                ROS_WARN("%s", nmea_msg->sentence.c_str());
            }
            else
            {
                pGTIMU->header.stamp = nmea_msg->header.stamp;
                pubGTIMU(pGTIMU);
            }
        }

        if (use_GPGGA && detected_GPGGA)
        {
            pGPGGA = new integrated_navigation_driver::NMEA_GPGGA();
            parseGPGGA2msg(nmea_str, *pGPGGA);
            if (!pGPGGA)
            {
                ROS_WARN("%s", nmea_msg->sentence.c_str());
            }
            else
            {
//                ROS_WARN("pub GPGGA");
                pGPGGA->header.stamp = nmea_msg->header.stamp;
                pubGPGGA(pGPGGA);
            }
        }

        if (use_NVSTD && detected_NVSTD)
        {
            pNVSTD = new integrated_navigation_driver::NMEA_NVSTD();
            parseNVSTD2msg(nmea_str, *pNVSTD);
            if (!pNVSTD)
            {
                ROS_WARN("%s", nmea_msg->sentence.c_str());
            }
            else
            {
                pGPGGA->header.stamp = nmea_msg->header.stamp;
                pubNVSTD(pNVSTD);
            }
        }

        if (use_GPCHC && detected_GPCHC)
        {
            pGPCHC = new integrated_navigation_driver::NMEA_GPCHC();
            parseGPCHC2msg(nmea_str, *pGPCHC);
            if (!pGPCHC)
            {
                ROS_WARN("%s", nmea_msg->sentence.c_str());
            }
            else
            {
                pGPCHC->header.stamp = nmea_msg->header.stamp;
                pubGPCHC(pGPCHC);
            }
        }

        delete(pGPFPD);
        delete(pGTIMU);
        delete(pGPGGA);
        delete(pNVSTD);
        delete(pGPCHC);

    }

    static void parseGPFPD2msg(const std::string& msg_in, integrated_navigation_driver::NMEA_GPFPD& msg_out)
    {
        std::vector<std::string> vSub;
        boost::split(vSub, msg_in, boost::is_any_of(","), boost::token_compress_on);

        if (vSub.size() < 16)
        {
            ROS_WARN("Can not Parse NMEA GPFPD Message! Maybe too much empty");
            return;
        }

        msg_out.header.frame_id = "GPFPD";
        char *ptr_t;

        // 时间
        msg_out.gnss_week = boost::numeric_cast<std::uint16_t>(strtoul(vSub[1].c_str(), &ptr_t, 10));
        msg_out.gnss_time = boost::numeric_cast<double>(strtof64(vSub[2].c_str(), &ptr_t));

        // 航向角
        msg_out.heading = boost::numeric_cast<double>(strtof64(vSub[3].c_str(), &ptr_t));
        msg_out.pitch = boost::numeric_cast<double>(strtof64(vSub[4].c_str(), &ptr_t));
        msg_out.roll = boost::numeric_cast<double>(strtof64(vSub[5].c_str(), &ptr_t));

        // 定位
        msg_out.latitude = boost::numeric_cast<double>(strtof64(vSub[6].c_str(), &ptr_t));
        msg_out.longitude = boost::numeric_cast<double>(strtof64(vSub[7].c_str(), &ptr_t));
        msg_out.altitude = boost::numeric_cast<double>(strtof64(vSub[8].c_str(), &ptr_t));
        // 东北天速度
        msg_out.velocity_e = boost::numeric_cast<double>(strtof64(vSub[9].c_str(), &ptr_t));
        msg_out.velocity_n = boost::numeric_cast<double>(strtof64(vSub[10].c_str(), &ptr_t));
        msg_out.velocity_u = boost::numeric_cast<double>(strtof64(vSub[11].c_str(), &ptr_t));

        msg_out.baseline = boost::numeric_cast<double>(strtof64(vSub[12].c_str(), &ptr_t));
        // 卫星数
        msg_out.nsv1 = boost::numeric_cast<std::uint16_t>(strtoul(vSub[13].c_str(), &ptr_t, 10));
        msg_out.nsv2 = boost::numeric_cast<std::uint16_t>(strtoul(vSub[14].c_str(), &ptr_t, 10));
        // 组合导航状态
        if (vSub[15].length() > 1){
            boost::array<char, 2> status = {vSub[15][0], vSub[15][1]};
            msg_out.status = status;
        }

    }

    static void parseGTIMU2msg(const std::string& msg_in, integrated_navigation_driver::NMEA_GTIMU& msg_out)
    {
        std::vector<std::string> vSub;
        boost::split(vSub, msg_in, boost::is_any_of(","), boost::token_compress_on);

        if (vSub.size() < 10)
        {
            ROS_WARN("Can not Parse NMEA GTIMU Message!");
            return;
        }

        msg_out.header.frame_id = "GTIMU";
        char *ptr_t;

        // 时间
        msg_out.gnss_week = boost::numeric_cast<std::uint16_t>(strtoul(vSub[1].c_str(), &ptr_t, 10));
        msg_out.gnss_time = boost::numeric_cast<double>(strtof64(vSub[2].c_str(), &ptr_t));

        //
        msg_out.gyroscope_x = boost::numeric_cast<double>(strtof64(vSub[3].c_str(), &ptr_t));
        msg_out.gyroscope_y = boost::numeric_cast<double>(strtof64(vSub[4].c_str(), &ptr_t));
        msg_out.gyroscope_z = boost::numeric_cast<double>(strtof64(vSub[5].c_str(), &ptr_t));

        //
        msg_out.acceleration_x = boost::numeric_cast<double>(strtof64(vSub[6].c_str(), &ptr_t));
        msg_out.acceleration_y = boost::numeric_cast<double>(strtof64(vSub[7].c_str(), &ptr_t));
        msg_out.acceleration_z = boost::numeric_cast<double>(strtof64(vSub[8].c_str(), &ptr_t));

        //
        msg_out.temperature = boost::numeric_cast<double>(strtof64(vSub[9].c_str(), &ptr_t));

    }

    static void parseGPGGA2msg(const std::string& msg_in, integrated_navigation_driver::NMEA_GPGGA& msg_out)
    {
        std::vector<std::string> vSub;
        boost::split(vSub, msg_in, boost::is_any_of(","), boost::token_compress_on);

        if (vSub.size() < 12)
        {
            ROS_WARN("Can not Parse NMEA GPGGA Message!");
            return;
        }

        msg_out.header.frame_id = "GPGGA";
        char *ptr_t;

        // 时间
        msg_out.utc_time.append(vSub[1]);


        //

        msg_out.latitude.append(vSub[2]);
        if (vSub[3].c_str() == "S" || vSub[3].c_str() == "s")
        {
            msg_out.latitude_direction = integrated_navigation_driver::NMEA_GPGGA::LATITUDE_SOUTH;
        }
        else
        {
            msg_out.latitude_direction = integrated_navigation_driver::NMEA_GPGGA::LATITUDE_NORTH;
        }
        msg_out.longitude.append(vSub[4]);
        if (vSub[5].c_str() == "W" || vSub[5].c_str() == "w")
        {
            msg_out.longitude_direction = integrated_navigation_driver::NMEA_GPGGA::LONGITUDE_WEST;
        }
        else
        {
            msg_out.longitude_direction = integrated_navigation_driver::NMEA_GPGGA::LONGITUDE_EAST;
        }

        switch (boost::numeric_cast<std::uint8_t>(strtoul(vSub[6].c_str(), &ptr_t, 10)))
        {
            case 0:
                msg_out.gnss_quality = integrated_navigation_driver::NMEA_GPGGA::GNSS_FIX_INVALID;
                break;
            case 1:
                msg_out.gnss_quality = integrated_navigation_driver::NMEA_GPGGA::GNSS_SINGLE_POINT;
                break;
            case 2:
                msg_out.gnss_quality = integrated_navigation_driver::NMEA_GPGGA::GNSS_PSEUDORANGE_DIFFERENTIAL;
                break;
            case 4:
                msg_out.gnss_quality = integrated_navigation_driver::NMEA_GPGGA::GNSS_FIX_SOLUTION;
                break;
            case 5:
                msg_out.gnss_quality = integrated_navigation_driver::NMEA_GPGGA::GNSS_FLOATING_SOLUTION;
                break;
            case 6:
                msg_out.gnss_quality = integrated_navigation_driver::NMEA_GPGGA::GNSS_RECKONING_MODE;
                break;
            case 7:
                msg_out.gnss_quality = integrated_navigation_driver::NMEA_GPGGA::GNSS_MANUAL_FIXED;
                break;
            case 8:
                msg_out.gnss_quality = integrated_navigation_driver::NMEA_GPGGA::GNSS_SIMULATOR_MODE;
                break;
            case 9:
                msg_out.gnss_quality = integrated_navigation_driver::NMEA_GPGGA::GNSS_WAAS;
                break;
            default:
                msg_out.gnss_quality = integrated_navigation_driver::NMEA_GPGGA::GNSS_FIX_INVALID;
        }

        msg_out.number_of_using_satellites = boost::numeric_cast<std::uint8_t>(strtoul(vSub[7].c_str(), &ptr_t, 10));

        msg_out.horizontal_dilution_of_precision = boost::numeric_cast<double>(strtof64(vSub[8].c_str(), &ptr_t));

        msg_out.altitude = boost::numeric_cast<double>(strtof64(vSub[9].c_str(), &ptr_t));
        if (vSub[10].c_str() == "KM" || vSub[10].c_str() == "km")
        {
            msg_out.altitude_units = integrated_navigation_driver::NMEA_GPGGA::UNITS_KILOMETER;
        }
        else if (vSub[10].c_str() == "DM" || vSub[10].c_str() == "dm")
        {
            msg_out.altitude_units = integrated_navigation_driver::NMEA_GPGGA::UNITS_DECIMETER;
        }
        else if (vSub[10].c_str() == "CM" || vSub[10].c_str() == "cm")
        {
            msg_out.altitude_units = integrated_navigation_driver::NMEA_GPGGA::UNITS_CENTIMETER;
        }
        else
        {
            msg_out.altitude_units = integrated_navigation_driver::NMEA_GPGGA::UNITS_METER;
        }

        msg_out.undulation = boost::numeric_cast<double>(strtof64(vSub[11].c_str(), &ptr_t));
        if (vSub[12].c_str() == "KM" || vSub[12].c_str() == "km")
        {
            msg_out.undulation_units = integrated_navigation_driver::NMEA_GPGGA::UNITS_KILOMETER;
        }
        else if (vSub[12].c_str() == "DM" || vSub[12].c_str() == "dm")
        {
            msg_out.undulation_units = integrated_navigation_driver::NMEA_GPGGA::UNITS_DECIMETER;
        }
        else if (vSub[12].c_str() == "CM" || vSub[12].c_str() == "cm")
        {
            msg_out.undulation_units = integrated_navigation_driver::NMEA_GPGGA::UNITS_CENTIMETER;
        }
        else
        {
            msg_out.undulation_units = integrated_navigation_driver::NMEA_GPGGA::UNITS_METER;
        }

        if (msg_out.gnss_quality > 1 && msg_out.gnss_quality < 6){
            msg_out.differential_age = boost::numeric_cast<std::uint8_t>(strtoul(vSub[13].c_str(), &ptr_t, 10));
            msg_out.differential_station_id.append(vSub[14]);
        }

    }

    static void parseNVSTD2msg(const std::string& msg_in, integrated_navigation_driver::NMEA_NVSTD& msg_out)
    {
        std::vector<std::string> vSub;
        boost::split(vSub, msg_in, boost::is_any_of(","), boost::token_compress_on);

        if (vSub.size() < 11)
        {
            ROS_WARN("Can not Parse NMEA NVSTD Message!");
            return;
        }

        msg_out.header.frame_id = "NVSTD";
        char *ptr_t;

        //
        msg_out.heading_std = boost::numeric_cast<double>(strtof64(vSub[1].c_str(), &ptr_t));
        msg_out.pitch_std = boost::numeric_cast<double>(strtof64(vSub[2].c_str(), &ptr_t));
        msg_out.roll_std = boost::numeric_cast<double>(strtof64(vSub[2].c_str(), &ptr_t));

        //
        msg_out.latitude_std = boost::numeric_cast<double>(strtof64(vSub[2].c_str(), &ptr_t));
        msg_out.longitude_std = boost::numeric_cast<double>(strtof64(vSub[2].c_str(), &ptr_t));
        msg_out.altitude_std = boost::numeric_cast<double>(strtof64(vSub[2].c_str(), &ptr_t));

        //
        msg_out.velocity_e_std = boost::numeric_cast<double>(strtof64(vSub[2].c_str(), &ptr_t));
        msg_out.velocity_n_std = boost::numeric_cast<double>(strtof64(vSub[2].c_str(), &ptr_t));
        msg_out.velocity_u_std = boost::numeric_cast<double>(strtof64(vSub[2].c_str(), &ptr_t));

    }

    static void parseGPCHC2msg(const std::string& msg_in, integrated_navigation_driver::NMEA_GPCHC& msg_out)
    {
        std::vector<std::string> vSub;
        boost::split(vSub, msg_in, boost::is_any_of(","), boost::token_compress_on);

        if (vSub.size() < 24)
        {
            ROS_WARN("Can not Parse NMEA GPCHC Message!");
            return;
        }

        msg_out.header.frame_id = "GPCHC";
        char *ptr_t;

        // 时间
        msg_out.gnss_week = boost::numeric_cast<std::uint16_t>(strtoul(vSub[1].c_str(), &ptr_t, 10));
        msg_out.gnss_time = boost::numeric_cast<double>(strtof64(vSub[2].c_str(), &ptr_t));

        // 航向角
        msg_out.heading = boost::numeric_cast<double>(strtof64(vSub[3].c_str(), &ptr_t));
        msg_out.pitch = boost::numeric_cast<double>(strtof64(vSub[4].c_str(), &ptr_t));
        msg_out.roll = boost::numeric_cast<double>(strtof64(vSub[5].c_str(), &ptr_t));

        // 陀螺仪偏角
        msg_out.gyroscope_x = boost::numeric_cast<double>(strtof64(vSub[6].c_str(), &ptr_t));
        msg_out.gyroscope_y = boost::numeric_cast<double>(strtof64(vSub[7].c_str(), &ptr_t));
        msg_out.gyroscope_z = boost::numeric_cast<double>(strtof64(vSub[8].c_str(), &ptr_t));

        // 陀螺仪加表
        msg_out.acceleration_x = boost::numeric_cast<double>(strtof64(vSub[9].c_str(), &ptr_t));
        msg_out.acceleration_y = boost::numeric_cast<double>(strtof64(vSub[10].c_str(), &ptr_t));
        msg_out.acceleration_z = boost::numeric_cast<double>(strtof64(vSub[11].c_str(), &ptr_t));

        // 定位
        msg_out.latitude = boost::numeric_cast<double>(strtof64(vSub[12].c_str(), &ptr_t));
        msg_out.longitude = boost::numeric_cast<double>(strtof64(vSub[13].c_str(), &ptr_t));
        msg_out.altitude = boost::numeric_cast<double>(strtof64(vSub[14].c_str(), &ptr_t));

        // 东北天速度
        msg_out.velocity_e = boost::numeric_cast<double>(strtof64(vSub[15].c_str(), &ptr_t));
        msg_out.velocity_n = boost::numeric_cast<double>(strtof64(vSub[16].c_str(), &ptr_t));
        msg_out.velocity_u = boost::numeric_cast<double>(strtof64(vSub[17].c_str(), &ptr_t));

        // 速度
        msg_out.velocity = boost::numeric_cast<double>(strtof64(vSub[18].c_str(), &ptr_t));

        // 卫星数
        msg_out.nsv1 = boost::numeric_cast<std::uint16_t>(strtoul(vSub[19].c_str(), &ptr_t, 10));
        msg_out.nsv2 = boost::numeric_cast<std::uint16_t>(strtoul(vSub[20].c_str(), &ptr_t, 10));

        // 组合导航状态
        boost::array<char, 2> status = {vSub[21][0], vSub[21][1]};
        msg_out.status = status;

        // 差分延时
        msg_out.age = boost::numeric_cast<std::uint8_t>(strtoul(vSub[22].c_str(), &ptr_t, 10));

        msg_out.warming = boost::numeric_cast<std::uint8_t>(strtoul(vSub[23].c_str(), &ptr_t, 10));
    }


    void pubGPFPD(const integrated_navigation_driver::NMEA_GPFPD* msg)
    {
        GPFPD_pub.publish(*msg);
    }

    void pubGTIMU(const integrated_navigation_driver::NMEA_GTIMU* msg)
    {
        GTIMU_pub.publish(*msg);
    }

    void pubGPGGA(const integrated_navigation_driver::NMEA_GPGGA* msg)
    {
        GPGGA_pub.publish(*msg);
    }

    void pubNVSTD(const integrated_navigation_driver::NMEA_NVSTD* msg)
    {
        NVSTD_pub.publish(*msg);
    }

    void pubGPCHC(const integrated_navigation_driver::NMEA_GPCHC* msg)
    {
        GPCHC_pub.publish(*msg);
    }

private:
    ros::NodeHandle nh_;

    ros::Subscriber nmea_sentense_sub;

    ros::Publisher GPFPD_pub;
    ros::Publisher GTIMU_pub;
    ros::Publisher GPGGA_pub;
    ros::Publisher NVSTD_pub;
    ros::Publisher GPCHC_pub;

    bool use_GPFPD = false;
    bool use_GTIMU = false;
    bool use_GPGGA = false;
    bool use_NVSTD = false;
    bool use_GPCHC = false;

};







int main(int argc, char** argv)
{
    ros::init(argc, argv, "nmea_parser");
    NMEA_Parser parser;
    ros::spin();
    return 0;
}

#endif