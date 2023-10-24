#ifndef _ROSTEST_HPP_
#define _ROSTEST_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nmea_msgs/Sentence.h>

#include "integrated_navigation_reader/SPANLog_INSPVAXA.h"

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

        nh_local.getParam("INSPVAX", use_INSPVAXA);

        spanlog_sentense_sub = nh_.subscribe("/spanlog/sentence", 1, &NMEA_Parser::SpanlogReader, this, ros::TransportHints().tcpNoDelay());

        if (use_INSPVAXA)
        {
            INSPVAXA_pub = nh_.advertise<integrated_navigation_reader::SPANLog_INSPVAXA>("/spanlog/inspvaxa", 10);
        }

    }

    void SpanlogReader(const nmea_msgs::Sentence::ConstPtr& nmea_msg)
    {

        bool detected_INSPVAXA = false;

        integrated_navigation_reader::SPANLog_INSPVAXA * pINSPVAX = nullptr;


        const std::string nmea_str = nmea_msg->sentence;
        if (nmea_str.find("#INSPVAXA") != std::string::npos || nmea_str.find("#inspvaxa") != std::string::npos)
        {
            detected_INSPVAXA = true;
        }



        if (use_INSPVAXA && detected_INSPVAXA)
        {
            pINSPVAX = new integrated_navigation_reader::SPANLog_INSPVAXA ();
            parseINSPVAXA2msg(nmea_str, *pINSPVAX);
            if (!pINSPVAX)
            {
                ROS_WARN("%s", nmea_msg->sentence.c_str());
            }
            else
            {
                pINSPVAX->header.stamp = nmea_msg->header.stamp;
                pubINSPVAX(pINSPVAX);
            }
        }

        delete(pINSPVAX);


    }

    static void parseINSPVAXA2msg(const std::string& msg_in, integrated_navigation_reader::SPANLog_INSPVAXA & msg_out)
    {
        std::vector<std::string> vSub;
        boost::split(vSub, msg_in, boost::is_any_of(",,|;"), boost::token_compress_on);

//        std::cout << vSub[0].c_str() << " " << vSub[1].c_str() << " " << vSub[2].c_str() << " " << vSub[3].c_str() << " "
//        << vSub[4].c_str() << " " << vSub[5].c_str() << " " << vSub[6].c_str() << " " << vSub[7].c_str() << " "
//        << vSub[8].c_str() << " " << vSub[9].c_str() << " " << vSub[10].c_str() << " " << vSub[11].c_str() << " "
//        << vSub[12].c_str() << " " << vSub[13].c_str() << " " << vSub[14].c_str() << " " << vSub[15].c_str() << " "
//        << vSub[16].c_str() << " " << std::endl;

        if (vSub.size() < 32)
        {
            ROS_WARN("Can not Parse Span log INSPVAXA Message!");
            return;
        }
//
        msg_out.header.frame_id = "INSPVAXA";
        char *ptr_t;

        // 时间
        msg_out.log_header.week = boost::numeric_cast<std::uint16_t>(strtoul(vSub[5].c_str(), &ptr_t, 10));
        msg_out.log_header.seconds = boost::numeric_cast<double>(strtof64(vSub[6].c_str(), &ptr_t));

        // 定位
        msg_out.latitude = boost::numeric_cast<double>(strtof64(vSub[12].c_str(), &ptr_t));
        msg_out.longitude = boost::numeric_cast<double>(strtof64(vSub[13].c_str(), &ptr_t));
        msg_out.height = boost::numeric_cast<double>(strtof64(vSub[14].c_str(), &ptr_t));
        msg_out.undulation = boost::numeric_cast<double>(strtof64(vSub[15].c_str(), &ptr_t));

        // 东北天速度
        msg_out.north_velocity = boost::numeric_cast<double>(strtof64(vSub[16].c_str(), &ptr_t));
        msg_out.east_velocity = boost::numeric_cast<double>(strtof64(vSub[17].c_str(), &ptr_t));
        msg_out.up_velocity = boost::numeric_cast<double>(strtof64(vSub[18].c_str(), &ptr_t));

        // 航向角
        msg_out.roll = boost::numeric_cast<double>(strtof64(vSub[19].c_str(), &ptr_t));
        msg_out.pitch = boost::numeric_cast<double>(strtof64(vSub[20].c_str(), &ptr_t));
        msg_out.azimuth = boost::numeric_cast<double>(strtof64(vSub[21].c_str(), &ptr_t));

        msg_out.latitude_std = boost::numeric_cast<double>(strtof64(vSub[22].c_str(), &ptr_t));
        msg_out.longitude_std = boost::numeric_cast<double>(strtof64(vSub[23].c_str(), &ptr_t));
        msg_out.height_std = boost::numeric_cast<double>(strtof64(vSub[24].c_str(), &ptr_t));

        msg_out.north_velocity_std = boost::numeric_cast<double>(strtof64(vSub[25].c_str(), &ptr_t));
        msg_out.east_velocity_std = boost::numeric_cast<double>(strtof64(vSub[26].c_str(), &ptr_t));
        msg_out.up_velocity_std = boost::numeric_cast<double>(strtof64(vSub[27].c_str(), &ptr_t));

        msg_out.roll_std = boost::numeric_cast<double>(strtof64(vSub[28].c_str(), &ptr_t));
        msg_out.pitch_std = boost::numeric_cast<double>(strtof64(vSub[29].c_str(), &ptr_t));
        msg_out.azimuth_std = boost::numeric_cast<double>(strtof64(vSub[30].c_str(), &ptr_t));

        msg_out.time_since_update = boost::numeric_cast<uint16_t>(strtof64(vSub[32].c_str(), &ptr_t));

    }


    void pubINSPVAX(const integrated_navigation_reader::SPANLog_INSPVAXA * msg)
    {
        INSPVAXA_pub.publish(*msg);
    }



private:
    ros::NodeHandle nh_;

    ros::Subscriber spanlog_sentense_sub;

    ros::Publisher INSPVAXA_pub;

    bool use_INSPVAXA = false;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nmea_parser");
    NMEA_Parser parser;
    ros::spin();
    return 0;
}

#endif