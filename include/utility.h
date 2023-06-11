//
// Created by lin on 23-6-8.
//

#ifndef INTEGRATED_NAV_DRIVER_GENERATENAVSATFIXMSG_H
#define INTEGRATED_NAV_DRIVER_GENERATENAVSATFIXMSG_H

#endif //INTEGRATED_NAV_DRIVER_GENERATENAVSATFIXMSG_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include "integrated_navigation_driver/NMEA_GPFPD.h"
#include "integrated_navigation_driver/NMEA_GPCHC.h"
#include "integrated_navigation_driver/NMEA_GPGGA.h"

#include <cmath>
#include <ctime>
#include <cstring>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <vector>
#include <sstream>

// Utility Func
long int getUTCDate2second()
{
    time_t t;
    struct tm *utc_date;
    t = time(NULL);
    utc_date = gmtime(&t);
    if (utc_date == NULL) {
        perror("getUTCDat error");
        return 0;
    }

    utc_date->tm_hour = 0;
    utc_date->tm_min = 0;
    utc_date->tm_sec = 0;
//        ROS_WARN("%d-%d-%d",utc_date->tm_year,utc_date->tm_mon,utc_date->tm_mday);
    time_t sec_date = std::mktime(utc_date);
//        ROS_WARN("%d-%d-%d",utc_date->tm_year,utc_date->tm_mon,utc_date->tm_mday);
//        ROS_WARN("%d",sec_date);
//        ROS_WARN("%s", ctime(&sec_date));
    return sec_date;
}

double transfer2MeterUnit(double ori_num, const std::string& ori_unit)
{
    double cal_num;
    if (ori_unit.c_str() == "KM" || ori_unit.c_str() == "km"){
        cal_num = ori_num * 1e3;
    } else if (ori_unit.c_str() == "DM" || ori_unit.c_str() == "dm") {
        cal_num = ori_num / 10;
    } else if (ori_unit.c_str() == "CM" || ori_unit.c_str() == "cm") {
        cal_num = ori_num / 100;
    } else{
        cal_num = ori_num;
    }
    return cal_num;
};

// transfer to standard UTC: add 1980-1-6 to 1970-1-1 days
uint64_t GNSSUTCWeekAndTime2Nanocecond(unsigned int week_num, double time_second, int leap_second_GNSSUTC)
{
    return ((week_num * 7 + 3657) * 24 * 60 * 60 + time_second - leap_second_GNSSUTC) * 1e9;
}

double DEG2RAD(double x)
{
    return x/180.0 * M_PI;
}
double RAD2DEG(double x)
{
    return x / M_PI * 180.0;
}