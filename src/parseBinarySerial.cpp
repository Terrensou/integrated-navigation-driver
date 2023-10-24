//
// Created by lin on 23-7-5.
//

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <nmea_msgs/Sentence.h>
#include "integrated_navigation_reader/Log_ExtendedSolutionStatus.h"
#include "integrated_navigation_reader/Log_SolutionSource.h"
#include "integrated_navigation_reader/Log_DUALANTENNAHEADINGB.h"
#include "integrated_navigation_reader/SPANLog_ExtendedSolutionStatus.h"
#include "integrated_navigation_reader/SPANLog_INSPVAXB.h"
#include "integrated_navigation_reader/SPANLog_BESTGNSSPOSB.h"
#include "integrated_navigation_reader/SPANLog_BESTGNSSVELB.h"
#include "integrated_navigation_reader/SPANLog_CORRIMUDATAB.h"
#include "integrated_navigation_reader/SPANLog_RAWIMUB.h"
#include "utility.h"

class Binary_Serial_Parser {
public:
    Binary_Serial_Parser() {
        is_little_end = isNativeLittleEndian();

        uint8_t inspvaxb_header[] = {0xAA, 0x44, 0x12, 0x1C, 0xB9, 0x05};
        inspvaxb = bytes2String(inspvaxb_header, 6);
        uint8_t bestgnssposb_header[] = {0xAA, 0x44, 0x12, 0x1C, 0x95, 0x05};
        bestgnssposb = bytes2String(bestgnssposb_header, 6);
        uint8_t bestgnssvelb_header[] = {0xAA, 0x44, 0x12, 0x1C, 0x96, 0x05};
        bestgnssvelb = bytes2String(bestgnssvelb_header, 6);
        uint8_t corrimudatab_header[] = {0xAA, 0x44, 0x12, 0x1C, 0x2C, 0x03};
        corrimudatab = bytes2String(corrimudatab_header, 6);
        uint8_t rawimub_header[] = {0xAA, 0x44, 0x12, 0x1C, 0x0C, 0x01};
        rawimub = bytes2String(rawimub_header, 6);
        uint8_t dualantennaheadingb_header[] = {0xAA, 0x44, 0x12, 0x1C, 0xCB, 0x03};
        dualantennaheadingb = bytes2String(dualantennaheadingb_header, 6);
        uint8_t gpfps_bin_header[] = {0xAA, 0x55, 0x04};
        gpfps_bin = bytes2String(gpfps_bin_header, 3);
        uint8_t gtimu_bin_header[] = {0xAA, 0x55, 0x05};
        gtimu_bin = bytes2String(gtimu_bin_header, 3);

        ros::NodeHandle nh_local("~");
        nh_local.getParam("INSPVAXB", use_INSPVAXB);
        nh_local.getParam("BESTGNSSPOSB", use_BESTGNSSPOSB);
        nh_local.getParam("BESTGNSSVELB", use_BESTGNSSVELB);
        nh_local.getParam("CORRIMUDATAB", use_CORRIMUDATAB);
        nh_local.getParam("RAWIMUB", use_RAWIMUB);
        nh_local.getParam("DUALANTENNAHEADINGB", use_DUALANTENNAHEADINGB);
        nh_local.getParam("GPFPS", use_GPFPS);
        nh_local.getParam("GPFPS_BIN", use_GPFPS_BIN);
        nh_local.getParam("GTIMU", use_GTIMU);
        nh_local.getParam("GTIMU_BIN", use_GTIMU_BIN);
        nh_local.getParam("GPGGA", use_GPGGA);
        nh_local.getParam("GPFPD", use_GPFPD);
        nh_local.getParam("GPCHC", use_GPCHC);

        nmea_pub = nh_.advertise<nmea_msgs::Sentence>("/nmea/sentence", 10);
        spanlog_pub = nh_.advertise<nmea_msgs::Sentence>("/spanlog/sentence", 10);

        if (use_INSPVAXB) {
            INSPVAXB_pub = nh_.advertise<integrated_navigation_reader::SPANLog_INSPVAXB>("/spanlog/inspvaxb", 10);
        }

        if (use_BESTGNSSPOSB) {
            BESTGNSSPOSB_pub = nh_.advertise<integrated_navigation_reader::SPANLog_BESTGNSSPOSB>(
                    "/spanlog/bestgnssposb", 10);
        }

        if (use_BESTGNSSVELB) {
            BESTGNSSVELB_pub = nh_.advertise<integrated_navigation_reader::SPANLog_BESTGNSSVELB>(
                    "/spanlog/bestgnssvelb", 10);
        }

        if (use_CORRIMUDATAB) {
            CORRIMUDATAB_pub = nh_.advertise<integrated_navigation_reader::SPANLog_CORRIMUDATAB>(
                    "/spanlog/corrimudatab", 10);
        }

        if (use_RAWIMUB) {
            RAWIMUB_pub = nh_.advertise<integrated_navigation_reader::SPANLog_RAWIMUB>("/spanlog/rawimub", 10);
        }

        if (use_DUALANTENNAHEADINGB) {
            DUALANTENNAHEADING_pub = nh_.advertise<integrated_navigation_reader::Log_DUALANTENNAHEADINGB>("/spanlog/dualantennaheadingb", 10);
        }

        std::string port;
        int baudrate = 115200;
        int timeout_msec = 1000;

        nh_.getParam("Nmea_serial_reader_node/port", port);
        nh_.getParam("Nmea_serial_reader_node/baud", baudrate);
        nh_.getParam("Nmea_serial_reader_node/timeout", timeout_msec);


        // open serial
        try {

            serialHandle.setPort(port.c_str());
            serialHandle.setBaudrate(baudrate);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_msec);

            serialHandle.setTimeout(timeout);
            serialHandle.open();
        } catch (serial::IOException &ioe) {
            ROS_ERROR_STREAM("Unable to open serial port: " << port << "." << std::endl);
            return;
        }

        // detect serial open status
        if (serialHandle.isOpen()) {
            ROS_INFO_STREAM("Serial Port " << port << " initialized." << std::endl);
        }

    }

    ~Binary_Serial_Parser() {
        serialHandle.flush();
        serialHandle.close();
    }

    void parseStream() {


        std::string buffer;
        std::string data;
        ros::Time now_time = ros::Time::now();
        ros::Time *p_time = &now_time;

        // TODO: use tire tree to accelerate find method
        while (ros::ok() && serialHandle.isOpen()) {
            // 从数据流中读取一个字节到缓冲区
            buffer += serialHandle.read(1); // 使用read函数获取一个字节的数据
            *p_time = ros::Time::now();

//            std::cout << buffer << std::endl;

            if (buffer.find(inspvaxb) != std::string::npos && use_INSPVAXB) {
//                std::cout << "Detected: inspvaxb" << std::endl;
                parseINSPVAXB(p_time);
                buffer.clear();
                continue;
            }

            if (buffer.find(bestgnssposb) != std::string::npos && use_BESTGNSSPOSB) {
//                std::cout << "Detected: bestgnssposb" << std::endl;
                parseBESTGNSSPOSB(p_time);
                buffer.clear();
                continue;
            }

            if (buffer.find(bestgnssvelb) != std::string::npos && use_BESTGNSSVELB) {
//                std::cout << "Detected: bestgnssvelb" << std::endl;
                parseBESTGNSSVELB(p_time);
                buffer.clear();
                continue;
            }

            if (buffer.find(corrimudatab) != std::string::npos && use_CORRIMUDATAB) {
//                std::cout << "Detected: bestgnssvelb" << std::endl;
                parseCORRIMUDATAB(p_time);
                buffer.clear();
                continue;
            }

            if (buffer.find(rawimub) != std::string::npos && use_RAWIMUB) {
//                std::cout << "Detected: rawimub" << std::endl;
                parseRAWIMUB(p_time);
                buffer.clear();
                continue;
            }

            if (buffer.find(dualantennaheadingb) != std::string::npos && use_DUALANTENNAHEADINGB) {
//                std::cout << "Detected: dualantennaheadingb" << std::endl;
                parseDUALANTENNAHEADINGB(p_time);
                buffer.clear();
                continue;
            }

            if (buffer.find(gpfps_bin) != std::string::npos && use_GPFPS_BIN) {
//                std::cout << "Detected: gpfps_bin" << std::endl;
                parseGPFPS_BIN(p_time);
                buffer.clear();
                continue;
            }
            if (buffer.find(gpfps) != std::string::npos && use_GPFPS) {
//                std::cout << "Detected: gpfps" << std::endl;
                parseGPFPS(p_time);
                buffer.clear();
                continue;
            }

            if (buffer.find(gtimu_bin) != std::string::npos && use_GTIMU_BIN) {
//                std::cout << "Detected: gtimu_bin" << std::endl;
                parseGTIMU_BIN_Huace(p_time);
                buffer.clear();
                continue;
            }
            if (buffer.find(gtimu) != std::string::npos && use_GTIMU) {
//                std::cout << "Detected: gtimu" << std::endl;
                parseGTIMU(p_time);
                buffer.clear();
                continue;
            }
            if (buffer.find(gpgga) != std::string::npos && use_GPGGA) {
//                std::cout << "Detected: gtimu" << std::endl;
                parseGPGGA(p_time);
                buffer.clear();
                continue;
            }
            if (buffer.find(gpfpd) != std::string::npos && use_GPFPD) {
//                std::cout << "Detected: gtimu" << std::endl;
                parseGPFPD(p_time);
                buffer.clear();
                continue;
            }
            if (buffer.find(gpchc) != std::string::npos && use_GPCHC) {
//                std::cout << "Detected: gtimu" << std::endl;
                parseGPCHC(p_time);
                buffer.clear();
                continue;
            }

        }
    }

    void pubNMEA(const nmea_msgs::Sentence *msg) {
        nmea_pub.publish(*msg);
    }

    void pubSpanLog(const nmea_msgs::Sentence *msg) {
        spanlog_pub.publish(*msg);
    }

    void pubINSPVAXB(const integrated_navigation_reader::SPANLog_INSPVAXB msg) {
        INSPVAXB_pub.publish(msg);
    }

    void pubBESTGNSSPOSB(const integrated_navigation_reader::SPANLog_BESTGNSSPOSB msg) {
        BESTGNSSPOSB_pub.publish(msg);
    }

    void pubBESTGNSSVELB(const integrated_navigation_reader::SPANLog_BESTGNSSVELB msg) {
        BESTGNSSVELB_pub.publish(msg);
    }

    void pubCORRIMUDATAB(const integrated_navigation_reader::SPANLog_CORRIMUDATAB msg) {
        CORRIMUDATAB_pub.publish(msg);
    }

    void pubRAWIMUB(const integrated_navigation_reader::SPANLog_RAWIMUB msg) {
        RAWIMUB_pub.publish(msg);
    }

    void pubDUALANTENNAHEADINGB(const integrated_navigation_reader::Log_DUALANTENNAHEADINGB msg) {
        DUALANTENNAHEADING_pub.publish(msg);
    }


private:

    ros::NodeHandle nh_;
    ros::Publisher nmea_pub;
    ros::Publisher spanlog_pub;
    ros::Publisher INSPVAXB_pub;
    ros::Publisher BESTGNSSPOSB_pub;
    ros::Publisher BESTGNSSVELB_pub;
    ros::Publisher CORRIMUDATAB_pub;
    ros::Publisher RAWIMUB_pub;
    ros::Publisher DUALANTENNAHEADING_pub;

    serial::Serial serialHandle;

    bool is_little_end = false;
    bool use_INSPVAXB = false;
    bool use_BESTGNSSPOSB = false;
    bool use_BESTGNSSVELB = false;
    bool use_CORRIMUDATAB = false;
    bool use_RAWIMUB = false;
    bool use_DUALANTENNAHEADINGB = false;
    bool use_GPFPS = false;
    bool use_GPFPS_BIN = false;
    bool use_GTIMU = false;
    bool use_GTIMU_BIN = false;
    bool use_GPGGA = false;
    bool use_GPFPD = false;
    bool use_GPCHC = false;

    std::string inspvaxb;
    std::string bestgnssposb;
    std::string bestgnssvelb;
    std::string corrimudatab;
    std::string rawimub;
    std::string dualantennaheadingb;
    std::string gpfps_bin;
    const std::string gpfps = "$GPFPS";
    std::string gtimu_bin;
    const std::string gtimu = "$GTIMU";
    const std::string gpgga = "$GPGGA";
    const std::string gpfpd = "$GPFPD";
    const std::string gpchc = "$GPCHC";

    void parseINSPVAXB(const ros::Time *now) {

        integrated_navigation_reader::SPANLog_INSPVAXB msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "INSPVAX";

        // get bin chars
        std::string data;
        serialHandle.read(data, 152);
//        std::cout << string2Hex(data.substr(0, 22)) << std::endl;
        // parse bin inspvax
        integrated_navigation_reader::BinaryHeader log_header;
        parseBinaryHeader(&log_header, data.substr(0, 22));
        msg_out.log_header = log_header;

        msg_out.ins_status.status = byte4toNumber<uint32_t>(data.substr(22, 4));
        msg_out.position_type.type = byte4toNumber<uint32_t>(data.substr(26, 4));

        msg_out.latitude = byte8toNumber<double_t>(data.substr(30, 8));
        msg_out.longitude = byte8toNumber<double_t>(data.substr(38, 8));
        msg_out.height = byte8toNumber<double_t>(data.substr(46, 8));
        msg_out.undulation = byte4toNumber<float_t>(data.substr(54, 4));

        msg_out.north_velocity = byte8toNumber<double_t>(data.substr(58, 8));
        msg_out.east_velocity = byte8toNumber<double_t>(data.substr(66, 8));
        msg_out.up_velocity = byte8toNumber<double_t>(data.substr(74, 8));

        msg_out.roll = byte8toNumber<double_t>(data.substr(82, 8));
        msg_out.pitch = byte8toNumber<double_t>(data.substr(90, 8));
        msg_out.azimuth = byte8toNumber<double_t>(data.substr(98, 8));

        msg_out.latitude_std = byte4toNumber<float_t>(data.substr(106, 4));
        msg_out.longitude_std = byte4toNumber<float_t>(data.substr(110, 4));
        msg_out.height_std = byte4toNumber<float_t>(data.substr(114, 4));

        msg_out.north_velocity_std = byte4toNumber<float_t>(data.substr(118, 4));
        msg_out.east_velocity_std = byte4toNumber<float_t>(data.substr(122, 4));
        msg_out.up_velocity_std = byte4toNumber<float_t>(data.substr(126, 4));

        msg_out.roll_std = byte4toNumber<float_t>(data.substr(130, 4));
        msg_out.pitch_std = byte4toNumber<float_t>(data.substr(134, 4));
        msg_out.azimuth_std = byte4toNumber<float_t>(data.substr(138, 4));

        parseExtendedSolutionStatus(&msg_out.extended_solution_status, data.substr(142, 4), is_little_end);
//        std::cout << string2Hex(data.substr(142, 4), false,is_little_end)  << std::endl;

        msg_out.time_since_update = byte2toNumber<uint16_t>(data.substr(146, 2));

        auto crc32 = byte4toNumber<uint32_t>(data.substr(148, 4));
        auto calcrc32 = CalculateBlockCRC32(154, (unsigned char *) (inspvaxb + data.substr(0, 148)).c_str());
        data.clear();

        if (crc32 != calcrc32) {
            std::stringstream ss;
            ss << "INSPVAXB data may be incorrect! Data CRC is " << std::hex << crc32 << " , but calculate CRC is "
               << calcrc32;
            ROS_WARN("%s", ss.str().c_str());
        }

        nmea_msgs::Sentence transform_out;
        transform_out.header.frame_id = "gnss_link";
        transform_out.header.stamp = *now;
        transformINSPVAXB2ASCII(&msg_out, &transform_out, crc32);

        pubINSPVAXB(msg_out);
        pubSpanLog(&transform_out);
    }

    void parseBESTGNSSPOSB(const ros::Time *now) {

        integrated_navigation_reader::SPANLog_BESTGNSSPOSB msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "BESTGNSSPOS";

        // get bin chars
        std::string data;
        serialHandle.read(data, 98);
//        std::cout << string2Hex(data.substr(0, 22)) << std::endl;
        // parse bin BESTGNSSPOS
        integrated_navigation_reader::BinaryHeader log_header;
        parseBinaryHeader(&log_header, data.substr(0, 22));
        msg_out.log_header = log_header;

        msg_out.solution_status.status = byte4toNumber<uint32_t>(data.substr(22, 4));
        msg_out.position_type.type = byte4toNumber<uint32_t>(data.substr(26, 4));

        msg_out.latitude = byte8toNumber<double_t>(data.substr(30, 8));
        msg_out.longitude = byte8toNumber<double_t>(data.substr(38, 8));
        msg_out.height = byte8toNumber<double_t>(data.substr(46, 8));
        msg_out.undulation = byte4toNumber<float_t>(data.substr(54, 4));

        msg_out.datum_id = byte4toNumber<uint32_t>(data.substr(58, 4));

        msg_out.latitude_std = byte4toNumber<float_t>(data.substr(62, 4));
        msg_out.longitude_std = byte4toNumber<float_t>(data.substr(66, 4));
        msg_out.height_std = byte4toNumber<float_t>(data.substr(70, 4));

        boost::array<char, 4> base_station_id = {data[74], data[75], data[76], data[77]};
        msg_out.base_station_id = base_station_id;
        msg_out.differential_age = byte4toNumber<float_t>(data.substr(78, 4));
        msg_out.solution_age = byte4toNumber<float_t>(data.substr(82, 4));

        msg_out.satellites_tracked = data[86];
        msg_out.satellites_solutions_used = data[87];
        msg_out.satellites_L1E1B1_used = data[88];
        msg_out.satellites_multi_frequency_used = data[89];

        msg_out.reserved = data[90];

        parseExtendedSolutionStatus(&msg_out.extended_solution_status,
                                    char2BinaryString(data[91]));
//        std::cout << string2Hex(data.substr(91, 1), false,is_little_end) << std::endl;
        parseGalileoBeiDouSignalUsedMask(&msg_out.signal_mask_galileo_beidou,
                                         char2BinaryString(data[92]));
//        std::cout << string2Hex(data.substr(92, 1), false,is_little_end) << std::endl;
        parseGPSGLONASSSignalUsedMask(&msg_out.signal_mask_gps_glonass,
                                      char2BinaryString(data[93]));
//        std::cout << string2Hex(data.substr(93, 1), false,is_little_end) << std::endl;
        auto crc32 = byte4toNumber<uint32_t>(data.substr(94, 4));
        auto calcrc32 = CalculateBlockCRC32(100, (unsigned char *) (bestgnssposb + data.substr(0, 94)).c_str());
        data.clear();

        if (crc32 != calcrc32) {
            std::stringstream ss;
            ss << "BESTGNSSPOSB data may be incorrect! Data CRC is " << std::hex << crc32 << " , but calculate CRC is "
               << calcrc32;
            ROS_WARN("%s", ss.str().c_str());
        }

        nmea_msgs::Sentence transform_out;
        transform_out.header.frame_id = "gnss_link";
        transform_out.header.stamp = *now;
        transformBESTGNSSPOSB2ASCII(&msg_out, &transform_out, crc32);

        pubBESTGNSSPOSB(msg_out);
        pubSpanLog(&transform_out);
    }

    void parseBESTGNSSVELB(const ros::Time *now) {

        integrated_navigation_reader::SPANLog_BESTGNSSVELB msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "BESTGNSSVEL";

        // get bin chars
        std::string data;
        serialHandle.read(data, 70);
//        std::cout << string2Hex(data.substr(0, 22)) << std::endl;
        // parse bin BESTGNSSPOS
        integrated_navigation_reader::BinaryHeader log_header;
        parseBinaryHeader(&log_header, data.substr(0, 22));
        msg_out.log_header = log_header;

        msg_out.solution_status.status = byte4toNumber<uint32_t>(data.substr(22, 4));
        msg_out.velocity_type.type = byte4toNumber<uint32_t>(data.substr(26, 4));

        msg_out.latency = byte4toNumber<float_t>(data.substr(30, 4));
        msg_out.age = byte4toNumber<float_t>(data.substr(34, 4));

        msg_out.horizontal_speed = byte8toNumber<double_t>(data.substr(38, 8));
        msg_out.direction_over_ground = byte8toNumber<double_t>(data.substr(46, 8));

        msg_out.vertical_speed = byte8toNumber<double_t>(data.substr(54, 8));

        msg_out.reserved = byte4toNumber<float_t>(data.substr(62, 4));

        auto crc32 = byte4toNumber<uint32_t>(data.substr(66, 4));
        auto calcrc32 = CalculateBlockCRC32(72, (unsigned char *) (bestgnssvelb + data.substr(0, 66)).c_str());
        data.clear();

        if (crc32 != calcrc32) {
            std::stringstream ss;
            ss << "BESTGNSSVELB data may be incorrect! Data CRC is " << std::hex << crc32 << " , but calculate CRC is "
               << calcrc32;
            ROS_WARN("%s", ss.str().c_str());
        }

        nmea_msgs::Sentence transform_out;
        transform_out.header.frame_id = "gnss_link";
        transform_out.header.stamp = *now;
        transformBESTGNSSVELB2ASCII(&msg_out, &transform_out, crc32);

        pubBESTGNSSVELB(msg_out);
        pubSpanLog(&transform_out);
    }

    void parseCORRIMUDATAB(const ros::Time *now) {

        integrated_navigation_reader::SPANLog_CORRIMUDATAB msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "CORRIMUDATA";

        // get bin chars
        std::string data;
        serialHandle.read(data, 86);
//        std::cout << string2Hex(data.substr(0, 22)) << std::endl;
        // parse bin BESTGNSSPOS
        integrated_navigation_reader::BinaryHeader log_header;
        parseBinaryHeader(&log_header, data.substr(0, 22));
        msg_out.log_header = log_header;

        msg_out.week = byte4toNumber<uint32_t>(data.substr(22, 4));
        msg_out.seconds = byte8toNumber<double_t>(data.substr(26, 8));

        msg_out.pitch_rate = byte8toNumber<double_t>(data.substr(34, 8));
        msg_out.roll_rate = byte8toNumber<double_t>(data.substr(42, 8));
        msg_out.yaw_rate = byte8toNumber<double_t>(data.substr(50, 8));

        msg_out.lateral_acceleration = byte8toNumber<double_t>(data.substr(58, 8));
        msg_out.longitudinal_acceleration = byte8toNumber<double_t>(data.substr(66, 8));
        msg_out.vertical_acceleration = byte8toNumber<double_t>(data.substr(74, 8));

        auto crc32 = byte4toNumber<uint32_t>(data.substr(82, 4));
        auto calcrc32 = CalculateBlockCRC32(88, (unsigned char *) (corrimudatab + data.substr(0, 82)).c_str());
        data.clear();

        if (crc32 != calcrc32) {
            std::stringstream ss;
            ss << "CORRIMUDATAB data may be incorrect! Data CRC is " << std::hex << crc32 << " , but calculate CRC is "
               << calcrc32;
            ROS_WARN("%s", ss.str().c_str());
        }

        nmea_msgs::Sentence transform_out;
        transform_out.header.frame_id = "imu_link";
        transform_out.header.stamp = *now;
        transformCORRIMUDATAB2ASCII(&msg_out, &transform_out, crc32);

        pubCORRIMUDATAB(msg_out);
        pubSpanLog(&transform_out);
    }

    void parseRAWIMUB(const ros::Time *now) {

        integrated_navigation_reader::SPANLog_RAWIMUB msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "RAWIMU";

        // get bin chars
        std::string data;
        serialHandle.read(data, 66);
//        std::cout << string2Hex(data.substr(0, 22)) << std::endl;
        // parse bin BESTGNSSPOS
        integrated_navigation_reader::BinaryHeader log_header;
        parseBinaryHeader(&log_header, data.substr(0, 22));
        msg_out.log_header = log_header;

        msg_out.week = byte4toNumber<uint32_t>(data.substr(22, 4));
        msg_out.seconds = byte8toNumber<double_t>(data.substr(26, 8));

        msg_out.imu_status = string2Hex(data.substr(32, 4), false, is_little_end);

        msg_out.z_acceleration = byte4toNumber<int32_t>(data.substr(38, 4));
        msg_out.negative_y_acceleration = byte4toNumber<int32_t>(data.substr(42, 4));
        msg_out.x_acceleration = byte4toNumber<int32_t>(data.substr(46, 4));

        msg_out.z_gyroscope = byte4toNumber<int32_t>(data.substr(50, 4));
        msg_out.negative_y_gyroscope = byte4toNumber<int32_t>(data.substr(54, 4));
        msg_out.x_gyroscope = byte4toNumber<int32_t>(data.substr(58, 4));

        auto crc32 = byte4toNumber<uint32_t>(data.substr(62, 4));
        auto calcrc32 = CalculateBlockCRC32(68, (unsigned char *) (rawimub + data.substr(0, 62)).c_str());
        data.clear();

        if (crc32 != calcrc32) {
            std::stringstream ss;
            ss << "RAWIMUB data may be incorrect! Data CRC is " << std::hex << crc32 << " , but calculate CRC is "
               << calcrc32;
            ROS_WARN("%s", ss.str().c_str());
        }

        nmea_msgs::Sentence transform_out;
        transform_out.header.frame_id = "imu_link";
        transform_out.header.stamp = *now;
        transformRAWIMUB2ASCII(&msg_out, &transform_out, crc32);

        pubRAWIMUB(msg_out);
        pubSpanLog(&transform_out);
    }

    void parseDUALANTENNAHEADINGB(const ros::Time *now) {

        integrated_navigation_reader::Log_DUALANTENNAHEADINGB msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "DUALANTENNAHEADING";

        // get bin chars
        std::string data;
        serialHandle.read(data, 70);
//        std::cout << string2Hex(data.substr(0, 22)) << std::endl;
        // parse bin BESTGNSSPOS
        integrated_navigation_reader::BinaryHeader log_header;
        parseBinaryHeader(&log_header, data.substr(0, 22));
        msg_out.log_header = log_header;

        msg_out.solution_status.status = byte4toNumber<uint32_t>(data.substr(22, 4));
        msg_out.position_type.type = byte4toNumber<uint32_t>(data.substr(26, 4));

        msg_out.length = byte4toNumber<float_t>(data.substr(30, 4));

        msg_out.heading = byte4toNumber<float_t>(data.substr(34, 4));
        msg_out.pitch = byte4toNumber<float_t>(data.substr(38, 4));

        msg_out.reserved = byte4toNumber<float_t>(data.substr(42, 4));

        msg_out.heading_std = byte4toNumber<float_t>(data.substr(46, 4));
        msg_out.pitch_std = byte4toNumber<float_t>(data.substr(50, 4));

        msg_out.station_id = data.substr(54, 4);

        msg_out.satellites_tracked = data[58];
        msg_out.satellites_solutions_used = data[59];
        msg_out.satellites_above_elevation_mask = data[60];
        msg_out.satellites_multi_frequency_used = data[61];

        parseSolutionSource(&msg_out.solution_source, char2BinaryString(data[62]));

        parseExtendedSolutionStatus(&msg_out.extended_solution_status,
                                    char2BinaryString(data[63]));

        parseGalileoBeiDouSignalUsedMask(&msg_out.signal_mask_galileo_beidou,
                                         char2BinaryString(data[64]));
        parseGPSGLONASSSignalUsedMask(&msg_out.signal_mask_gps_glonass,
                                      char2BinaryString(data[65]));

        auto crc32 = byte4toNumber<uint32_t>(data.substr(66, 4));
        auto calcrc32 = CalculateBlockCRC32(72, (unsigned char *) (dualantennaheadingb + data.substr(0, 66)).c_str());
        data.clear();

        if (crc32 != calcrc32) {
            std::stringstream ss;
            ss << "DUALANTENNAHEADINGB data may be incorrect! Data CRC is " << std::hex << crc32
               << " , but calculate CRC is " << calcrc32;
            ROS_WARN("%s", ss.str().c_str());
        }

        nmea_msgs::Sentence transform_out;
        transform_out.header.frame_id = "gnss_link";
        transform_out.header.stamp = *now;
        transformDUALANTENNAHEADINGB2ASCII(&msg_out, &transform_out, crc32);

        pubDUALANTENNAHEADINGB(msg_out);
        pubSpanLog(&transform_out);
    }


    void parseGPFPS(const ros::Time *now) {
        nmea_msgs::Sentence msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "gnss_link";

        std::string data = "$GPFPS";
        serialHandle.readline(data, 128, "\r\n");
        data.pop_back();
        data.pop_back();

        msg_out.sentence = data;

        pubNMEA(&msg_out);
    }

    void parseGPFPS_BIN(const ros::Time *now) {

        nmea_msgs::Sentence msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "gnss_link";

        // get bin chars
        std::string data;
        serialHandle.read(data, 57);

        // parse bin GPFPS_BIN
        auto gnss_week = byte2toNumber<uint16_t>(data.substr(0, 2));
        auto gnss_msec = byte4toNumber<uint32_t>(data.substr(2, 4));
        auto heading = byte4toNumber<float_t>(data.substr(6, 4));
        auto pitch = byte4toNumber<float_t>(data.substr(10, 4));
        auto roll = byte4toNumber<float_t>(data.substr(14, 4));

        auto latitude = byte4toNumber<int32_t>(data.substr(18, 4));
        auto longitude = byte4toNumber<int32_t>(data.substr(22, 4));
        auto altitude = byte4toNumber<int32_t>(data.substr(26, 4));

        auto head_dc = byte4toNumber<float_t>(data.substr(30, 4));
        auto heave = byte4toNumber<float_t>(data.substr(34, 4));

        auto velocity_e = byte4toNumber<float_t>(data.substr(38, 4));
        auto velocity_n = byte4toNumber<float_t>(data.substr(42, 4));
        auto velocity_u = byte4toNumber<float_t>(data.substr(46, 4));

        auto base_line = byte4toNumber<float_t>(data.substr(50, 4));

        uint8_t nsv1 = data[54];
        uint8_t nsv2 = data[55];

        uint8_t status = data[56];

        std::stringstream sentence;
        sentence.setf(std::ios::fixed);
        sentence << "$GPFPS," << gnss_week << "," << std::setprecision(3) << gnss_msec * 1e-3 << ","
                 << std::setprecision(2) << heading << "," << pitch << "," << roll << ","
                 << std::setprecision(7) << latitude * 1e-7 << "," << longitude * 1e-7 << "," << std::setprecision(2)
                 << altitude * 1e-3 << ","
                 << head_dc << "," << heave << ","
                 << std::setprecision(3) << velocity_e << "," << velocity_n << "," << velocity_u << "," << base_line
                 << ","
                 << std::setprecision(0) << (unsigned int) nsv1 << "," << (unsigned int) nsv2 << ","
                 << std::setw(2) << std::setfill('0') << (int) status;

        sentence << '*' << std::hex << std::uppercase << checkSum(sentence.str().substr(1));

        msg_out.sentence = sentence.str();

        pubNMEA(&msg_out);
    }

    void parseGTIMU(const ros::Time *now) {
        nmea_msgs::Sentence msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "gnss_link";

        std::string data = "$GTIMU";
        serialHandle.readline(data, 128, "\r\n");
        data.pop_back();
        data.pop_back();

        msg_out.sentence = data;

        pubNMEA(&msg_out);
    }

    void parseGTIMU_BIN_Huace(const ros::Time *now) {
        nmea_msgs::Sentence msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "gnss_link";

        // get bin chars
        std::string data;

        serialHandle.read(data, 56);

        // parse bin GTIMU_BIN
        auto gnss_week = byte2toNumber<uint16_t>(data.substr(0, 2));
        auto gnss_msec = byte4toNumber<uint32_t>(data.substr(2, 4));

        auto gyroscope_x = byte8toNumber<double_t>(data.substr(6, 8));
        auto gyroscope_y = byte8toNumber<double_t>(data.substr(14, 8));
        auto gyroscope_z = byte8toNumber<double_t>(data.substr(22, 8));

        auto acceleration_x = byte8toNumber<double_t>(data.substr(30, 8));
        auto acceleration_y = byte8toNumber<double_t>(data.substr(38, 8));
        auto acceleration_z = byte8toNumber<double_t>(data.substr(46, 8));

        auto temperature = byte2toNumber<uint16_t>(data.substr(54, 2));

        std::stringstream sentence;
        sentence.setf(std::ios::fixed);
        sentence << "$GTIMU," << gnss_week << "," << std::setprecision(3) << gnss_msec * 1e-3 << ","
                 << std::setprecision(4) << gyroscope_x << "," << gyroscope_y << "," << gyroscope_z << ","
                 << acceleration_x << "," << acceleration_y << "," << acceleration_z << ","
                 << std::setprecision(1) << temperature * 1e-3;

        sentence << '*' << std::hex << std::uppercase << checkSum(sentence.str().substr(1));

        msg_out.sentence = sentence.str();

        pubNMEA(&msg_out);
    }

    void parseGTIMU_BIN_Starnote(const ros::Time *now) {
        nmea_msgs::Sentence msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "gnss_link";

        // get bin chars
        std::string data;

        serialHandle.read(data, 56);

        // parse bin GTIMU_BIN
        auto gnss_week = byte2toNumber<uint16_t>(data.substr(0, 2));
        auto gnss_msec = byte4toNumber<uint32_t>(data.substr(2, 4));

        auto gyroscope_x = byte8toNumber<double_t>(data.substr(6, 8));
        auto gyroscope_y = byte8toNumber<double_t>(data.substr(14, 8));
        auto gyroscope_z = byte8toNumber<double_t>(data.substr(22, 8));

        auto acceleration_x = byte8toNumber<double_t>(data.substr(30, 8));
        auto acceleration_y = byte8toNumber<double_t>(data.substr(38, 8));
        auto acceleration_z = byte8toNumber<double_t>(data.substr(46, 8));

        auto gyroscope_x_temperature = byte2toNumber<uint16_t>(data.substr(54, 2));
        auto gyroscope_y_temperature = byte2toNumber<uint16_t>(data.substr(56, 2));
        auto gyroscope_z_temperature = byte2toNumber<uint16_t>(data.substr(58, 2));

        auto acceleration_x_temperature = byte2toNumber<uint16_t>(data.substr(60, 2));
        auto acceleration_y_temperature = byte2toNumber<uint16_t>(data.substr(62, 2));
        auto acceleration_z_temperature = byte2toNumber<uint16_t>(data.substr(64, 2));

        auto avg_temperature = (gyroscope_x_temperature + gyroscope_y_temperature + gyroscope_z_temperature +
                                acceleration_x_temperature + acceleration_y_temperature + acceleration_z_temperature) /
                               6.0;

        auto status = data[66];


        std::stringstream sentence;
        sentence.setf(std::ios::fixed);
        sentence << "$GTIMU," << gnss_week << "," << std::setprecision(3) << gnss_msec * 1e-3 << ","
                 << std::setprecision(4) << gyroscope_x << "," << gyroscope_y << "," << gyroscope_z << ","
                 << acceleration_x << "," << acceleration_y << "," << acceleration_z << ","
                 << std::setprecision(1) << avg_temperature;

        sentence << '*' << std::hex << std::uppercase << checkSum(sentence.str().substr(1));

        msg_out.sentence = sentence.str();

        pubNMEA(&msg_out);
    }

    void parseGPGGA(const ros::Time *now) {
        nmea_msgs::Sentence msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "gnss_link";

        std::string data = "$GPGGA";
        serialHandle.readline(data, 128, "\r\n");
        data.pop_back();
        data.pop_back();

        msg_out.sentence = data;

        pubNMEA(&msg_out);
    }

    void parseGPFPD(const ros::Time *now) {
        nmea_msgs::Sentence msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "gnss_link";

        std::string data = "$GPFPD";
        serialHandle.readline(data, 128, "\r\n");
        data.pop_back();
        data.pop_back();

        msg_out.sentence = data;

        pubNMEA(&msg_out);
    }

    void parseGPCHC(const ros::Time *now) {
        nmea_msgs::Sentence msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "gnss_link";

        std::string data = "$GPCHC";
        serialHandle.readline(data, 256, "\r\n");
        data.pop_back();
        data.pop_back();

        msg_out.sentence = data;

        pubNMEA(&msg_out);
    }

    void parseExtendedSolutionStatus(integrated_navigation_reader::Log_ExtendedSolutionStatus *msg, std::string data) {
        if (data.length() < 8) {
            ROS_WARN("Can not parse Log extended solution status because input data do not have enough bits");
            return;
        }
//        std::cout << data << ":";

        msg->RTK_or_PDP_GLIDE_solution = data[7] - '0';
        msg->pseudorange_iono_correction = std::stoi(data.substr(4, 3), nullptr, 2);
        msg->RTK_assist_active = data[3] - '0';
        msg->antenna_information_status = data[2] - '0';
        msg->reserved = data[1] - '0';
        msg->used_terrain_compensation_corrections = data[0] - '0';

    }


    void
    parseExtendedSolutionStatus(integrated_navigation_reader::SPANLog_ExtendedSolutionStatus *msg, std::string char_data, const bool reverse = false) {
        if (char_data.length() < 4) {
            ROS_WARN("Can not parse SPANLog extended solution status because input data do not have enough bits");
            return;
        }

        if (reverse) {
            std::reverse(char_data.begin(), char_data.end());
        }

        std::string bin_data = char2BinaryString(char_data[0]) + char2BinaryString(char_data[1]) +
                               char2BinaryString(char_data[2]) + char2BinaryString(char_data[3]);

//        std::cout << bin_data << ":" ;

        msg->position_update = bin_data[31] - '0';
        msg->phase_update = bin_data[30] - '0';
        msg->zero_velocity_update = bin_data[29] - '0';
        msg->wheel_sensor_update = bin_data[28] - '0';
        msg->ALIGN_heading_update = bin_data[27] - '0';
        msg->external_position_update = bin_data[26] - '0';
        msg->INS_solution_convergence_flag = bin_data[25] - '0';
        msg->doppler_update = bin_data[24] - '0';
        msg->pseudorange_update = bin_data[23] - '0';
        msg->velocity_update = bin_data[22] - '0';
        msg->reserved_bit_10 = bin_data[21] - '0';
        msg->dead_reckoning_update = bin_data[20] - '0';
        msg->phase_wind_up_update = bin_data[19] - '0';
        msg->course_over_ground_update = bin_data[18] - '0';
        msg->external_velocity_update = bin_data[17] - '0';
        msg->external_attitude_update = bin_data[16] - '0';
        msg->external_heading_update = bin_data[15] - '0';
        msg->external_height_update = bin_data[14] - '0';
        msg->reserved_bit_18 = bin_data[13] - '0';
        msg->reserved_bit_19 = bin_data[12] - '0';
        msg->reserved_bit_20 = bin_data[11] - '0';
        msg->reserved_bit_21 = bin_data[10] - '0';
        msg->secondary_INS_solution = bin_data[9] - '0';
        msg->reserved_bit_23 = bin_data[8] - '0';
        msg->turn_on_biases_estimated = bin_data[7] - '0';
        msg->alignment_direction_verified = bin_data[6] - '0';
        msg->alignment_indication = std::stoi(bin_data.substr(3, 3), nullptr, 2);
        msg->NVM_seed_indication = std::stoi(bin_data.substr(0, 3), nullptr, 2);

    }

    void parseBinaryHeader(integrated_navigation_reader::BinaryHeader *msg, std::string data) {
        if (data.length() < 22) {
            ROS_WARN("Can not parse binary header, because input data do not have enough bits");
            return;
        }

        auto message_type = char2BinaryString(data[0]);
        if (message_type.substr(4, 2) == "00") {
            msg->message_type[0] = integrated_navigation_reader::BinaryHeader::Binary_Format;
        } else if (message_type.substr(4, 2) == "01") {
            msg->message_type[0] = integrated_navigation_reader::BinaryHeader::ASCII_Format;
        } else if (message_type.substr(4, 2) == "10") {
            msg->message_type[0] = integrated_navigation_reader::BinaryHeader::Abbreviated_ASCII_NMEA_Format;
        } else {
            msg->message_type[0] = integrated_navigation_reader::BinaryHeader::Reserved;
        }
        if (message_type[6] == '1') {
            msg->message_type[1] = integrated_navigation_reader::BinaryHeader::Response_Message;
        } else {
            msg->message_type[1] = integrated_navigation_reader::BinaryHeader::Original_Message;
        }

        msg->port_address = data[1];
        msg->message_length = byte2toNumber<uint16_t>(data.substr(2, 2));
        msg->sequence = byte2toNumber<uint16_t>(data.substr(4, 2));
        msg->idle_time = data[6] / 2.0;
//        msg->idle_time = (unsigned int)data[6];
        msg->time_status = data[7];

        msg->week = byte2toNumber<uint16_t>(data.substr(8, 2));
        msg->milliseconds = byte4toNumber<uint32_t>(data.substr(10, 4));

        parseReceiverStatus(&msg->receiver_status,
                            data.substr(14, 4), is_little_end);
//        std::cout << string2Hex(data.substr(14, 4), false,is_little_end) << std::endl;

        msg->reserved = string2Hex(data.substr(18, 2), false, is_little_end);
        msg->receiver_sw_version = byte2toNumber<uint16_t>(data.substr(20, 2));
    }

    void parseReceiverStatus(integrated_navigation_reader::Log_ReceiverStatus *msg, std::string char_data, const bool reverse = false) {
        if (char_data.length() < 4) {
            ROS_WARN("Can not parse receiver status because input data do not have enough bits");
            return;
        }
//        std::cout << char_data << std::endl;
        if (reverse) {
            std::reverse(char_data.begin(), char_data.end());
        }

        std::string bin_data = char2BinaryString(char_data[0]) + char2BinaryString(char_data[1]) +
                               char2BinaryString(char_data[2]) + char2BinaryString(char_data[3]);
//        std::cout << bin_data << ":";
        msg->error_flag = bin_data[31] - '0';
        msg->temperature_status = bin_data[30] - '0';
        msg->voltage_supply_status = bin_data[29] - '0';
        msg->primary_antenna_power_status = bin_data[28] - '0';
        msg->LNA_failure = bin_data[27] - '0';
        msg->primary_antenna_open_circuit_flag = bin_data[26] - '0';
        msg->primary_antenna_short_circuit_flag = bin_data[25] - '0';
        msg->CPU_overload_flag = bin_data[24] - '0';
        msg->COM_port_transmit_buffer_overrun = bin_data[23] - '0';
        msg->spoofing_detection_status = bin_data[22] - '0';
        msg->reserved = bin_data[21] - '0';
        msg->link_overrun_flag = bin_data[20] - '0';
        msg->input_overrun_flag = bin_data[19] - '0';
        msg->aux_transmit_overrun_flag = bin_data[18] - '0';
        msg->antenna_gain_state = bin_data[17] - '0';
        msg->jammer_detected = bin_data[16] - '0';
        msg->INS_reset_flag = bin_data[15] - '0';
        msg->IMU_communication_failure = bin_data[14] - '0';
        msg->GPS_almanac_flag_UTC_known = bin_data[13] - '0';
        msg->position_solution_flag = bin_data[12] - '0';
        msg->position_fixed_flag = bin_data[11] - '0';
        msg->clock_steering_status = bin_data[10] - '0';
        msg->clock_model_flag = bin_data[9] - '0';
        msg->external_oscillator_locked_flag = bin_data[8] - '0';
        msg->software_resource = bin_data[7] - '0';
        msg->status_error_format_version_bit = bin_data[6] - '0';
        msg->version_bit_1 = bin_data[5] - '0';
        msg->tracking_mode = bin_data[4] - '0';
        msg->digital_filtering_enabled = bin_data[3] - '0';
        msg->auxiliary_3_status_event_flag = bin_data[2] - '0';
        msg->auxiliary_2_status_event_flag = bin_data[1] - '0';
        msg->auxiliary_1_status_event_flag = bin_data[0] - '0';

    }

    void parseSolutionSource(integrated_navigation_reader::Log_SolutionSource *msg, std::string data) {
        if (data.length() < 8) {
            ROS_WARN("Can not parse solution source, because input data do not have enough bits");
            return;
        }

        msg->reserved_bit_0 = data[7] - '0';
        msg->reserved_bit_1 = data[6] - '0';

        msg->source_antenna = std::stoi(data.substr(4, 2), nullptr, 2);

        msg->reserved_bit_4 = data[3] - '0';
        msg->reserved_bit_5 = data[2] - '0';
        msg->reserved_bit_6 = data[1] - '0';
        msg->reserved_bit_7 = data[0] - '0';

    }

    void parseGalileoBeiDouSignalUsedMask(integrated_navigation_reader::Log_GalileoBeiDouSignalUsedMask *msg,
                                          std::string data) {
        if (data.length() < 8) {
            ROS_WARN("Can not parse Galileo BeiDou Signal Used Mask because input data do not have enough bits");
            return;
        }
//        std::cout << data << ":";
        msg->galileo_E1 = data[7] - '0';
        msg->galileo_E5a = data[6] - '0';
        msg->galileo_E5b = data[5] - '0';
        msg->galileo_ALTBOC = data[4] - '0';
        msg->beidou_B1 = data[3] - '0';
        msg->beidou_B2 = data[2] - '0';
        msg->beidou_B3 = data[1] - '0';
        msg->galileo_E6 = data[0] - '0';

    }

    void
    parseGPSGLONASSSignalUsedMask(integrated_navigation_reader::Log_GPSGLONASSSignalUsedMask *msg, std::string data) {
        if (data.length() < 8) {
            ROS_WARN("Can not parse GPS GLONASSS Signal Used Mask because input data do not have enough bits");
            return;
        }
//        std::cout << data << ":";

        msg->gps_L1 = data[7] - '0';
        msg->gps_L2 = data[6] - '0';
        msg->gps_L5 = data[5] - '0';
        msg->reserved_bit_3 = data[4] - '0';
        msg->glonass_L1 = data[3] - '0';
        msg->glonass_L2 = data[2] - '0';
        msg->glonass_L3 = data[1] - '0';
        msg->reserved_bit_7 = data[0] - '0';

    }

    //TODO: support INSPVAXB to INSPVAXA sentence: port_address,time_status,ins_status,position_type convet to string
    static std::string transformBinaryHeader2String(integrated_navigation_reader::BinaryHeader *msg) {
        std::stringstream sentence;
        sentence.setf(std::ios::fixed);
        sentence << msg->port_address << "," << msg->sequence << "," << std::setprecision(1) << msg->idle_time << ","
                 << (unsigned int) msg->time_status << "," << msg->week << "," << std::setprecision(3)
                 << msg->milliseconds * 1e-3 << ","
                 << transformReceiverStatus2HexString(&msg->receiver_status) << "," << msg->reserved << "," << msg->receiver_sw_version;

        return sentence.str();
    }

    static std::string transformReceiverStatus2HexString(integrated_navigation_reader::Log_ReceiverStatus *msg) {
        return charBits2HexString(msg->software_resource, msg->status_error_format_version_bit,
                                  msg->version_bit_1, msg->tracking_mode,
                                  msg->digital_filtering_enabled, msg->auxiliary_3_status_event_flag,
                                  msg->auxiliary_2_status_event_flag, msg->auxiliary_1_status_event_flag) +
                charBits2HexString(msg->INS_reset_flag, msg->IMU_communication_failure,
                                   msg->GPS_almanac_flag_UTC_known, msg->position_solution_flag,
                                   msg->position_fixed_flag, msg->clock_steering_status,
                                   msg->clock_steering_status, msg->external_oscillator_locked_flag) +
                charBits2HexString(msg->COM_port_transmit_buffer_overrun, msg->spoofing_detection_status,
                                   msg->reserved, msg->link_overrun_flag,
                                   msg->input_overrun_flag, msg->aux_transmit_overrun_flag,
                                   msg->antenna_gain_state, msg->jammer_detected) +
                charBits2HexString(msg->error_flag, msg->temperature_status,
                                   msg->voltage_supply_status, msg->primary_antenna_power_status,
                                   msg->LNA_failure, msg->primary_antenna_open_circuit_flag,
                                   msg->primary_antenna_short_circuit_flag, msg->CPU_overload_flag);
    }

    static std::string transformExtendedSolutionStatus2HexString(integrated_navigation_reader::Log_ExtendedSolutionStatus *msg) {
        auto pseudorange_iono_correction = decimal2BinaryString(msg->pseudorange_iono_correction, 3);

        return charBits2HexString(msg->RTK_or_PDP_GLIDE_solution, pseudorange_iono_correction[2] - '0',
                                  pseudorange_iono_correction[1]- '0', pseudorange_iono_correction[0]- '0',
                                  msg->RTK_assist_active, msg->antenna_information_status,
                                  msg->reserved, msg->used_terrain_compensation_corrections);
    }

    static std::string transformExtendedSolutionStatus2HexString(integrated_navigation_reader::SPANLog_ExtendedSolutionStatus *msg) {
        auto alignment_indication = decimal2BinaryString(msg->alignment_indication, 3);
        auto NVM_seed_indication = decimal2BinaryString(msg->NVM_seed_indication, 3);

//        std::cout << "alignment_indication: " << alignment_indication << "  NVM_seed_indication: " << NVM_seed_indication << std::endl;
        return charBits2HexString(msg->turn_on_biases_estimated,msg->alignment_direction_verified,
                                  alignment_indication[2] - '0', alignment_indication[1]- '0',
                                  alignment_indication[0] - '0', NVM_seed_indication[2] - '0',
                                  NVM_seed_indication[1] - '0', NVM_seed_indication[0] - '0') +
                charBits2HexString(msg->external_heading_update, msg->external_height_update,
                                   msg->reserved_bit_18, msg->reserved_bit_19,
                                   msg->reserved_bit_20, msg->reserved_bit_21,
                                   msg->secondary_INS_solution, msg->reserved_bit_23) +
                charBits2HexString(msg->pseudorange_update, msg->velocity_update,
                                   msg->reserved_bit_10, msg->dead_reckoning_update,
                                   msg->phase_wind_up_update, msg->course_over_ground_update,
                                   msg->external_velocity_update, msg->external_attitude_update) +
                charBits2HexString(msg->position_update, msg->phase_update,
                                   msg->zero_velocity_update, msg->wheel_sensor_update,
                                   msg->ALIGN_heading_update, msg->external_position_update,
                                   msg->INS_solution_convergence_flag, msg->doppler_update);
    }

    static std::string transformGalileoBeiDouSignalUsedMask2HexString(integrated_navigation_reader::Log_GalileoBeiDouSignalUsedMask *msg) {
        return charBits2HexString(msg->galileo_E1, msg->galileo_E5a,
                                  msg->galileo_E5b, msg->galileo_ALTBOC,
                                  msg->beidou_B1, msg->beidou_B2,
                                  msg->beidou_B3, msg->galileo_E6);
    }

    static std::string transformGPSGLONASSSignalUsedMask2HexString(integrated_navigation_reader::Log_GPSGLONASSSignalUsedMask *msg) {
        return charBits2HexString(msg->gps_L1, msg->gps_L2,
                                  msg->gps_L5, msg->reserved_bit_3,
                                  msg->glonass_L1, msg->glonass_L2,
                                  msg->glonass_L3, msg->reserved_bit_7);
    }

    static std::string transformSolutionSource2HexString(integrated_navigation_reader::Log_SolutionSource *msg) {
        auto source_antenna = decimal2BinaryString(msg->source_antenna, 2);
//        std::cout << source_antenna << std::endl;
        return charBits2HexString(msg->reserved_bit_0, msg->reserved_bit_1,
                                  source_antenna[1] - '0', source_antenna[0] - '0',
                                  msg->reserved_bit_4, msg->reserved_bit_5,
                                  msg->reserved_bit_6, msg->reserved_bit_7);
    }

    static void
    transformINSPVAXB2ASCII(integrated_navigation_reader::SPANLog_INSPVAXB *msg_from, nmea_msgs::Sentence *msg_to,
                            const uint32_t crc32) {
        std::stringstream sentence;
        sentence.setf(std::ios::fixed);
        sentence << "#INSPVAXA," << transformBinaryHeader2String(&msg_from->log_header) << ";"
                 << (unsigned int) msg_from->ins_status.status << "," << (unsigned int) msg_from->position_type.type
                 << "," << std::setprecision(11) << msg_from->latitude << "," << msg_from->longitude << ","
                 << std::setprecision(4)<< msg_from->height << "," << msg_from->undulation << ","
                 << msg_from->north_velocity << "," << msg_from->east_velocity << "," << msg_from->up_velocity << ","
                 << std::setprecision(9)
                 << msg_from->roll << "," << msg_from->pitch << "," << msg_from->azimuth << ","
                 << std::setprecision(4)
                 << msg_from->latitude_std << "," << msg_from->longitude_std << "," << msg_from->height_std << ","
                 << msg_from->north_velocity_std << "," << msg_from->east_velocity_std << ","
                 << msg_from->up_velocity_std << ","
                 << msg_from->roll_std << "," << msg_from->pitch_std << "," << msg_from->azimuth_std << ","
                 << transformExtendedSolutionStatus2HexString(&msg_from->extended_solution_status) << ","
                 << msg_from->time_since_update << "*" << std::hex
                 << crc32;

        msg_to->sentence = sentence.str();
    }

    static void transformBESTGNSSPOSB2ASCII(integrated_navigation_reader::SPANLog_BESTGNSSPOSB *msg_from,
                                            nmea_msgs::Sentence *msg_to, const uint32_t crc32) {
        std::stringstream sentence;
        sentence.setf(std::ios::fixed);
        sentence << "#BESTGNSSPOSA," << transformBinaryHeader2String(&msg_from->log_header) << ";"
                 << (unsigned int) msg_from->solution_status.status << ","
                 << (unsigned int) msg_from->position_type.type << "," << std::setprecision(11)
                 << msg_from->latitude << "," << msg_from->longitude << "," << std::setprecision(4) << msg_from->height
                 << "," << msg_from->undulation << ","
                 << msg_from->datum_id << "," << msg_from->latitude_std << "," << msg_from->longitude_std << ","
                 << msg_from->height_std << ",\""
                 << msg_from->base_station_id[0] << msg_from->base_station_id[1]
                 << msg_from->base_station_id[2] << msg_from->base_station_id[3]
                 << "\"," << std::setprecision(3) << msg_from->differential_age << "," << msg_from->solution_age << ","
                 << (unsigned int) msg_from->satellites_tracked << ","
                 << (unsigned int) msg_from->satellites_solutions_used << ","
                 << (unsigned int) msg_from->satellites_L1E1B1_used << ","
                 << (unsigned int) msg_from->satellites_multi_frequency_used << "," << (unsigned int) msg_from->reserved
                 << ","
                 << transformExtendedSolutionStatus2HexString(&msg_from->extended_solution_status) << ","
                 << transformGalileoBeiDouSignalUsedMask2HexString(&msg_from->signal_mask_galileo_beidou) << ","
                 << transformGPSGLONASSSignalUsedMask2HexString(&msg_from->signal_mask_gps_glonass) << "*" << std::hex << crc32;

        msg_to->sentence = sentence.str();
    }

    static void transformBESTGNSSVELB2ASCII(integrated_navigation_reader::SPANLog_BESTGNSSVELB *msg_from,
                                            nmea_msgs::Sentence *msg_to, const uint32_t crc32) {
        std::stringstream sentence;
        sentence.setf(std::ios::fixed);
        sentence << "#BESTGNSSVELA," << transformBinaryHeader2String(&msg_from->log_header) << ";"
                 << (unsigned int) msg_from->solution_status.status << ","
                 << (unsigned int) msg_from->velocity_type.type << "," << std::setprecision(3)
                 << msg_from->latency << "," << msg_from->age << "," << std::setprecision(4)
                 << msg_from->horizontal_speed << "," << std::setprecision(6) << msg_from->direction_over_ground << ","
                 << std::setprecision(4) << msg_from->vertical_speed << "," << msg_from->reserved << "*"
                 << std::hex << crc32;

        msg_to->sentence = sentence.str();
    }

    static void transformCORRIMUDATAB2ASCII(integrated_navigation_reader::SPANLog_CORRIMUDATAB *msg_from,
                                            nmea_msgs::Sentence *msg_to, const uint32_t crc32) {
        std::stringstream sentence;
        sentence.setf(std::ios::fixed);
        sentence << "#CORRIMUDATAA," << transformBinaryHeader2String(&msg_from->log_header) << ";"
                 << msg_from->week << "," << std::setprecision(9) << msg_from->seconds << ","
                 << msg_from->pitch_rate << "," << msg_from->roll_rate << "," << msg_from->yaw_rate << ","
                 << msg_from->lateral_acceleration << "," << msg_from->longitudinal_acceleration << ","
                 << msg_from->vertical_acceleration << "*" << std::hex << crc32;

        msg_to->sentence = sentence.str();
    }

    static void transformDUALANTENNAHEADINGB2ASCII(integrated_navigation_reader::Log_DUALANTENNAHEADINGB *msg_from,
                                                   nmea_msgs::Sentence *msg_to, const uint32_t crc32) {
        std::stringstream sentence;
        sentence.setf(std::ios::fixed);
        sentence << "#DUALANTENNAHEADINGA," << transformBinaryHeader2String(&msg_from->log_header) << ";"
                 << (unsigned int)msg_from->solution_status.status << "," << (unsigned int)msg_from->position_type.type << ","
                 << std::setprecision(9) << msg_from->length << ","
                 << msg_from->heading << "," << msg_from->pitch << ","
                 << std::setprecision(1) << msg_from->reserved << ","
                 << std::setprecision(9) << msg_from->heading_std << "," << msg_from->pitch_std << ","
                 << '"' << msg_from->station_id << '"' << ","
                 << (unsigned int)msg_from->satellites_tracked << ","
                 << (unsigned int)msg_from->satellites_solutions_used << ","
                 << (unsigned int)msg_from->satellites_above_elevation_mask << ","
                 << (unsigned int)msg_from->satellites_multi_frequency_used << ","
                 << transformSolutionSource2HexString(&msg_from->solution_source) << ","
                 << transformExtendedSolutionStatus2HexString(&msg_from->extended_solution_status) << ","
                 << transformGalileoBeiDouSignalUsedMask2HexString(&msg_from->signal_mask_galileo_beidou) << ","
                 << transformGPSGLONASSSignalUsedMask2HexString(&msg_from->signal_mask_gps_glonass) << "*" << std::hex
                 << crc32;

        msg_to->sentence = sentence.str();
    }

    static void
    transformRAWIMUB2ASCII(integrated_navigation_reader::SPANLog_RAWIMUB *msg_from, nmea_msgs::Sentence *msg_to,
                           const uint32_t crc32) {
        std::stringstream sentence;
        sentence.setf(std::ios::fixed);
        sentence << "#RAWIMUA," << transformBinaryHeader2String(&msg_from->log_header) << ";"
                 << msg_from->week << "," << std::setprecision(9) << msg_from->seconds << "," << msg_from->imu_status
                 << ","
                 << msg_from->z_acceleration << "," << msg_from->negative_y_acceleration << ","
                 << msg_from->x_acceleration << ","
                 << msg_from->z_gyroscope << "," << msg_from->negative_y_gyroscope << "," << msg_from->x_gyroscope
                 << "*" << std::hex << crc32;

        msg_to->sentence = sentence.str();
    }

};


int main(int argc, char **argv) {

    ros::init(argc, argv, "binary_serial_parser");
    Binary_Serial_Parser parser;

    parser.parseStream();

    ros::spin();
    return 0;
}
