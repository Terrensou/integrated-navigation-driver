//
// Created by lin on 23-7-5.
//

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <nmea_msgs/Sentence.h>
#include "integrated_navigation_reader/Spanlog_INSPVAXB.h"
#include "utility.h"

class Binary_Serial_Parser
{
public:
    Binary_Serial_Parser(){
        is_little_end = isNativeLittleEndian();

        uint8_t inspvaxb_heder[] = {0xAA, 0x44, 0x12, 0x1C, 0xB9, 0x05};
        inspvaxb = bytes2String(inspvaxb_heder, 6);
        uint8_t gpfps_bin_heder[] = {0xAA, 0x55, 0x04};
        gpfps_bin = bytes2String(gpfps_bin_heder, 3);
        uint8_t gtimu_bin_heder[] = {0xAA, 0x55, 0x05};
        gtimu_bin = bytes2String(gtimu_bin_heder, 3);

        ros::NodeHandle nh_local("~");
        nh_local.getParam("INSPVAXB", use_INSPVAXB);
        nh_local.getParam("GPFPS", use_GPFPS);
        nh_local.getParam("GPFPS_BIN", use_GPFPS_BIN);
        nh_local.getParam("GTIMU", use_GTIMU);
        nh_local.getParam("GTIMU_BIN", use_GTIMU_BIN);
        nh_local.getParam("GPGGA", use_GPGGA);
        nh_local.getParam("GPFPD", use_GPFPD);

        nmea_pub = nh_.advertise<nmea_msgs::Sentence>("/nmea/sentence", 10);
        spanlog_pub = nh_.advertise<nmea_msgs::Sentence>("/spanlog/sentence", 10);

        if (use_INSPVAXB)
        {
            INSPVAXB_pub = nh_.advertise<integrated_navigation_reader::Spanlog_INSPVAXB>("/spanlog/inspvaxb", 10);
        }

        std::string port;
        int baudrate = 115200;
        int timeout_msec = 1000;

        nh_.getParam("Nmea_serial_reader_node/port", port);
        nh_.getParam("Nmea_serial_reader_node/baud", baudrate);
        nh_.getParam("Nmea_serial_reader_node/timeout", timeout_msec);


        // open serial
        try{

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

        }
    }

    void pubNMEA(const nmea_msgs::Sentence * msg)
    {
        nmea_pub.publish(*msg);
    }

    void pubSpanLog(const nmea_msgs::Sentence * msg)
    {
        spanlog_pub.publish(*msg);
    }

    void pubINSPVAXB(const integrated_navigation_reader::Spanlog_INSPVAXB msg)
    {
        INSPVAXB_pub.publish(msg);
    }


private:

    ros::NodeHandle nh_;
    ros::Publisher nmea_pub;
    ros::Publisher spanlog_pub;
    ros::Publisher INSPVAXB_pub;

    serial::Serial serialHandle;

    bool is_little_end = false;
    bool use_INSPVAXB = false;
    bool use_GPFPS = false;
    bool use_GPFPS_BIN = false;
    bool use_GTIMU = false;
    bool use_GTIMU_BIN = false;
    bool use_GPGGA = false;
    bool use_GPFPD = false;

    std::string inspvaxb;
    std::string gpfps_bin;
    const std::string gpfps = "$GPFPS";
    std::string gtimu_bin;
    const std::string gtimu = "$GTIMU";
    const std::string gpgga = "$GPGGA";
    const std::string gpfpd = "$GPFPD";

    void parseINSPVAXB(const ros::Time *now) {

        integrated_navigation_reader::Spanlog_INSPVAXB msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "INSPVAX";

        // get bin chars
        std::string data;
        serialHandle.read(data, 152);

        // parse bin inspvax
        auto message_type = chat2BinaryString(data[0]);
        if (message_type.substr(4, 2) == "00") {
            msg_out.log_header.message_type[0] = integrated_navigation_reader::Spanlog_BinaryHeader::Binary_Format;
        } else if (message_type.substr(4, 2) == "01") {
            msg_out.log_header.message_type[0] = integrated_navigation_reader::Spanlog_BinaryHeader::ASCII_Format;
        } else if (message_type.substr(4, 2) == "10") {
            msg_out.log_header.message_type[0] = integrated_navigation_reader::Spanlog_BinaryHeader::Abbreviated_ASCII_NMEA_Format;
        } else {
            msg_out.log_header.message_type[0] = integrated_navigation_reader::Spanlog_BinaryHeader::Reserved;
        }
        if (message_type[6] == '1') {
            msg_out.log_header.message_type[1] = integrated_navigation_reader::Spanlog_BinaryHeader::Response_Message;
        } else {
            msg_out.log_header.message_type[1] = integrated_navigation_reader::Spanlog_BinaryHeader::Original_Message;
        }

        msg_out.log_header.port_address = data[1];
        msg_out.log_header.message_length = byte2toNumber<uint16_t>(data.substr(2, 2));
        msg_out.log_header.sequence = byte2toNumber<uint16_t>(data.substr(4, 2));
        msg_out.log_header.idle_time = data[6] / 2.0;
        msg_out.log_header.time_status = data[7];

        msg_out.log_header.week = byte2toNumber<uint16_t>(data.substr(8, 2));
        msg_out.log_header.milliseconds = byte4toNumber<uint32_t>(data.substr(10, 4));

        msg_out.log_header.receiver_status = string2Hex(data.substr(14, 4), false, is_little_end);
        msg_out.log_header.reserved = string2Hex(data.substr(18, 2), false, is_little_end);
        msg_out.log_header.receiver_sw_version = byte2toNumber<uint16_t>(data.substr(20, 2));

        msg_out.ins_status = byte4toNumber<uint32_t>(data.substr(22, 4));
        msg_out.position_type = byte4toNumber<uint32_t>(data.substr(26, 4));

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

        msg_out.extended_solution_status = string2Hex(data.substr(142, 4), false, is_little_end);

        msg_out.time_since_update = byte2toNumber<uint16_t>(data.substr(146, 2));

        auto crc32 = byte4toNumber<uint32_t>(data.substr(148, 4));
        auto calcrc32 = CalculateBlockCRC32(154, (unsigned char *) (inspvaxb +data.substr(0,148)).c_str());
        //TODO: add CRC check.
        if (crc32 != calcrc32) {
            std::stringstream ss;
            ss << "INSPVAXB data may be incorrect! Data CRC is " << std::hex << crc32 << " , but calculate CRC is " << calcrc32;
            ROS_WARN("%s", ss.str().c_str());
        }

        nmea_msgs::Sentence transform_out;
        transform_out.header.frame_id = "gnss_link";
        transform_out.header.stamp = *now;
        transformINSPVAXB2INSPVAXASentence(&msg_out, &transform_out, crc32);

        pubINSPVAXB(msg_out);
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
                 << std::setprecision(7) << latitude * 1e-7 << "," << longitude * 1e-7 << "," << std::setprecision(2) << altitude * 1e-3 << ","
                 << head_dc << "," << heave << ","
                 << std::setprecision(3) << velocity_e << "," << velocity_n << "," << velocity_u << "," << base_line << ","
                 << std::setprecision(0) << (unsigned int)nsv1 << "," << (unsigned int)nsv2 << ","
                 << std::setw(2) << std::setfill('0') << (int)status;

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
                 << std::setprecision(1) << temperature * 1e-3 ;

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

        auto avg_temperature = (gyroscope_x_temperature + gyroscope_y_temperature + gyroscope_z_temperature + acceleration_x_temperature + acceleration_y_temperature + acceleration_z_temperature) / 6.0;

        auto status = data[66];


        std::stringstream sentence;
        sentence.setf(std::ios::fixed);
        sentence << "$GTIMU," << gnss_week << "," << std::setprecision(3) << gnss_msec * 1e-3 << ","
                 << std::setprecision(4) << gyroscope_x << "," << gyroscope_y << "," << gyroscope_z << ","
                 << acceleration_x << "," << acceleration_y << "," << acceleration_z << ","
                 << std::setprecision(1) << avg_temperature ;

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


    //TODO: support INSPVAXB to INSPVAXA sentence: port_address,time_status,ins_status,position_type convet to string
    static void transformINSPVAXB2INSPVAXASentence(integrated_navigation_reader::Spanlog_INSPVAXB *msg_from, nmea_msgs::Sentence *msg_to, const uint32_t crc32) {
        std::stringstream sentence;
        sentence.setf(std::ios::fixed);
        sentence << "#INSPVAXA,"<< msg_from->log_header.port_address << "," << msg_from->log_header.sequence << ","
                << std::setprecision(3) << msg_from->log_header.idle_time << ","
                << (unsigned int)msg_from->log_header.time_status << ","
                << msg_from->log_header.week << "," << std::setprecision(3) << msg_from->log_header.milliseconds*1e-3 << ","
                << msg_from->log_header.receiver_status << "," << msg_from->log_header.reserved << ","<< msg_from->log_header.receiver_sw_version << ";"
                << (unsigned int)msg_from->ins_status << "," << (unsigned int)msg_from->position_type << ","
                << msg_from->latitude << "," << msg_from->longitude << "," << msg_from->height << "," << msg_from->undulation << ","
                << msg_from->north_velocity << "," << msg_from->east_velocity << "," << msg_from->up_velocity << ","
                << msg_from->roll << "," << msg_from->pitch << "," << msg_from->azimuth << ","
                << msg_from->latitude_std << "," << msg_from->longitude_std << "," << msg_from->height_std << ","
                << msg_from->north_velocity_std << "," << msg_from->east_velocity_std << "," << msg_from->up_velocity_std << ","
                << msg_from->roll_std << "," << msg_from->pitch_std << "," << msg_from->azimuth_std << ","
                << msg_from->extended_solution_status << "," << msg_from->time_since_update << "*" << std::hex << crc32;

        msg_to->sentence = sentence.str();
    }

};



int main(int argc, char** argv)
{

    ros::init(argc, argv, "binary_serial_parser");
    Binary_Serial_Parser parser;

    parser.parseStream();

    ros::spin();
    return 0;
}
