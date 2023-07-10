//
// Created by lin on 23-7-5.
//

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <nmea_msgs/Sentence.h>
#include <bitset>
#include "utility.h"

#include <iostream>
#include <unistd.h>

// refer to
class INSPVA_HEADER {
public:
    uint8_t message_type;
    uint8_t port_address;
    uint16_t message_length;
    uint16_t sequence;
    uint8_t idle_time;
    uint8_t time_status;
    uint16_t week;
    uint32_t milliseconds;
    uint32_t receiver_status;
    uint16_t revserved;
    uint16_t receiver_sw_version;

    INSPVA_HEADER(const std::string data) {
        if (data.size() < 22) {
            throw std::invalid_argument("INSPVA_HEADER get not enough byte length");
            return;
        }
        // parse INSPVA_HEADER
        message_type = data[0];
        port_address = data[1];
        message_type = byte2toNumber<uint16_t>(data.substr(2, 2));
        sequence = byte2toNumber<uint16_t>(data.substr(4, 2));
        idle_time = data[6];

        time_status = data[7];
        week = byte2toNumber<uint16_t>(data.substr(8, 2));
        milliseconds = byte4toNumber<uint32_t>(data.substr(10, 4));

        receiver_status = byte4toNumber<uint32_t>(data.substr(14, 4));
        revserved = byte2toNumber<uint16_t>(data.substr(18, 2));
        receiver_sw_version = byte2toNumber<uint16_t>(data.substr(20, 2));
    }
};


class Binary_Serial_Parser
{
public:
    Binary_Serial_Parser(){

        uint8_t inspvaxb_heder[] = {0xAA, 0x44, 0x12, 0x1C, 0xB9, 0x05};
        inspvaxb = bytes2String(inspvaxb_heder, 6);
        uint8_t gpfps_bin_heder[] = {0xAA, 0x55, 0x04};
        gpfps_bin = bytes2String(gpfps_bin_heder, 3);
        uint8_t gtimu_bin_heder[] = {0xAA, 0x55, 0x05};
        gtimu_bin = bytes2String(gtimu_bin_heder, 3);

//        std::cout << "gpfps_bin: " << gpfps_bin.size() << "   " << string_to_hex(gpfps_bin)<< std::endl;
//        std::cout << "gtimu_bin: " << gtimu_bin.size() << "   " << string_to_hex(gtimu_bin) << std::endl;

        nmea_pub = nh_.advertise<nmea_msgs::Sentence>("/nmea/sentence", 10);
        spanlog_pub = nh_.advertise<nmea_msgs::Sentence>("/spanlog/sentence", 10);

        std::string port = "/dev/ttyUSB0";
        unsigned int baudrate = 115200;
        uint32_t timeout_msec = 1000;

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

            if (buffer.find(inspvaxb) != std::string::npos) {
//                std::cout << "Detected: inspvaxb" << std::endl;
                parseINSPVAXB(p_time);
                buffer.clear();
                continue;
            }

            if (buffer.find(gpfps_bin) != std::string::npos) {
//                std::cout << "Detected: gpfps_bin" << std::endl;
                parseGPFPS_BIN(p_time);
                buffer.clear();
                continue;
            }
            if (buffer.find(gpfps) != std::string::npos) {
//                std::cout << "Detected: gpfps" << std::endl;
                parseGPFPS(p_time);
                buffer.clear();
                continue;
            }

            if (buffer.find(gtimu_bin) != std::string::npos) {
//                std::cout << "Detected: gtimu_bin" << std::endl;
                parseGTIMU_BIN_Huace(p_time);
                buffer.clear();
                continue;
            }
            if (buffer.find(gtimu) != std::string::npos) {
//                std::cout << "Detected: gtimu" << std::endl;
                parseGTIMU(p_time);
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


private:

    ros::NodeHandle nh_;
    ros::Publisher nmea_pub;
    ros::Publisher spanlog_pub;

    serial::Serial serialHandle;

    bool from_little_end = true;
    std::string inspvaxb;
    std::string gpfps_bin;
    const std::string gpfps = "$GPFPS";
    std::string gtimu_bin;
    const std::string gtimu = "$GTIMU";


    void parseINSPVAXB(const ros::Time *now) {

        nmea_msgs::Sentence msg_out;
        msg_out.header.stamp = *now;
        msg_out.header.frame_id = "gnss_link";

        // get bin chars
        std::string data;
        serialHandle.read(data, 152);

        // parse bin inspvax
        INSPVA_HEADER inspva_header(data.substr(0, 22));

        auto ins_status = byte4toNumber<uint32_t>(data.substr(22, 4));
        auto position_type = byte4toNumber<uint32_t>(data.substr(26, 4));

        auto latitude = byte8toNumber<double_t>(data.substr(30, 8));
        auto longitude = byte8toNumber<double_t>(data.substr(38, 8));
        auto height = byte8toNumber<double_t>(data.substr(46, 8));
        auto undulation = byte4toNumber<float_t>(data.substr(54, 4));

        auto north_velocity = byte8toNumber<double_t>(data.substr(58, 8));
        auto east_velocity = byte8toNumber<double_t>(data.substr(66, 8));
        auto up_velocity = byte8toNumber<double_t>(data.substr(74, 8));

        auto roll = byte8toNumber<double_t>(data.substr(82, 8));
        auto pitch = byte8toNumber<double_t>(data.substr(90, 8));
        auto azimuth = byte8toNumber<double_t>(data.substr(98, 8));

        auto latitude_std = byte4toNumber<float_t>(data.substr(106, 4));
        auto longitude_std = byte4toNumber<float_t>(data.substr(110, 4));
        auto height_std = byte4toNumber<float_t>(data.substr(114, 4));

        auto north_velocity_std = byte4toNumber<float_t>(data.substr(118, 4));
        auto east_velocity_std = byte4toNumber<float_t>(data.substr(122, 4));
        auto up_velocity_std = byte4toNumber<float_t>(data.substr(126, 4));

        auto roll_std = byte4toNumber<float_t>(data.substr(130, 4));
        auto pitch_std = byte4toNumber<float_t>(data.substr(134, 4));
        auto azimuth_std = byte4toNumber<float_t>(data.substr(138, 4));

        auto extended_solution_status = data.substr(142, 4);

        auto time_since_update = byte2toNumber<uint16_t>(data.substr(146, 2));

        auto crc32 = data.substr(148, 4);
        //TODO: add CRC check.

        std::stringstream sentence;
        sentence.setf(std::ios::fixed);
        sentence << "#INSPVAXA,COM1,0,73.5,FINESTEERING,1695,309428.000,02000040,4e77,43562;INS_SOLUTION_GOOD,INS_PSRSP,"
                << latitude << "," << longitude << "," << height << "," << undulation << ","
                << north_velocity << "," << east_velocity << "," << up_velocity << ","
                << roll << "," << pitch << "," << azimuth << ","
                << latitude_std << "," << longitude_std << "," << height_std << ","
                << north_velocity_std << "," << east_velocity_std << "," << up_velocity_std << ","
                << roll_std << "," << pitch_std << "," << azimuth_std << ","
                << string2Hex(extended_solution_status) << "," << time_since_update << "*" << string2Hex(crc32.c_str());

        msg_out.sentence = sentence.str();

        pubSpanLog(&msg_out);
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

        pubSpanLog(&msg_out);
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

};



int main(int argc, char** argv)
{

    ros::init(argc, argv, "binary_serial_parser");
    Binary_Serial_Parser parser;

    parser.parseStream();

    ros::spin();
    return 0;
}
