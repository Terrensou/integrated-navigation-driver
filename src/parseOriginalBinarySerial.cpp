#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <bitset>
#include "Serial.h"

#include <iostream>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

//struct HeaderA {
//    uint8_t header1;
//    uint8_t header2;
//    uint8_t header3;
//    uint16_t gnss_week1;
//    uint8_t gnss_week2;
//    uint32_t gnss_second;
//    float_t yaw;
//    float_t pitch;
//    float_t roll;
//    uint32_t lo;
//    uint32_t la;
//    uint32_t al;
//    int16_t tem_gyro_x;
//    int16_t tem_gyro_y;
//    int16_t tem_gyro_z;
//    int16_t tem_acc_x;
//    int16_t tem_acc_y;
//    int16_t tem_acc_z;
//    uint8_t status;
//};
//struct HeaderB {
//    uint8_t header1;
//    uint8_t header2;
//    uint8_t header3;
//    uint16_t gnss_week;
//    uint32_t gnss_second;
//    double_t gyo_x;
//    double_t gyo_y;
//    double_t gyo_z;
//    double_t acc_x;
//    double_t acc_y;
//    double_t acc_z;
//    int16_t tem_gyro_x;
//    int16_t tem_gyro_y;
//    int16_t tem_gyro_z;
//    int16_t tem_acc_x;
//    int16_t tem_acc_y;
//    int16_t tem_acc_z;
//    uint8_t status;
//};

//struct HeaderB {
//    uint8_t marker;
//    uint32_t dataB;
//};
//uint16_t swapEndianness(uint16_t value) { return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF); }
//void parseSerialData(int serialPort) {
//// Read data from the serial port
//    unsigned char buffer[256]; // Buffer to hold the received data
//    ssize_t bytesRead; // Number of bytes read
//
//    bytesRead = read(serialPort, buffer, sizeof(buffer));
//    if (bytesRead > 0) {
//// Process the received binary data
//        size_t i = 0;
//        while (i < bytesRead) {
//            unsigned char byteValue1 = buffer[i];
//            unsigned char byteValue2 = buffer[i+1];
//            unsigned char byteValue3 = buffer[i+2];
//// Check for different markers to determine the structure
////std::cout << pow(byteValue,1) << std::endl;
//            if (byteValue1 == 0xAA && byteValue2 == 0x55 && byteValue3 == 0x04) { // Marker for HeaderA
//                if (i + sizeof(HeaderA) < bytesRead) {
//                    HeaderA* headerA = reinterpret_cast<HeaderA*>(&buffer[i]);
//                    uint16_t gnss_time = swapEndianness(headerA->gnss_week1);
//// Perform further processing based on HeaderA
//                    std::cout << "HeaderA Marker: " << static_cast<int>(headerA->header1) << std::endl;
//                    std::cout << "HeaderA Marker: " << static_cast<int>(headerA->header2) << std::endl;
//                    std::cout << "HeaderA Marker: " << static_cast<int>(headerA->header3) << std::endl;
//                    std::cout << static_cast<int>(buffer[i+3]) <<  " "<< static_cast<int>(buffer[i+4]) << std::endl;
//                    std::cout << "gnss_week: " << headerA->gnss_week1 << std::endl;
//                    std::cout << "gnss_week: " << gnss_time << std::endl;
//                    std::cout << std::endl;
//
//                    i += sizeof(HeaderA); // Move to the next structure
//                }
//            }
//            else if (byteValue == 0xBB) { // Marker for HeaderB
//                if (i + sizeof(HeaderB) < bytesRead) {
//                    HeaderB* headerB = reinterpret_cast<HeaderB*>(&buffer[i]);
//// Perform further processing based on HeaderB
//                    std::cout << "HeaderB Marker: " << static_cast<int>(headerB->marker) << std::endl;
//                    std::cout << "DataB: " << headerB->dataB << std::endl;
//                    std::cout << std::endl;
//
//                    i += sizeof(HeaderB); // Move to the next structure
//                }
//            }
//            else {
//// Unknown marker or unrecognized data, skip to the next byte
//                ++i;
//            }
//        }
//    } else if (bytesRead < 0) {
//        // Error handling if read fails
//        std::cout << "Error reading from serial port." << std::endl;
//    }
//}

int main() {
//    const char* serialPortPath = "/dev/ttyUSB0";
//    // Replace with your serial port path
//    // Open the serial port
//     int serialPort = open(serialPortPath, O_RDWR);
//     std::cout <<"Open the serial port";
//     if (serialPort != -1) {
//         // Configure the serial port settings
//          struct termios tty; tcgetattr(serialPort, &tty);
//          cfsetispeed(&tty, B115200);
//          // Set the baud rate
//           tty.c_cflag |= (CLOCAL | CREAD);
//           // Enable receiver and set local mode
//            tcsetattr(serialPort, TCSANOW, &tty);
//         while (true)
//            parseSerialData(serialPort);
//            // Close the serial port
//             close(serialPort);
//             } else {
//         // Error handling if open fails
//          std::cout << "Error opening serial port." << std::endl;
//     }
//     return 0;
}


//int main()
//{
//    SerialPort serialPort("/dev/ttyUSB0", 115200, 1000); // 替换为实际的串口名称和波特率
//
//    serialPort.write("Hello, serial port!");
//
//    unsigned char checkByte1;
//    unsigned char checkByte2;
//    unsigned char receivedDatas[60];
//    int count = 0;
//
//
//
//
//    while (true)
//    {
//        if (serialPort.readByte(checkByte1))
//        {
//
//            std::cout << "Received data: " << std::hex << (unsigned int)checkByte1 << std::endl;
//            switch (checkByte1) {
//                case 0xAA:
//
//            }
//            if (checkByte1 == 0xAA) { // Marker for HeaderA
//                std::cout << std::dec << count << "   0xAA data " << checkByte1 << std::endl;
//                count = 0;
//                serialPort.readBytes(receivedDatas, 60);
//                for (int i = 0; i < 60 ; i++){
//                    std::cout << std::hex << (unsigned int)receivedDatas[i] << " ";
//                }
//                std::cout << std::endl;
//            }else
//            count ++;
//        }
//    }
//
//    return 0;
//}

