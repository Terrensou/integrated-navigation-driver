//
// Created by lin on 23-6-8.
//

#ifndef INTEGRATED_NAV_DRIVER_GENERATENAVSATFIXMSG_H
#define INTEGRATED_NAV_DRIVER_GENERATENAVSATFIXMSG_H

#endif //INTEGRATED_NAV_DRIVER_GENERATENAVSATFIXMSG_H

#include <bitset>
#include <cmath>
#include <ctime>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <boost/endian/conversion.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/hex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/crc.hpp>

// Utility Func
bool isNativeLittleEndian() { int a = 1;
    int num = (*(char*)&a);
    //&a 取出a的地址； (char*)&a 代表a变量地址的第一个字节的地址
     if (num == 1)
         return true;
     else
         return false;

}

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

std::string bytes2String(const uint8_t *str, unsigned int num){

    return std::string((char *)str, num);
}

std::string chat2BinaryString(const char c){
    std::bitset<8> binary(c);
    return binary.to_string();
}


std::string string2Hex(std::string input, const bool is_uppercase = false, const bool reverse = false) {
    if (reverse) {
        std::reverse(input.begin(), input.end());
    }
    std::ostringstream hexStream;
    hexStream << std::hex << std::setfill('0');
    if (is_uppercase){
        hexStream << std::uppercase;
    }
    for (char c : input) {
        hexStream << std::setw(2) << static_cast<int>(static_cast<unsigned char>(c));
    }
    return hexStream.str();
}

template <typename T>
T byte2toNumber(const std::string& bytes) {
    if (bytes.length() != sizeof(uint16_t)) {
        // 字节长度不匹配，抛出异常或返回默认值
        std::cout << "receive bytes length = " <<bytes.length() << " , but size of tranform type: " << sizeof(uint16_t) << std::endl;
        throw std::invalid_argument("Invalid byte length");
    }

    // 将字节串转换为十六进制字符串
    std::string hexString;
    boost::algorithm::hex(bytes, std::back_inserter(hexString));

    // 将十六进制字符串转换回字节
    std::string unhexString;
    boost::algorithm::unhex(hexString, std::back_inserter(unhexString));

    // 根据当前系统的字节顺序将字节转换
    T result;
    std::memcpy(&result, unhexString.data(), sizeof(T));
    boost::endian::little_to_native_inplace(result);

    return result;
}

template <typename T>
T byte4toNumber(const std::string& bytes) {
    if (bytes.length() != sizeof(uint32_t)) {
        // 字节长度不匹配，抛出异常或返回默认值
        std::cout << "receive bytes length = " <<bytes.length() << " , but size of tranform type: " << sizeof(uint32_t) << std::endl;
        throw std::invalid_argument("Invalid byte length");
    }

    // 将字节串转换为十六进制字符串
    std::string hexString;
    boost::algorithm::hex(bytes, std::back_inserter(hexString));

    // 将十六进制字符串转换回字节
    std::string unhexString;
    boost::algorithm::unhex(hexString, std::back_inserter(unhexString));

    // 根据当前系统的字节顺序将字节转换为无符号整数
    T result;
    std::memcpy(&result, unhexString.data(), sizeof(T));
    boost::endian::little_to_native_inplace(result);

    return result;
}

template <typename T>
T byte8toNumber(const std::string& bytes) {
    if (bytes.length() != sizeof(uint64_t)) {
        // 字节长度不匹配，抛出异常或返回默认值
        std::cout << "receive bytes length = " <<bytes.length() << " , but size of tranform type: " << sizeof(uint64_t) << std::endl;
        throw std::invalid_argument("Invalid byte length");
    }

    // 将字节串转换为十六进制字符串
    std::string hexString;
    boost::algorithm::hex(bytes, std::back_inserter(hexString));

    // 将十六进制字符串转换回字节
    std::string unhexString;
    boost::algorithm::unhex(hexString, std::back_inserter(unhexString));

    // 根据当前系统的字节顺序将字节转换为无符号整数
    T result;
    std::memcpy(&result, unhexString.data(), sizeof(T));
    boost::endian::little_to_native_inplace(result);

    return result;
}

int checkSum(const std::string str) {
    const char *c_str = str.c_str();
    int result = c_str[0];
    for (int i=1; c_str[i] != '*' ; i++) {
        result ^= c_str[i];
//        std::cout << c_str[i] << " ";
    }
    return result;
}

uint32_t crc32Check(const std::string data) {
    boost::crc_32_type crc32;

    crc32.process_bytes(data.data(), data.length());

    return crc32.checksum();
};
#define CRC32_POLYNOMIAL 0xEDB88320L
unsigned long CRC32Value(int i) {
    int j;
    unsigned long ulCRC;
    ulCRC = i;
    for (j = 8; j > 0; j--) {
        if (ulCRC & 1)
            ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
        else
            ulCRC >>= 1;
    }
    return ulCRC;
}

unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char *ucBuffer) {
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;

    while (ulCount-- != 0) {
        ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value(((int)ulCRC ^* ucBuffer++)&0xFF);
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return (ulCRC);
}