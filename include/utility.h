//
// Created by lin on 23-6-8.
//

#ifndef INTEGRATED_NAV_DRIVER_GENERATENAVSATFIXMSG_H
#define INTEGRATED_NAV_DRIVER_GENERATENAVSATFIXMSG_H

#endif //INTEGRATED_NAV_DRIVER_GENERATENAVSATFIXMSG_H

#include <cmath>
#include <ctime>
#include <cstring>
#include <boost/endian/conversion.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/hex.hpp>
#include <boost/lexical_cast.hpp>

int isLittleEndian() {
    int data = 1;
    if (1 == *(uint8_t *) &data) {
        return 1;
    } else {
        return 0;
    }
}

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

std::string bytes2String(const uint8_t *str, unsigned int num){
//    return std::string((char *)str);
//    std::string s((char *)str, sizeof(str));
    return std::string((char *)str, num);
}

std::string string2Hex(const std::string& str) //transfer string to hex-string
{
    std::string result="0x";
    std::string tmp;
    std::stringstream ss;
    for(int i=0;i<str.size();i++)
    {
        ss<<std::hex<<int(str[i])<<std::endl;
        ss>>tmp;
        result+=tmp;
    }
    return result;
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

int16_t byte2Int16(const std::string& bytes) {
    if (bytes.length() != sizeof(int16_t)) {
        // 字节长度不匹配，抛出异常或返回默认值
        throw std::invalid_argument("Invalid byte length");
    }

    // 将字节串转换为十六进制字符串
    std::string hexString;
    boost::algorithm::hex(bytes, std::back_inserter(hexString));

    // 将十六进制字符串转换为 unsigned int
    uint16_t intValue;
    std::stringstream ss;
    ss << std::hex << hexString;
    ss >> intValue;

    // 执行字节交换
    uint16_t swappedValue = boost::endian::endian_reverse(intValue);

    // 将 unsigned int 转换为浮点数
    int16_t result;
    std::memcpy(&result, &swappedValue, sizeof(int16_t));

//    std::cout << result << std::endl;

    return result;
}

float_t byte2Float(const std::string& bytes) {
    if (bytes.length() != sizeof(float_t)) {
    // 字节长度不匹配，抛出异常或返回默认值
        throw std::invalid_argument("Invalid byte length");
    }

    // 将字节串转换为十六进制字符串
    std::string hexString;
    boost::algorithm::hex(bytes, std::back_inserter(hexString));

    // 将十六进制字符串转换为 unsigned int
    uint32_t intValue;
    std::stringstream ss;
    ss << std::hex << hexString;
    ss >> intValue;

    // 执行字节交换
    uint32_t swappedValue = boost::endian::endian_reverse(intValue);

    // 将 unsigned int 转换为浮点数
    float_t result;
    std::memcpy(&result, &swappedValue, sizeof(float_t));

//    std::cout << result << std::endl;

    return result;
}

double_t byte2Double(const std::string& bytes) {
    if (bytes.length() != sizeof(double_t)) {
        // 字节长度不匹配，抛出异常或返回默认值
        std::cout << bytes.length() << "   " << sizeof(double_t) << std::endl;
        throw std::invalid_argument("Invalid byte length");
    }

    // 将字节串转换为十六进制字符串
    std::string hexString;
    boost::algorithm::hex(bytes, std::back_inserter(hexString));

    // 将十六进制字符串转换为 unsigned int
    uint64_t intValue;
    std::stringstream ss;
    ss << std::hex << hexString;
    ss >> intValue;

    // 执行字节交换
    uint64_t swappedValue = boost::endian::endian_reverse(intValue);

    // 将 unsigned int 转换为浮点数
    double_t result;
    std::memcpy(&result, &swappedValue, sizeof(double_t));

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