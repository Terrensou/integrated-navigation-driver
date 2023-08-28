#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <boost/circular_buffer.hpp>
#include <queue>
#include <vector>

class SerialPort
{
public:
    SerialPort(const std::string& portName, int baudRate, size_t bufferSize)
            : portName(portName), baudRate(baudRate), stopRequested(false), buffer(bufferSize)
    {
        openSerialPort();
        startReadingThread();
    }

    ~SerialPort()
    {
        stopRequested = true;
        if (readingThread.joinable())
            readingThread.join();
        closeSerialPort();
    }

    void write(const std::string& data)
    {
        std::lock_guard<std::mutex> lock(writeMutex);
        writeQueue.push(data);
        writeCondition.notify_one();
    }

    bool readByte(unsigned char& data)
    {
        std::lock_guard<std::mutex> lock(readMutex);
        if (!buffer.empty())
        {
            data = buffer.front();
            buffer.pop_front();
            return true;
        }
        return false;
    }

    uint16_t readBytes(unsigned char *data, uint16_t length)
    {
        uint16_t read_count = 0;

        for (auto it = buffer.begin(); read_count != length;) {
            if (!buffer.empty())
            {
//                std::lock_guard<std::mutex> lock(readMutex);
//                data.push_back(buffer.front());
                data[read_count++] = buffer.front();
                buffer.pop_front();
//                read_count ++;
            } else {
                usleep(10);
            }
        }
        return read_count;
    }

private:
    void openSerialPort()
    {
        serialPort = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serialPort == -1)
        {
            std::cerr << "Failed to open serial port." << std::endl;
            return;
        }

        struct termios options;
        tcgetattr(serialPort, &options);
        cfsetispeed(&options, baudRate);
        cfsetospeed(&options, baudRate);
//        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;

        options.c_cc[VTIME] = 0;	/*设置等待时间*/
        options.c_cc[VMIN] = 0;	/*设置最小字符*/

        // Raw mode
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

        tcsetattr(serialPort, TCSANOW, &options);
        fcntl(serialPort, F_SETFL, 0);
    }

    void closeSerialPort()
    {
        close(serialPort);
    }

    void startReadingThread()
    {
        readingThread = std::thread([this]() {
            char buffer[1024];
            ssize_t bytesRead;

            while (!stopRequested)
            {
                bytesRead = ::read(serialPort, buffer, sizeof(buffer));
                if (bytesRead > 0)
                {
                    std::lock_guard<std::mutex> lock(readMutex);
                    for (int i = 0; i < bytesRead; ++i)
//                        this->buffer.push_back(std::string(1, buffer[i]));
                    this->buffer.push_back(buffer[i]);
                }
                else if (bytesRead == -1)
                {
                    std::cerr << "Error reading from serial port." << std::endl;
                }

                usleep(1000); // 延迟一段时间，避免CPU过载
            }
        });
    }

    std::string portName;
    int baudRate;
    std::thread readingThread;
    std::atomic<bool> stopRequested;
    std::mutex readMutex;
    std::mutex writeMutex;
    std::condition_variable writeCondition;
    boost::circular_buffer<char> buffer;
    std::queue<std::string> writeQueue; // 新增的 writeQueue 变量
    int serialPort;
};






//#include <cstdlib>
//#include <unistd.h>
//#include <sys/types.h>
//#include <sys/signal.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <termios.h>
//#include <cerrno>
//#include <climits>
//#include <string>
//#include <thread>
//
//const long long RecvBufferLen = 1024;   //设置接收数据缓冲区大小
//
//typedef enum
//{
//    _2400,
//    _4800,
//    _9600,
//    _19200,
//    _38400,
//    _57600,
//    _115200,
//    _460800,
//} E_BaudRate;  //波特率
//
//typedef enum
//{
//    _5,
//    _6,
//    _7,
//    _8,
//}E_DataSize;  //数据位
//
//typedef enum
//{
//    None,
//    Odd,
//    Even,
//}E_Parity;  //校验位
//
//typedef enum
//{
//    _1,
//    _2,
//}E_StopBit;  //停止位
//
//
//class Serial
//{
//public:
//    Serial(){
//        b_OpenSign = false;
//        nSerialID = 0;
//    }
//
//    ~Serial(){
//        Close();
//    }
//
//
//    int OpenSerial(std::string port, E_BaudRate Bps, E_DataSize DataSize, E_Parity Parity, E_StopBit StopBit)
//    {
//        Close();
//        nSerialID = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
//        if (-1 == nSerialID)
//        {
//            /* 不能打开串口一*/
//            std::string str = port + " open fail !!!";
//            perror(str.c_str());
//            return -1;
//        }
//        struct termios Opt;
//        tcgetattr(nSerialID, &ProtoOpt);  //获取设备当前的设置
//
//        Opt = ProtoOpt;
//
//        /*设置输入输出波特率*/
//        switch(Bps)
//        {
//            case E_BaudRate::_2400:
//                cfsetispeed(&Opt,B2400);
//                cfsetospeed(&Opt,B2400);
//                break;
//            case E_BaudRate::_4800:
//                cfsetispeed(&Opt,B4800);
//                cfsetospeed(&Opt,B4800);
//                break;
//            case E_BaudRate::_9600:
//                cfsetispeed(&Opt,B9600);
//                cfsetospeed(&Opt,B9600);
//                break;
//            case E_BaudRate::_19200:
//                cfsetispeed(&Opt,B19200);
//                cfsetospeed(&Opt,B19200);
//                break;
//            case E_BaudRate::_38400:
//                cfsetispeed(&Opt,B38400);
//                cfsetospeed(&Opt,B38400);
//                break;
//            case E_BaudRate::_57600:
//                cfsetispeed(&Opt,B57600);
//                cfsetospeed(&Opt,B57600);
//                break;
//            case E_BaudRate::_115200:
//                cfsetispeed(&Opt,B115200);
//                cfsetospeed(&Opt,B115200);
//                break;
//            case E_BaudRate::_460800:
//                cfsetispeed(&Opt,B460800);
//                cfsetospeed(&Opt,B460800);
//                break;
//            default		:
//                ROS_ERROR("Don't exist baudrate %d !\n",Bps);
//                return (-1);
//        }
//
//        /*设置数据位*/
//        Opt.c_cflag &= (~CSIZE);
//        switch( DataSize )
//        {
//            case E_DataSize::_5:
//                Opt.c_cflag |= CS5;
//                break;
//            case E_DataSize::_6:
//                Opt.c_cflag |= CS6;
//            case E_DataSize::_7:
//                Opt.c_cflag |= CS7;
//                break;
//            case E_DataSize::_8:
//                Opt.c_cflag |= CS8;
//                break;
//            default:
//                ROS_ERROR("Don't exist DataSize %d !\n",DataSize);
//                return -1;
//        }
//
//        /*设置校验位*/
//        switch( Parity )
//        {
//            case E_Parity::None:					/*无校验*/
//                Opt.c_cflag &= (~PARENB);
//                break;
//            case E_Parity::Odd:					/*奇校验*/
//                Opt.c_cflag |= PARENB;
//                Opt.c_cflag |= PARODD;
//                Opt.c_iflag |= (INPCK | ISTRIP);
//                break;
//            case E_Parity::Even:					/*偶校验*/
//                Opt.c_cflag |= PARENB;
//                Opt.c_cflag &= (~PARODD);
//                Opt.c_iflag |= (INPCK | ISTRIP);
//                break;
//            default:
//                ROS_ERROR("Don't exist Parity %c !\n",Parity);
//                return -1;
//        }
//
//        /*设置停止位*/
//        switch( StopBit )
//        {
//            case E_StopBit::_1:
//                Opt.c_cflag &= (~CSTOPB);
//                break;
//            case E_StopBit::_2:
//                Opt.c_cflag |= CSTOPB;
//                break;
//            default:
//                ROS_ERROR("Don't exist iStopBit %d !\n",StopBit);
//                return -1;
//        }
//
//        //如果只是串口传输数据，而不需要串口来处理，那么使用原始模式(Raw Mode)方式来通讯，设置方式如下：
//        Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//        Opt.c_oflag &= ~OPOST;
//
//        tcflush(nSerialID,TCIOFLUSH);		/*刷新输入队列(TCIOFLUSH为刷新输入输出队列)*/
//
//        Opt.c_cc[VTIME] = 0;	/*设置等待时间*/
//        Opt.c_cc[VMIN] = 0;	/*设置最小字符*/
//
//        int Result = tcsetattr(nSerialID,TCSANOW,&Opt);  //使这些设置生效
//
//        if( Result )
//        {
//            ROS_ERROR_STREAM("Set new terminal description error !");
//            return (-1);
//        }
//
//        b_OpenSign = true;
//
//        RunRecv();
//
//        return 0;
//    }
//
//    int Recv(unsigned char *Buff, int length)
//    {
//        int res =  RefreshBuffer(Buff, length, true);
//
//        return res;
//    }
//
//    int Send(unsigned char *Buff, int length){
//        int iLen = 0;
//        if(length <= 0)
//        {
//            printf("Send byte number error !\n");
//            return -1;
//        }
//
//        iLen = write(nSerialID,Buff,length);
//
//        return iLen;
//    }
//
//    int Close(){
//        if(nSerialID > 0)
//        {
//            tcsetattr (nSerialID, TCSADRAIN, &ProtoOpt);  //恢复原始串口配置
//        }
//        close(nSerialID);
//        b_OpenSign = false;
//    }
//
//private:
//    void RunConnect(){
//
//    }
//    void RunRecv(){
//        std::thread ThRecv	= std::thread
//                {
//                        [&]()
//                        {
//                            unsigned char RecvBuf[4096] = {0};
//                            while (b_OpenSign)
//                            {
//                                usleep(10*1000);
//                                if((nSerialID < 0))
//                                {
//                                    continue;
//                                }
//
//                                memset(RecvBuf, 0, 4096);
//
//                                int res = read(nSerialID, RecvBuf, sizeof(RecvBuf));
//                                //std::cout << "res = " << res << std::endl;
//                                if(res > 0)
//                                {
//                                    RefreshBuffer(RecvBuf, res, false);
//                                }
//                            }
//                        }
//                };
//
//        ThRecv.detach();
//    }
//
//
//
//    int RefreshBuffer(unsigned char *pBuf, int Len, bool RecvTypet)
//    {
//        static unsigned char  Buffer[RecvBufferLen + 1] = {0};
//        static int 					nSum=0;				    //	缓冲区中数据总长度
//        signed int 					nStop=0;
//
//        int ren = 0;
//
//        if(false == RecvTypet)
//        {
//            //************************ 将接收到的数据加入缓冲区中 ************************/
//            //std::cout<<"recv = "<< Len <<std::endl;
//
//            if((Len + nSum) <= RecvBufferLen)		//	总长度小于1K
//            {
//                memcpy(&Buffer[nSum], pBuf, Len);
//                nSum = Len + nSum;
//            }
//            else
//            {
//                if(Len <= RecvBufferLen)			//	拷贝满1K空间，丢弃掉aucT[0]开始的字符，并进行填充，!!!!!!!!!!!
//                {
//                    memcpy(Buffer, pBuf, Len);
//                    nSum = Len;
//                }
//                else						//	本次接收到的数据长度大于1K
//                {
//                    memcpy(Buffer, pBuf + (Len - RecvBufferLen), RecvBufferLen);
//                    nSum = RecvBufferLen;
//                }
//            }
//            //std::cout<<"----> nSum = "<< nSum <<std::endl;
//            ren = 0;
//        }
//        else
//        {
//            if(Len <= 0)
//            {
//                return -1;
//            }
//            if(nSum <= 0)
//            {
//                return 0;
//            }
//
//            if(Len <= nSum)
//            {
//                memcpy(pBuf, Buffer, Len);
//
//                nStop =  Len;
//                ren = Len;
//            }
//            else
//            {
//                memcpy(pBuf, Buffer, nSum);
//
//                nStop = nSum;
//                ren = nSum;
//            }
//
//
//            //************ 移动取出数据 ***************/
//            if(nStop==0)
//            {
//                return 0;
//            }
//            else if(nSum > nStop)	  // 把没有解析到的数据移动到最开始位置
//            {
//                for(int i=0; i<(nSum-nStop); i++)
//                {
//                    Buffer[i] = Buffer[nStop + i];
//                }
//                nSum = nSum - nStop;
//            }
//            else if(nSum == nStop)
//            {
//                nSum = 0;
//            }
//
//        }
//
//        return ren;
//    }
//
//
//private:
//    int nSerialID;  //串口
//
//    bool b_OpenSign;   //串口打开标志
//
//    struct termios ProtoOpt;   //存放串口原始配置
//};
//
//
