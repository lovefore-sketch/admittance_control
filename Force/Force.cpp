/* 通过UDP读取一次传感器的数据 */
#include <sys/types.h>
#include "..\socket\tcpsocket.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <winsock.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sstream>  
#include <Windows.h>
#include<thread> 
#include <vector>
#include "Force.h"


#define COMMAND 2  
#define SET_SOFWARE_BIAS  0x0042
#define NUM_SAMPLES 1 /* 采集一次数据 */
#pragma comment(lib, "ws2_32.lib")

HANDLE W_InitSerialPort(const char* szStr);
HANDLE InitSerialPort(const char* szStr);
int strsearch(const char* p1, const char* p2);
int GSMatoi(char* s);

double ForceInt[5];
double LastForceInt[5] = { 0.001,0.001,0.001,0.001,0.001};
double DeltaR[5];
using namespace std;
/* 定义数据格式和UDP数据类型 */
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
extern int ForceOut;
extern double ContactForce[6];

typedef struct response_struct 
{
	uint32 rdt_sequence;
	uint32 ft_sequence;
	uint32 status;
	int32 FTData[6];
	double FTforce[6];
} RESPONSE;

void FloatToByte(float floatNum, unsigned char* byteArry)
{
	char* pchar = (char*)&floatNum;
	for (int i = 0; i < sizeof(float); i++)
	{
		*byteArry = *pchar;
		pchar++;
		byteArry++;
	}
}

int freq = 1; //请求频率
std::vector<float> t1; //x轴次数坐标
std::vector<float> fx; //空Fx
std::vector<float> fy;
std::vector<float> fz;
std::vector<float> tx;
std::vector<float> ty;
std::vector<float> tz;
//mode:1  ATI mode:2 Sensor
int pp[6];
int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */
byte request[8];			/* The request data sent to the Net F/T. */
byte bias[8];			    /* The bias request sent to the Net F/T. */
RESPONSE resp;				/* The structured response received from the Net F/T. */
byte response[36];			/* The raw response data received from the Net F/T. */
int i;						/* Generic loop/array index. */

const char* AXES[6] = { "Fx", "Fy", "Fz", "Tx", "Ty", "Tz" };
WSAData wsd;           //初始化信息
int ReadATIForce(const char* ip, const int port,int mode, fstream& InFile)
{

	if (mode == 0)
	{
		//0.启动Winsock
		if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0) {/*进行WinSocket的初始化,
			windows 初始化socket网络库，申请2，2的版本，windows socket编程必须先初始化。*/
			cout << "WSAStartup Error = " << WSAGetLastError() << endl;
			return 0;
		}
		else {
			cout << "WSAStartup Success" << endl;
		}

		//1.初始化开始UDP通信 */
		socketHandle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (socketHandle == SOCKET_ERROR)
		{
			cout << "socket Error = " << WSAGetLastError() << endl;
			exit(1);
		}
		else {
			cout << "socket Success" << endl;
		}
	}
	if (mode == 1)
	{
		/* 具体参考 9.1 in Net F/T user manual. */
		*(uint16*)&request[0] = htons(0x1234);
		*(uint16*)&request[2] = htons(COMMAND);
		*(uint32*)&request[4] = htonl(NUM_SAMPLES);

		//2.给socket绑定服务端的ip地址和端口号
		struct sockaddr_in svr_addr;
		int addrlen = sizeof(struct sockaddr_in);

		svr_addr.sin_family = AF_INET;
		svr_addr.sin_port = htons(port);  //把本地字节序转为网络字节序，大端存储和小端存储
		svr_addr.sin_addr.s_addr = inet_addr("192.168.1.1");
		//3.连接UDP//
		connect(socketHandle, (LPSOCKADDR)&svr_addr, addrlen);


		//4.发送数据读取指令//
		send(socketHandle, request, 8, 0);



		/* 5.接收响应 */
		recv(socketHandle, response, 36, 0);

		//
		resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
		resp.ft_sequence = ntohl(*(uint32*)&response[4]);
		resp.status = ntohl(*(uint32*)&response[8]);
		for (i = 0; i < 6; i++)
		{
			resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
			resp.FTforce[i] = resp.FTData[i] / 1000000.00000;
		}

		/* 7.打印显示数据 */
		//printf("Status: 0x%08x\n", resp.status); //机械臂状态
		//printf("Rdt_sequence: 0x%08x\n", resp.rdt_sequence); //流状态信息
		//printf("Ft_sequence: 0x%08x\n", resp.ft_sequence);


		t1.push_back((freq));
		fx.push_back(resp.FTforce[0]);
		fy.push_back(resp.FTforce[1]);
		fz.push_back(resp.FTforce[2]);
		tx.push_back(resp.FTforce[3]);
		ty.push_back(resp.FTforce[4]);
		tz.push_back(resp.FTforce[5]);
		//cout << "次数：" << freq << " 力的大小：" <<resp.FTforce[2] << endl;
		//plt::figure_size(800, 600); 会重复调用

		freq = freq + 1;

		for (i = 0; i < 6; i++)
		{
			//printf("%s: %d  ", AXES[i], resp.FTData[i]); //力传感器未单位转换信息
			printf("%s: %.5f\n", AXES[i], resp.FTforce[i]); 
			ContactForce[i] = resp.FTforce[i];
		}


		//8.打印数据到文件
		if (ForceOut == 0)  //力输出检测
		{
			ForceOut = 1;
			InFile << "status" <<","<< "rdt_sequence" << "," << "ft_sequence" << "," << "FxCount" << "," << "FyCount" << "," << "FzCount" << ","
				<< "RxCount" << "," << "RyCount" << "," << "RzCount" << ","
				<< "Fx" << "," << "Fy" << "," << "Fz" << "," << "Rx" << "," << "Ry" << "," << "Rz" << endl;
		}
		else
		{
			InFile << resp.status << "," << resp.rdt_sequence << "," << resp.ft_sequence << "," << resp.FTData[0] << "," << resp.FTData[1]
				<< "," << resp.FTData[2] << "," << resp.FTData[3] << "," << resp.FTData[4] << "," << resp.FTData[5] 
				<< "," << resp.FTforce[0] << "," << resp.FTforce[1] << "," << resp.FTforce[2] << "," << resp.FTforce[3]
				<< "," << resp.FTforce[4] << "," << resp.FTforce[5] << endl;

		}
		//9.输出到串口
		unsigned char CSdata[28];
		unsigned char Tempbuff[4];
		char Begin[] = { 0x03,0xFC };
		char End[] = { 0xFC,0x03 };
		//赋值
		CSdata[0] = Begin[0];
		CSdata[1] = Begin[1];
		for (int i = 0; i < 6; i++)
		{
			FloatToByte((float)(resp.FTforce[i]), Tempbuff);
			for (int j = 0; j < 4; j++)
				CSdata[4 * i + 1 * j + 2] = Tempbuff[j];
		}
		CSdata[26] = End[0];
		CSdata[27] = End[1];
		DWORD Writesize;
		HANDLE hCom1 = InitSerialPort("COM7");
		WriteFile(hCom1, CSdata, 16, &Writesize, NULL);
		PurgeComm(hCom1, PURGE_RXCLEAR);
		CloseHandle(hCom1);
	}
	return 0;
}


HANDLE W_InitSerialPort(const char* szStr)
{
	//const char temp[5]="COM4";

	WCHAR wszClassName[5];
	memset(wszClassName, 0, sizeof(wszClassName));
	MultiByteToWideChar(CP_ACP, 0, szStr, strlen(szStr) + 1, wszClassName,
		sizeof(wszClassName) / sizeof(wszClassName[0]));
	HANDLE hCom1 = CreateFile(wszClassName,//COM1口
		GENERIC_READ | GENERIC_WRITE, //允许读和写
		0, //独占方式
		NULL,
		OPEN_EXISTING, //打开而不是创建
		0, //同步方式
		NULL);

	//if (hCom1 == (HANDLE)-1)
	if (hCom1 == INVALID_HANDLE_VALUE)
	{
		printf("打开COM失败!\n");
		//return FALSE;
	}
	else
	{
		printf("COM打开成功！\n");
	}

	SetupComm(hCom1, 1024, 1024); //输入缓冲区和输出缓冲区的大小都是1024
	COMMTIMEOUTS TimeOuts;
	//设定读超时
	TimeOuts.ReadIntervalTimeout = 100;
	TimeOuts.ReadTotalTimeoutMultiplier = 5000;
	TimeOuts.ReadTotalTimeoutConstant = 5000;
	//设定写超时
	TimeOuts.WriteTotalTimeoutMultiplier = 500;
	TimeOuts.WriteTotalTimeoutConstant = 2000;
	SetCommTimeouts(hCom1, &TimeOuts); //设置超时
	DCB dcb;
	GetCommState(hCom1, &dcb);
	dcb.BaudRate = 115200; //波特率为9600
	dcb.ByteSize = 8; //每个字节有8位
	dcb.Parity = NOPARITY; //无奇偶校验位
	dcb.StopBits = ONESTOPBIT; //1个停止位
	SetCommState(hCom1, &dcb);
	return hCom1;
}

HANDLE InitSerialPort(const char* szStr)
{
	//const char temp[5]="COM4";

	WCHAR wszClassName[5];
	memset(wszClassName, 0, sizeof(wszClassName));
	MultiByteToWideChar(CP_ACP, 0, szStr, strlen(szStr) + 1, wszClassName,
		sizeof(wszClassName) / sizeof(wszClassName[0]));
	HANDLE hCom1 = CreateFile(wszClassName,//COM1口
		GENERIC_READ | GENERIC_WRITE, //允许读和写
		0, //独占方式
		NULL,
		OPEN_EXISTING, //打开而不是创建
		0, //同步方式
		NULL);

	//if (hCom1 == (HANDLE)-1)
	if (hCom1 == INVALID_HANDLE_VALUE)
	{
		//printf("打开COM失败!\n");
		return FALSE;
	}
	else
	{
		//printf("COM打开成功！\n");
	}

	SetupComm(hCom1, 1024, 1024); //输入缓冲区和输出缓冲区的大小都是1024
	COMMTIMEOUTS TimeOuts;
	//设定读超时
	TimeOuts.ReadIntervalTimeout = 100;
	TimeOuts.ReadTotalTimeoutMultiplier = 5000;
	TimeOuts.ReadTotalTimeoutConstant = 5000;
	//设定写超时
	TimeOuts.WriteTotalTimeoutMultiplier = 500;
	TimeOuts.WriteTotalTimeoutConstant = 2000;
	SetCommTimeouts(hCom1, &TimeOuts); //设置超时
	DCB dcb;
	GetCommState(hCom1, &dcb);
	dcb.BaudRate = 460800; //波特率为9600
	dcb.ByteSize = 8; //每个字节有8位
	dcb.Parity = NOPARITY; //无奇偶校验位
	dcb.StopBits = ONESTOPBIT; //1个停止位
	SetCommState(hCom1, &dcb);
	return hCom1;
}


int strsearch(const char* p1, const char* p2)
{
	int p1_len = 0, p2_len = 0;
	int i = 0, j = 0, k = 0;
	int pos = 0;
	if (!*p2)return 0;
	p1_len = strlen((char const*)p1);
	p2_len = strlen((char const*)p2);
	for (i = 0; i < p1_len; i++)
	{
		if (*(p1 + i) == *(p2))
		{
			for (k = i, j = 0; j < p2_len; k++, j++)
			{
				if (*(p1 + k) == *(p2 + j))
				{
					pos = i + 1;
					if (j == p2_len - 1)return pos;
				}
				else { pos = 0; break; }
			}
		}
	}
	return pos;
}

int GSMatoi(char* s)
{
	int i;
	int n = 0;
	for (i = 0; s[i] >= '0' && s[i] <= '9'; ++i)
	{
		n = 10 * n + (s[i] - '0');
	}
	return n;
}

void setBias()
{
	*(uint16*)&bias[0] = htons(0x1234);
	*(uint16*)&bias[2] = htons(SET_SOFWARE_BIAS);
	*(uint32*)&bias[4] = htonl(NUM_SAMPLES);

	//2.给socket绑定服务端的ip地址和端口号
	struct sockaddr_in svr_addr;
	int addrlen = sizeof(struct sockaddr_in);

	svr_addr.sin_family = AF_INET;
	svr_addr.sin_port = htons(49152);  //把本地字节序转为网络字节序，大端存储和小端存储
	svr_addr.sin_addr.s_addr = inet_addr("192.168.1.1");
	//3.连接UDP//
	connect(socketHandle, (LPSOCKADDR)&svr_addr, addrlen);


	//4.发送数据读取指令//
	send(socketHandle, bias, 8, 0);
}
