/* ͨ��UDP��ȡһ�δ����������� */
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
#define NUM_SAMPLES 1 /* �ɼ�һ������ */
#pragma comment(lib, "ws2_32.lib")

HANDLE W_InitSerialPort(const char* szStr);
HANDLE InitSerialPort(const char* szStr);
int strsearch(const char* p1, const char* p2);
int GSMatoi(char* s);

double ForceInt[5];
double LastForceInt[5] = { 0.001,0.001,0.001,0.001,0.001};
double DeltaR[5];
using namespace std;
/* �������ݸ�ʽ��UDP�������� */
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

int freq = 1; //����Ƶ��
std::vector<float> t1; //x���������
std::vector<float> fx; //��Fx
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
WSAData wsd;           //��ʼ����Ϣ
int ReadATIForce(const char* ip, const int port,int mode, fstream& InFile)
{

	if (mode == 0)
	{
		//0.����Winsock
		if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0) {/*����WinSocket�ĳ�ʼ��,
			windows ��ʼ��socket����⣬����2��2�İ汾��windows socket��̱����ȳ�ʼ����*/
			cout << "WSAStartup Error = " << WSAGetLastError() << endl;
			return 0;
		}
		else {
			cout << "WSAStartup Success" << endl;
		}

		//1.��ʼ����ʼUDPͨ�� */
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
		/* ����ο� 9.1 in Net F/T user manual. */
		*(uint16*)&request[0] = htons(0x1234);
		*(uint16*)&request[2] = htons(COMMAND);
		*(uint32*)&request[4] = htonl(NUM_SAMPLES);

		//2.��socket�󶨷���˵�ip��ַ�Ͷ˿ں�
		struct sockaddr_in svr_addr;
		int addrlen = sizeof(struct sockaddr_in);

		svr_addr.sin_family = AF_INET;
		svr_addr.sin_port = htons(port);  //�ѱ����ֽ���תΪ�����ֽ��򣬴�˴洢��С�˴洢
		svr_addr.sin_addr.s_addr = inet_addr("192.168.1.1");
		//3.����UDP//
		connect(socketHandle, (LPSOCKADDR)&svr_addr, addrlen);


		//4.�������ݶ�ȡָ��//
		send(socketHandle, request, 8, 0);



		/* 5.������Ӧ */
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

		/* 7.��ӡ��ʾ���� */
		//printf("Status: 0x%08x\n", resp.status); //��е��״̬
		//printf("Rdt_sequence: 0x%08x\n", resp.rdt_sequence); //��״̬��Ϣ
		//printf("Ft_sequence: 0x%08x\n", resp.ft_sequence);


		t1.push_back((freq));
		fx.push_back(resp.FTforce[0]);
		fy.push_back(resp.FTforce[1]);
		fz.push_back(resp.FTforce[2]);
		tx.push_back(resp.FTforce[3]);
		ty.push_back(resp.FTforce[4]);
		tz.push_back(resp.FTforce[5]);
		//cout << "������" << freq << " ���Ĵ�С��" <<resp.FTforce[2] << endl;
		//plt::figure_size(800, 600); ���ظ�����

		freq = freq + 1;

		for (i = 0; i < 6; i++)
		{
			//printf("%s: %d  ", AXES[i], resp.FTData[i]); //��������δ��λת����Ϣ
			printf("%s: %.5f\n", AXES[i], resp.FTforce[i]); 
			ContactForce[i] = resp.FTforce[i];
		}


		//8.��ӡ���ݵ��ļ�
		if (ForceOut == 0)  //��������
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
		//9.���������
		unsigned char CSdata[28];
		unsigned char Tempbuff[4];
		char Begin[] = { 0x03,0xFC };
		char End[] = { 0xFC,0x03 };
		//��ֵ
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
	HANDLE hCom1 = CreateFile(wszClassName,//COM1��
		GENERIC_READ | GENERIC_WRITE, //�������д
		0, //��ռ��ʽ
		NULL,
		OPEN_EXISTING, //�򿪶����Ǵ���
		0, //ͬ����ʽ
		NULL);

	//if (hCom1 == (HANDLE)-1)
	if (hCom1 == INVALID_HANDLE_VALUE)
	{
		printf("��COMʧ��!\n");
		//return FALSE;
	}
	else
	{
		printf("COM�򿪳ɹ���\n");
	}

	SetupComm(hCom1, 1024, 1024); //���뻺����������������Ĵ�С����1024
	COMMTIMEOUTS TimeOuts;
	//�趨����ʱ
	TimeOuts.ReadIntervalTimeout = 100;
	TimeOuts.ReadTotalTimeoutMultiplier = 5000;
	TimeOuts.ReadTotalTimeoutConstant = 5000;
	//�趨д��ʱ
	TimeOuts.WriteTotalTimeoutMultiplier = 500;
	TimeOuts.WriteTotalTimeoutConstant = 2000;
	SetCommTimeouts(hCom1, &TimeOuts); //���ó�ʱ
	DCB dcb;
	GetCommState(hCom1, &dcb);
	dcb.BaudRate = 115200; //������Ϊ9600
	dcb.ByteSize = 8; //ÿ���ֽ���8λ
	dcb.Parity = NOPARITY; //����żУ��λ
	dcb.StopBits = ONESTOPBIT; //1��ֹͣλ
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
	HANDLE hCom1 = CreateFile(wszClassName,//COM1��
		GENERIC_READ | GENERIC_WRITE, //�������д
		0, //��ռ��ʽ
		NULL,
		OPEN_EXISTING, //�򿪶����Ǵ���
		0, //ͬ����ʽ
		NULL);

	//if (hCom1 == (HANDLE)-1)
	if (hCom1 == INVALID_HANDLE_VALUE)
	{
		//printf("��COMʧ��!\n");
		return FALSE;
	}
	else
	{
		//printf("COM�򿪳ɹ���\n");
	}

	SetupComm(hCom1, 1024, 1024); //���뻺����������������Ĵ�С����1024
	COMMTIMEOUTS TimeOuts;
	//�趨����ʱ
	TimeOuts.ReadIntervalTimeout = 100;
	TimeOuts.ReadTotalTimeoutMultiplier = 5000;
	TimeOuts.ReadTotalTimeoutConstant = 5000;
	//�趨д��ʱ
	TimeOuts.WriteTotalTimeoutMultiplier = 500;
	TimeOuts.WriteTotalTimeoutConstant = 2000;
	SetCommTimeouts(hCom1, &TimeOuts); //���ó�ʱ
	DCB dcb;
	GetCommState(hCom1, &dcb);
	dcb.BaudRate = 460800; //������Ϊ9600
	dcb.ByteSize = 8; //ÿ���ֽ���8λ
	dcb.Parity = NOPARITY; //����żУ��λ
	dcb.StopBits = ONESTOPBIT; //1��ֹͣλ
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

	//2.��socket�󶨷���˵�ip��ַ�Ͷ˿ں�
	struct sockaddr_in svr_addr;
	int addrlen = sizeof(struct sockaddr_in);

	svr_addr.sin_family = AF_INET;
	svr_addr.sin_port = htons(49152);  //�ѱ����ֽ���תΪ�����ֽ��򣬴�˴洢��С�˴洢
	svr_addr.sin_addr.s_addr = inet_addr("192.168.1.1");
	//3.����UDP//
	connect(socketHandle, (LPSOCKADDR)&svr_addr, addrlen);


	//4.�������ݶ�ȡָ��//
	send(socketHandle, bias, 8, 0);
}
