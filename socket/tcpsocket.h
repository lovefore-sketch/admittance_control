#pragma once
#include<iostream>
#include<WinSock2.h>

#pragma comment(lib,"ws2_32.lib")
#define err(errmsg) cout << "[eroor] "<< errmsg <<" failed " << "code " << WSAGetLastError() << endl
#define PORT 30003 //0-1024��ϵͳ����

using namespace std;

//��ʼ����·��
bool init_Socket();
//�ر������
bool close_Socket();
//������������������socket
SOCKET creat_serverSocket();
//�ͻ��ˣ������ͻ���socket
SOCKET create_clientSocket(const char* ip);
int ReadATIForce(const char* ip, const int port, int mode, fstream& InFile);
void setBias();
//double testCont(double u0);
double function(double input);
