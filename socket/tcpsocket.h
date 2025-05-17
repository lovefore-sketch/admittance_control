#pragma once
#include<iostream>
#include<WinSock2.h>

#pragma comment(lib,"ws2_32.lib")
#define err(errmsg) cout << "[eroor] "<< errmsg <<" failed " << "code " << WSAGetLastError() << endl
#define PORT 30003 //0-1024是系统保留

using namespace std;

//初始化网路库
bool init_Socket();
//关闭网络库
bool close_Socket();
//服务器：创建服务器socket
SOCKET creat_serverSocket();
//客户端：创建客户端socket
SOCKET create_clientSocket(const char* ip);
int ReadATIForce(const char* ip, const int port, int mode, fstream& InFile);
void setBias();
//double testCont(double u0);
double function(double input);
