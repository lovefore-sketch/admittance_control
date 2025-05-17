#include "tcpsocket.h"

bool init_Socket()
{
	//WSADATA* wsadate = new WSADATA();
	WSADATA wsadate;
	if(0 != WSAStartup(MAKEWORD(2,2),&wsadate)) //windows socket windows异步套接字
	{
		cout << "[eroor]WSAStarup failed" << " code " << WSAGetLastError() << endl;
		return false;
	}
	return true;
}

bool close_Socket()
{
	
	if (0 != WSACleanup()) //windows socket windows异步套接字
	{
		err("WSACleanup");
		return false;
	}
	return true;
}

SOCKET creat_serverSocket()
{
	//1.创建一个空的socket
	SOCKET fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);//AF_INET:ipv4
	if (INVALID_SOCKET == fd)
	{
		err("socket");
		return INVALID_SOCKET;
	}

	//2.给socket绑定本地的ip地址和端口号
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);  //把本地字节序转为网络字节序，大端存储和小端存储
	addr.sin_addr.S_un.S_addr = ADDR_ANY; //绑定本地任意ip
	if (SOCKET_ERROR == bind(fd, (struct sockaddr*)&addr, sizeof(addr)))
	{
		err("bind");
		return INVALID_SOCKET;
	}

	//3.开始监听
	listen(fd, 10);
	
	return fd;
}

SOCKET create_clientSocket(const char* ip)
{
	// 1.创建一个空的socket
	SOCKET fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);//AF_INET:ipv4
	if (INVALID_SOCKET == fd)
	{
		err("socket");
		return INVALID_SOCKET;
	}
	

	//2.给socket绑定服务端的ip地址和端口号
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);  //把本地字节序转为网络字节序，大端存储和小端存储
	addr.sin_addr.S_un.S_addr = inet_addr(ip); //绑定服务器ip
	if (INVALID_SOCKET == connect(fd, (LPSOCKADDR)&addr, sizeof(addr)))
	{
		err("connet");
		return INVALID_SOCKET;
	}

	
	return fd;
}
