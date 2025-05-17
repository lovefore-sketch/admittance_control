#include "tcpsocket.h"

bool init_Socket()
{
	//WSADATA* wsadate = new WSADATA();
	WSADATA wsadate;
	if(0 != WSAStartup(MAKEWORD(2,2),&wsadate)) //windows socket windows�첽�׽���
	{
		cout << "[eroor]WSAStarup failed" << " code " << WSAGetLastError() << endl;
		return false;
	}
	return true;
}

bool close_Socket()
{
	
	if (0 != WSACleanup()) //windows socket windows�첽�׽���
	{
		err("WSACleanup");
		return false;
	}
	return true;
}

SOCKET creat_serverSocket()
{
	//1.����һ���յ�socket
	SOCKET fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);//AF_INET:ipv4
	if (INVALID_SOCKET == fd)
	{
		err("socket");
		return INVALID_SOCKET;
	}

	//2.��socket�󶨱��ص�ip��ַ�Ͷ˿ں�
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);  //�ѱ����ֽ���תΪ�����ֽ��򣬴�˴洢��С�˴洢
	addr.sin_addr.S_un.S_addr = ADDR_ANY; //�󶨱�������ip
	if (SOCKET_ERROR == bind(fd, (struct sockaddr*)&addr, sizeof(addr)))
	{
		err("bind");
		return INVALID_SOCKET;
	}

	//3.��ʼ����
	listen(fd, 10);
	
	return fd;
}

SOCKET create_clientSocket(const char* ip)
{
	// 1.����һ���յ�socket
	SOCKET fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);//AF_INET:ipv4
	if (INVALID_SOCKET == fd)
	{
		err("socket");
		return INVALID_SOCKET;
	}
	

	//2.��socket�󶨷���˵�ip��ַ�Ͷ˿ں�
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);  //�ѱ����ֽ���תΪ�����ֽ��򣬴�˴洢��С�˴洢
	addr.sin_addr.S_un.S_addr = inet_addr(ip); //�󶨷�����ip
	if (INVALID_SOCKET == connect(fd, (LPSOCKADDR)&addr, sizeof(addr)))
	{
		err("connet");
		return INVALID_SOCKET;
	}

	
	return fd;
}
