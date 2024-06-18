// LTCPIP/TCP.hpp
// lee, hexb66@bit.edu.cn
// Mar. 22, 2022
#pragma once
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include<netinet/in.h> 
#include <arpa/inet.h>  
#include<netinet/tcp.h> 
#include <unistd.h> 
// #include <utl/data_type.h>
// #include <WINSOCK2.H>
#pragma comment(lib, "wsock32")
#define _L_TCPIP_BEGIN namespace lee{namespace tcpip{
#define _L_TCPIP_END }}

_L_TCPIP_BEGIN

inline bool initSocket()
{
    WSADATA wsd;
    auto nResult = WSAStartup(MAKEWORD(2, 2), &wsd);
    if (nResult != NO_ERROR)
    {
        std::cout << "WSAStartup failed: " << WSAGetLastError() << std::endl;
        return false;
    }
    return true;
};

inline void clearSocket()
{
	WSACleanup();
}

template <typename SEND_TYPE, typename RECV_TYPE>
class TCP
{
protected:
    std::string IP;
    int Port;
    SOCKET      Sock;
    sockaddr_in Addr;
    std::string Msg; 
    char *SendBuf, *RecvBuf;

    bool RecvFlag;

public:
    auto &getRecvFlag(){return this->RecvFlag;};
    auto &getMsg(){return this->Msg;};
    void printMsg(){
        std::cout<<this->Msg<<std::endl;
    };
    TCP():RecvFlag(false)
    {
        this->SendBuf = new char[sizeof(SEND_TYPE)+10];
        this->RecvBuf = new char[sizeof(RECV_TYPE)+10];
        this->Msg = "None";
    };
    ~TCP()
    {
        delete this->SendBuf;
        delete this->RecvBuf;
    };

    int init(const char *_IP="127.0.0.1", const int &_Port=5050)
    {   
        this->Sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if(this->Sock == INVALID_SOCKET)
        {
            this->Msg = "Socket create error!";
            return 0;
        }
        this->IP = std::string(_IP);
        this->Port = _Port;

        this->Addr.sin_family=AF_INET;
        this->Addr.sin_port = htons(this->Port);
        if(this->IP.size()==0)
        {
            this->Addr.sin_addr.s_addr = htonl(INADDR_ANY);
        }
        else
        {
            this->Addr.sin_addr.S_un.S_addr = inet_addr(this->IP.c_str());
        }  
        return 1;
    }

    RECV_TYPE recvData(const SOCKET &_Sock=0)
    {
        this->Msg = "Waiting for data ...       ";
        int num = 0;
        if(_Sock==0)
            num = recv(this->Sock, this->RecvBuf, sizeof(RECV_TYPE), 0);
        else
            num = recv(_Sock, this->RecvBuf, sizeof(RECV_TYPE), 0);
        if(num>=0)
        {
            this->Msg = "Data received              ";
            this->RecvFlag = true;
        }
        else{
            this->Msg = "Receive failed             ";
            this->RecvFlag = false;
        }
        return *((RECV_TYPE*)this->RecvBuf);
    };

    void sendData(const char *_Data, const int &_Len=sizeof(SEND_TYPE), const SOCKET &_Sock=0)
    {
        this->Msg = "Sending data ...          ";
        if(_Sock==0)
            send(this->Sock, _Data, _Len, 0);
        else
            send(_Sock, _Data, _Len, 0);
        this->Msg = "Send over           ";
    };

    void sendData(const SEND_TYPE &_Data, const SOCKET &_Sock=0)
    {
        this->sendData((const char *)&_Data, sizeof(SEND_TYPE),_Sock);
    };

    void close()
    {
        closesocket(this->Sock);
    };
};
_L_TCPIP_END