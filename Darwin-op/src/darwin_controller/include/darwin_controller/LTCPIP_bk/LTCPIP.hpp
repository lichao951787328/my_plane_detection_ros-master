// LTCPIP/LTCPIP.hpp
// lee, hexb66@bit.edu.cn
// Mar. 22, 2022
#pragma once
#include "TCP.hpp"

_L_TCPIP_BEGIN

template<typename SEND_TYPE, typename RECV_TYPE>
class ServerTCP:public TCP<SEND_TYPE,RECV_TYPE>
{
protected:
    SOCKET      ClientSock;
    sockaddr_in ClientAddr;

public:
    inline auto &getClientSock(){return this->ClientSock;};
    int init(const int &_Port=5050)
    {   
        if(!this->TCP<SEND_TYPE,RECV_TYPE>::init("", _Port))
        {
            return 0;
        }
        bind(this->Sock, (struct sockaddr *)&this->Addr, sizeof(this->Addr));
        listen(this->Sock, 100);
        this->Msg = "Listening ...         ";
        return 1;
    };

    void waitForConnection()
    {
        this->Msg = "Waiting for connection ... ";
        int Len = sizeof(this->ClientAddr);
        this->ClientSock = accept(this->Sock, (struct sockaddr *)&this->ClientAddr, &Len);
        if(this->ClientSock==INVALID_SOCKET)
        {
            this->Msg = "Connection closed!          ";
            return;
        }
        this->Msg = "Connected               ";
    };
    RECV_TYPE recvData()
    {
        return this->TCP<SEND_TYPE,RECV_TYPE>::recvData(this->ClientSock);
    }; 
    void sendData(const SEND_TYPE &_Data)
    {
        this->TCP<SEND_TYPE,RECV_TYPE>::sendData(_Data, this->ClientSock);
    };
    void close()
    {
        this->TCP<SEND_TYPE,RECV_TYPE>::close();
        closesocket(this->ClientSock);
    };
};

template <typename SEND_TYPE, typename RECV_TYPE>
class ClientTCP:public TCP<SEND_TYPE,RECV_TYPE>
{
protected:

public:
    int connect()
    {
        if(::connect(this->Sock,(struct sockaddr*)&this->Addr, sizeof(this->Addr))==SOCKET_ERROR)
        {
            this->Msg = "Connect error!";
            return 0;
        }
        this->Msg = "Connect sucess";
        return 1;
    };
};

_L_TCPIP_END