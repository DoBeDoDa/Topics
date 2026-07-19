#pragma once
#include <winsock2.h>
#include <string>

class SocketClient {
private:
    SOCKET clientSocket;
    bool connected;
    std::string receiveBuffer;
public:
    SocketClient();
    ~SocketClient();

    // 連接與斷開 TCP 伺服器
    bool connectToServer(const std::string& ip, int port);
    void closeConnection();
    bool isConnected() const;

    // 接收與發送資料與清除緩衝區
    int receiveData(char* buf, int max_len);
    int receiveLine(std::string& line);
    int sendData(const std::string& data);
    void flushBuffer();
};
