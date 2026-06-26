#pragma once
#include <winsock2.h>
#include <string>

class SocketClient {
private:
    SOCKET clientSocket;
    bool connected;
public:
    SocketClient();
    ~SocketClient();

    // 連接與斷開 TCP 伺服器
    bool connectToServer(const std::string& ip, int port);
    void closeConnection();
    bool isConnected() const;

    // 接收資料與清除緩衝區
    int receiveData(char* buf, int max_len);
    void flushBuffer();
};
