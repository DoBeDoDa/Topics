#include "SocketClient.h"

SocketClient::SocketClient() : clientSocket(INVALID_SOCKET), connected(false) {}

SocketClient::~SocketClient() {
    closeConnection();
}

bool SocketClient::connectToServer(const std::string& ip, int port) {
    clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (clientSocket == INVALID_SOCKET) return false;

    sockaddr_in serverAddr = { AF_INET, htons(port) };
    serverAddr.sin_addr.s_addr = inet_addr(ip.c_str());

    if (connect(clientSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        closesocket(clientSocket);
        clientSocket = INVALID_SOCKET;
        connected = false;
        return false;
    }
    connected = true;
    return true;
}

void SocketClient::closeConnection() {
    if (clientSocket != INVALID_SOCKET) {
        closesocket(clientSocket);
        clientSocket = INVALID_SOCKET;
    }
    connected = false;
}

bool SocketClient::isConnected() const {
    return connected;
}

int SocketClient::receiveData(char* buf, int max_len) {
    if (!connected) return -1;
    return recv(clientSocket, buf, max_len, 0);
}

int SocketClient::sendData(const std::string& data) {
    if (!connected) return -1;
    return send(clientSocket, data.c_str(), static_cast<int>(data.length()), 0);
}

void SocketClient::flushBuffer() {
    if (!connected) return;
    char dummy[1024];
    u_long bytes_available;
    ioctlsocket(clientSocket, FIONREAD, &bytes_available);
    while (bytes_available > 0) {
        recv(clientSocket, dummy, sizeof(dummy) - 1, 0);
        ioctlsocket(clientSocket, FIONREAD, &bytes_available);
    }
}
