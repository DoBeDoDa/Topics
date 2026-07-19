#include "SocketClient.h"

SocketClient::SocketClient() : clientSocket(INVALID_SOCKET), connected(false) {}

SocketClient::~SocketClient() {
    closeConnection();
}

bool SocketClient::connectToServer(const std::string& ip, int port) {
    receiveBuffer.clear();
    clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (clientSocket == INVALID_SOCKET) return false;

    sockaddr_in serverAddr = {};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
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
    receiveBuffer.clear();
}

bool SocketClient::isConnected() const {
    return connected;
}

int SocketClient::receiveData(char* buf, int max_len) {
    if (!connected) return -1;
    return recv(clientSocket, buf, max_len, 0);
}

int SocketClient::receiveLine(std::string& line) {
    if (!connected) return -1;

    while (true) {
        std::string::size_type newline = receiveBuffer.find('\n');
        if (newline != std::string::npos) {
            line = receiveBuffer.substr(0, newline);
            receiveBuffer.erase(0, newline + 1);
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            return static_cast<int>(line.size());
        }

        char buffer[1024];
        int bytes = recv(clientSocket, buffer, sizeof(buffer), 0);
        if (bytes <= 0) {
            return bytes;
        }
        receiveBuffer.append(buffer, bytes);
    }
}

int SocketClient::sendData(const std::string& data) {
    if (!connected) return -1;
    int totalSent = 0;
    const int totalLength = static_cast<int>(data.length());
    while (totalSent < totalLength) {
        int sent = send(
            clientSocket,
            data.c_str() + totalSent,
            totalLength - totalSent,
            0
        );
        if (sent == SOCKET_ERROR || sent == 0) {
            return -1;
        }
        totalSent += sent;
    }
    return totalSent;
}

void SocketClient::flushBuffer() {
    if (!connected) return;
    receiveBuffer.clear();
    char dummy[1024];
    u_long bytes_available;
    ioctlsocket(clientSocket, FIONREAD, &bytes_available);
    while (bytes_available > 0) {
        recv(clientSocket, dummy, sizeof(dummy) - 1, 0);
        ioctlsocket(clientSocket, FIONREAD, &bytes_available);
    }
}
