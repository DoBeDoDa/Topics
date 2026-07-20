// 正式撞球程式入口，初始化網路並啟動 BilliardApp 主流程。
#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif

#include <iostream>
#include <winsock2.h>
#include "BilliardApp.h"

#pragma comment(lib, "ws2_32.lib")

using namespace std;

int main() {
    // 設定編碼支援繁體中文輸出
    setlocale(LC_ALL, "zh_TW.UTF-8");
    cout << "--- 撞球 AI 視覺伺服系統 (自動直擊與顆星複合模組版) ---" << endl;

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        cout << "[錯誤] WSAStartup 失敗。" << endl;
        return -1;
    }

    BilliardApp app;
    if (app.initialize()) {
        app.run();
    }

    WSACleanup();
    return 0;
}
