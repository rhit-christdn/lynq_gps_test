#include <windows.h>
#include <iostream>

int main() {
    LPCWSTR portName = L"\\\\.\\COM4"; // Change COM port here

    HANDLE hSerial = CreateFileW(portName, GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "Error opening serial port\n";
        return 1;
    }

    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Failed to get current serial parameters\n";
        CloseHandle(hSerial);
        return 1;
    }

    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Failed to set serial parameters\n";
        CloseHandle(hSerial);
        return 1;
    }

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;

    SetCommTimeouts(hSerial, &timeouts);

    char buffer[128];
    DWORD bytesRead;

    while (true) {
        if (ReadFile(hSerial, buffer, sizeof(buffer) - 1, &bytesRead, NULL)) {
            if (bytesRead > 0) {
                buffer[bytesRead] = '\0';

                char* latPtr = strstr(buffer, "\"lat\"");
                char* lonPtr = strstr(buffer, "\"long\"");

                double latitude, longitude;

                // Extract numbers using sscanf
                sscanf(latPtr, "\"lat\" : %lf", &latitude);
                sscanf(lonPtr, "\"long\" : %lf", &longitude);

                std::cout << "Latitude: " << latitude << "\n";
                std::cout << "Longitude: " << longitude << "\n";
            }
        } else {
            std::cerr << "Read error\n";
            break;
        }
    }

    CloseHandle(hSerial);
    return 0;

}
