#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

int main() {
    const char* portName = "/dev/ttyACM0"; // Change to your actual serial port
    int serialPort = open(portName, O_RDONLY | O_NOCTTY);

    if (serialPort < 0) {
        std::cerr << "Error opening serial port\n";
        return 1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serialPort, &tty) != 0) {
        std::cerr << "Error getting serial attributes\n";
        close(serialPort);
        return 1;
    }

    // Set baud rate and serial settings
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_cflag |= (CLOCAL | CREAD);            // Enable receiver, no modem control
    tty.c_cflag &= ~(PARENB | PARODD);          // No parity
    tty.c_cflag &= ~CSTOPB;                     // One stop bit
    tty.c_cflag &= ~CRTSCTS;                    // No flow control
    tty.c_iflag = 0;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;  // Read blocks until at least 1 byte is available
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial attributes\n";
        close(serialPort);
        return 1;
    }

    // Main read loop
    char buffer[256];
    while (true) {
        int bytesRead = read(serialPort, buffer, sizeof(buffer) - 1);
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';
            std::cout << buffer;

            char* latPtr = strstr(buffer, "\"lat\"");
            char* lonPtr = strstr(buffer, "\"long\"");

            if (latPtr && lonPtr) {
                double latitude, longitude;

                if (sscanf(latPtr, "\"lat\" : %lf", &latitude) == 1 &&
                    sscanf(lonPtr, "\"long\" : %lf", &longitude) == 1) {
                    std::cout << "Latitude: " << latitude << "\n";
                    std::cout << "Longitude: " << longitude << "\n";
                }
            }
        } else {
            std::cerr << "Read error or disconnect\n";
            break;
        }
    }

    close(serialPort);
    return 0;
}
