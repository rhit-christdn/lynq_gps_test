#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include "json.hpp"

using json = nlohmann::json;

int main() {
    const char* portName1 = "/dev/ttyACM0";
    const char* portName2 = "/dev/ttyACM1";

    int serialPort1 = open(portName1, O_RDONLY | O_NOCTTY);
    int serialPort2 = open(portName2, O_RDONLY | O_NOCTTY);

    if (serialPort1 < 0) {
        std::cerr << "Error opening serial port 1\n";
        return 1;
    }
    if (serialPort2 < 0) {
        std::cerr << "Error opening serial port 2\n";
        return 1;
    }

    struct termios tty{};
    if (tcgetattr(serialPort1, &tty) != 0) {
        std::cerr << "Error getting serial attributes for port 1\n";
        close(serialPort1);
        return 1;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag = 0;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serialPort1, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial attributes for port 1\n";
        close(serialPort1);
        return 1;
    }
    if (tcsetattr(serialPort2, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial attributes for port 2\n";
        close(serialPort2);
        return 1;
    }

    std::string buffer1, buffer2;
    char chunk1[256], chunk2[256];

    while (true) {
        int n = read(serialPort1, chunk1, sizeof(chunk1) - 1);
        int m = read(serialPort2, chunk2, sizeof(chunk2) - 1);

        if (n > 0) {
            chunk1[n] = '\0';
            buffer1 += chunk1;

            size_t end = buffer1.find('}');
            if (end != std::string::npos) {
                std::string message = buffer1.substr(0, end + 1);
                buffer1.erase(0, end + 1);

                try {
                    json j = json::parse(message);
                    if (j.contains("lat") && j.contains("long")) {
                        double lat = j["lat"];
                        double lon = j["long"];
                        std::cout << "Latitude 1: " << lat << ", Longitude 1: " << lon << "\n";
                    } else {
                        std::cerr << "Missing lat/long in JSON\n";
                    }
                } catch (json::parse_error& e) {
                    std::cerr << "JSON parse error: " << e.what() << "\n";
                }
            }
        }

        if (m > 0) {
            chunk2[m] = '\0';
            buffer2 += chunk2;

            size_t end = buffer2.find('}');
            if (end != std::string::npos) {
                std::string message = buffer2.substr(0, end + 1);
                buffer2.erase(0, end + 1);

                try {
                    json j = json::parse(message);
                    if (j.contains("lat") && j.contains("long")) {
                        double lat = j["lat"];
                        double lon = j["long"];
                        std::cout << "Latitude 2: " << lat << ", Longitude 2: " << lon << "\n";
                    } else {
                        std::cerr << "Missing lat/long in JSON\n";
                    }
                } catch (json::parse_error& e) {
                    std::cerr << "JSON parse error: " << e.what() << "\n";
                }
            }
        }
    }

    close(serialPort1);
    close(serialPort2);
    std::cout << "Serial ports closed.\n";
    return 0;
}
