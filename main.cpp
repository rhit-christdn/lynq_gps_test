#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include "json.hpp"

using json = nlohmann::json;

int main() {
    const char* portName1 = "/dev/ttyACM0"; // Adjust as needed
    const char* portName2 = "/dev/ttyACM1"; // Adjust as needed
    int serialPort1 = open(portName1, O_RDONLY | O_NOCTTY);
    int serialPort2 = open(portName2, O_RDONLY | O_NOCTTY);

    if (serialPort1 < 0) {
        std::cerr << "Error opening serial port\n";
        return 1;
    }
    if (serialPort2 < 0) {
        std::cerr << "Error opening serial port\n";
        return 1;
    }

    struct termios tty{};
    if (tcgetattr(serialPort1, &tty) != 0) {
        std::cerr << "Error getting serial attributes\n";
        close(serialPort1);
        return 1;
    }
    if (tcgetattr(serialPort2, &tty) != 0) {
        std::cerr << "Error getting serial attributes\n";
        close(serialPort2);
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
        std::cerr << "Error setting serial attributes\n";
        close(serialPort1);
        return 1;
    }
    if (tcsetattr(serialPort2, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial attributes\n";
        close(serialPort2);
        return 1;
    }

    std::string buffer;
    char chunk[256];

    while (true) {
        int n = read(serialPort1, chunk, sizeof(chunk) - 1);\
        int m = read(serialPort2, chunk, sizeof(chunk) - 1);
        if (n > 0) {
            chunk[n] = '\0';
            buffer += chunk;

            // Attempt to find a complete JSON object ending with }
            size_t end = buffer.find('}');
            if (end != std::string::npos) {
                std::string message = buffer.substr(0, end + 1);
                buffer.erase(0, end + 1);

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
            chunk[m] = '\0';
            buffer += chunk;
            // Attempt to find a complete JSON object ending with }
            size_t end = buffer.find('}');
            if (end != std::string::npos) {
                std::string message = buffer.substr(0, end + 1);
                buffer.erase(0, end + 1);
                try {
                    json j = json::parse(message);
                    if (j.contains("lat") && j.contains("long")) {
                        double lat = j["lat"];
                        double lon = j["long"];
                        std::cout << "Latitude 2: " << lat << ", Longitude 2: " << lon << "\n";
                    }
                    else {
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