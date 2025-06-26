#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include "json.hpp"

using json = nlohmann::json;

int main() {
    const char* portName = "/dev/ttyACM0"; // Adjust as needed
    int serialPort = open(portName, O_RDONLY | O_NOCTTY);
    if (serialPort < 0) {
        std::cerr << "Error opening serial port\n";
        return 1;
    }

    struct termios tty{};
    if (tcgetattr(serialPort, &tty) != 0) {
        std::cerr << "Error getting serial attributes\n";
        close(serialPort);
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

    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial attributes\n";
        close(serialPort);
        return 1;
    }

    std::string buffer;
    char chunk[256];

    while (true) {
        int n = read(serialPort, chunk, sizeof(chunk) - 1);
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
                        std::cout << "Latitude: " << lat << ", Longitude: " << lon << "\n";
                    } else {
                        std::cerr << "Missing lat/long in JSON\n";
                    }
                } catch (json::parse_error& e) {
                    std::cerr << "JSON parse error: " << e.what() << "\n";
                }
            }
        }
    }

    close(serialPort);
    return 0;
}