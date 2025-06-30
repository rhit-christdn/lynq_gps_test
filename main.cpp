#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include "json.hpp"

using json = nlohmann::json;

int main()
{
    const char *portName1 = "/dev/ttyACM0";

    int serialPort1 = open(portName1, O_RDONLY | O_NOCTTY);

    if (serialPort1 < 0)
    {
        std::cerr << "Error opening serial port 1\n";
        return 1;
    }


    struct termios tty{};
    if (tcgetattr(serialPort1, &tty) != 0)
    {
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

    if (tcsetattr(serialPort1, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error setting serial attributes for port 1\n";
        close(serialPort1);
        return 1;
    }

    std::string buffer1;
    char chunk1[256];

    while (true)
    {
        int n = read(serialPort1, chunk1, sizeof(chunk1) - 1);

        bool lynq1Printed = false;
        double lat1 = 0.0, lon1 = 0.0;
        char[64] identity1;

        if (n > 0)
        {
            chunk1[n] = '\0';
            buffer1 += chunk1;

            // Parse all complete JSON messages
            size_t end;
            while ((end = buffer1.find('}')) != std::string::npos)
            {
                std::string message = buffer1.substr(0, end + 1);
                buffer1.erase(0, end + 1);

                try
                {
                    json j = json::parse(message);
                    if (j.contains("identity") && j.contains("lat") && j.contains("long") && !lynq1Printed)
                    {
                        identity1 = j["identity"].get<std::string>();
                        lat1 = j["lat"];
                        lon1 = j["long"];
                        std::cout << "identity 1: " << identity1
                                  << "Latitude 1: " << lat1
                                  << ", Longitude 1: " << lon1 << "\n";
                    }
                }
                catch (json::parse_error &e)
                {
                    std::cerr << "JSON parse error (1): " << e.what() << "\n";
                }
            }
            lynq1Printed = false; // Clear printed flag for next iteration
        }


        // if (gotLat1 && gotLat2)
        // {
        //     std::cout << "Average Value.\n";

        //     double avgLat = (lat1 + lat2) / 2.0;
        //     double avgLon = (lon1 + lon2) / 2.0;
        //     std::cout << "Avg Longitude: " << std::setprecision(10) << avgLon << "\n";
        //     std::cout << "Avg Latitude: " << std::setprecision(10) << avgLat << "\n";
        // }

        
        std::cout << "\n";
    }

    close(serialPort1);
    std::cout << "Serial ports closed.\n";
    return 0;
}
