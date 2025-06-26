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
    const char *portName2 = "/dev/ttyACM1";

    int serialPort1 = open(portName1, O_RDONLY | O_NOCTTY);
    int serialPort2 = open(portName2, O_RDONLY | O_NOCTTY);

    if (serialPort1 < 0)
    {
        std::cerr << "Error opening serial port 1\n";
        return 1;
    }
    if (serialPort2 < 0)
    {
        std::cerr << "Error opening serial port 2\n";
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
    if (tcsetattr(serialPort2, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error setting serial attributes for port 2\n";
        close(serialPort2);
        return 1;
    }

    std::string buffer1, buffer2;
    char chunk1[256], chunk2[256];

    while (true)
    {
        int n = read(serialPort1, chunk1, sizeof(chunk1) - 1);
        int m = read(serialPort2, chunk2, sizeof(chunk2) - 1);

        bool gotLat1 = false, gotLat2 = false;
        bool lynq1Printed = false, lynq2Printed = false;
        double lat1 = 0.0, lon1 = 0.0;
        double lat2 = 0.0, lon2 = 0.0;

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
                    if (j.contains("lat") && j.contains("long") && !lynq1Printed)
                    {
                        gotLat1 = true;
                        lat1 = j["lat"];
                        lon1 = j["long"];
                        std::cout << "Latitude 1: " << std::setprecision(10) << lat1
                                  << ", Longitude 1: " << std::setprecision(10) << lon1 << "\n";
                        lynq1Printed = true; // Mark as printed
                    }
                }
                catch (json::parse_error &e)
                {
                    std::cerr << "JSON parse error (1): " << e.what() << "\n";
                }
            }
            lynq1Printed = false; // Clear printed flag for next iteration
            buffer1.clear(); // Clear buffer after processing
        }

        if (m > 0)
        {
            chunk2[m] = '\0';
            buffer2 += chunk2;

            // Parse all complete JSON messages
            size_t end;
            while ((end = buffer2.find('}')) != std::string::npos && !lynq2Printed)
            {
                std::string message = buffer2.substr(0, end + 1);
                buffer2.erase(0, end + 1);

                try
                {
                    json j = json::parse(message);
                    if (j.contains("lat") && j.contains("long") && !lynq2Printed)
                    {
                        gotLat2 = true;
                        lat2 = j["lat"];
                        lon2 = j["long"];
                        std::cout << "Latitude 2: " << std::setprecision(10) << lat2
                                  << ", Longitude 2: " << std::setprecision(10) << lon2 << "\n";
                        lynq2Printed = true; // Mark as printed
                    }
                }
                catch (json::parse_error &e)
                {
                    std::cerr << "JSON parse error (2): " << e.what() << "\n";
                }
            }
            lynq2Printed = false; // Clear printed flag for next iteration
            buffer2.clear(); // Clear buffer after processing
        }


        if (gotLat1 && gotLat2)
        {
            std::cout << "Average Value.\n";

            double avgLat = (lat1 + lat2) / 2.0;
            double avgLon = (lon1 + lon2) / 2.0;
            std::cout << "Avg Longitude: " << std::setprecision(10) << avgLon << "\n";
            std::cout << "Avg Latitude: " << std::setprecision(10) << avgLat << "\n";
        }

        
        std::cout << "\n";
    }

    close(serialPort1);
    close(serialPort2);
    std::cout << "Serial ports closed.\n";
    return 0;
}
