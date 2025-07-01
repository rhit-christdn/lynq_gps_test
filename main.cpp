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

    // Variables to track updates for A1, A2, A3 and B1, B2, B3
    // These will be used to calculate the average latitude and longitude
    // when all three values are received for each group.
    bool A1updated = false, A2updated = false, A3updated = false;
    bool B1updated = false, B2updated = false, B3updated = false;

    double A1lat = 0.0, A1lon = 0.0;
    double A2lat = 0.0, A2lon = 0.0;
    double A3lat = 0.0, A3lon = 0.0;
    double B1lat = 0.0, B1lon = 0.0;
    double B2lat = 0.0, B2lon = 0.0;
    double B3lat = 0.0, B3lon = 0.0;

    unsigned int count = 0;
    unsigned int NoStatusCount = 0;

    while (true)
    {
        int n = read(serialPort1, chunk1, sizeof(chunk1) - 1);

        double lat1 = 0.0, lon1 = 0.0;

        std::string identity1;

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
                    if (j.contains("identity") && j.contains("lat") && j.contains("long"))
                    {
                        identity1 = j["identity"].get<std::string>();
                        lat1 = j["lat"];
                        lon1 = j["long"];

                        if (identity1.find("No Status") != std::string::npos)
                        {
                            NoStatusCount++;
                            count++;
                            std::cout << "No Status: " << NoStatusCount << "\n";
                            std::cout << "Ratio: " << std::setprecision(10) << (double)NoStatusCount / count << "\n";
                        }
                        else
                        {
                            count++;
                        }

                        if (identity1[0] == 'A')
                        {
                            if (identity1[1] == '1')
                            {
                                A1updated = true;
                                A1lat = lat1;
                                A1lon = lon1;
                            }
                            else if (identity1[1] == '2')
                            {
                                A2updated = true;
                                A2lat = lat1;
                                A2lon = lon1;
                            }
                            else if (identity1[1] == '3')
                            {
                                A3updated = true;
                                A3lat = lat1;
                                A3lon = lon1;
                            }
                        }
                        else if (identity1[0] == 'B')
                        {
                            if (identity1[1] == '1')
                            {
                                B1updated = true;
                                B1lat = lat1;
                                B1lon = lon1;
                            }
                            else if (identity1[1] == '2')
                            {
                                B2updated = true;
                                B2lat = lat1;
                                B2lon = lon1;
                            }
                            else if (identity1[1] == '3')
                            {
                                B3updated = true;
                                B3lat = lat1;
                                B3lon = lon1;
                            }
                        }

                        if (A1updated || A2updated || A3updated)
                        {
                            double avgALat = (A1lat + A2lat + A3lat) / 3.0;
                            double avgALon = (A1lon + A2lon + A3lon) / 3.0;

                            std::cout << "Average A: Latitude: " << std::setprecision(10) << avgALat;
                            std::cout << ", Longitude: " << std::setprecision(10) << avgALon << "\n";

                            A1updated = false; // Reset after printing
                            A2updated = false;
                            A3updated = false;
                        }

                        if (B1updated || B2updated || B3updated)
                        {
                            double avgBLat = (B1lat + B2lat + B3lat) / 3.0;
                            double avgBLon = (B1lon + B2lon + B3lon) / 3.0;

                            std::cout << "Average B: Latitude: " << std::setprecision(10) << avgBLat;
                            std::cout << ", Longitude: " << std::setprecision(10) << avgBLon << "\n";

                            B1updated = false; // Reset after printing
                            B2updated = false;
                            B3updated = false;
                        }
                    }
                }
                catch (json::parse_error &e)
                {
                    std::cerr << "JSON parse error (1): " << e.what() << "\n";
                }
            }
        }

        // if (gotLat1 && gotLat2)
        // {
        //     std::cout << "Average Value.\n";

        //     double avgLat = (lat1 + lat2) / 2.0;
        //     double avgLon = (lon1 + lon2) / 2.0;
        //     std::cout << "Avg Longitude: " << std::setprecision(10) << avgLon << "\n";
        //     std::cout << "Avg Latitude: " << std::setprecision(10) << avgLat << "\n";
        // }
    }

    close(serialPort1);
    std::cout << "Serial ports closed.\n";
    return 0;
}
