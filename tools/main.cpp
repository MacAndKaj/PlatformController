/*
 * Copyright (c) 2023 MacAndKaj. All rights reserved.
 */
#include "SpiProxy.hpp"
#include "PinController.hpp"

#include <cstring>
#include <iostream>
#include <memory>
#include <cstdint>


constexpr int DATA_SIZE = 125;
constexpr std::uint8_t HEADER_BYTE = 0xF0;


struct Frame
{
    uint8_t header{HEADER_BYTE};
    uint8_t id;
    uint8_t data[DATA_SIZE];
    uint8_t crc;
};

constexpr int FRAME_SIZE = sizeof(Frame);



int main(int argc, char* argv[])
{
    std::cout << "Starting PlatformController" << std::endl;
    if (argc != 4)
    {
        std::cerr << "Incorrect arguments amount" << std::endl;
        return EXIT_FAILURE; 
    }

    std::string spi_filename = argv[1];
    // std::string gpio_filename = argv[2];

    std::string l_speed = argv[2];
    std::string r_speed = argv[3];
    try
    {
        SpiProxy proxy(spi_filename);
        // proxy.spi_read_reg8(0x2B);

        std::int8_t l = std::stoi(l_speed);
        std::int8_t r = std::stoi(r_speed);
        Frame frame = {
            .id=0x01
        };
        frame.data[0]=l;
        frame.data[2]=r;
        std::vector<std::uint8_t> d(FRAME_SIZE);
        std::memcpy(d.data(), &frame, FRAME_SIZE);
        proxy.spi_write(d);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }


    return EXIT_SUCCESS;
}