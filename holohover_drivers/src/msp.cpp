/*
  Copyright (c) 2024, EPFL
  Copyright (c) 2017, Fabrizio Di Vittorio (fdivitto2013@gmail.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <iostream>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "msp.hpp"

static speed_t baud_to_speed(unsigned int baud)
{
    switch (baud) {
        case 50:
            return B50;
        case 75:
            return B75;
        case 110:
            return B110;
        case 134:
            return B134;
        case 150:
            return B150;
        case 200:
            return B200;
        case 300:
            return B300;
        case 600:
            return B600;
        case 1200:
            return B1200;
        case 1800:
            return B1800;
        case 2400:
            return B2400;
        case 4800:
            return B4800;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        case 1152000:
            return B1152000;
        case 1500000:
            return B1500000;
        case 2000000:
            return B2000000;
        case 2500000:
            return B2500000;
        case 3000000:
            return B3000000;
        case 3500000:
            return B3500000;
        case 4000000:
            return B4000000;
        default:
            return 0;
    }
}

MSP::MSP() : fd(-1) {}

MSP::~MSP()
{
    stop();
}

bool MSP::begin(const char *device, unsigned int baud)
{
    fd = open(device, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        std::cerr << "Failed to open port: " << device << std::endl;
        return false;
    }

    if (ioctl(fd, TIOCEXCL, NULL) < 0) {
        std::cerr << "Could not get exclusive access to port: " << device << std::endl;
        close(fd);
        fd = -1;
        return false;
    }

    if (!isatty(fd)) {
        std::cerr << "File descriptor " << device << " is not pointing to tty device" << std::endl;
        close(fd);
        fd = -1;
        return false;
    }

    if (tcgetattr(fd, &config) < 0) {
        std::cerr << "Error getting current termios config" << std::endl;
        close(fd);
        fd = -1;
        return false;
    }

    // Input flags - Turn off input processing
    //
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

    // Output flags - Turn off output processing
    //
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    config.c_oflag = 0;

    // No line processing
    //
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    // Turn off character processing
    //
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;

    // Timout of 100ms
    config.c_cc[VMIN] = 0;
    config.c_cc[VTIME] = 1;

    speed_t speed = baud_to_speed(baud);
    if (speed == 0) {
        std::cerr << "Baud rate is not supported, is it non-standard?" << std::endl;
        close(fd);
        fd = -1;
        return false;
    }

    // Communication speed
    if (cfsetispeed(&config, speed) < 0 || cfsetospeed(&config, speed) < 0) {
        std::cerr << "Error setting baud rate" << std::endl;
        close(fd);
        fd = -1;
        return false;
    }

    if (tcsetattr(fd, TCSAFLUSH, &config) < 0) {
        std::cerr << "Error applying termios configuration" << std::endl;
        close(fd);
        fd = -1;
        return false;
    }

    return true;
}

void MSP::stop()
{
    close(fd);
    fd = -1;
}

ssize_t MSP::send(uint8_t messageID, uint8_t* payload, uint8_t size)
{
    ssize_t bytes_send = 0;
    ssize_t r;
    r = write(fd, "$M<", 3); if (r < 0) return r; bytes_send += r;
    r = write(fd, &size, 1); if (r < 0) return r; bytes_send += r;
    r = write(fd, &messageID, 1); if (r < 0) return r; bytes_send += r;
    uint8_t checksum = size ^ messageID;
    uint8_t* payloadPtr = payload;
    for (uint8_t i = 0; i < size; ++i) {
        uint8_t b = *(payloadPtr++);
        checksum ^= b;
    }
    if (size > 0) {
        r = write(fd, payload, size); if (r < 0) return r; bytes_send += r;
    }
    r = write(fd, &checksum, 1); if (r < 0) return r; bytes_send += r;

    return bytes_send;
}

ssize_t MSP::recv(uint8_t* message_id, uint8_t* payload, uint8_t payload_size, int timeout)
{
    ssize_t bytes_recvd = 0;
    ssize_t r;

    auto time_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);

    uint8_t message_size;
    while (true) {
        // read header
        size_t length;
        do {
            int retval = ioctl(fd, FIONREAD, &length);
            if (retval < 0) {
                std::cerr << "FIONREAD ioctl failed" << std::endl;
                return -1;
            }
            if (std::chrono::steady_clock::now() > time_end) {
                return -1;
            }
        } while (length < 6);

        uint8_t header[3];
        r = read(fd, header, 3); if (r < 0) return r; bytes_recvd += r;

        // check header
        if (header[0] == '$' && header[1] == 'M' && header[2] == '>') {
            // header ok, read payload size
            r = read(fd, &message_size, 1);  if (r < 0) return r; bytes_recvd += r;

            // read message ID (type)
            r = read(fd, message_id, 1);  if (r < 0) return r; bytes_recvd += r;

            uint8_t checksum_calc = message_size ^ *message_id;

            // read payload
            uint8_t max_read_size = message_size < payload_size ? message_size : payload_size;

            ssize_t payload_recvd = 0;
            while (payload_recvd < max_read_size) {
                r = read(fd, payload + payload_recvd, max_read_size - payload_recvd);
                if (r < 0) return r;
                payload_recvd += r;
                bytes_recvd += r;

                if (std::chrono::steady_clock::now() > time_end) {
                    return -1;
                }
            }

            uint8_t* payload_ptr = payload;
            uint8_t idx = 0;
            while (idx < message_size) {
                uint8_t b;
                if (idx < payload_size) {
                    b = *(payload_ptr++);
                } else {
                    r = read(fd, &b, 1);  if (r < 0) return r; bytes_recvd += r;
                }
                checksum_calc ^= b;
                ++idx;
            }
            // zero remaining bytes if *size < maxSize
            for (; idx < payload_size; ++idx)
                *(payload_ptr++) = 0;

            // read and check checksum
            uint8_t checksum;
            r = read(fd, &checksum, 1);  if (r < 0) return r; bytes_recvd += r;

            if (checksum_calc == checksum) {
                return message_size;
            }
        }
    }
}

// wait for message_id
ssize_t MSP::wait_for(uint8_t message_id, uint8_t* payload, uint8_t payload_size, int timeout)
{
    auto time_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);

    uint8_t read_message_id;
    ssize_t message_size;
    while (std::chrono::steady_clock::now() <= time_end) {
        message_size = recv(&read_message_id, payload, payload_size, timeout);
        if (message_size >= 0 && message_id == read_message_id)
            return message_size;
    }

    // timeout
    return -1;
}

// send a message and wait for the reply
ssize_t MSP::request(uint8_t message_id, uint8_t* payload, uint8_t payload_size, int timeout)
{
    send(message_id, nullptr, 0);
    return wait_for(message_id, payload, payload_size, timeout);
}

// send message and wait for ack
ssize_t MSP::command(uint8_t message_id, uint8_t* payload, uint8_t payload_size, int timeout)
{
    send(message_id, payload, payload_size);
    // ack required
    return wait_for(message_id, nullptr, 0, timeout);
}
