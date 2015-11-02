/********************************************************************************
 * Copyright (c) 2013, Majenko Technologies
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of Majenko Technologies.
 ********************************************************************************/
 
 /*******************************************************************************
  * 2015 - Modified by Brett Bowden for Raspberry Pi
  * *****************************************************************************/

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <sys/poll.h>

#include "icsc.h"

int _serial;
unsigned char _station;
command commands[MAX_COMMANDS];
int connected;
struct termios tty;
struct pollfd fds[1];

int icsc_begin(unsigned char station, unsigned int baud, char *device)
{
    _station = station;
    _serial = open(device, O_RDWR | O_EXCL | O_NONBLOCK);
    fds[0].fd = _serial;
    fds[0].events = POLLIN;

    if (!_serial)
    {
        errno = ENODEV;
        return -1;
    }

    //fcntl(_serial, F_SETFL, FNDELAY);
    //fcntl(_serial, F_SETOWN, getpid());
    //fcntl(_serial, F_SETFL, O_ASYNC | O_NONBLOCK);

    tcgetattr(_serial, &tty);

    cfsetspeed(&tty, baud);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag =  IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON|IXOFF|IXANY);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tcsetattr(_serial, TCSANOW, &tty);
    return 1;
}

void icsc_send(unsigned char station, unsigned char command, unsigned char len, char *data)
{
    unsigned char i;
    struct icsc_header header;
    struct icsc_footer footer;
    char fillstart[4] = {SOH, SOH, SOH, SOH};

    header.soh = SOH;
    header.dst = station;
    header.src = _station;
    header.command = command;
    header.len = len;
    header.stx = STX;

    footer.etx = ETX;
    footer.eot = EOT;
    footer.checksum = 0;
    footer.checksum += station;
    footer.checksum += _station;
    footer.checksum += command;
    footer.checksum += len;
    for (i=0; i<len; i++)
    {
        footer.checksum += (unsigned char)data[i];
    }

    write(_serial, &fillstart, 4);
    write(_serial, &header, sizeof(struct icsc_header));
    write(_serial, data, len);
    write(_serial, &footer, sizeof(struct icsc_footer));
    fsync(_serial);
    //usleep(1000);
}

void icsc_process()
{
    static unsigned char phase = 0;
    static unsigned char len = 0;
    static unsigned char pos = 0;
    static char command = 0;
    static unsigned char sender = 0;
    static unsigned char destination = 0;
    static unsigned char header[6];
    static char data[256];
    static unsigned char calcCS = 0;
    int i;
    char inch;

    if (poll(fds, 1, 10) > 0)
    {
        while (read(_serial, &inch, 1) == 1)
        {
            switch(phase)
            {
            case 0:
                header[0] = header[1];
                header[1] = header[2];
                header[2] = header[3];
                header[3] = header[4];
                header[4] = header[5];
                header[5] = inch;
                if ((header[0] == SOH) && (header[5] == STX))
                {
                    if (header[1] == _station)
                    {
                        destination = header[1];
                        sender = header[2];
                        command = header[3];
                        len = header[4];
                        phase = 1;
                        pos = 0;
                        calcCS = 0;
                        calcCS += destination;
                        calcCS += sender;
                        calcCS += (unsigned char) command;
                        calcCS += len;
                        if (len == 0)
                        {
                            phase = 2;
                        }
                    }
                }
                break;
            case 1:
                data[pos++] = inch;
                calcCS += (unsigned char) inch;
                if (pos == len)
                {
                    phase = 2;
                }
                break;
            case 2:
                if (inch == ETX)
                {
                    phase = 3;
                }
                else
                {
                    pos = 0;
                    len = 0;
                    command = 0;
                    sender = 0;
                    destination = 0;
                    len = 0;
                    calcCS = 0;
                    phase = 0;
                }
                break;
            case 3:
                if (calcCS != (unsigned char)inch)
                {
                    pos = 0;
                    len = 0;
                    command = 0;
                    sender = 0;
                    destination = 0;
                    len = 0;
                    calcCS = 0;
                    phase = 0;
                }
                else
                {
                    phase = 4;
                }
                break;
            case 4:
                if ((unsigned char)inch != EOT)
                {
                    pos = 0;
                    len = 0;
                    command = 0;
                    sender = 0;
                    destination = 0;
                    len = 0;
                    calcCS = 0;
                    phase = 0;
                }
                else
                {

                    for (i=0; i<MAX_COMMANDS; i++)
                    {
                        if (commands[i].commandCode == command)
                        {
                            if (commands[i].callback)
                            {
                                commands[i].callback(sender, command, len, data);
                            }
                        }
                    }
                    pos = 0;
                    len = 0;
                    command = 0;
                    sender = 0;
                    destination = 0;
                    len = 0;
                    calcCS = 0;
                    phase = 0;
                    phase = 0;
                }

            }
        }
    }
    //usleep(20);
}

void icsc_register_command(char command, callbackFunction function)
{
    int i;
    for (i=0; i<MAX_COMMANDS; i++)
    {
        if (commands[i].commandCode == 0)
        {
            commands[i].commandCode = command;
            commands[i].callback = function;
            return;
        }
    }
}

void icsc_unregister_command(char command)
{
    int i;
    for (i=0; i<MAX_COMMANDS; i++)
    {
        if (commands[i].commandCode == command)
        {
            commands[i].commandCode = 0;
            commands[i].callback = NULL;
        }
    }
}

