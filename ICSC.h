
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

#ifndef _ICSC_H
#define _ICSC_H

//#include <unistd.h>
#include <sys/io.h>
//#include <termios.h>
#include <termios.h>

#define MAX_COMMANDS 100

#define SOH 1
#define STX 2
#define ETX 3
#define EOT 4

struct icsc_header {
    unsigned char soh;
    unsigned char dst;
    unsigned char src;
    char command;
    unsigned char len;
    unsigned char stx;
};

struct icsc_footer {
    unsigned char etx;
    unsigned char checksum;
    unsigned char eot;
};

// Format of command callback functions
typedef void(*callbackFunction)(unsigned char, char, unsigned char, char *);

// Structure to store command code / function pairs
typedef struct {
    char commandCode;
    callbackFunction callback;
} command;

extern int icsc_begin(unsigned char station, unsigned int baud, char *device);
extern void icsc_send(unsigned char station, unsigned char command, unsigned char len, char *data);
extern void icsc_process();
extern void icsc_register_command(char command, callbackFunction function);
extern void icsc_unregister_command(char command);
#endif
