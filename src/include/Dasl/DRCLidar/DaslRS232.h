//Copyright (c) 2020 Dongbin Kim, Giho Jang
//
// Permission is hereby granted, free of charge,
// to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software
// without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or
//  sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall
// be included in all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
//  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
//  AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
//  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef DASL_RS232_H
#define DASL_RS232_H
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <strings.h>
#include <unistd.h>
class DaslRS232
{
public:
    DaslRS232();
    ~DaslRS232();

    int fd;
    struct termios oldtio;

public:
    bool open ( const char * port );
    void setup ( unsigned int baud, unsigned int stop_bits );
    void setTimeOut ( unsigned int time,unsigned int reviced_byte );
    int  write ( char * data, int len );
    int  read ( char * data, int len );
    int  write ( unsigned char * data, int len );
    int  read ( unsigned char * data, int len );
    void clearBuffer();

    void close();

};
inline DaslRS232::DaslRS232()
{
    fd=0;
}

inline DaslRS232::~DaslRS232()
{
    close();
}

inline bool DaslRS232::open(const char *port)
{
    fd = ::open(port, O_RDWR | O_NOCTTY );

    if (fd <0)
    {
        perror(port);
        return false;
    }
    tcgetattr(fd,&oldtio); /* save current port settings */
    return true;
}

inline void DaslRS232::setup(unsigned int baud, unsigned int stop_bits)
{
    (void) stop_bits;
    struct termios newtio;
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = baud | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;
    newtio.c_cc[VMIN]     = 1;


    tcflush(fd, TCIFLUSH);  //This function effect to tcp/udp sockets
    tcsetattr(fd,TCSANOW,&newtio);
}

inline void DaslRS232::setTimeOut(unsigned int time, unsigned int reviced_byte)
{
    struct termios newtio;
    newtio.c_cc[VTIME]    = time;        /* inter-character timer unused */
    newtio.c_cc[VMIN]     = reviced_byte;   /* blocking read until 5 chars received */

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
}

inline int DaslRS232::write(char *data, int len)
{

    int res;
    res = ::write(fd,data,len);
    return res;
}

inline int DaslRS232::read(char *data, int len)
{
    int res;
    res = ::read(fd,data,len);
    return res;
}
inline int DaslRS232::write(unsigned char *data, int len)
{

    int res;
    res = write((char*)data,len);
    return res;
}

inline int DaslRS232::read(unsigned char *data, int len)
{
    int res;
    res = read((char*)data,len);
    return res;
}

inline void DaslRS232::clearBuffer()
{
     tcflush(fd, TCIFLUSH); //This function effect to tcp/udp sockets
}

inline void DaslRS232::close()
{
    if(fd != 0)
    {
      tcsetattr(fd,TCSANOW,&oldtio);
      ::close(fd);
      fd = 0;
    }
}

#endif