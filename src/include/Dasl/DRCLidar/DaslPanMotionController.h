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

#ifndef DASL_PANMOTION_CONTROLLER_H
#define DASL_PANMOTION_CONTROLLER_H


#include "DaslRS232.h"
#include <vector>
#include <memory.h>
#include <math.h>

#define	BOARD_NO		0
#define DPP				(360. / (4000.*62. / 14.))  // reduction ratio, Deg per pulse
#define UNIT_TIME		0.025

//#define MOTORCMD_HOME		1
//#define MOTORCMD_SCAN		2
//#define MOTORCMD_GOTO		3
#define MOTORCMD_REQUEST_NAME                0x01
#define MOTORCMD_REQUEST_MOTPOSITION         0x03
#define MOTORCMD_GOTO                        0x23
#define MOTORCMD_REQUEST_SCANPOSITION        0x26
#define MOTORCMD_SCAN                        0x27



typedef union{
	struct {
		unsigned    CMD			: 2;		// last command
		unsigned 	CAL			: 1;		// calibrated
		unsigned	SCAN		: 1;		// scan phase
		unsigned 	READY		: 1;		// ready phase
		unsigned	TRANS		: 1;		// transition phase
		unsigned    REST		: 1;	    // rest phase
		unsigned    SCAN_POS	: 1;		// motor pos when scan signal is on		
	} bits;
	unsigned char byte;
} MOTOR_STAT;

class DaslPanMotionController
{
	int m_conn_flag;

    int m_pos[2];
    unsigned char data[16];
	MOTOR_STAT motor_stat[2];

	int readPanPosition();
	int readScanPosition();
	int reqPanPos();
	int reqScanPos(int start);
    int  verifyCheckSum(unsigned char  * data);
    void putCheckSum(unsigned char*data);
    int execCommand(unsigned char *data);
    int waitReturnValue(unsigned char *data);

    std::vector<char> comm_buffer;
    DaslRS232 m_comm;
	DaslPanMotionController();
public:
	
	static DaslPanMotionController* getInstance(){
		static DaslPanMotionController obj;

		return &obj;
	}
	~DaslPanMotionController();

	int connect();
	int disconnect();

    int setPosition(float motor0_pos, float motor0_time=1, float motor1_pos =0.0, float motor1_time=0.0); //deg, sec
	int scan(int scan_range0=-50, int scan_range1=50, float scan_time=3, float goto_time=1,int motor0_pos=0, int motor1_pos=0); //deg, sec

    int findHome();
	int serveOff();
	int stop();

	float getPosition(int index=0);
	int getScanPosition(int start);


};
inline  DaslPanMotionController::DaslPanMotionController()
{
    m_pos[0] = 0;
    m_pos[1] = 0;
    m_conn_flag = 0;

    motor_stat[0].byte = 0;
    motor_stat[1].byte = 0;
    bzero(data, 16);
}

inline  DaslPanMotionController::~DaslPanMotionController()
{
    m_comm.close();
}

inline float DaslPanMotionController::getPosition(int index)
{
    int ret = reqPanPos();
    if (ret != 0)
    {
        return -1;
    }
    readPanPosition();
    
    return ((float)m_pos[index]*DPP);
}
inline int DaslPanMotionController::getScanPosition(int start)
{
    int ret = reqScanPos(start);
    if (ret != 0)
    {
        return -1;
    }
    return readScanPosition();
}

inline int DaslPanMotionController::connect()
{
    int ret;

    m_conn_flag = 0;
    char buff[100];

    for (int i = 0; i < 10; i++)
    {
        sprintf(buff, "/dev/ttyUSB%d", i);
        printf("%s\n",buff);
        if (m_comm.open(buff) != true)
        {
            continue;
        }
        else
        {
            m_comm.setup(B115200, 1);
            m_comm.setTimeOut(10, 16); // 1sec,16 bytes

            data[0] = 0xFF;
            data[1] = 0xFF;
            data[2] = BOARD_NO;
            data[3] = MOTORCMD_REQUEST_NAME; // Request name
            if ((ret = execCommand(data)) != 1)
            {
                m_comm.close();
                continue;
            }
            if ((ret = waitReturnValue(data)) != 1) //TODO: Should be add timeout!!
            {
                m_comm.close();
                continue;
            }
            motor_stat[0].byte = data[6];
            m_pos[0] = (long)(data[10] & 0xFF);
            m_pos[0] = (m_pos[0] << 8) + (long)(data[9] & 0xFF);
            m_pos[0] = (m_pos[0] << 8) + (long)(data[8] & 0xFF);
            m_pos[0] = (m_pos[0] << 8) + (long)(data[7] & 0xFF);

            motor_stat[1].byte = data[11];
            m_pos[1] = (long)(data[15] & 0xFF);
            m_pos[1] = (m_pos[1] << 8) + (long)(data[14] & 0xFF);
            m_pos[1] = (m_pos[1] << 8) + (long)(data[13] & 0xFF);
            m_pos[1] = (m_pos[1] << 8) + (long)(data[12] & 0xFF);
            m_conn_flag = 1;
            return 0;
        }
    }
    return -1;
}

inline int DaslPanMotionController::readScanPosition()
{
    int ret;
    long ltemp;

    if (m_conn_flag == 0)
        return -1;

    if ((ret = waitReturnValue(data)) != 1)
    {
        return ret;
    }

    motor_stat[0].byte = data[6];
    motor_stat[1].byte = data[11];

    if (motor_stat[0].bits.SCAN_POS == 1)
    {
        ltemp = (long)(data[10] & 0xFF);
        ltemp = (ltemp << 8) + (long)(data[9] & 0xFF);
        ltemp = (ltemp << 8) + (long)(data[8] & 0xFF);
        ltemp = (ltemp << 8) + (long)(data[7] & 0xFF);

        m_pos[0] = ltemp;
    }

    if (motor_stat[1].bits.SCAN_POS == 1)
    {
        ltemp = (long)(data[15] & 0xFF);
        ltemp = (ltemp << 8) + (long)(data[14] & 0xFF);
        ltemp = (ltemp << 8) + (long)(data[13] & 0xFF);
        ltemp = (ltemp << 8) + (long)(data[12] & 0xFF);

        m_pos[1] = ltemp;
    }

    return 0;
}

inline int DaslPanMotionController::readPanPosition()
{
    int ret;
    int ltemp;

    if (m_conn_flag == 0)
        return -1;

    if ((ret = waitReturnValue(data)) != 1)
    {
        return ret;
    }

    motor_stat[0].byte = data[6];
    motor_stat[1].byte = data[11];

    memcpy((char *)&ltemp, &data[7], 4);

    m_pos[0] = ltemp;

    ltemp = (long)(data[15] & 0xFF);
    ltemp = (ltemp << 8) + (long)(data[14] & 0xFF);
    ltemp = (ltemp << 8) + (long)(data[13] & 0xFF);
    ltemp = (ltemp << 8) + (long)(data[12] & 0xFF);
    m_pos[1] = ltemp;

    return 0;
}

inline int DaslPanMotionController::setPosition(float motor0_pos, float motor0_time, float motor1_pos, float motor1_time)
{
    int ret;
    long pos1, pos2;
    int time1, time2;

    if (m_conn_flag == 0)
    {
        return -1;
    }

    pos1 = (long)(motor0_pos / DPP);
    pos2 = (long)(motor1_pos / DPP);

    time1 = (int)(motor0_time / UNIT_TIME);
    time2 = (int)(motor1_time / UNIT_TIME);
    if (time1 < 1)
    {
        time1 = 1;
    }
    else if (time1 > 255)
    {
        time1 = 255;
    }

    if (time2 < 1)
    {
        time2 = 1;
    }
    else if (time2 > 255)
    {
        time2 = 255;
    }

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = BOARD_NO;
    data[3] = MOTORCMD_GOTO; // GOTO

    data[6] = pos1 & 0xFF;
    data[7] = (pos1 >> 8) & 0xFF;
    data[8] = (pos1 >> 16) & 0xFF;
    data[9] = (pos1 >> 24) & 0xFF;
    data[10] = time1 & 0xFF;

    data[11] = pos2 & 0xFF;
    data[12] = (pos2 >> 8) & 0xFF;
    data[13] = (pos2 >> 16) & 0xFF;
    data[14] = (pos2 >> 24) & 0xFF;
    data[15] = time2 & 0xFF;

    if ((ret = execCommand(data)) != 1)
    {
        return ret;
    }

    return 0;
}

inline int DaslPanMotionController::scan(int scan_start, int scan_end, float scan_time, float goto_time, int motor0_pos, int motor1_pos)
{
    int ret;
    printf("[Trace] Called DaslPanMotionController::scan\n");
    if (m_conn_flag == 0)
        return -1;

    if (scan_time < 0)
    {
        scan_time = 0.1;
    }
    else if (scan_time > 25.5)
    {
        scan_time = 25.5;
    }

    if (goto_time < 0)
    {
        goto_time = 0.1;
    }
    else if (goto_time > 25.5)
    {
        goto_time = 25.5;
    }

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = BOARD_NO;
    data[3] = MOTORCMD_SCAN; // SCAN

    data[6] = abs(scan_start) & 0x7F;
    if (scan_start < 0)
    {
        data[6] += 0x80;
    }
    data[7] = abs(scan_end) & 0x7F;
    if (scan_end < 0)
    {
        data[7] += 0x80;
    }
    data[10] = ((int)(scan_time * 10)) & 0xFF;
    data[11] = 2; // one-scan and goto
    data[12] = 3; // goto

    data[13] = abs(motor0_pos) & 0x7F;
    if (motor0_pos < 0)
    {
        data[13] += 0x80;
    }
    data[14] = abs(motor1_pos) & 0x7F;
    if (motor1_pos < 0)
    {
        data[14] += 0x80;
    }
    data[15] = ((int)(goto_time * 10)) & 0xFF;

    if ((ret = execCommand(data)) != 1)
    {
        return ret;
    }
    printf("[Trace] Finished DaslPanMotionController::scan\n");

    return 0;
}

inline int DaslPanMotionController::serveOff()
{
    int ret;
    if (m_conn_flag == 0)
        return -1;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = BOARD_NO;
    data[3] = 0x0F; // Control off

    if ((ret = execCommand(data)) != 1)
    {
        return ret;
    }
    return 0;
}

inline int DaslPanMotionController::findHome()
{
    int ret;
    if (m_conn_flag == 0)
        return -1;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = BOARD_NO;
    data[3] = 0x11; // Find home
    data[6] = 0xFF; // for all axes

    if ((ret = execCommand(data)) != 1)
    {
        return ret;
    }
    return 0;
}

inline  int DaslPanMotionController::disconnect()
{
    m_conn_flag = 0;
    m_comm.close();

    return 0;
}

inline  int DaslPanMotionController::reqPanPos()
{
    int ret;
    if (m_conn_flag == 0)
    {
        return -1;
    }

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = BOARD_NO;
    data[3] = MOTORCMD_REQUEST_MOTPOSITION; // Request motor pos
    m_comm.clearBuffer();
    if ((ret = execCommand(data)) != 1)
    {
        return ret;
    }

    return 0;
}

inline  int DaslPanMotionController::reqScanPos(int start)
{
    int ret;
    if (m_conn_flag == 0)
    {
        return -1;
    }

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = BOARD_NO;
    data[3] = MOTORCMD_REQUEST_SCANPOSITION; // Request scanner pos

    data[6] = start & 0x01;

    if ((ret = execCommand(data)) != 1)
    {
        return ret;
    }

    return 0;
}

inline int DaslPanMotionController::stop()
{
    int ret;

    if (m_conn_flag == 0)
        return -1;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = BOARD_NO;
    data[3] = 0x28; // stop motor

    if ((ret = execCommand(data)) != 1)
    {
        return ret;
    }

    return 0;
}

inline int DaslPanMotionController::verifyCheckSum(unsigned char *data)
{
    int chksum1 = (data[2] ^ data[3] ^ data[6] ^ data[7] ^ data[8] ^ data[9] ^ data[10] ^ data[11] ^ data[12] ^ data[13] ^ data[14] ^ data[15]);
    int chksum2 = data[6] ^ data[7] ^ data[8] ^ data[9] ^ data[10] ^ data[11] ^ data[12] ^ data[13] ^ data[14] ^ data[15];

    if (data[4] != chksum1)
    {
        return -3;
    }
    if (data[5] != chksum2)
    {
        return -4;
    }
    if ((data[0] & 0xFF) != 0xFF || (data[1] & 0xFF) != 0xFF || (data[2] & 0xFF) != BOARD_NO)
    {
        return -5;
    }
    return 1;
}

inline  void DaslPanMotionController::putCheckSum(unsigned char *data)
{
    data[4] = (data[2] ^ data[3] ^ data[6] ^ data[7] ^ data[8] ^ data[9] ^ data[10] ^ data[11] ^ data[12] ^ data[13] ^ data[14] ^ data[15]);
    data[5] = data[6] ^ data[7] ^ data[8] ^ data[9] ^ data[10] ^ data[11] ^ data[12] ^ data[13] ^ data[14] ^ data[15];
}

inline int DaslPanMotionController::execCommand(unsigned char *data)
{
    putCheckSum(data);
    if (m_comm.write(data, 16) != 16)
    {
        m_conn_flag = 0;
        return -2;
    }

    return 1;
}

inline int DaslPanMotionController::waitReturnValue(unsigned char *data)
{
    int ret;
    m_comm.clearBuffer();
    if ((ret = m_comm.read(data, 16)) != 16)
    {
        return -2;
    }

    if ((ret = verifyCheckSum(data)) != 1)
    {
        return ret;
    }
    return 1;
}



#endif
