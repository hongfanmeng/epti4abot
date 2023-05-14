#ifndef CHIPS_JY61P
#define CHIPS_JY61P

#include <string>
#include <stdio.h>
#include "memory.h"

namespace Wit
{
    
class JY61P
{
    private :
        struct STCTime
        {
            unsigned char yy;
            unsigned char mm;
            unsigned char dd;
            unsigned char hh;
            unsigned char mn;
            unsigned char ss;
            short ms;
        } stcTime;
        struct STCAcc
        {
            short a[3];
            short t;
        } stcAcc;
        struct STCGyro
        {
            short w[3];
            short v;
        } stcGyro;
        struct STCMag
        {
            short h[3];
            short t;
        } stcMag;
        struct STCAngle
        {
            short Angle[3];
            short v;
        } stcAngle;
        struct STCQuat
        {
            short q[4];
        } stcQuat;
    public :
    bool dirty[6];
    struct Combined
        {
            float w;
            float x;
            float y;
            float z;
        } acc, mag, gyro, quat;
    struct Angle
        {
            float r;
            float p;
            float y;
        } angle;
    JY61P() 
    {

    }
    ~JY61P() 
    {

    }
    void FetchData(char *data, int usLength)
    {
        for (int i = 0 ; i < 6 ; ++i) {
            dirty[i] = false;
        }
        char *pData_head = data;
        while (usLength >= 11)
        {
            if (pData_head[0] != 0x55)
            {
                pData_head++;
                usLength--;
                continue;
            }
            switch (pData_head[1])
            {
            case 0x50: //time
                memcpy(&stcTime, &pData_head[2], 8);
                dirty[0] = true;
                break;
            case 0x51: //acceleration
                memcpy(&stcAcc, &pData_head[2], 8);
                acc.x = stcAcc.a[0] / 32768.00 * 16 * 9.8;
                acc.y = stcAcc.a[1] / 32768.00 * 16 * 9.8;
                acc.z = stcAcc.a[2] / 32768.00 * 16 * 9.8;
                dirty[1] = true;
                break;
            case 0x52: //gyroscope
                memcpy(&stcGyro, &pData_head[2], 8);
                gyro.x = stcGyro.w[0] / 32768.00 * 2000 / 180 * 3.1415926;
                gyro.y = stcGyro.w[1] / 32768.00 * 2000 / 180 * 3.1415926;
                gyro.z = stcGyro.w[2] / 32768.00 * 2000 / 180 * 3.1415926;
                dirty[2] = true;
                break;
            case 0x53: //angles
                memcpy(&stcAngle, &pData_head[2], 8);
                angle.r = stcAngle.Angle[0] / 32768.00 * 3.1415926;
                angle.p = stcAngle.Angle[1] / 32768.00 * 3.1415926;
                angle.y = stcAngle.Angle[2] / 32768.00 * 3.1415926;
                dirty[3] = true;
                break;
            case 0x54: //magnet
                memcpy(&stcMag, &pData_head[2], 8);
                mag.x = stcMag.h[0];
                mag.y = stcMag.h[1];
                mag.z = stcMag.h[2];
                dirty[4] = true;
                break;
            case 0x59: //quaternion
                memcpy(&stcQuat, &pData_head[2], 8);
                quat.w = stcQuat.q[0] / 32768.00;
                quat.x = stcQuat.q[1] / 32768.00;
                quat.y = stcQuat.q[2] / 32768.00;
                quat.z = stcQuat.q[3] / 32768.00;
                dirty[5] = true;
                break;
            }
            usLength -= 11;
            pData_head += 11;
        }
    }
};

}// namespace Wit

#endif
