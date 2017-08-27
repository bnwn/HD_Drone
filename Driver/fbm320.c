#include "PN020Series.h"
#include "fbm320.h"
#include "common.h"

uint8_t	Formula_Select = 0;

FMTI_Sensor fbm320_packet;


bool fbm320_init(void)
{
    if (I2C_ReadByte(FBM320_SLAVE_ADDRESS, FBM320_REG_CHIPID) != FBM320_CHIPID) {
        return false;
    }
    fbm320_packet.Version = ((I2C_ReadByte(FBM320_SLAVE_ADDRESS, 0xA5) & 0x70) >> 2) | ((I2C_ReadByte(FBM320_SLAVE_ADDRESS, 0xF4) & 0xC0) >> 6);

    /* set default param */
    fbm320_packet.RPC = 3;
    Formula_Select |= 0x01;

    coefficient();

    /* read temperature */
    I2C_WriteByte(FBM320_SLAVE_ADDRESS, FBM320_REG_CMD, FBM320_READ_TEMPERATURE);
    delay_ms(5);
    fbm320_packet.UT = fbm320_read_long_data();

    /* read pressure */
    I2C_WriteByte(FBM320_SLAVE_ADDRESS, FBM320_REG_CMD, FBM320_READ_PRESSURE);
    delay_ms(20);
    fbm320_packet.UP = fbm320_read_long_data();

    calculate(fbm320_packet.UP, fbm320_packet.UT);

    I2C_WriteByte(FBM320_SLAVE_ADDRESS, FBM320_REG_CMD, FBM320_READ_TEMPERATURE);

    return true;
}

int32_t fbm320_read_long_data(void)
{
    uint8_t buf[3];

    I2C_SequentialRead(FBM320_SLAVE_ADDRESS, FBM320_REG_DATA_MSB, buf, 3);

    return (int32_t)(buf[0] << 16 | buf[1] << 8 | buf[2]);
}

/**
 * @brief timer procedure, call in 50Hz
 */
void fbm320_timer_procedure(void)
{
    static uint16_t timer_count = 0;

    switch (timer_count) {
    case 0:
        I2C_WriteByte(FBM320_SLAVE_ADDRESS, FBM320_REG_CMD, FBM320_READ_PRESSURE);
        break;
    case 1: // (20)10.7ms
        fbm320_packet.UP = fbm320_read_long_data();
        I2C_WriteByte(FBM320_SLAVE_ADDRESS, FBM320_REG_CMD, FBM320_READ_TEMPERATURE);
        break;
    case 2: // (40)delay 2.5ms
        fbm320_packet.UT = fbm320_read_long_data();
        I2C_WriteByte(FBM320_SLAVE_ADDRESS, FBM320_REG_CMD, FBM320_READ_PRESSURE);

        calculate(fbm320_packet.UP, fbm320_packet.UT);
        fbm320_packet.Altitude = abs_altitude(fbm320_packet.RP) / 10;
    default:
        break;
    }

    if (timer_count < 3)
        timer_count++;
    else
        timer_count = 1;
}

void coefficient(void)													//Receive Calibrate Coefficient
{
    uint8_t i;
    uint16_t R[10];

    for(i=0; i<9; i++) {
        R[i] = ((uint16_t)I2C_ReadByte(FBM320_SLAVE_ADDRESS, FBM320_REG_COEFF_A + (i*2))<<8) | I2C_ReadByte(FBM320_SLAVE_ADDRESS, (FBM320_REG_COEFF_A + 1) + (i*2));
    }
    R[9] = ((uint16_t)I2C_ReadByte(FBM320_SLAVE_ADDRESS, FBM320_REG_COEFF_D) << 8) | I2C_ReadByte(FBM320_SLAVE_ADDRESS, FBM320_REG_COEFF_F);

    if (((Formula_Select & 0xF0) == 0x10) || ((Formula_Select & 0x0F) == 0x01)) {
        fbm320_packet.C0 = R[0] >> 4;
        fbm320_packet.C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
        fbm320_packet.C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
        fbm320_packet.C3 = R[2] >> 3;
        fbm320_packet.C4 = ((uint32_t)R[3] << 2) | (R[0] & 3);
        fbm320_packet.C5 = R[4] >> 1;
        fbm320_packet.C6 = R[5] >> 3;
        fbm320_packet.C7 = ((uint32_t)R[6] << 3) | (R[5] & 7);
        fbm320_packet.C8 = R[7] >> 3;
        fbm320_packet.C9 = R[8] >> 2;
        fbm320_packet.C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
        fbm320_packet.C11 = R[9] & 0xFF;
        fbm320_packet.C12 = ((R[0] & 0x0C) << 1) | (R[7] & 7);
    } else {
        fbm320_packet.C0 = R[0] >> 4;
        fbm320_packet.C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
        fbm320_packet.C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
        fbm320_packet.C3 = R[2] >> 3;
        fbm320_packet.C4 = ((uint32_t)R[3] << 1) | (R[5] & 1);
        fbm320_packet.C5 = R[4] >> 1;
        fbm320_packet.C6 = R[5] >> 3;
        fbm320_packet.C7 = ((uint32_t)R[6] << 2) | ((R[0] >> 2) & 3);
        fbm320_packet.C8 = R[7] >> 3;
        fbm320_packet.C9 = R[8] >> 2;
        fbm320_packet.C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
        fbm320_packet.C11 = R[9] & 0xFF;
        fbm320_packet.C12 = ((R[5] & 6) << 2) | (R[7] & 7);
    }
}

void calculate(int32_t UP, int32_t UT)		//Calculate Real Pressure & Temperautre
{
    int32_t DT, DT2, X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32, CF, PP1, PP2, PP3, PP4;
    int32_t dC05, dRT, DPC2, DP;
    int32_t BT1 = 0, BT2 = 0, DPTS1 = 0, DPTI1 = 0, DPTS2 = 0, DPTI2 = 0, DPC2S0 = 0, DPC2S1 = 0;

    if (fbm320_packet.Version == 3) {
        DPC2S0 = 559;
        DPC2S1 = 549;
        BT1 = 2000;
        BT2 = 700;
        DPTS1 = -1513;
        DPTI1 = 46;
        DPTS2 = 819;
        DPTI2 = -9;
    }

    if (((Formula_Select & 0xF0) == 0x10) || ((Formula_Select & 0x0F) == 0x01)) { // FMB320 verion 2
        DT	=	((UT - 8388608) >> 4) + (fbm320_packet.C0 << 4);
        X01	=	(fbm320_packet.C1 + 4459) * DT >> 1;
        X02	=	((((fbm320_packet.C2 - 256) * DT) >> 14) * DT) >> 4;
        X03	=	(((((fbm320_packet.C3 * DT) >> 18) * DT) >> 18) * DT);
        fbm320_packet.RT	=	((2500 << 15) - X01 - X02 - X03) >> 15;

        DT2	=	(X01 + X02 + X03) >> 12;

        X11	=	((fbm320_packet.C5 - 4443) * DT2);
        X12	=	(((fbm320_packet.C6 * DT2) >> 16) * DT2) >> 2;
        X13	=	((X11 + X12) >> 10) + ((fbm320_packet.C4 + 120586) << 4);

        X21	=	((fbm320_packet.C8 + 7180) * DT2) >> 10;
        X22	=	(((fbm320_packet.C9 * DT2) >> 17) * DT2) >> 12;
        if(X22 >= X21)
            X23	=	X22 - X21;
        else
            X23	=	X21 - X22;
        X24	=	(X23 >> 11) * (fbm320_packet.C7 + 166426);
        X25	=	((X23 & 0x7FF) * (fbm320_packet.C7 + 166426)) >> 11;
        if((X22 - X21) < 0)
            X26	=	((0 - X24 - X25) >> 11) + fbm320_packet.C7 + 166426;
        else
            X26	=	((X24 + X25) >> 11) + fbm320_packet.C7 + 166426;

        PP1	=	((fbm320_packet.UP - 8388608) - X13) >> 3;
        PP2	=	(X26 >> 11) * PP1;
        PP3	=	((X26 & 0x7FF) * PP1) >> 11;
        PP4	=	(PP2 + PP3) >> 10;

        CF	=	(2097152 + fbm320_packet.C12 * DT2) >> 3;
        X31	=	(((CF * fbm320_packet.C10) >> 17) * PP4) >> 2;
        X32	=	(((((CF * fbm320_packet.C11) >> 15) * PP4) >> 18) * PP4);
        fbm320_packet.RP	=	((X31 + X32) >> 15) + PP4 + 99880;
    } else {                                                                 // FMB320 verion 1
        DT	=	((UT - 8388608) >> 4) + (fbm320_packet.C0 << 4);
        X01	=	(fbm320_packet.C1 + 4418) * DT >> 1;
        X02	=	((((fbm320_packet.C2 - 256) * DT) >> 14) * DT) >> 4;
        X03	=	(((((fbm320_packet.C3 * DT) >> 18) * DT) >> 18) * DT);
        fbm320_packet.RT = ((2500 << 15) - X01 - X02 - X03) >> 15;

        DT2	=	(X01 + X02 + X03) >>12;

        X11	=	(fbm320_packet.C5 * DT2);
        X12	=	(((fbm320_packet.C6 * DT2) >> 16) * DT2) >> 2;
        X13	=	((X11 + X12) >> 10) + ((fbm320_packet.C4 + 211288) << 4);

        X21	=	((fbm320_packet.C8 + 7209) * DT2) >> 10;
        X22	=	(((fbm320_packet.C9 * DT2) >> 17) * DT2) >> 12;
        if(X22 >= X21)
            X23	=	X22 - X21;
        else
            X23	=	X21 - X22;
        X24	=	(X23 >> 11) * (fbm320_packet.C7 + 285594);
        X25	=	((X23 & 0x7FF) * (fbm320_packet.C7 + 285594)) >> 11;
        if((X22 - X21) < 0)
            X26	=	((0 - X24 - X25) >> 11) + fbm320_packet.C7 + 285594;
        else
            X26	=	((X24 + X25) >> 11) + fbm320_packet.C7 + 285594;
        PP1	=	((fbm320_packet.UP - 8388608) - X13) >> 3;
        PP2	=	(X26 >> 11) * PP1;
        PP3	=	((X26 & 0x7FF) * PP1) >> 11;
        PP4	=	(PP2 + PP3) >> 10;

        CF	=	(2097152 + fbm320_packet.C12 * DT2) >> 3;
        X31	=	(((CF * fbm320_packet.C10) >> 17) * PP4) >> 2;
        X32	=	(((((CF * fbm320_packet.C11) >> 15) * PP4) >> 18) * PP4);
        fbm320_packet.RP = ((X31 + X32) >> 15) + PP4 + 99880;
    }

    if (fbm320_packet.Version == 3)   // for fbm320 version 3
    {
        if ((fbm320_packet.RPC & 1) == 1) {
            dC05 = fbm320_packet.C5 - 16384;
            dRT = fbm320_packet.RT - 2500;
            DPC2 = (((DPC2S1 * dC05) >> 9) + DPC2S0) >> 4;
            DP = (((DPC2 * dRT) >> 15) * dRT) >> 11;
            fbm320_packet.RP -= DP;
        }

        if ((fbm320_packet.RPC & 2) == 2) {
            if (fbm320_packet.RT < BT1) {
                DP = ((fbm320_packet.RT * DPTS1) >> 16) + DPTI1;
                if(fbm320_packet.RT < BT2)
                    DP = ((fbm320_packet.RT * DPTS2) >> 16) + DPTI2 + DP;
                fbm320_packet.RP -= DP;
            }
        }
    }
}

int32_t abs_altitude(int32_t Press)	//Calculate absolute altitude, unit: mm
{
    int8_t _P0;
    int16_t _hs1, _dP0;
    int32_t _h0, _hs0, _HP1, _HP2;

    if(Press >= 103000)
    {
        _P0	=	103;
        _h0	=	-138507;
        _hs0	=	-21007;
        _hs1	=	311;
    }
    else if(Press >= 98000)
    {
        _P0	=	98;
        _h0	=	280531;
        _hs0	=	-21869;
        _hs1	=	338;
    }
    else if(Press >= 93000)
    {
        _P0	=	93;
        _h0	=	717253;
        _hs0	=	-22813;
        _hs1	=	370;
    }

    else if(Press >= 88000)
    {
        _P0	=	88;
        _h0	=	1173421;
        _hs0	=	-23854;
        _hs1	=	407;
    }
    else if(Press >= 83000)
    {
        _P0	=	83;
        _h0	=	1651084;
        _hs0	=	-25007;
        _hs1	=	450;
    }
    else if(Press >= 78000)
    {
        _P0	=	78;
        _h0	=	2152645;
        _hs0	=	-26292;
        _hs1	=	501;
    }
    else if(Press >= 73000)
    {
        _P0	=	73;
        _h0	=	2680954;
        _hs0	=	-27735;
        _hs1	=	560;
    }
    else if(Press >= 68000)
    {
        _P0	=	68;
        _h0	=	3239426;
        _hs0	=	-29366;
        _hs1	=	632;
    }
    else if(Press >= 63000)
    {
        _P0	=	63;
        _h0	=	3832204;
        _hs0	=	-31229;
        _hs1	=	719;
    }
    else if(Press >= 58000)
    {
        _P0	=	58;
        _h0	=	4464387;
        _hs0	=	-33377;
        _hs1	=	826;
    }
    else if(Press >= 53000)
    {
        _P0	=	53;
        _h0	=	5142359;
        _hs0	=	-35885;
        _hs1	=	960;
    }
    else if(Press >= 48000)
    {
        _P0	=	48;
        _h0	=	5874268;
        _hs0	=	-38855;
        _hs1	=	1131;
    }
    else if(Press >= 43000)
    {
        _P0	=	43;
        _h0	=	6670762;
        _hs0	=	-42434;
        _hs1	=	1354;
    }
    else if(Press >= 38000)
    {
        _P0	=	38;
        _h0	=	7546157;
        _hs0	=	-46841;
        _hs1	=	1654;
    }
    else if(Press >= 33000)
    {
        _P0	=	33;
        _h0	=	8520395;
        _hs0	=	-52412;
        _hs1	=	2072;
    }
    else
    {
        _P0	=	28;
        _h0	=	9622536;
        _hs0	=	-59704;
        _hs1	=	2682;
    }

    _dP0	=	Press - _P0 * 1000;
    _HP1	=	(_hs0 * _dP0) >> 2;
    _HP2	=	(((_hs1 * _dP0) >> 10)* _dP0) >> 4;

    return	((_h0 << 6) + _HP1 + _HP2) >> 6;										//Return absolute altitude
}
