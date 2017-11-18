#include <stdio.h>
#include <windows.h>

typedef struct SensorData1 {
	int IRSensor[6];
	int TorqueSensor[2];
	int TemperatureSensor;
	int CDSSensor;
	int GSensor[3];
	int MSensor[3];
	int GySensor[3];
	int SteeringVar;
	int SteeringTorque;
	int Battery;
	int Remote;
	int MTemperature;
}SensorData;

HANDLE AltinoComDevId;

char szPort[15] = "\\\\.\\COM3";
unsigned char tx_d[28] = { 0, };
unsigned char Sendbuf[28] = { 0, };
unsigned char rx_d[31] = { 0, };
unsigned char rx_d56[56]= { 0, };
unsigned char rx_data[1028] = { 0, };
unsigned char rx_d_sensor1[31] = { 0, };
unsigned char rx_d_sensor2[31] = { 0, };
unsigned int speed = 0;
unsigned char connectstate = 0;
DWORD dwByte;

unsigned char check_sum_tx_calcuration(int u16_cnt)
{
	int u16_tx_check_sum = 0;
	int u16_tx_cnt;
	u16_tx_check_sum = u16_tx_check_sum + tx_d[1];
	for (u16_tx_cnt = 3; u16_tx_cnt <= u16_cnt; u16_tx_cnt++) {
		u16_tx_check_sum = u16_tx_check_sum + tx_d[u16_tx_cnt];
	}
	u16_tx_check_sum = u16_tx_check_sum % 256;
	return (byte)(u16_tx_check_sum);
}

void delay(int ms)
{
    Sleep(ms);
}
void Open(char *szPort)
{
	if (connectstate == 0)
	{
		AltinoComDevId = CreateFile(
			szPort,
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			0,
			NULL);

		COMMTIMEOUTS CommTimeOuts;
		GetCommTimeouts(AltinoComDevId, &CommTimeOuts);
		CommTimeOuts.ReadIntervalTimeout = 0;
		CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
		CommTimeOuts.ReadTotalTimeoutConstant = 0;
		CommTimeOuts.WriteTotalTimeoutMultiplier = 0;
		CommTimeOuts.WriteTotalTimeoutConstant = 0;
		SetCommTimeouts(AltinoComDevId, &CommTimeOuts);

		DCB dcb;
		GetCommState(AltinoComDevId, &dcb);
		dcb.BaudRate = CBR_115200;
		dcb.ByteSize = 8;
		dcb.Parity = NOPARITY;
		dcb.StopBits = 0;
		dcb.fBinary = TRUE;
		dcb.fParity = FALSE;

		SetCommState(AltinoComDevId, &dcb);
		Sendbuf[4] = 1;
		Sleep(1000);
		connectstate = 1;
	}
}

void Close()
{
	if (connectstate == 1)
	{
		Sleep(1000);
		CloseHandle(AltinoComDevId);
		connectstate = 0;
	}
}

void SendData(unsigned char *Sendbuf)
{
	tx_d[0] = 0x2;
	tx_d[1] = 28;

	tx_d[3] = 1;
	tx_d[4] = Sendbuf[4];
	tx_d[5] = Sendbuf[5];
	tx_d[6] = Sendbuf[6];
	tx_d[7] = Sendbuf[7];
	tx_d[8] = Sendbuf[8];
	tx_d[9] = Sendbuf[9];
	tx_d[10] = Sendbuf[10];
	tx_d[11] = Sendbuf[11];
	tx_d[12] = Sendbuf[12];
	tx_d[13] = Sendbuf[13];
	tx_d[14] = Sendbuf[14];
	tx_d[15] = Sendbuf[15];
	tx_d[16] = Sendbuf[16];
	tx_d[17] = Sendbuf[17];
	tx_d[18] = Sendbuf[18];
	tx_d[19] = Sendbuf[19];
	tx_d[20] = Sendbuf[20];
	if (Sendbuf[4] == 1 || Sendbuf[4]==10) {
	    Sendbuf[21] = Sendbuf[21] | 0x01;
	}
	else  {
	    Sendbuf[21] = Sendbuf[21];
    }
	tx_d[21] = Sendbuf[21];
	tx_d[22] = Sendbuf[22];
	tx_d[23] = Sendbuf[23];
	tx_d[24] = Sendbuf[24];
	tx_d[25] = Sendbuf[25];
	tx_d[26] = 0;
	tx_d[27] = 0x3;

	tx_d[2] = check_sum_tx_calcuration(27);

	PurgeComm(AltinoComDevId, PURGE_TXCLEAR);
	WriteFile(AltinoComDevId, tx_d, 28, &dwByte, 0);
}

void Go(int left, int right)
{
    Sendbuf[6] = 0;
	Sendbuf[9] = 0;
	if (left>1000)
		left = 1000;
	else if (left<-1000)
		left = -1000;

	if (left<0)
		left = 32768 - left;

	if (right>1000)
		right = 1000;
	else if (right<-1000)
		right = -1000;

	if (right<0)
		right = 32768 - right;

	if (right==0)
		Sendbuf[6] = 255;
	else
		Sendbuf[6] = 0;

	Sendbuf[7] = (byte)(right / 256);
	Sendbuf[8] = (byte)(right % 256);

	if (left == 0)
		Sendbuf[9] = 255;
	else
		Sendbuf[9] = 0;

	Sendbuf[10] = (byte)(left / 256);
	Sendbuf[11] = (byte)(left % 256);

	SendData(Sendbuf);
}


void Go2(int left, int right)
{
    Sendbuf[6] = 1;
	Sendbuf[9] = 1;
	if (left>1000)
		left = 1000;
	else if (left<-1000)
		left = -1000;

	if (left<0)
		left = 32768 - left;

	if (right>1000)
		right = 1000;
	else if (right<-1000)
		right = -1000;

	if (right<0)
		right = 32768 - right;

	if (right==0)
		Sendbuf[6] = 255;
	else
		Sendbuf[6] = 1;

	Sendbuf[7] = (byte)(right / 256);
	Sendbuf[8] = (byte)(right % 256);

	if (left == 0)
		Sendbuf[9] = 255;
	else
		Sendbuf[9] = 1;

	Sendbuf[10] = (byte)(left / 256);
	Sendbuf[11] = (byte)(left % 256);

	SendData(Sendbuf);
}


void Steering(int steeringvalue)
{
    Sendbuf[24] = 0;
	Sendbuf[5] = (byte)steeringvalue;
	SendData(Sendbuf);
}

void Steering2(int value1, int value2)
{
    if(value1>127)
		value1=127;
	if(value2>127)
		value2=127;
	if(value1<-127)
		value1=-127;
	if(value2<-127)
		value1=-127;
	if(value1<0)
		value1=128-value1;
	if(value2<0)
		value2=128-value2;
    Sendbuf[24] = 1;
	Sendbuf[5] = (byte)value1;
	Sendbuf[25]=(byte)value2;
	SendData(Sendbuf);
}

void Sound(unsigned char buzzer)
{
	Sendbuf[22] = (byte)buzzer;
	SendData(Sendbuf);
}

void Display(unsigned char ASCII)
{
	Sendbuf[12] = ASCII;
	Sendbuf[13] = 0;
	Sendbuf[14] = 0;
	Sendbuf[15] = 0;
	Sendbuf[16] = 0;
	Sendbuf[17] = 0;
	Sendbuf[18] = 0;
	Sendbuf[19] = 0;
	Sendbuf[20] = 0;
	SendData(Sendbuf);
}

void DisplayLine(unsigned char dot0, unsigned char dot1, unsigned char dot2, unsigned char dot3, unsigned char dot4, unsigned char dot5, unsigned char dot6, unsigned char dot7)
{
	Sendbuf[12] = 0;
	Sendbuf[13] = dot0;
	Sendbuf[14] = dot1;
	Sendbuf[15] = dot2;
	Sendbuf[16] = dot3;
	Sendbuf[17] = dot4;
	Sendbuf[18] = dot5;
	Sendbuf[19] = dot6;
	Sendbuf[20] = dot7;

	SendData(Sendbuf);
}

void Led(unsigned int led)
{
	Sendbuf[23] = (byte)(led%256);
	Sendbuf[21] = (byte) (led/256);
	SendData(Sendbuf);
}

void chekdata()
{
	int rx_check_sum;

	if ((rx_d[0] == 2) && (rx_d[30] == 3) && (rx_d[1] == 31)) {
		rx_check_sum = rx_d[0];
		rx_check_sum = rx_check_sum + rx_d[1];
		rx_check_sum = rx_check_sum + rx_d[3];
		rx_check_sum = rx_check_sum + rx_d[4];
		rx_check_sum = rx_check_sum + rx_d[5];
		rx_check_sum = rx_check_sum + rx_d[6];
		rx_check_sum = rx_check_sum + rx_d[7];
		rx_check_sum = rx_check_sum + rx_d[8];
		rx_check_sum = rx_check_sum + rx_d[9];
		rx_check_sum = rx_check_sum + rx_d[10];
		rx_check_sum = rx_check_sum + rx_d[11];
		rx_check_sum = rx_check_sum + rx_d[12];
		rx_check_sum = rx_check_sum + rx_d[13];
		rx_check_sum = rx_check_sum + rx_d[14];
		rx_check_sum = rx_check_sum + rx_d[15];
		rx_check_sum = rx_check_sum + rx_d[16];
		rx_check_sum = rx_check_sum + rx_d[17];
		rx_check_sum = rx_check_sum + rx_d[18];
		rx_check_sum = rx_check_sum + rx_d[19];
		rx_check_sum = rx_check_sum + rx_d[20];
		rx_check_sum = rx_check_sum + rx_d[21];
		rx_check_sum = rx_check_sum + rx_d[22];
		rx_check_sum = rx_check_sum + rx_d[23];
		rx_check_sum = rx_check_sum + rx_d[24];
		rx_check_sum = rx_check_sum + rx_d[25];
		rx_check_sum = rx_check_sum + rx_d[26];
		rx_check_sum = rx_check_sum + rx_d[27];
		rx_check_sum = rx_check_sum + rx_d[28];
		rx_check_sum = rx_check_sum + rx_d[29];
		rx_check_sum = rx_check_sum + rx_d[30];
		rx_check_sum = rx_check_sum % 256;

		if (rx_check_sum == rx_d[2])
		{
			if (rx_d[4] == 1)
			{
				rx_d_sensor1[7] = rx_d[7];
				rx_d_sensor1[8] = rx_d[8];
				rx_d_sensor1[9] = rx_d[9];
				rx_d_sensor1[10] = rx_d[10];
				rx_d_sensor1[11] = rx_d[11];
				rx_d_sensor1[12] = rx_d[12];
				rx_d_sensor1[13] = rx_d[13];
				rx_d_sensor1[14] = rx_d[14];
				rx_d_sensor1[15] = rx_d[15];
				rx_d_sensor1[16] = rx_d[16];
				rx_d_sensor1[17] = rx_d[17];
				rx_d_sensor1[18] = rx_d[18];
				rx_d_sensor1[19] = rx_d[19];
				rx_d_sensor1[20] = rx_d[20];
				rx_d_sensor1[21] = rx_d[21];
				rx_d_sensor1[22] = rx_d[22];
				rx_d_sensor1[23] = rx_d[23];
				rx_d_sensor1[24] = rx_d[24];
				rx_d_sensor1[25] = rx_d[25];
				rx_d_sensor1[26] = rx_d[26];
			}
			else if(rx_d[4] == 2)
			{
				rx_d_sensor2[7] = rx_d[7];
				rx_d_sensor2[8] = rx_d[8];
				rx_d_sensor2[9] = rx_d[9];
				rx_d_sensor2[10] = rx_d[10];
				rx_d_sensor2[11] = rx_d[11];
				rx_d_sensor2[12] = rx_d[12];
				rx_d_sensor2[13] = rx_d[13];
				rx_d_sensor2[14] = rx_d[14];
				rx_d_sensor2[15] = rx_d[15];
				rx_d_sensor2[16] = rx_d[16];
				rx_d_sensor2[17] = rx_d[17];
				rx_d_sensor2[18] = rx_d[18];
				rx_d_sensor2[19] = rx_d[19];
				rx_d_sensor2[20] = rx_d[20];
				rx_d_sensor2[21] = rx_d[21];
				rx_d_sensor2[22] = rx_d[22];
				rx_d_sensor2[23] = rx_d[23];
				rx_d_sensor2[24] = rx_d[24];
				rx_d_sensor2[25] = rx_d[25];
			}
		}
	}
}

void chekdata56()
{
	int rx_check_sum=0;


	if ((rx_d56[0] == 2) && (rx_d56[55] == 3) && (rx_d56[1] == 56)) {
		rx_check_sum = rx_d56[0];
		rx_check_sum = rx_check_sum + rx_d56[1];
		rx_check_sum = rx_check_sum + rx_d56[3];
		rx_check_sum = rx_check_sum + rx_d56[4];
		rx_check_sum = rx_check_sum + rx_d56[5];
		rx_check_sum = rx_check_sum + rx_d56[6];
		rx_check_sum = rx_check_sum + rx_d56[7];
		rx_check_sum = rx_check_sum + rx_d56[8];
		rx_check_sum = rx_check_sum + rx_d56[9];
		rx_check_sum = rx_check_sum + rx_d56[10];
		rx_check_sum = rx_check_sum + rx_d56[11];
		rx_check_sum = rx_check_sum + rx_d56[12];
		rx_check_sum = rx_check_sum + rx_d56[13];
		rx_check_sum = rx_check_sum + rx_d56[14];
		rx_check_sum = rx_check_sum + rx_d56[15];
		rx_check_sum = rx_check_sum + rx_d56[16];
		rx_check_sum = rx_check_sum + rx_d56[17];
		rx_check_sum = rx_check_sum + rx_d56[18];
		rx_check_sum = rx_check_sum + rx_d56[19];
		rx_check_sum = rx_check_sum + rx_d56[20];
		rx_check_sum = rx_check_sum + rx_d56[21];
		rx_check_sum = rx_check_sum + rx_d56[22];
		rx_check_sum = rx_check_sum + rx_d56[23];
		rx_check_sum = rx_check_sum + rx_d56[24];
		rx_check_sum = rx_check_sum + rx_d56[25];
		rx_check_sum = rx_check_sum + rx_d56[26];
		rx_check_sum = rx_check_sum + rx_d56[27];
		rx_check_sum = rx_check_sum + rx_d56[28];
		rx_check_sum = rx_check_sum + rx_d56[29];
		rx_check_sum = rx_check_sum + rx_d56[30];
		rx_check_sum = rx_check_sum + rx_d56[31];
		rx_check_sum = rx_check_sum + rx_d56[32];
		rx_check_sum = rx_check_sum + rx_d56[33];
		rx_check_sum = rx_check_sum + rx_d56[34];
		rx_check_sum = rx_check_sum + rx_d56[35];
		rx_check_sum = rx_check_sum + rx_d56[36];
		rx_check_sum = rx_check_sum + rx_d56[37];
		rx_check_sum = rx_check_sum + rx_d56[38];
		rx_check_sum = rx_check_sum + rx_d56[39];
		rx_check_sum = rx_check_sum + rx_d56[40];
		rx_check_sum = rx_check_sum + rx_d56[41];
		rx_check_sum = rx_check_sum + rx_d56[42];
		rx_check_sum = rx_check_sum + rx_d56[43];
		rx_check_sum = rx_check_sum + rx_d56[44];
		rx_check_sum = rx_check_sum + rx_d56[45];
		rx_check_sum = rx_check_sum + rx_d56[46];
		rx_check_sum = rx_check_sum + rx_d56[47];
		rx_check_sum = rx_check_sum + rx_d56[48];
		rx_check_sum = rx_check_sum + rx_d56[49];
		rx_check_sum = rx_check_sum + rx_d56[50];
		rx_check_sum = rx_check_sum + rx_d56[51];
		rx_check_sum = rx_check_sum + rx_d56[52];
		rx_check_sum = rx_check_sum + rx_d56[53];
		rx_check_sum = rx_check_sum + rx_d56[54];
		rx_check_sum = rx_check_sum + rx_d56[55];

		rx_check_sum = rx_check_sum % 256;

		if (rx_check_sum==rx_d56[2])
		{
            rx_d_sensor1[7] = rx_d56[7]; //ir1
            rx_d_sensor1[8] = rx_d56[8]; //ir1
            rx_d_sensor1[9] = rx_d56[9];  //ir2
            rx_d_sensor1[10] = rx_d56[10]; //ir2
            rx_d_sensor1[11] = rx_d56[11]; //ir3
            rx_d_sensor1[12] = rx_d56[12]; //ir3
            rx_d_sensor1[13] = rx_d56[13]; //ir4
            rx_d_sensor1[14] = rx_d56[14]; //ir4
            rx_d_sensor1[15] = rx_d56[15]; //ir5
            rx_d_sensor1[16] = rx_d56[16]; //ir5
            rx_d_sensor1[17] = rx_d56[17]; //ir6
            rx_d_sensor1[18] = rx_d56[18]; //ir6

            rx_d_sensor1[19] = rx_d56[19]; //right torque
            rx_d_sensor1[20] = rx_d56[20]; //right torque

            rx_d_sensor1[21] = rx_d56[21]; //left torque
            rx_d_sensor1[22] = rx_d56[22]; //left torque

            rx_d_sensor1[23] = rx_d56[49];  //temperature
            rx_d_sensor1[24] = rx_d56[50];  //temperature

            rx_d_sensor1[25] = rx_d56[43]; //cds
            rx_d_sensor1[26] = rx_d56[44]; //cds

            rx_d_sensor2[7] = rx_d56[25]; //acc x
            rx_d_sensor2[8] = rx_d56[26]; //acc x
            rx_d_sensor2[9] = rx_d56[27]; //acc y
            rx_d_sensor2[10] = rx_d56[28]; //acc y
            rx_d_sensor2[11] = rx_d56[29]; //acc z
            rx_d_sensor2[12] = rx_d56[30]; //acc z

            rx_d_sensor2[13] = rx_d56[31]; //mag x
            rx_d_sensor2[14] = rx_d56[32]; //mag x
            rx_d_sensor2[15] = rx_d56[33]; //mag y
            rx_d_sensor2[16] = rx_d56[34]; //mag y
            rx_d_sensor2[17] = rx_d56[35]; //mag z
            rx_d_sensor2[18] = rx_d56[36]; //mag z

            rx_d_sensor2[19] = rx_d56[45]; //steering var
            rx_d_sensor2[20] = rx_d56[46]; //steering var
            rx_d_sensor2[21] = rx_d56[23]; //steering torque
            rx_d_sensor2[22] = rx_d56[24]; //steering torque
            rx_d_sensor2[23] = rx_d56[47]; //battery
            rx_d_sensor2[24] = rx_d56[48]; //battery
            rx_d_sensor2[25] = rx_d56[51]; //remote control
            rx_d_sensor2[26] = rx_d56[52]; //M temperature

            rx_d_sensor2[27] = rx_d56[37]; //gyro sensor x
            rx_d_sensor2[28] = rx_d56[38]; //gyro sensor x

            rx_d_sensor2[29] = rx_d56[39]; //gyro sensor y
            rx_d_sensor2[30] = rx_d56[40]; //gyro sensor y

            rx_d_sensor2[5] = rx_d56[41]; //gyro sensor x
            rx_d_sensor2[6] = rx_d56[42]; //gyro sensor y

		}
	}
}



SensorData Sensor(int command)
{
	SensorData sensordata1;

	Sendbuf[4] = (byte)command;

	SendData(Sendbuf);
    if(Sendbuf[4]==1 || Sendbuf[4]==2)
    {
        ReadFile(AltinoComDevId, rx_d, 31, &dwByte, 0);
        chekdata();
    }
	else if(Sendbuf[4]==10)
    {
        ReadFile(AltinoComDevId, rx_d56, 56, &dwByte, 0);
        chekdata56();
    }
    PurgeComm(AltinoComDevId, PURGE_RXCLEAR);

	sensordata1.IRSensor[0] = rx_d_sensor1[7] * 256 + rx_d_sensor1[8];
	sensordata1.IRSensor[1] = rx_d_sensor1[9] * 256 + rx_d_sensor1[10];
	sensordata1.IRSensor[2] = rx_d_sensor1[11] * 256 + rx_d_sensor1[12];
	sensordata1.IRSensor[3] = rx_d_sensor1[13] * 256 + rx_d_sensor1[14];
	sensordata1.IRSensor[4] = rx_d_sensor1[15] * 256 + rx_d_sensor1[16];
	sensordata1.IRSensor[5] = rx_d_sensor1[17] * 256 + rx_d_sensor1[18];

	sensordata1.TorqueSensor[0] = rx_d_sensor1[19] * 256 + rx_d_sensor1[20];
	sensordata1.TorqueSensor[1] = rx_d_sensor1[21] * 256 + rx_d_sensor1[22];

	sensordata1.TemperatureSensor = rx_d_sensor1[23] * 256 + rx_d_sensor1[24];
	sensordata1.CDSSensor = rx_d_sensor1[25] * 256 + rx_d_sensor1[26];

	sensordata1.GSensor[0] = rx_d_sensor2[7] * 256 + rx_d_sensor2[8];
	sensordata1.GSensor[1] = rx_d_sensor2[9] * 256 + rx_d_sensor2[10];
	sensordata1.GSensor[2] = rx_d_sensor2[11] * 256 + rx_d_sensor2[12];

	sensordata1.MSensor[0] = rx_d_sensor2[13] * 256 + rx_d_sensor2[14];
	sensordata1.MSensor[1] = rx_d_sensor2[15] * 256 + rx_d_sensor2[16];
	sensordata1.MSensor[2] = rx_d_sensor2[17] * 256 + rx_d_sensor2[18];

	sensordata1.SteeringVar = rx_d_sensor2[19] * 256 + rx_d_sensor2[20];
	sensordata1.SteeringTorque = rx_d_sensor2[21] * 256 + rx_d_sensor2[22];
	sensordata1.Battery = rx_d_sensor2[23] * 256 + rx_d_sensor2[24];

	sensordata1.Remote = rx_d_sensor2[25];
	sensordata1.MTemperature = rx_d_sensor2[26];

	sensordata1.GySensor[0] = rx_d_sensor2[27] * 256 + rx_d_sensor2[28];
	sensordata1.GySensor[1] = rx_d_sensor2[29] * 256 + rx_d_sensor2[30];
	sensordata1.GySensor[2] = rx_d_sensor2[5] * 256 + rx_d_sensor2[6];

	if(sensordata1.GSensor[0]>32768)
        sensordata1.GSensor[0]=sensordata1.GSensor[0]-65535;

    if(sensordata1.GSensor[1]>32768)
        sensordata1.GSensor[1]=sensordata1.GSensor[1]-65535;

	if(sensordata1.GSensor[2]>32768)
        sensordata1.GSensor[2]=sensordata1.GSensor[2]-65535;

    if(sensordata1.MSensor[0]>32768)
        sensordata1.MSensor[0]=sensordata1.MSensor[0]-65535;

    if(sensordata1.MSensor[1]>32768)
        sensordata1.MSensor[1]=sensordata1.MSensor[1]-65535;

    if(sensordata1.MSensor[2]>32768)
        sensordata1.MSensor[2]=sensordata1.MSensor[2]-65535;


 	if(sensordata1.GySensor[0]>32768)
        sensordata1.GySensor[0]=sensordata1.GySensor[0]-65535;

    if(sensordata1.GySensor[1]>32768)
        sensordata1.GySensor[1]=sensordata1.GySensor[1]-65535;

	if(sensordata1.GySensor[2]>32768)
        sensordata1.GySensor[2]=sensordata1.GySensor[2]-65535;

	return sensordata1;
}


