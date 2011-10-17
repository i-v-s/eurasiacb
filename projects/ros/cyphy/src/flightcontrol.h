/*
 *	Mikrokopter Flight control Serial Interface
 *	Copyright (C) 2010, CYPHY lab
 *	Inkyu Sa <i.sa@qut.edu.au>
 *
 *	http://wiki.qut.edu.au/display/cyphy
 *
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.	If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MIKO_FLIGHTCONTROL_FLIGHTCONTROL_H
#define MIKO_FLIGHTCONTROL_FLIGHTCONTROL_H //нахрена ему это определение, которое ничего не  определяет?

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>
#include <stdarg.h>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include "sensor_msgs/Imu.h"
#include <sys/time.h>
#include <control_toolbox/pid.h>
#include "serialinterface.h"
#include "serial_interface.cpp"

#if 0
#define TRUE 1
#define FALSE 0
#define YES 1
#define NO 0
#endif

#define FC_ADDRESS 1 // значение, котрое передаётся функции SendOutData. В функции выглядит так uint8_t addr. Записывает второй символ массива  g_txd_buffer как
// g_txd_buffer[pt++] = 'a' + addr;  // Address (a=0; b=1,...) - нужно понять, что значит второй символ в массиве этом
#define MAX_THROTTLE 255
#define YAW_SCALE_FACTOR 80
//#define PITCH_SCALE_FACTOR 128
//#define ROLL_SCALE_FACTOR 128

#define PITCH_SCALE_FACTOR 30
#define ROLL_SCALE_FACTOR 30


#define MOTOR_FRONT 12
#define MOTOR_REAR 13
#define MOTOR_LEFT 14
#define MOTOR_RIGHT 15
#define ANGLE_PITCH 0
#define ANGLE_ROLL 1
#define ANGLE_YAW 11
#define GYRO_YAW 4
#define HEIGHT 5

#define ACC_Z 6
#define ACC_X 2
#define ACC_Y 3




struct str_Data3D
{
	signed int	angle[3]; // pitch, roll, yaw in 0,1�
	signed char Centroid[3];
	signed char reserve[5];
};

typedef struct
{
			uint8_t Digital[2];
			int16_t Analog[32];	 // Debugvalues
} __attribute__((packed)) DebugOut_t;

#if 0
	 DebugOut.Analog[0] = IntegralNick / (EE_Parameter.GyroAccFaktor * 4);
	 DebugOut.Analog[1] = IntegralRoll / (EE_Parameter.GyroAccFaktor * 4);
	 DebugOut.Analog[2] = Mittelwert_AccNick / 4;
	 DebugOut.Analog[3] = Mittelwert_AccRoll / 4;
	 DebugOut.Analog[4] = (signed int) AdNeutralGier - AdWertGier;
	 DebugOut.Analog[5] = HoehenWert/5;
	 DebugOut.Analog[6] = AdWertAccHoch;//(Mess_Integral_Hoch / 512);// Aktuell_az;
	 DebugOut.Analog[8] = KompassValue;
	 DebugOut.Analog[9] = UBat;
	 DebugOut.Analog[10] = SenderOkay;
	 DebugOut.Analog[11] = ErsatzKompass / GIER_GRAD_FAKTOR;
	 DebugOut.Analog[12] = Motor[0].SetPoint;
	 DebugOut.Analog[13] = Motor[1].SetPoint;
	 DebugOut.Analog[14] = Motor[2].SetPoint;
	 DebugOut.Analog[15] = Motor[3].SetPoint;
	 DebugOut.Analog[20] = ServoNickValue;
	 DebugOut.Analog[22] = Capacity.ActualCurrent;
	 DebugOut.Analog[23] = Capacity.UsedCapacity;
//	 DebugOut.Analog[22] = FromNaviCtrl_Value.GpsZ;
//	 DebugOut.Analog[29] = FromNaviCtrl_Value.SerialDataOkay;
	 DebugOut.Analog[7] = GasMischanteil;
	 DebugOut.Analog[19] = WinkelOut.CalcState;
	 DebugOut.Analog[29] = Capacity.MinOfMaxPWM;
	 DebugOut.Analog[30] = GPS_Nick;
	 DebugOut.Analog[31] = GPS_Roll;
#endif

namespace miko
{
	class FlightControl
	{
	private:

		ros::Timer timer_;

		double freq_;
		std::string port_;
		int speed_;

		#if 0
		bool enable_LL_STATUS_;
		int interval_LL_STATUS_;
		int offset_LL_STATUS_;
		bool enable_IMU_RAWDATA_;
		int interval_IMU_RAWDATA_;
		int offset_IMU_RAWDATA_;
		bool enable_IMU_CALCDATA_;
		int interval_IMU_CALCDATA_;
		int offset_IMU_CALCDATA_;
		bool enable_RC_DATA_;
		int interval_RC_DATA_;
		int offset_RC_DATA_;
		bool enable_CONTROLLER_OUTPUT_;
		int interval_CONTROLLER_OUTPUT_;
		int offset_CONTROLLER_OUTPUT_;
		bool enable_GPS_DATA_;
		int interval_GPS_DATA_;
		int offset_GPS_DATA_;
		bool enable_GPS_DATA_ADVANCED_;
		int interval_GPS_DATA_ADVANCED_;
		int offset_GPS_DATA_ADVANCED_;
		bool enable_CONTROL_;
		int interval_CONTROL_;
		int offset_CONTROL_;
		#endif

		bool Throttle_Direction;
		
		typedef struct
		{
			uint8_t Digital[2];
			uint8_t RemoteButtons;
			int8_t	Pitch;
			int8_t	Roll;
			int8_t	Yaw;
			uint8_t Throttle;
			int8_t	Height;
			uint8_t free;
			uint8_t Frame;
			uint8_t Config;
		} __attribute__((packed)) ExternControl_t;


		typedef struct
		{
			double pitch;
			double roll;	// in 0.1 deg
			double z;		// in 0.1 deg
			double yaw;	// in 0.1 deg
		}__attribute__((packed)) DesiredPosition_t;

		ros::Publisher pub;
		sensor_msgs::Imu data;
		uint64_t time;
	
		DesiredPosition_t DesiredPosition;
    
		public:
                SerialInterface* serialInterface_;

		ros::Time last_time;
		ros::Time current_time;
		control_toolbox::Pid pid_yaw;
		control_toolbox::Pid pid_pitch;
		control_toolbox::Pid pid_roll;
		ExternControl_t	ExternControl;
		FlightControl ();
		virtual ~FlightControl();
		void AddCRC(uint16_t datelen);
		void SendOutData(uint8_t cmd, uint8_t addr, uint8_t numofbuffers, ...);
		void enablePolling (uint16_t request, uint16_t interval);
		void spin (const ros::TimerEvent & e);
		unsigned long long time_helper(void);
		
	}; // end class FlightControl
} //end namespace miko

#endif
