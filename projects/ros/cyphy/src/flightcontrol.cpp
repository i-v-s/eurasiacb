/*
 *	Mikrokopter Flight control Serial Interface
 *	Copyright (C) 2010, CYPHY lab
 *	Inkyu Sa <i.sa@qut.edu.au>
 *
 *	http://wiki.qut.edu.au/display/cyphy
 *
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

#include "flightcontrol.h"
#include "serial_interface.cpp"

ros::Timer timer;
ros::WallTimer walltimer;


unsigned char g_buf[300];
unsigned char g_txd_buffer[TXD_BUFFER_LEN];
unsigned char g_rxd_buffer[RXD_BUFFER_LEN];
//unsigned char txd_complete;
unsigned char g_ReceivedBytes;
unsigned char *g_pRxData;
unsigned char g_RxDataLen;

struct str_Data3D g_Data3D,g_Data3D_Temp;
DebugOut_t g_DebugData,g_DebugData_Temp;


int main(int argc, char **argv)
{
//	int interval=0;
//	int nLength;
	ros::init(argc, argv, "flightcontrol");
        miko::FlightControl flightcontrol;

	#if 0
		miko::SerialInterface::SerialInterface serialInterface;
		flightcontrol.SendOutData('d', FC_ADDRESS, 1,&interval,sizeof(interval)); // Request debug data from FC
		ros::WallDuration(0.1).sleep();			//50ms delay
		nLength=flightcontrol.serialInterface_->getdata(g_rxd_buffer, 255);
		printf("nLength=%d\n",nLength);

		flightcontrol.serialInterface_->Decode64();
		flightcontrol.serialInterface_->ParsingData();
		printf("%d %d %d\n",g_DebugData_Temp.Analog[ANGLE_PITCH],
		g_DebugData_Temp.Analog[ANGLE_ROLL],g_DebugData_Temp.Analog[ANGLE_YAW]);

		printf("Reset\n");
		flightcontrol.SendOutData('R', FC_ADDRESS,0); //reset command
		flightcontrol.SendOutData('b', FC_ADDRESS, 1,(uint8_t *) &flightcontrol.ExternControl, sizeof(flightcontrol.ExternControl));
		flightcontrol.SendOutData('c', FC_ADDRESS, 0); // Request IMU data from FC
		flightcontrol.SendOutData('c', FC_ADDRESS, 0); // Request IMU data from FC
	#endif
	
	flightcontrol.last_time = ros::Time::now();
	ros::spin();
	return 0;
}

namespace miko
{
	FlightControl::FlightControl()
	{
		ROS_INFO ("Creating FlightControl Interface");
    	
		ros::NodeHandle nh;
    	
		pub = nh.advertise<sensor_msgs::Imu>("data", 100);
    	
    	
		// **** parameter Setting
		freq_ = 50.0;
		port_ = "/dev/ttyUSB0";
		speed_ = 57600;
		if (freq_ <= 0.0) 
			ROS_FATAL ("Invalid frequency param");
		
		ros::Duration d (1.0 / freq_);

		// Reference input initialization
		ExternControl.Digital[0] =0;
		ExternControl.Digital[1] =0;
		ExternControl.RemoteButtons =0;
		ExternControl.Pitch =0;
		ExternControl.Roll =0;
		ExternControl.Yaw =0;
		ExternControl.Throttle =0;
		ExternControl.Height =0;
		ExternControl.free =0;
		ExternControl.Frame =0;
		ExternControl.Config =1;
		Throttle_Direction=true;

		g_DebugData.Digital[0]=0;
		g_DebugData.Digital[1]=0;
		for(int i=0;i<32;i++) g_DebugData.Analog[i]=0;

		g_ReceivedBytes=0;
		g_pRxData=0;
		g_RxDataLen=0;

		DesiredPosition.pitch=0;
		DesiredPosition.roll=0;
		DesiredPosition.z=0;
		DesiredPosition.yaw=0;
			
		// **** set up intefaces

                serialInterface_ = new miko::SerialInterface (port_, speed_);
		serialInterface_->serialport_bytes_rx_ = 0;
		serialInterface_->serialport_bytes_tx_ = 0;
		
		timer= nh.createTimer(ros::Duration(0.05), &FlightControl::spin,this);	//50ms timer
		
		ROS_INFO ("Serial setup finished");
		
		pid_yaw.initPid( 6.0 , 1.0 , 2.0 , 0.3 , -0.3 ); 
		pid_pitch.initPid( 6.0 , 1.0 , 2.0 , 0.3 , -0.3 ); 
		pid_roll.initPid( 6.0 , 1.0 , 2.0 , 0.3 , -0.3 ); 
		// proportional gain, integral gain, derivative gain, integral upper limit, integral lower limit
	}
	
	#if 1
		FlightControl::~FlightControl()
		{
			ROS_INFO ("Destroying FlightControl Interface");
		}
	#endif

	void FlightControl::spin(const ros::TimerEvent & e)
	{
		int nLength=0;
		//ROS_INFO ("spin()");
		uint8_t interval=5;
		current_time = ros::Time::now();
		//SendOutData('c', FC_ADDRESS, 1,&interval,sizeof(interval)); // Request IMU data from FC
		//ros::WallDuration(0.01).sleep(); // 10ms delay
		
		SendOutData('d', FC_ADDRESS, 1,&interval,sizeof(interval)); // Request debug data from FC
		data.header.stamp = ros::Time::now();
		ros::WallDuration(0.01).sleep(); // 10ms delay

		#if 0
			SendOutData('c', FC_ADDRESS, 1,&interval,sizeof(interval)); // Request IMU data from FC
			nLength=serialInterface_->getdata(g_rxd_buffer, 255);
			data.header.stamp = ros::Time::now();
			serialInterface_->Decode64();
			serialInterface_->ParsingData();
			
			ros::WallDuration(0.01).sleep();
		#endif

		nLength=serialInterface_->getdata(g_rxd_buffer, 255);
		if(nLength>0)
		{
		
			//printf("nLength=%d\n",nLength);
			serialInterface_->Decode64();

			#if 0
			printf("==============start frame==========\n");
			for(int i=0;i<nLength;i++)
			{
				 if(i%10==0) printf("\n");
				 else printf("%d",g_rxd_buffer[i]);
			}
			#endif
			
			serialInterface_->ParsingData();
			
			// create direction, the yaw angle is range from 0 to 180 cw, from 0 to -180 ccw.
			if(g_DebugData.Analog[ANGLE_YAW] >= 180) 
				g_DebugData.Analog[ANGLE_YAW] = g_DebugData.Analog[ANGLE_YAW]-360;
		
			printf("%d %d %d\n",g_DebugData.Analog[ACC_X],
					g_DebugData.Analog[ACC_Y],g_DebugData.Analog[ACC_Z]);
		
			#if 0
				printf("%d %d %d %d\n",g_DebugData.Analog[ANGLE_PITCH],
						 g_DebugData.Analog[ANGLE_ROLL],g_DebugData.Analog[ANGLE_YAW],g_DebugData.Analog[HEIGHT]);
			#endif
		
			#if 1
				double yaw_temp=pid_yaw.updatePid(g_DebugData.Analog[ANGLE_YAW] - DesiredPosition.yaw,current_time-last_time);
				double pitch_temp=pid_pitch.updatePid(g_DebugData.Analog[ANGLE_PITCH]- DesiredPosition.pitch,current_time-last_time);
				double roll_temp=pid_roll.updatePid(g_DebugData.Analog[ANGLE_ROLL]- DesiredPosition.roll,current_time-last_time);
                
				yaw_temp = yaw_temp/YAW_SCALE_FACTOR;
				pitch_temp = pitch_temp/PITCH_SCALE_FACTOR;
				roll_temp = roll_temp/ROLL_SCALE_FACTOR;
	
				ExternControl.Yaw= (int8_t)yaw_temp;
				ExternControl.Pitch= (int8_t)pitch_temp;
				ExternControl.Roll= (int8_t)roll_temp;
				ExternControl.Throttle= MAX_THROTTLE;
			#endif

			//printf("ExternControl.Pitch=%d ExternControl.Roll=%d ExternControl.Yaw=%d\n"
			//,ExternControl.Pitch,ExternControl.Roll,ExternControl.Yaw);
	 
			//printf("%d %d %d\n",g_Data3D_Temp.AnglePitch,g_Data3D_Temp.AngleRoll,g_Data3D_Temp.AngleYaw);
    
			//printf("%d %d %d %f\n",g_DebugData_Temp.Analog[ANGLE_YAW],
			//g_DebugData_Temp.Analog[ANGLE_ROLL],g_DebugData_Temp.Analog[ANGLE_YAW],temp);

			#if 0
				printf("FRONT=%d REAR=%d LEFT=%d RIGHT=%d,Height=%d\n",
								g_DebugData.Analog[MOTOR_FRONT],
								g_DebugData.Analog[MOTOR_REAR],
								g_DebugData.Analog[MOTOR_LEFT],
								g_DebugData.Analog[MOTOR_RIGHT],
								g_DebugData.Analog[HEIGHT]-65535);
			#endif
		
			last_time = current_time;
				SendOutData('b', FC_ADDRESS, 1,(uint8_t *) &ExternControl, sizeof(ExternControl));
			
			#if 0
				data.angular_velocity.x = g_Data3D_Temp.AnglePitch;
				data.angular_velocity.y = g_Data3D_Temp.AngleRoll;
				data.angular_velocity.z = g_Data3D_Temp.AngleYaw;
			#endif

			#if 1
				data.angular_velocity.x = g_DebugData.Analog[ANGLE_PITCH];
				data.angular_velocity.y = g_DebugData.Analog[ANGLE_ROLL];
				data.angular_velocity.z = g_DebugData.Analog[ANGLE_YAW];
			
			
				data.linear_acceleration.x = g_DebugData.Analog[ACC_X];
				data.linear_acceleration.y = g_DebugData.Analog[ACC_Y];
				data.linear_acceleration.z = g_DebugData.Analog[ACC_Z];
			#endif
			
			
			pub.publish(data);
			// ROS_INFO("%d",msg.data);
			// Send serial data test code. 
			#if 0
				if(Throttle_Direction==true) // increase
				{
					if(ExternControl.Throttle == MAX_THROTTLE)
					{
						Throttle_Direction=false;
					}
					else
						ExternControl.Throttle++;
				}
				else // decrease
				{
					if(ExternControl.Throttle == 0)
					{
						Throttle_Direction=false;
					}
					else ExternControl.Throttle--;
				}

				serialInterface_->serialport_bytes_rx_ = 0;
				serialInterface_->serialport_bytes_tx_ = 0;
			#endif
		}
	}

	void FlightControl::SendOutData(uint8_t cmd, uint8_t addr, uint8_t numofbuffers, ...) // uint8_t *pdata, uint8_t len, ...
	{ 
		va_list ap;
		uint16_t pt = 0;
		uint8_t a,b,c;
		uint8_t ptr = 0;

		uint8_t *pdata = 0;
		int len = 0;

		g_txd_buffer[pt++] = '#';								// Start character
		g_txd_buffer[pt++] = 'a' + addr;			// Address (a=0; b=1,...)
		g_txd_buffer[pt++] = cmd;								// Command

		va_start(ap, numofbuffers);
		if(numofbuffers)
		{
			pdata = va_arg(ap, uint8_t*);
			len = va_arg(ap, int);
			ptr = 0;
			numofbuffers--;
		}

		while(len)
		{
			if(len)
			{
				a = pdata[ptr++];
				len--;
				if((!len) && numofbuffers)
				{
					pdata = va_arg(ap, uint8_t*);
					len = va_arg(ap, int);
					ptr = 0;
					numofbuffers--;
				}
			}
			else a = 0;
			if(len)
			{
				b = pdata[ptr++];
				len--;
				if((!len) && numofbuffers)
				{
					pdata = va_arg(ap, uint8_t*);
					len = va_arg(ap, int);
					ptr = 0;
					numofbuffers--;
				}
			}
			else b = 0;
			if(len)
			{
				c = pdata[ptr++];
				len--;
				if((!len) && numofbuffers)
				{
					pdata = va_arg(ap, uint8_t*);
					len = va_arg(ap, int);
					ptr = 0;
					numofbuffers--;
				}
			}
			else c = 0;
			g_txd_buffer[pt++] = '=' + (a >> 2);
			g_txd_buffer[pt++] = '=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4));
			g_txd_buffer[pt++] = '=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6));
			g_txd_buffer[pt++] = '=' + ( c & 0x3f);
		} //while
		
		va_end(ap);
		AddCRC(pt); // add checksum after data block and initates the transmission
	}

	void FlightControl::AddCRC(uint16_t datalen)
	{
		uint16_t tmpCRC = 0, i;
		for(i = 0; i < datalen; i++)
		{
			tmpCRC += g_txd_buffer[i];
		}
		tmpCRC %= 4096;
		g_txd_buffer[datalen++] = '=' + tmpCRC / 64;
		g_txd_buffer[datalen++] = '=' + tmpCRC % 64;
		g_txd_buffer[datalen++] = '\r';
		//printf("datelen=%d\n",datalen);
		//for(i = 0; i < datalen; i++) printf("txd_buffer[%d]=%d\n",i,txd_buffer[i]);
		serialInterface_->output(g_txd_buffer,datalen);
	}
}
