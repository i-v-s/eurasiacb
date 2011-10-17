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

#ifndef MIKO_FLIGHTCONTROL_SERIALINTERFACE_H
#define MIKO_FLIGHTCONTROL_SERIALINTERFACE_H // я не знаю зачем определять вот это значение. В смысле и определения ведь никакого нет.

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>
#include <math.h>

#include <ros/ros.h>

//#include "crc16.h"
//#include "telemetry.h"
#include "flightcontrol.h"

#define TXD_BUFFER_LEN	300 // это количество элементво массива g_txd_buffer. Этот массив отправляется функцией SendOutData в плату
#define RXD_BUFFER_LEN	300 // это кол-во элементов массива g_rxd_buffer. Этот массив заполняется функцией getdata класса Serialinterface. Там еще вопрос - нафига в функции тоже стоит свой параметр len и он там равен 250
#define MAX_PITCH_THRESHOLD 360 //видимо, пороговое значение величины больше которого не может быть. Но по факту это определение нигде в коде не используется. Со всеми остальными threshold'ами, судя по всему, та же ситуация
#define MAX_ROLL_THRESHOLD 360
#define MAX_YAW_THRESHOLD 360
#define MAX_HEIGHT_THRESHOLD 200
#define MAX_ACC_THRESHOLD 400

#define BOUNDARY 10 // это определение не используется в коде

namespace miko
{
        class SerialInterface //описание класса общения по последовательному порту
	{
	public:
                SerialInterface (std::string port, uint32_t speed); // прототип конструктора класса. На вход принимается название порта и скорость обмена
                ~SerialInterface (); // деструктор порта. Описывется в serialinterface.cpp - делается close(dev_); и flush(); закрывает порт т.е.

                void output (char *output, int len); // здесь и строчкой ниже прототип функции, которая записывает данные в последовательный порт - она описана в serialinterface.cpp.
                void output (unsigned char *output, int len); // эта отличается тем, что значение данных может быть отрицательным, видимо.
	 // bool getPackets (Telemetry *telemetry);
                void Decode64(void); // протоип функции, которая разкодирует данные, которые приходят с платы
                void ParsingData(void); // а эта функция парсит раскодированные данные
	 // void sendControl (Telemetry *telemetry);
                void dumpDebug (void); // прототип функции, которой не существует (я её не вижу). Вроде ошибка на такую фигню должна выскакивать.
                int getdata (unsigned char *buf, int len); // тоже прототип функции, которая извлекает данные из последовательного порта в массив g_rxd_buffer
 //	 bool getPacket (char *spacket, unsigned char &packet_type, unsigned short &packet_crc, unsigned short &packet_size);

                uint32_t serialport_bytes_rx_; // эта переменная нигде не используется. Он её зачем-то инициализирует в ноль. И всё.
                uint32_t serialport_bytes_tx_; // то же самое
                int *scan; // переменная не используется
                bool status; // в конструкторе класса есть строка status = false; но по-моему она больше нефига не значит, т.к. нигде не используется.
                int pt[800]; //нет такого массива
                int counter; //нет такой переменной


                bool Initialized; // Initialized=false; в конструкторе класса. И больше нигде, т.е. то же мёртвая переменная
                int count; // нет такой переменной
		
	private:
                                        speed_t bitrate (int); // прототип функции bitrate, описанной в serialport.cpp
                        void flush (); // прототип функции.  tcflush (dev_, TCIOFLUSH); видимо это что-то из termios.h для общения с сериал портом
                        void drain (); // описана в .cpp ROS_ASSERT_MSG (tcdrain (dev_) == 0, "Drain Error: %s", strerror (errno)); . Хер его знает
                        void stall (bool); // не описана
                        int wait (int); // описана в cpp. Зачем нужна - не понял

                        int dev_; // название порта
                        std::string serialport_name_; // тоже название порта
                        uint32_t serialport_speed_; // скорость порат
                        speed_t serialport_baud_; // тоже скорость порта
	};
}
#endif
