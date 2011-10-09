#include "ros/ros.h"
#include "qstab/Move.h" // Объявление msg-типа Move
#include <stdio.h>   /* Стандартные объявления ввода/вывода */
#include <string.h>  /* Объявления строковых функций */
#include <unistd.h>  /* Объявления стандартных функций UNIX */
#include <fcntl.h>   /* Объявления управления файлами */
#include <errno.h>   /* Объявления кодов ошибок */
#include <termios.h> /* Объявления управления POSIX-терминалом */

int open_port(void);      //Функция открытия порта

int fd;


void chatterCallback(const qstab::Move msg)
{
  int n;
  n = write(fd, (char*) msg.horizontal, 6);
  if (n < 0)
  {
     fputs("write() of 4 bytes failed!\n", stderr);
  }
  else
 
  {
  printf(" n= %d  \n",n)  ;
  }
  ROS_INFO("I heard: [%ld]", (long int) msg.horizontal);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "stablistener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);


  fd=open_port(); // Открыли порт. fd - файловый дескриптор для порта

<<<<<<< .mine
  
  //Блок установки скорости передачи http://linuxland.itam.nsc.ru/misc/other19/index.html
  struct termios options;

=======
>>>>>>> .r31
  /*
   * Получение текущих опций для порта...
   */

  tcgetattr(fd, &options);

  /*
   * Установка скорости передачи в 19200...
   */
  
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  /*
   * Разрешение приемника и установка локального режима...
   */

  options.c_cflag |= (CLOCAL | CREAD);
  
  /*
   * Установка новых опций для порта...
   */

  tcsetattr(fd, TCSANOW, &options);
  // Конец блока установки скорости передачи


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  close(fd);
  return 0;
}


  int  open_port(void)
    {
      int fd;
   fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY |O_NONBLOCK);
  if (fd == -1)
  {
   /*
    * Could not open the port.
    */
 
   perror("open_port: Unable to open /dev/ttyS0 - "); 
  }
   else
      fcntl(fd, F_SETFL, 0);
   return (fd);
    }
