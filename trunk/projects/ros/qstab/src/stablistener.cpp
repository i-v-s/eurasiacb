#include "ros/ros.h"
#include "qstab/Move.h" // Объявление msg-типа Move
#include <stdio.h>   /* Стандартные объявления ввода/вывода */
#include <string.h>  /* Объявления строковых функций */
#include <unistd.h>  /* Объявления стандартных функций UNIX */
#include <fcntl.h>   /* Объявления управления файлами */
#include <errno.h>   /* Объявления кодов ошибок */
#include <termios.h> /* Объявления управления POSIX-терминалом */

int open_port(void);      //Функция открытия порта

void chatterCallback(const qstab::Move msg)
{
  int fd=open_port(); // Открыли порт. fd - файловый дескриптор для порта
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
  close(fd);
  ROS_INFO("I heard: [%ld]", (long int) msg.horizontal);
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "stablistener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);




  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

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
