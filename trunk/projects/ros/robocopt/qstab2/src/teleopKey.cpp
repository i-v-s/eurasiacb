#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_S 0x20
#define KEYCODE_Q 0x71

class TeleopKey
{
    public:
        TeleopKey();
        void keyLoop();

    private:

        ros::NodeHandle nh_;
        //ros::Publisher vel_pub_;

};

TeleopKey::TeleopKey()
{
    vel_pub_ = nh_.advertise<std_msgs::Char>("qstab2/teleopKey", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleopKey");
    TeleopKey teleop_turtle;

    signal(SIGINT,quit);

    teleop_turtle.keyLoop();

    return(0);
}


void TeleopKey::keyLoop()
{
    char c;
    bool dirty=false;


    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the copter.");


    for(;;)
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        ROS_DEBUG("value: 0x%02X\n", c);

        switch(c)
        {
            case KEYCODE_L:
                ROS_DEBUG("LEFT");
                dirty = true;
            break;
            case KEYCODE_R:
                ROS_DEBUG("RIGHT");
                dirty = true;
            break;
            case KEYCODE_U:
                ROS_DEBUG("UP");
                dirty = true;
            break;
            case KEYCODE_D:
                ROS_DEBUG("DOWN");
                dirty = true;
            break;
            case KEYCODE_S:
                ROS_DEBUG("SPACE");
                dirty = true;
            break;
        }


//        turtlesim::Velocity vel;
//        vel.angular = a_scale_*angular_;
//        vel.linear = l_scale_*linear_;
        if(dirty ==true)
        {
            //vel_pub_.publish(vel);
            dirty=false;
        }
    }

    ROS_INFO("This is the end");


    return;
}


