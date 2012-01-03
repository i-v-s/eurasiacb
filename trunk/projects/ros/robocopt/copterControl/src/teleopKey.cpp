#include <ros/ros.h>
#include <copterControl/CControl.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

// Key codes
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_DW 0x42
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_SP 0x20
#define KEYCODE_Q 0x71

// Params, DEF- defoult
#define DEF_X 0
#define MAX_X 10
#define MIN_X -10
#define DEF_Y 0
#define MAX_Y 100
#define MIN_Y -100
#define DEF_Z 0
#define MAX_Z 10
#define MIN_Z -10

#define DATAGET_SPEED 4

class TeleopKey
{
    public:
        TeleopKey();
        void keyLoop();

    private:

        ros::NodeHandle nh_;
        ros::Publisher ctrl_pub_;
        copterControl::CControl ctrl_msg_;

        int z_factor;
};

TeleopKey::TeleopKey()
{
    ctrl_pub_ = nh_.advertise<copterControl::CControl>("copterControl/teleopKey", 1);

    ctrl_msg_.x = DEF_X;
    ctrl_msg_.y = DEF_Y;
    ctrl_msg_.z = DEF_Z;
    ctrl_msg_.fix = false;
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
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        ROS_DEBUG("value: 0x%02X\n", c);

        ctrl_msg_.x = DEF_X;
        ctrl_msg_.z = DEF_Z;

        switch(c)
        {
            case KEYCODE_L:
                ROS_DEBUG("LEFT");
                ctrl_msg_.x = MIN_X;
                dirty = true;
            break;
            case KEYCODE_R:
                ROS_DEBUG("RIGHT");
                ctrl_msg_.x = MAX_X;
                dirty = true;
            break;
            case KEYCODE_U:
                ROS_DEBUG("FORWARD");
                ctrl_msg_.z = MAX_Z;
                dirty = true;
            break;
            case KEYCODE_DW:
                ROS_DEBUG("BACK");
                ctrl_msg_.z = MIN_Z;
                dirty = true;
            break;
            case KEYCODE_W:
                ROS_DEBUG("UP");
                ctrl_msg_.y < MAX_Y ? ctrl_msg_.y++ : ctrl_msg_.y= MAX_Y;
                dirty = true;
            break;
            case KEYCODE_S:
                ROS_DEBUG("DOWN");
                ctrl_msg_.y > MIN_Y ? ctrl_msg_.y-- : ctrl_msg_.y = MIN_Y;
                dirty = true;
            break;
            case KEYCODE_SP:
                ROS_DEBUG("STOP");
                ctrl_msg_.x = DEF_X;
                ctrl_msg_.z = DEF_Z;
                dirty = true;
            break;
            case KEYCODE_D:
                ROS_DEBUG("FIX");
                ctrl_msg_.fix = true;
                dirty = true;
            break;

        }

        if(dirty) {
            ROS_INFO("x=%d, y=%d, z=%d, fix=%s", ctrl_msg_.x, ctrl_msg_.y, ctrl_msg_.z, ctrl_msg_.fix?"true":"false");
            ctrl_pub_.publish(ctrl_msg_);
            ctrl_msg_.fix = false;
            dirty=false;
        }
    }

    return;
}


