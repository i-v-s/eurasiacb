#include <ros/ros.h>
#include <copterControl/CControl.h>
#include <signal.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/select.h>

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
#define DEF_NICK 0
#define MAX_NICK 30
#define MIN_NICK -30
#define DEF_GAS 0
#define MAX_GAS 100
#define MIN_GAS 0
#define DEF_YAW 0
#define MAX_YAW 10
#define MIN_YAW -10

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
};

TeleopKey::TeleopKey()
{
    ctrl_pub_ = nh_.advertise<copterControl::CControl>("copterControl/teleopKey", 1);

    ctrl_msg_.nick = DEF_NICK;
    ctrl_msg_.gas = DEF_GAS;
    ctrl_msg_.yaw = DEF_YAW;
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


    fd_set rfds;
    int retval;
    struct timeval mtime;

    for(;;)
    {
        mtime.tv_usec = 100;
        mtime.tv_sec = 0;

        FD_ZERO(&rfds);
        FD_SET(0, &rfds);

        retval = select(1, &rfds, NULL, NULL, &mtime);
        if(retval==-1)
            perror("select");
        else if (retval) {

            read(kfd, &c, 1);
            ROS_DEBUG("value: 0x%02X\n", c);

            switch(c)
            {
                case KEYCODE_L:
                    ROS_DEBUG("LEFT");
                    ctrl_msg_.nick = MIN_NICK;
                    dirty = true;
                break;
                case KEYCODE_R:
                    ROS_DEBUG("RIGHT");
                    ctrl_msg_.nick = MAX_NICK;
                    dirty = true;
                break;
                case KEYCODE_U:
                    ROS_DEBUG("FORWARD");
                    ctrl_msg_.yaw = MAX_YAW;
                    dirty = true;
                break;
                case KEYCODE_DW:
                    ROS_DEBUG("BACK");
                    ctrl_msg_.yaw = MIN_YAW;
                    dirty = true;
                break;
                case KEYCODE_W:
                    ROS_DEBUG("UP");
                    ctrl_msg_.gas < MAX_GAS ? ctrl_msg_.gas++ : ctrl_msg_.gas= MAX_GAS;
                    dirty = true;
                break;
                case KEYCODE_S:
                    ROS_DEBUG("DOWN");
                    ctrl_msg_.gas > MIN_GAS ? ctrl_msg_.gas-- : ctrl_msg_.gas = MIN_GAS;
                    dirty = true;
                break;
                case KEYCODE_SP:
                    ROS_DEBUG("STOP");
                    ctrl_msg_.nick = DEF_NICK;
                    ctrl_msg_.yaw = DEF_YAW;
                    dirty = true;
                break;
                case KEYCODE_D:
                    ROS_DEBUG("FIX");
                    ctrl_msg_.fix = true;
                    dirty = true;
                break;

            }

            if(dirty) {
                ROS_INFO("nick=%d, gas=%d, yaw=%d, fix=%s", ctrl_msg_.nick, ctrl_msg_.gas, ctrl_msg_.yaw, ctrl_msg_.fix?"true":"false");
                ctrl_pub_.publish(ctrl_msg_);
                ctrl_msg_.fix = false;
                dirty=false;
            }

        }
        else {
            if(ctrl_msg_.nick!=DEF_NICK) ctrl_msg_.nick > DEF_NICK? ctrl_msg_.nick-- : ctrl_msg_.nick++;
            if(ctrl_msg_.yaw!=DEF_YAW) ctrl_msg_.yaw > DEF_YAW? ctrl_msg_.yaw-- : ctrl_msg_.yaw++;

            ROS_INFO("nick=%d, gas=%d, yaw=%d, fix=%s", ctrl_msg_.nick, ctrl_msg_.gas, ctrl_msg_.yaw, ctrl_msg_.fix?"true":"false");
            ctrl_pub_.publish(ctrl_msg_);
        }

    }

    return;
}


