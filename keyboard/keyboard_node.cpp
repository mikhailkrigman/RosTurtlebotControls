#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42

class Keyboard
{
public:
  Keyboard();
  void keyLoop();

private:  
  ros::NodeHandle nh_;
  ros::Publisher key_pub_;
};

Keyboard::Keyboard()
{
  key_pub_ = nh_.advertise<std_msgs::Char>("/arrow_keys", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_node");
  Keyboard keyboard;

  signal(SIGINT,quit);

  keyboard.keyLoop();
  
  return(0);
}

void Keyboard::keyLoop()
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
  puts("Arrow keys will be published on a ROS Topic");

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);
  
    std_msgs::Char key;
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        key.data = KEYCODE_L;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        key.data = KEYCODE_R;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        key.data = KEYCODE_U;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        key.data = KEYCODE_D;
        dirty = true;
        break;
    }
   
    if(dirty ==true)
    {
      key_pub_.publish(key); 
      dirty=false;
    }
  }

  return;
}


