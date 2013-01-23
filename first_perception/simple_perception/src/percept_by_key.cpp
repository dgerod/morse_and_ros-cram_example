// =============================================================================
// Simple_perception by key
// =============================================================================

#include <ros/ros.h>
#include <simple_perception/Percept.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <sstream>

// -----------------------------------------------------------------------------

int kfd = 0;
struct termios cooked, raw;

// -----------------------------------------------------------------------------
// TeleopPerception CLASS
// -----------------------------------------------------------------------------

#define KEYCODE_R 		0x43
#define KEYCODE_L 		0x44
#define KEYCODE_U 		0x41
#define KEYCODE_D 		0x42
#define KEYCODE_Q 		0x71
#define KEYCODE_RETURN	0x0A

class TeleopPerception
{
public:
  TeleopPerception();
  void keyLoop();

private:
  ros::NodeHandle nh_;

  int object_id_;
  ros::Publisher obj_pub_;
};

// -----------------------------------------------------------------------------

TeleopPerception::TeleopPerception()
 : object_id_(0)
{
  nh_.param("object_id", object_id_,object_id_);
  obj_pub_ = nh_.advertise<simple_perception::Percept>("simple_perception/percept", 1);
}

void
TeleopPerception::keyLoop()
{
  char c;
  bool dirty = false;

  // Get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);

  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to mimic object detection.");

  for(;;)
  {
    // Get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);
    printf("Key pressed: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_RETURN:
        ROS_DEBUG("RETURN");
        object_id_++;
        dirty = true;
        break;
    }

    simple_perception::Percept percept;
    std::stringstream ss;

    ss << object_id_;
    percept.object_id = ss.str();

    if(dirty == true)
    {
      obj_pub_.publish(percept);
      dirty = false;
    }
  }

  return;
}

// -----------------------------------------------------------------------------
// MAIN
// -----------------------------------------------------------------------------

void
quit ( int sig )
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int
main ( int argc, char** argv )
{
  ros::init(argc, argv, "simple_perception");
  TeleopPerception percept_teleop;

  signal(SIGINT,quit);
  percept_teleop.keyLoop();

  return(0);
}

// =============================================================================
