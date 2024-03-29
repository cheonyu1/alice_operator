#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <pwd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <diagnostic_msgs/KeyValue.h>
#include <alice_msgs/FoundObjectArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/UInt8.h>

// for simulating
//#include <gazebo_msgs/ModelStates.h>
// topic name = /gazebo/model_states
// name[], pos[], 

#include "alice_operator/RoboCupGameControlData.h"

using namespace std;

#define PI 3.14159294

class ROI
{
public:
  double x, y, w, h;

  ROI():x(0),y(0),w(0),h(0){}
  ROI(double _x, double _y, double _w, double _h)
  {
    Set(_x, _y, _w, _h);
  }

  void Set(double _x, double _y, double _w, double _h)
  {
    x = _x;
    y = _y;
    w = _w;
    h = _h;
  }
};

class Vector3
{
public:
  double x, y, z;

  Vector3():x(0),y(0),z(0){};
  Vector3(double _x, double _y, double _z)
  {
    Set(_x, _y, _z);
  }

  void Set(double _x, double _y, double _z)
  {
    x = _x;
    y = _y;
    z = _z;
  }

  bool IsIn(double _x, double _y, double _w, double _h)
  {
    if( x > _x && x < _x + _w && 
        y > _y && y < _y + _h )
      return true;
    else
      return false;
  }
  bool IsIn(ROI area)
  {
    if( x >= area.x && x <= area.x + area.w && 
        y >= area.y && y <= area.y + area.h )
      return true;
    else
      return false;
  }
};

class Object
{
public:
  bool found;
  Vector3 local;
  Vector3 global;
  double dist;
  double angle;
  ros::Time last_saw;
};

void ROS_Thread();
void UDP_Thread();
void RobotInit();
void ReadConf(string file_path, string *result, string *opt, int len);
string EraseChar(string &str, char target);
void PrintRobotInfo(RobotInfo &player);
void PrintTeamInfo(TeamInfo &team, int num);
void PrintControlData(RoboCupGameControlData &control_data);
void VisionCallback(const alice_msgs::FoundObjectArray &msg);
void RobotPosCallback(const geometry_msgs::Pose2D &msg);
//void RobotPosCallback(const geometry_msgs::Vector3 &msg);

//void GazeboCallback(const gazebo_msgs::ModelStates &msg);
void PositionCallback(const geometry_msgs::Vector3 &msg);

double Deg2Rad(double deg);
double Rad2Deg(double rad);

struct RoboCupGameControlData control_data; // buffer
struct RoboCupGameControlReturnData robot_data;

mutex mtx;

bool goaled = false;
static int team_number[2] = {-1, -1};
static int team_score[2] = {-1, -1};


class Alice
{
public:

  enum State
  {
    Initial, 
    Ready, 
    Set, 
    Play, 
  };
  enum Strategy
  {
    Stop, 
    GoToSetPosition,
    GoalKeep, 
    Attack, 
    Defence, 
  };

  /// devil's code line
  bool gogogo;
  bool keeper;


  int id;
  int penalty;
  Vector3 map_size;
  Vector3 set_point;
  Vector3 shot_point;

  Vector3 pos_global;

  Vector3 dest_local;
  Vector3 dest_global;

  double dist_to_dest;
  double angle_to_dest;
  double angle_to_dest_global;
  double cross;

  State state;
  Strategy strategy;
  diagnostic_msgs::KeyValue move_cmd;
  diagnostic_msgs::KeyValue head_cmd;
  diagnostic_msgs::KeyValue  arm_cmd;
  int speed;

  int team_index;

  bool got_ball;
  ROI ball_zone;
  ROI kick_zone;

  int score;
  bool music_start;
  int music_num;

  Object ball;
  //  bool ball.found;
  //  Vector3 ball.local;
  //  Vector3 ball_global;
  //  double ball.dist;
  //  double ball.angle;

  Object goal;
  //  bool goal.found;
  //  Vector3 goal_local;
  //  Vector3 goal_global;
  //  double goal.dist;
  //  double goal.angle;

  Object center;

  uint8_t last_packet_num;
  ros::Time move_timer;

  Alice():state(Initial), penalty(0), last_packet_num(0), speed(3)
  {
    music_start = false;
    music_num = 0;
    score = -1;
    gogogo = false;
    keeper = false;
    ball_zone.Set(0.0, -0.5 , 0.55, 1.0 );
    kick_zone.Set(0.0, -0.18, 0.5, 0.36);
  };

  void Init()
  {
    move_timer = ros::Time::now();
    id = robot_data.player-1;
    map_size.x = 14;
    map_size.y = 9;
    team_index = -1;
  }

  void Update()
  {
    arm_cmd.key = "arm_motion";
    arm_cmd.value = "0";
    ReadControlData();
    UpdateObject();
    UpdatePoints();

    SetStrategy();
    SetDestination();

    GoalCheck();
    EndCheck();
    //Move();
  }

private:

  void UpdatePoints()
  {
    goal.global.x = 7.0f /14*map_size.x;
    goal.global.y = 0;
    set_point.y = 0;
    set_point.z = 0;
    shot_point.y = 0;
    shot_point.z = 0;

    if(keeper)
    {
      set_point.x = -5.0f /14*map_size.x;
      shot_point.x = -6.0f /14*map_size.x;
    }
    else
    {
      if(control_data.kickOffTeam == robot_data.team)
      {
        set_point.x = -0.5f /14*map_size.x;
        //set_point.x = -0.5f /14*map_size.x;
      }
      else
      {
        set_point.x = -2.0f /14*map_size.x;
      }
      shot_point.x = 6.0f /14*map_size.x;
    }

    if(team_index == 1)
    {
      set_point.x = -set_point.x;
      set_point.z = PI;
      shot_point.x = -shot_point.x;
      shot_point.z = PI;
      goal.global.x = -goal.global.x;
    }
  }

  void ReadControlData()
  {
    // if team index is available, update local state. 
    if(team_index >= 0)
    {
      if(control_data.teams[team_index].teamNumber == robot_data.team)
      {
        penalty = (int)control_data.teams[team_index].players[id].penalty;
        if(penalty == 0)//control_data.teams[team_index].players[id].penalty == PENALTY_NONE)
        {
          switch(control_data.state)
          {
          case STATE_INITIAL:
            state = Initial;
            break;
          case STATE_READY:
            state = Ready;
            break;
          case STATE_SET:
            state = Set;
            break;
          case STATE_PLAYING:
            state = Play;
            break;
          case STATE_FINISHED:
            state = Initial;
            break;
          default:
            state = Initial;
            break;
          }
        }
        else
        {
          state = Initial;
        }
      }
      else
      {
        state = Initial;
      }
    }
    else
    {
      // read control command & check packet number & get team index. 
      if(last_packet_num != control_data.packetNumber)
      {
        last_packet_num = control_data.packetNumber;
        CheckTeamIndex();
      }
      state = Initial;
    }
  }

  void SetStrategy()
  {
    switch(state)
    {
    case Ready:
      // go to the start point. 
      strategy = GoToSetPosition;
      break;
    case Set:
      strategy = Stop;
      break;
    case Play:
      if(keeper)
        strategy = GoalKeep;
      else
        strategy = Attack;
      break;
    default:
      strategy = Stop;
      break;
    }
  }

  void SetDestination()
  {
    dist_to_dest = sqrt(pow(dest_global.x-pos_global.x, 2) + (pow(dest_global.y-pos_global.y, 2)));
    angle_to_dest_global = atan2(dest_global.y-pos_global.y, dest_global.x-pos_global.x);
    angle_to_dest = acosf((cos(angle_to_dest_global)*cos(pos_global.z)) + (sin(angle_to_dest_global)*sin(pos_global.z)));
    cross = (cos(angle_to_dest_global)*sin(pos_global.z)) - (sin(angle_to_dest_global)*cos(pos_global.z));

    dest_local.x = cos(angle_to_dest) * dist_to_dest;
    dest_local.y = sin(angle_to_dest) * dist_to_dest;

//    cout
//      //      << endl << endl
//      << "state  : " << (int)control_data.state << endl//;
//    << "ball local  : " << ball.local.x << ", "  << ball.local.y  << endl//;
//    << "ball global : " << ball.global.x << ", " << ball.global.y << endl
//      << "ball angle  : " << ball.angle << endl << endl
//      << "goal local  : " << goal.local.x << ", "  << goal.local.y  << endl
//      << "goal global : " << goal.global.x << ", " << goal.global.y << endl
//      << "goal angle  : " << goal.angle << endl << endl
//      << "position    : " << pos_global.x << ", "  << pos_global.y  << endl
//      << "alice angle : " << Rad2Deg(pos_global.z) << endl << endl
//      << "destination : " << dest_global.x << ", " << dest_global.y << endl
//      << "dist_to_dest         : " << dist_to_dest << endl
//      << "global_angle_to_dest : " << Rad2Deg(angle_to_dest_global) << endl
//      //      << "cross                : " << cross << endl
//      << "angle_to_dest        : " << Rad2Deg(angle_to_dest) << endl << endl
//      << "got ball   : " << got_ball << endl << endl
//      << "gogogo   : " << (int)gogogo << endl << endl;

    //static ros::Time search_timer = ros::Time::now();
    switch(strategy)
    {
    case Stop:
      //dest_global = pos_global;
      move_cmd.key = "stop";
      head_cmd.key = "head_stop";
      move_cmd.value = "3";
      break;

    case GoToSetPosition:
      dest_global = set_point;
      MoveToCenter();

      if(keeper)
      {
        move_cmd.key = "stop";
        move_cmd.value = "3";
      }

//      if(gogogo)
//      {
//        move_cmd.key = "forward";
//        move_cmd.value = "3";
//      }

      break;

    case GoalKeep:
      GoalKeeping();
      break;

    case Attack:
      if( control_data.kickOffTeam == robot_data.team || 
          (int)control_data.secondaryTime == 0 )
      {
        if(ball.found)
        {
          if( ball.local.IsIn(ball_zone) )
          {
            head_cmd.key = "head_ball_check";
            MoveToGoal();
          }
          else
          {
            MoveToBall();
          }
        }
        else
        {
          Search();
        }
      }
      else
      {
        StopMove();
      }

      if(gogogo)
      {
        move_cmd.key = "forward";
        move_cmd.value = "3";
      }

      break;

    default:
      dest_global = pos_global;
      break;
    }
  }

  void UpdateObject()
  {
    //ball.found = true;
    if(ball.found)
    {
      ball.global.x = pos_global.x + (cos(pos_global.z)*ball.local.x) - (sin(pos_global.z)*ball.local.y);
      ball.global.y = pos_global.y + (sin(pos_global.z)*ball.local.x) + (cos(pos_global.z)*ball.local.y);
    }
    //    else if(got_ball || ball.last_saw + ros::Duration(3) < ros::Time::now())
    //    {
    //      ball.local.x = (cos(-pos_global.z)*ball.global.x) - (sin(-pos_global.z)*ball.global.y);
    //      ball.local.y = (sin(-pos_global.z)*ball.global.x) + (cos(-pos_global.z)*ball.global.y);
    //    }

    ball.dist = sqrt(pow(ball.local.x, 2) + pow(ball.local.y, 2));
    ball.angle = atan2(ball.local.y, ball.local.x);


    if(!goal.found)
    {
      goal.local.x = (cos(-pos_global.z)*goal.global.x) - (sin(-pos_global.z)*goal.global.y);
      goal.local.y = (sin(-pos_global.z)*goal.global.x) + (cos(-pos_global.z)*goal.global.y);
    }

    goal.dist = sqrt(pow(goal.local.x, 2) + pow(goal.local.y, 2));
    goal.angle = atan2(goal.local.y, goal.local.x);
  }

  void MoveToCenter()
  {
    char tmp[10];

    //cout << "ball local x,y,z   : " << ball.local.x << "," << ball.local.y << "," << ball.local.z << endl;
    //cout << "ball zone  x,y,w,h : " << ball_zone.x << "," << ball_zone.y << "," << ball_zone.w << "," << ball_zone.h << endl;
    // the destination is infront of me. 
    if( strategy == Stop)
    {
      StopHead();
      StopMove();
    }
    else
    {
      head_cmd.key = "head_tracking";

      double dist_to_center;
      if(control_data.kickOffTeam == robot_data.team)
        dist_to_center = 1.8;
      else
        dist_to_center = 2.5;
      
      if(ball.found && ball.dist <= dist_to_center)
      {
        StopMove();
      }
      else if(!ball.found)
      {
        head_cmd.key = "head_searching";
        static ros::Time forward_timer = ros::Time::now();

        StopMove();

        if(forward_timer + ros::Duration(4) < ros::Time::now())
        {
          forward_timer = ros::Time::now();
        }

        else if(forward_timer + ros::Duration(2) < ros::Time::now())
        {
          move_cmd.key = "forward";//_precision";
          sprintf(tmp, "3");
          move_cmd.value = tmp;
        }
      }
      else if(abs(Rad2Deg(ball.angle)) > 40)
      {
        if(ball.dist < 1.6)
        {
          if(ball.angle > 0)
            move_cmd.key = "turn_left";
          else
            move_cmd.key = "turn_right";
        }
        else
        {
          if(ball.angle > 0)
            move_cmd.key = "expended_left";
          else
            move_cmd.key = "expended_right";
        }
        sprintf(tmp, "%d", speed);
      }
      else if(abs(Rad2Deg(ball.angle)) > 8)
      {
        sprintf(tmp, "%.4f", abs(Rad2Deg(ball.angle)));
        if(ball.angle > 0)
          move_cmd.key = "turn_left_precision";
        else
          move_cmd.key = "turn_right_precision";
      }
      else
      {
        move_cmd.key = "forward_precision";
        sprintf(tmp, "%.4f", abs(ball.local.x*0.8));
      }
      move_cmd.value = tmp;
    }
  }

  void MoveToBall()
  {
    char tmp[10];

    //cout << "ball local x,y,z   : " << ball.local.x << "," << ball.local.y << "," << ball.local.z << endl;
    //cout << "ball zone  x,y,w,h : " << ball_zone.x << "," << ball_zone.y << "," << ball_zone.w << "," << ball_zone.h << endl;
    // the destination is infront of me. 
    if( strategy == Stop)
    {
      StopHead();
      StopMove();
    }
    else
    {
      head_cmd.key = "head_tracking";

      if(abs(Rad2Deg(ball.angle)) > 45)
      {
        if(ball.dist < 1.6)
        {
          if(ball.angle > 0)
            move_cmd.key = "turn_left";
          else
            move_cmd.key = "turn_right";
        }
        else
        {
          if(ball.angle > 0)
            move_cmd.key = "expended_left";
          else
            move_cmd.key = "expended_right";
        }
      }
      else if(abs(Rad2Deg(ball.angle)) > 25)
      {
        sprintf(tmp, "%.4f", abs(Rad2Deg(ball.angle)));
        if(ball.angle > 0)
          move_cmd.key = "turn_left_precision";
        else
          move_cmd.key = "turn_right_precision";
      }
      else
      {
        move_cmd.key = "forward_precision";
        sprintf(tmp, "%.4f", abs(ball.local.x*0.7));
      }
      move_cmd.value = tmp;
    }
  }

  void MoveToGoal()
  {
    //cout << "move to goal" <<endl;
    char tmp[10];

    if( strategy == Stop )//&& goal.last_saw + ros::Duration(5) < ros::Time::now() )
    {
      StopMove();
      StopHead();
    }
    else if(ball.local.IsIn(kick_zone))
    {
      double angle_to_goal_global = atan2(goal.global.y-pos_global.y, goal.global.x-pos_global.x);
      double angle_to_goal = acosf((cos(angle_to_goal_global)*cos(pos_global.z)) + (sin(angle_to_goal_global)*sin(pos_global.z)));
      double goal_cross = (cos(angle_to_goal_global)*sin(pos_global.z)) - (sin(angle_to_goal_global)*cos(pos_global.z));

      if(Rad2Deg(angle_to_goal) > 45)
      {
        if(goal_cross < 0)
          move_cmd.key = "centered_right";
        else
          move_cmd.key = "centered_left";

        sprintf(tmp, "%d", speed);
        move_cmd.value = tmp;
      }
      else if(Rad2Deg(angle_to_goal) > 25)
      {
        if(goal_cross < 0)
          move_cmd.key = "centered_right_precision";
        else
          move_cmd.key = "centered_left_precision";

        sprintf(tmp, "%.4f", Rad2Deg(angle_to_goal*0.7));
        move_cmd.value = tmp;
      }
      else
      {
        Kicking();
      }
    }
    else
    {
      if(ball.local.y > 0)
      {
        move_cmd.key = "left";
        sprintf(tmp, "%d", speed);
      }
      else
      {
        move_cmd.key = "right";
        sprintf(tmp, "%d", speed);
      }
      move_cmd.value = tmp;
    }
  }

  void Move()
  {
    //cout << "move" << endl;
    char tmp[10];

    // if destination is not nearby.
    if( strategy == Stop )
    {
      StopMove();
      StopHead();
    }
    else
    {
      //double goal_angle = dest_global.z - pos_global.z;
      //double goal_angle = atan2(angle_to_dest_global) - pos_global.z;

      // the destination is infront of me. 
      if( Rad2Deg(angle_to_dest) > 30)// && 
      { 
        if(dist_to_dest < 0.8)
        {
          if(cross > 0)// && 
            move_cmd.key = "turn_right_precision";
          else
            move_cmd.key = "turn_left_precision";

          sprintf(tmp, "%d", (int)(abs(Rad2Deg(angle_to_dest*0.7))));
        }
        else
        {
          if(cross > 0)// && 
            move_cmd.key = "expanded_right";
          else
            move_cmd.key = "expanded_left";

          sprintf(tmp, "%d", speed);
        }
        move_cmd.value = tmp;
      }
      else if( Rad2Deg(angle_to_dest) > 50)// && 
      {
        if(cross > 0)// && 
          move_cmd.key = "turn_right_precision";
        else
          move_cmd.key = "turn_left_precision";

        sprintf(tmp, "%d", (int)(abs(Rad2Deg(angle_to_dest * 0.9))));
        move_cmd.value = tmp;
      }
      else if( dist_to_dest >  2)
      {
        if(dest_local.x > 0)
        {
          move_cmd.key = "forward";
          sprintf(tmp, "%d", speed);
        }
        else// if(local_dest.x < 0)
        {
          move_cmd.key = "backward";
          sprintf(tmp, "%d", speed);
        }
        move_cmd.value = tmp;
      }
      else if( dist_to_dest > 1 )
      {
        if(dest_local.x > 0)
        {
          move_cmd.key = "forward_precision";
        }
        else
        {
          move_cmd.key = "backward_precision";
        }
        sprintf(tmp, "%.4f", abs(dest_local.x));
        move_cmd.value = tmp;
      }
      else
      {
        StopMove();
      }
    }
  }

  void Search()
  {
    if(!keeper)
    {
      head_cmd.key = "head_searching";
      //cout << "search" << endl;
      char tmp[10];
      static ros::Time turn_timer = ros::Time::now();

      if( strategy != Stop )
      {
        if(turn_timer + ros::Duration(4) < ros::Time::now())
          turn_timer = ros::Time::now();

        if(turn_timer + ros::Duration(1.8) < ros::Time::now())
        {
          //move_cmd.key = "forward";
          move_cmd.key = "turn_left";
          sprintf(tmp, "%d", speed);
        }
        else
        {
          move_cmd.key = "stop";
          sprintf(tmp, "%d", speed);
        }
        move_cmd.value = tmp;
      }
    }
  }

  void GoalKeeping()
  {
    head_cmd.key = "head_tracking";
    if(team_index == 0)
      pos_global.z = 0;
    //dest_global.z = 0;
    else
      pos_global.z = PI;
    //dest_global.z = PI;

    if( ball.local.IsIn(kick_zone) )
    {
      Kicking();
    }
    else if( ball.found && ball.dist < 2 )
    {
      MoveToBall();
    }
    else if( ball.local.y > 0 && ball.found && ball.dist < 3 )//&& pos_global.y > -1)
    {
      //move_cmd.key = "stop";
      move_cmd.key = "left";
      move_cmd.value = "3";
    }
    else if( ball.local.y < 0 && ball.found && ball.dist < 3 )// && pos_global.y < 1 )
    {
      //move_cmd.key = "stop";
      move_cmd.key = "right";
      move_cmd.value = "3";
    }
  }

  void Kicking()
  {
    char tmp[10];
    static ros::Time kick_timer = ros::Time::now()-ros::Duration(4);
    if(kick_timer + ros::Duration(5) < ros::Time::now())
    {
      kick_timer = ros::Time::now();
    }
    else if(kick_timer+ros::Duration(3) < ros::Time::now())
    {
      if(ball.local.y > 0 + 0.07)
        move_cmd.key = "left_kick";
      else
        move_cmd.key = "right_kick";

      sprintf(tmp, "%d", speed);
      move_cmd.value = tmp;
    }
    else
    {
      StopMove();
    }
  }

  void StopMove()
  {
    char tmp[10];
    sprintf(tmp, "%d", speed);
    move_cmd.key = "stop";
    move_cmd.value = tmp;
  }

  void StopHead()
  {
    head_cmd.key = "head_stop";
  }


  void CheckTeamIndex()
  {
    for(int i=0 ; i<2 ; i++)
    {
      if(control_data.teams[i].teamNumber == robot_data.team)
      {
        team_index = i;
      }
    }
  }


  void GoalCheck()
  {
    static ros::Time ceremony_timer;
    if(goaled)
    {
      for(int i=0 ; i<2 ; i++)
      {
        if(team_number[i] == robot_data.team)
        {
          if(score != team_score[i])
          {
            score = team_score[i];
            // music play command. 
            music_num = 0;
            //cout <<"scooooooooooooooooooooooooooooooooooooooreeeeeeeeeeeeeeeeeeeeeeeeeeee" << score << endl;
            break;
          }
        }
        else
        {
          if(team_score[i] >= 10)
            music_num = 3;
          else if(team_score[i] >= 3)
            music_num = 2;
          else
            music_num = 1;
        }
      }
      goaled = false;
      music_start = true;
      ceremony_timer = ros::Time::now();
    }

    if(ceremony_timer + ros::Duration(30) > ros::Time::now())
    {
      arm_cmd.value = "1";
      //move_cmd.key = "stop";
      //move_cmd.value = "3";
      //cout << "ceremony go on!" <<endl;
      // do ceremony!
    }
    else
    {
      arm_cmd.value = "0";
      // just for check. 
      //cout << "no ceremony now.." <<endl;
    }
  }

  void EndCheck()
  {
    static bool checker = false;

//    cout << "first harf  : " << (int)control_data.firstHalf << endl;
//    cout << "first state : " << (int)control_data.state << endl;
//    if(!checker)
//      cout << "check false" << endl;
//    else
//      cout << "check true" << endl;

    if( (int)control_data.firstHalf != 1 && 
        (int)control_data.state == 4 &&
        !checker)
    {
      music_num = 4;
      music_start = true;
      checker = true;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "alice_operator_node");

  thread udp_recv(UDP_Thread);
  thread ros_main(ROS_Thread);

  if(!ros::ok())
  {
    cout << "ROS is not ok. " << endl;
    exit(0);
  }

  ros_main.join();
  udp_recv.join();

  return 0;
}

///////////////////////////////------------------------------------------------------------------

Alice alice;

void ROS_Thread()
{
  ros::NodeHandle nh;

  RobotInit();
  if(robot_data.player <= 0)
  {
    cout << "Wrong robot id. " << endl;
    exit(1);
  }
  //  ros::Subscriber sub_gazebo = nh.subscribe("/gazebo/model_states", 10, GazeboCallback);
  //  ros::Subscriber sub_position = nh.subscribe("/heroehs/alice_reference_body_sum", 10, PositionCallback);

  ros::Subscriber sub_obj_pos = nh.subscribe("/heroehs/environment_detector", 10, VisionCallback);
  //  ros::Subscriber sub_robot_pos = nh.subscribe("/heroehs/alice/robot_state", 10, RobotPosCallback);
  ros::Subscriber sub_robot_pos = nh.subscribe("/heroehs/alice/global_position", 10, RobotPosCallback);  // geometry_msgs::Pose2D

  ros::Publisher pub_move_cmd    = nh.advertise<diagnostic_msgs::KeyValue>("/heroehs/alice/move_command", 10);
  ros::Publisher pub_head_cmd    = nh.advertise<diagnostic_msgs::KeyValue>("/heroehs/alice/head_command", 10);
  ros::Publisher pub_arm_cmd     = nh.advertise<diagnostic_msgs::KeyValue>("/heroehs/alice/arm_command", 10);
  ros::Publisher pub_game_state  = nh.advertise<std_msgs::UInt8>("/game_controller/state", 10);
  ros::Publisher pub_team_index  = nh.advertise<std_msgs::UInt8>("/game_controller/team_index", 10);
  ros::Publisher pub_penalty     = nh.advertise<std_msgs::UInt8>("/heroehs/alice/penalty", 10);
  ros::Publisher pub_destination = nh.advertise<geometry_msgs::Vector3>("/heroehs/alice/global_destination", 10);

  ros::Publisher pub_music_cmd   = nh.advertise<std_msgs::UInt8>("/heroehs/alice/ceremony", 10);

  alice.Init();

  while(ros::ok())
  {
    mtx.lock();
    alice.Update();
    mtx.unlock();

    std_msgs::UInt8 game_state_msg;
    game_state_msg.data = (uint8_t)control_data.state;
    pub_game_state.publish(game_state_msg);

    std_msgs::UInt8 penalty_msg;                                                                     
    penalty_msg.data = (uint8_t)alice.penalty;                                                  
    pub_penalty.publish(penalty_msg);

    if(alice.team_index != -1)
    {
      std_msgs::UInt8 team_index_msg;
      team_index_msg.data = alice.team_index;
      pub_team_index.publish(team_index_msg);
    }

    if(control_data.state == STATE_READY)
    {
      geometry_msgs::Vector3 dest_global_msg;
      dest_global_msg.x = alice.dest_global.x;
      dest_global_msg.y = alice.dest_global.y;
      dest_global_msg.z = alice.dest_global.z;
      pub_destination.publish(dest_global_msg);
    }

    pub_head_cmd.publish(alice.head_cmd);
    pub_move_cmd.publish(alice.move_cmd);

    static string last_arm;
    if(last_arm.compare(alice.arm_cmd.value) != 0)
    {
      pub_arm_cmd.publish(alice.arm_cmd);
      last_arm = alice.arm_cmd.value;
    }

    if(alice.music_start)
    {
      std_msgs::UInt8 music_msg;
      music_msg.data = alice.music_num;
      alice.music_start = false;
      pub_music_cmd.publish(music_msg);
    }

    ros::spinOnce();
    usleep(10000);
  }
}



void VisionCallback(const alice_msgs::FoundObjectArray &msg)
{
  if(alice.ball.last_saw + ros::Duration(1.5) < ros::Time::now())
    alice.ball.found = false;
  if(alice.center.last_saw + ros::Duration(1.5) < ros::Time::now())
    alice.center.found = false;
  alice.goal.found = false;
  for(int i=0 ; i<msg.length ; i++)
  {
    if(msg.data[i].name.compare("ball") == 0)
    {
      alice.ball.found = true;
      alice.ball.last_saw = ros::Time::now();
      alice.ball.local.x = msg.data[i].pos.x;
      alice.ball.local.y = msg.data[i].pos.y;
      alice.ball.local.z = msg.data[i].pos.z;
    }
    else if(msg.data[i].name.compare("goal1") == 0 && 
        alice.team_index == 1)
    {
      alice.goal.found = true;
      alice.goal.last_saw = ros::Time::now();
      alice.goal.local.x = msg.data[i].pos.x;
      alice.goal.local.y = msg.data[i].pos.y;
      alice.goal.local.z = msg.data[i].pos.z;
    }
    else if(msg.data[i].name.compare("goal2") == 0 && 
        alice.team_index == 0)
    {
      alice.goal.found = true;
      alice.goal.last_saw = ros::Time::now();
      alice.goal.local.x = msg.data[i].pos.x;
      alice.goal.local.y = msg.data[i].pos.y;
      alice.goal.local.z = msg.data[i].pos.z;
    }
    else if(msg.data[i].name.compare("center") == 0)
    {
      alice.center.found = true;
      alice.center.last_saw = ros::Time::now();
      alice.center.local.x = msg.data[i].pos.x;
      alice.center.local.y = msg.data[i].pos.y;
      alice.center.local.z = msg.data[i].pos.z;
    }
  }
}

void RobotPosCallback(const geometry_msgs::Pose2D &msg)
  //void RobotPosCallback(const geometry_msgs::Vector3 &msg)
{
  alice.pos_global.x = msg.x;
  alice.pos_global.y = msg.y;
  //alice.pos_global.z = msg.z;
  alice.pos_global.z = msg.theta;  // 0 ~ PI*2

  while(alice.pos_global.z < 0 || alice.pos_global.z > (PI*2))
  {
    if(alice.pos_global.z < 0)
      alice.pos_global.z += (PI*2);
    else if(alice.pos_global.z > PI*2)
      alice.pos_global.z -= (PI*2);
  }

  // 0 ~ PI*2  ->  -PI ~ PI
  if(alice.pos_global.z > PI)
    alice.pos_global.z -= (PI*2);
}

void RobotInit()
{
  passwd *user = getpwuid(getuid());
  string file_path = string(user->pw_dir) + "/config/robot.cfg";//"/catkin_ws/src/alice_operator/config/robot.cfg";

  string option[] = { 
    "player_number", 
    "team_number", 
    "gogogo", 
    "keeper"};
  string value[4];

  ReadConf(file_path, value, option, 4);

  robot_data.player = atoi(value[0].c_str());
  robot_data.team   = atoi(value[1].c_str());
  // GAMECONTROLLER_RETURN_MSG_MAN_PENALISE   0
  // GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE 1
  // GAMECONTROLLER_RETURN_MSG_MAN_ALIVE      2
  robot_data.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
  if(atoi(value[2].c_str()) == 1)
    alice.gogogo = true;
  if(atoi(value[3].c_str()) == 1)
    alice.keeper = true;
  else if(atoi(value[3].c_str()) == 0)
    alice.keeper = false;

  //  cout << "   team number : "<< (int)robot_data.team << endl;
  //  cout << " player number : "<< (int)robot_data.player << endl;
  //  cout << "       message : "<< (int)robot_data.message << endl;
}

void ReadConf(string file_path, string *result, string *opt, int len)
{
  ifstream file;
  file.open(file_path);

  while(!file.eof())
  {
    string input_str;
    getline(file, input_str);
    input_str = EraseChar(input_str, ' ');
    //cout << input_str << endl;

    for(int i=0 ; i<len ; i++)
    {
      if(input_str.find(opt[i]) != string::npos)
      {
        int str_p = input_str.find('|');

        if(str_p < input_str.length())
        {
          result[i].assign(input_str, str_p+1, input_str.length() - (str_p+1));
        }

        break;
      }
    }
  }

  file.close();
}

string EraseChar(string &str, char target)
{
  string temp;
  for(int i=0 ; i<str.length() ; i++)
    if(str[i] != target)
      temp += str[i];
  return temp;
}






void UDP_Thread()
{
  int broadcast = 1;
  // make socket variable
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  struct timeval timeout_val = {1, 0};
  if(setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1)
    exit(1);
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout_val, sizeof(timeout_val));

  // make variable for communicate
  sockaddr_in controller_addr;

  //bzero(&peer_addr, sizeof(peer_addr)); // set it 0
  controller_addr.sin_family = AF_INET;
  controller_addr.sin_port = htons(GAMECONTROLLER_DATA_PORT);
  controller_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // set broadcast ip(0.0.0.0)
  bind(sock, (sockaddr*)&controller_addr, sizeof(controller_addr));

  // make variable for communicate
  sockaddr_in client_addr; // sender's address
  socklen_t client_addr_len;

  //client_addr.sin_family = AF_INET;
  //client_addr.sin_port = htons(GAMECONTROLLER_RETURN_PORT);
  //client_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // set broadcast ip(0.0.0.0)

  int recv_len;

  while(ros::ok())
  {
    client_addr_len = sizeof(client_addr);
    recv_len = recvfrom(sock, (char*)&control_data, sizeof(control_data), 0, (sockaddr*)&client_addr, &client_addr_len);

    if(recv_len > 0)
    {
      mtx.lock();
      if(team_number[0] == -1)
      {
        team_number[0] = control_data.teams[0].teamNumber;
        team_number[1] = control_data.teams[1].teamNumber;
      }

      for(int i=0 ; i<2 ; i++)
      {
        if(control_data.teams[i].teamNumber == team_number[0])
        {
          if((int)team_score[0] != (int)control_data.teams[i].score && 
              team_score[0] != -1)
          {
            goaled = true;
          }
          team_score[0] = control_data.teams[i].score;
        }
        else if (control_data.teams[i].teamNumber == team_number[1])
        {
          if((int)team_score[1] != (int)control_data.teams[i].score && 
              team_score[1] != -1)
          {
            goaled = true;
          }
          team_score[1] = control_data.teams[i].score;
        }
      }

      cout << "team number : " << team_number[0] << endl;
      cout << "     score  : " << team_score[0] << endl;
      cout << "team number : " << team_number[1] << endl;
      cout << "     score  : " << team_score[1] << endl;
      //PrintControlData(control_data);
      mtx.unlock();
    }

    client_addr.sin_port = htons(GAMECONTROLLER_RETURN_PORT);
    client_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);  // set broadcast ip(0.0.0.0)
    //client_addr.sin_addr.s_addr = inet_addr("0.0.0.0");//INADDR_BROADCAST);  // set broadcast ip(0.0.0.0)
    sendto(sock, (char*)&robot_data, sizeof(robot_data), 0, (sockaddr*)&client_addr, sizeof(client_addr));
  }
  close(sock);
}

void PrintControlData(RoboCupGameControlData &control_data)
{
  cout << endl;
  cout << "------[ GameControlData ]--------------------------------" << endl;
  cout << endl;
  // header of the data structure
  //  cout << "               header : " << (string)control_data.header << endl;
  // version of the data structure
  //  cout << "              version : " << (int)control_data.version << endl;
  //  number incremented with each packet sent (with wraparound)
  cout << "        packet number : " << (int)control_data.packetNumber << endl;
  // the number of players on a team
  //  cout << "      player per team : " << (int)control_data.playersPerTeam << endl;
  // game type (GAME_ROUNDROBIN, GAME_PLAYOFF, GAME_DROPIN)
  cout << "            game type : " << (int)control_data.gameType << endl;
  // state of the game (STATE_READY, STATE_PLAYING, etc)
  cout << "                state : " << (int)control_data.state << endl;
  // 1 = game in first half, 0 otherwise
  cout << "           first harf : " << (int)control_data.firstHalf << endl;
  // the team number of the next team to kick off or DROPBALL
  cout << "        kick off Team : " << (int)control_data.kickOffTeam << endl;
  // extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
  cout << "      secondary state : " << (int)control_data.secondaryState << endl;
  // Extra info on the secondary state
  cout << " secondary state info : " << (string)control_data.secondaryStateInfo << endl;
  // number of team that caused last drop in
  cout << "         drop in team : " << (int)control_data.dropInTeam << endl;
  // number of seconds passed since the last drop in. -1  (0xffff) before first dropin
  cout << "         drop in time : " << (int)control_data.dropInTime << endl;
  // estimate of number of seconds remaining in the half
  cout << "       secs remaining : " << (int)control_data.secsRemaining << endl;
  // number of seconds shown as secondary time (remaining  ready, until free ball, etc)
  cout << "       secondary time : " << (int)control_data.secondaryTime << endl;
  cout << endl;
  cout << "---------------------------------------------------------" << endl;
  cout << endl;


  for(int i=0 ; i<2 ; i++)
  {
    if(control_data.teams[i].teamNumber == robot_data.team)
    {
      cout << "------[ Team Number " << (int)control_data.teams[i].teamNumber << " ]----------------------------------" << endl;
      PrintTeamInfo(control_data.teams[i], (int)control_data.playersPerTeam);
    }
  }
  //cout << endl;
}

void PrintTeamInfo(TeamInfo &team, int num)
{
  //cout << "    team colour : " << (int)team.teamColour << endl;    // colour of the team
  cout << "          score : " << (int)team.score << endl;         // team's score
  cout << "   penalty shot : " << (int)team.penaltyShot << endl;   // penalty shot counter
  cout << "   single shots : " << (int)team.singleShots << endl;   // bits represent penalty shot success
  //cout << " coach sequence : " << (int)team.coachSequence << endl; // sequence number of the coach's message
  //  cout << "  coach message : ";
  //  for(int i=0 ; i<SPL_COACH_MESSAGE_SIZE ; i++)
  //    cout << (int)team.coachMessage[i] << ", ";  // the coach's message to the team
  //  cout << endl;
  cout << endl;
  cout << "---------------------------------------------------------" << endl;
  cout << endl;

  if(robot_data.player >= 1 && robot_data.player <= 4)
  {
    cout << "------[ Player " << (int)robot_data.player << " ]---------------------------------------" << endl;
    PrintRobotInfo(team.players[robot_data.player-1]);
  }
  //cout << "------[ Coach ]------------------------------------------" << endl;
  //PrintRobotInfo(team.coach);

  //  for(int i=0 ; num ; i++)
  //  {
  //    cout << "------[ Player " << i+1 << " ]---------------------------------------" << endl;
  //    PrintRobotInfo(team.players[i]);
  //  }
  cout << "---------------------------------------------------------" << endl;
  cout << endl;
}

void PrintRobotInfo(RobotInfo &player)
{
  // penalty state of the player
  cout << "             penalty : " << (int)player.penalty << endl;
  // estimate of time till unpenalised
  cout << " secsTillUnpenalised : " << (int)player.secsTillUnpenalised << endl;
  // number of yellow cards
  cout << "   yellow card count : " << (int)player.yellowCardCount << endl;
  // number of red cards
  cout << "      red card count : " << (int)player.redCardCount << endl;
  cout << endl;
}


//void GazeboCallback(const gazebo_msgs::ModelStates &msg)
//{
//  for(int i=0 ; i<msg.name.size() ; i++)
//  {
//    if(msg.name[i].compare("soccer_ball_0_0") == 0)
//    {
//      alice.ball_global.x = msg.pose[i].position.x;
//      alice.ball_global.y = msg.pose[i].position.y;
//    }
//    if(msg.name[i].compare("alice_1_robot") == 0 || msg.name[i].compare("alice_2_robot") == 0)
//    {
//      alice.position.x = msg.pose[i].position.x;
//      alice.position.y = msg.pose[i].position.y;
//      double q_x = msg.pose[i].orientation.x;
//      double q_y = msg.pose[i].orientation.y;
//      double q_z = msg.pose[i].orientation.z;
//      double q_w = msg.pose[i].orientation.w;
//      //alice.position.z = atan2(2*(q_x*q_w + q_y*q_z), 1-2*(q_z*q_z + q_w*q_w)) + PI;
//      //cout << "alice pos z : " << alice.position.z/PI*180 <<endl;
//    }
//  }
//}
//
//void PositionCallback(const geometry_msgs::Vector3 &msg)
//{
//  alice.position.z = msg.z;
//}

double Deg2Rad(double deg)
{
  return deg/180*PI;
}

double Rad2Deg(double rad)
{
  return rad/PI*180;
}











