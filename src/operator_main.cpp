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
#include <gazebo_msgs/ModelStates.h>
// topic name = /gazebo/model_states
// name[], pos[], 

#include "alice_operator/RoboCupGameControlData.h"

#define PI 3.14159294

struct Vector3
{
  float x=0;
  float y=0;
  float z=0;
};

using namespace std;

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

void GazeboCallback(const gazebo_msgs::ModelStates &msg);
void PositionCallback(const geometry_msgs::Vector3 &msg);

struct RoboCupGameControlData control_data; // buffer
struct RoboCupGameControlReturnData robot_data;

mutex mtx;

Vector3 ball;

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

  int id;
  Vector3 map_size;
  Vector3 set_point;
  Vector3 shot_point;
  Vector3 position;
  Vector3 destination;

  State state;
  Strategy strategy;
  diagnostic_msgs::KeyValue move_cmd;
  int speed;

  int team_index;
  int penalty;

  bool ball_found;
  float dist_to_ball;
  float angle_to_ball;
  Vector3 ball_global;
  uint8_t last_packet_num;
  ros::Time move_timer;

  Alice():state(Initial), last_packet_num(0), penalty(0), speed(2)
  {
  };

  void Init()
  {
    move_timer = ros::Time::now();
    id = robot_data.player-1;
    map_size.x = 9;//14;
    map_size.y = 6;//9;
    team_index = -1;
  }

  void Update()
  {
    ReadControlData();
    SetStrategy();
    SetDestination();

    Move();
  }

private:

  void UpdatePoints()
  {
    set_point.y = 0;
    set_point.z = 0;
    shot_point.y = 0;
    shot_point.z = 0;

    if(robot_data.player == 1)
    {
      if(control_data.kickOffTeam == robot_data.team)
      {
        set_point.x = -1.0f/14*map_size.x;
      }
      else
      {
        set_point.x = -2.0f/14*map_size.x;
      }
      shot_point.x = 6.0f/14*map_size.x;
    }
    else if(robot_data.player == 2)
    {
      set_point.x = -6.5f*14/map_size.x;
      shot_point.x = -6.0f*14/map_size.x;
    }

    if(team_index == 1)
    {
      set_point.x = -set_point.x;
      set_point.z = PI;
      shot_point.x = -shot_point.x;
      shot_point.z = PI;
    }
  }

  void ReadControlData()
  {
    // if team index is available, update local state. 
    if(team_index >= 0)
    {
      penalty = (int)control_data.teams[team_index].players[id].penalty;
      if(control_data.teams[team_index].players[id].penalty == PENALTY_NONE)
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
      if(robot_data.player == 2)
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
    switch(strategy)
    {
    case Stop:
      destination = position;
      break;

    case GoToSetPosition:
      UpdatePoints();
      destination = set_point;
      break;

    case GoalKeep:
      destination = position;
      if(team_index == 0)
        destination.z = 0;
      else
        destination.z = PI;
      break;

    case Attack:
      UpdateBall();
      cout
        << "local ball pos : " << ball.x << ", " << ball.y << endl;
      if( abs(ball.y) < 0.2 && 
          ball.x < 0.6 && 
          ball.x > 0 && 
          abs((destination.z-position.z)/PI*180) < 10 )// && 
        //((destination.z-PI) - (position.z-PI))/PI*180 < 10 && 
        //dist_to_ball < 0.8 )
      {
        cout << "?????????????????????????????????????????????????????" << endl;
        destination = shot_point;
      }
      else
      {
        destination = ball_global;
        speed = 1;
      }
      break;

    default:
      destination = position;
      break;
    }
  }

  void UpdateBall()
  {
    //ball_found = true;
    if(ball_found)
    {
      ball_global.x = position.x + (cos(position.z)*ball.x) - (sin(position.z)*ball.y);
      ball_global.y = position.y + (sin(position.z)*ball.x) + (cos(position.z)*ball.y);
    }

    dist_to_ball = sqrt(pow(ball_global.x - position.x, 2) + (pow(ball_global.y - position.y, 2)));
    angle_to_ball = atan2(ball_global.y - position.y, ball_global.x - position.x);

//    float global_angle_to_dest = atan2(destination.y-position.y, destination.x-position.x);
//    float cross = (cos(global_angle_to_dest)*sin(position.z)) - (sin(global_angle_to_dest)*cos(position.z));
//    float global_angle_to_ball = acosf((cos(global_angle_to_dest)*cos(position.z)) + (sin(global_angle_to_dest)*sin(position.z)));
//    if(cross < 0)
//    {
//      ball.x = cos(global_angle_to_ball) * dist_to_ball;
//      ball.y = sin(global_angle_to_ball) * dist_to_ball;
//    }
//    else
//    {
//      ball.x = cos(-global_angle_to_ball) * dist_to_ball;
//      ball.y = sin(-global_angle_to_ball) * dist_to_ball;
//    }
  }

  void Move()
  {
    float dist_to_dest = sqrt(pow(destination.x-position.x, 2) + (pow(destination.y-position.y, 2)));
    float global_angle_to_dest = atan2(destination.y-position.y, destination.x-position.x);
    float cross = (cos(global_angle_to_dest)*sin(position.z)) - (sin(global_angle_to_dest)*cos(position.z));
    float angle_to_dest = acosf((cos(global_angle_to_dest)*cos(position.z)) + (sin(global_angle_to_dest)*sin(position.z)));
    char tmp[10];
    cout
      << "position    : " << position.x << ", " << position.y << endl
      << "alice angle : " << position.z/PI*180 << endl
      << "destination : " << destination.x << ", " << destination.y << endl
      << "dist_to_dest         : " << dist_to_dest << endl
      << "global_angle_to_dest : " << global_angle_to_dest/PI*180 << endl
      << "cross                : " << cross << endl
      << "angle_to_dest        : " << angle_to_dest/PI*180 << endl << endl;

    Vector3 local_dest;
    if(cross < 0)
    {
      local_dest.x = cos(angle_to_dest) * dist_to_dest;
      local_dest.y = sin(angle_to_dest) * dist_to_dest;
    }
    else
    {
      local_dest.x = cos(-angle_to_dest) * dist_to_dest;
      local_dest.y = sin(-angle_to_dest) * dist_to_dest;
    }

    // if destination is not nearby.
    if( strategy != Stop )
    {
      // the destination is infront of me. 
      if( abs(local_dest.y) < 0.3 &&
          abs(local_dest.x) < 0.3 )
      {
        float goal_angle = (destination.z-PI) - (position.z-PI);

        sprintf(tmp, "%d", (int)(abs(goal_angle)/PI*180));

        if(goal_angle/PI*180 > 10)
          move_cmd.key = "centered_right_precision";
        else if(goal_angle/PI*180 < -10)
          move_cmd.key = "centered_left_precision";
        else
        {
          move_cmd.key = "stop";
          sprintf(tmp, "3");
        }
      }
      else if(abs(local_dest.y) < 0.25)
      {
        if(local_dest.x > 0)
        {
          move_cmd.key = "forward_precision";
          sprintf(tmp, "%.4f", abs(local_dest.x));
        }
        else// if(local_dest.x < 0.2)
        {
          move_cmd.key = "backward_precision";
          sprintf(tmp, "%.4f", abs(local_dest.x));
        }
      }
      else
      {
        if(abs(angle_to_dest/PI*180) > 15 || abs(angle_to_dest/PI*180) < 180-15)
        {
          if(cross > 0)
            move_cmd.key = "turn_right_precision";
          else
            move_cmd.key = "turn_left_precision";

          sprintf(tmp, "%d", (int)(angle_to_dest/PI*180));
        }
        else if(local_dest.x > 0)
        {
          move_cmd.key = "forward";
          sprintf(tmp, "%d", speed);
        }
        else// if(local_dest.x < 0)
        {
          move_cmd.key = "backward";
          sprintf(tmp, "%d", speed);
        }
      }
    }
    else
    {
      move_cmd.key = "stop";
      sprintf(tmp, "3");
    }

    move_cmd.value = tmp;
  }

  void Dribble()
  {
//    if( strategy != Stop )
//    {
//      // the destination is infront of me. 
//      if( abs(local_dest.y) < 0.3 &&
//          abs(local_dest.x) < 0.3 )
//      {
//        float goal_angle = (destination.z-PI) - (position.z-PI);
//
//        sprintf(tmp, "%d", (int)(abs(goal_angle)/PI*180));
//
//        if(goal_angle/PI*180 > 10)
//          move_cmd.key = "centered_right_precision";
//        else if(goal_angle/PI*180 < -10)
//          move_cmd.key = "centered_left_precision";
//        else
//        {
//          move_cmd.key = "stop";
//          sprintf(tmp, "3");
//        }
//      }
//    }
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
  //ros::Subscriber sub_gazebo = nh.subscribe("/gazebo/model_states", 10, GazeboCallback);
  //ros::Subscriber sub_position = nh.subscribe("/heroehs/alice_reference_body_sum", 10, PositionCallback);

  ros::Subscriber sub_obj_pos = nh.subscribe("/heroehs/environment_detector", 10, VisionCallback);
  //ros::Subscriber sub_robot_pos = nh.subscribe("/heroehs/alice/robot_state", 10, RobotPosCallback);
  ros::Subscriber sub_robot_pos = nh.subscribe("/heroehs/alice/global_position", 10, RobotPosCallback);  // geometry_msgs::Pose2D

  ros::Publisher pub_move_cmd    = nh.advertise<diagnostic_msgs::KeyValue>("/heroehs/move_command", 10);
  ros::Publisher pub_game_state  = nh.advertise<std_msgs::UInt8>("/game_controller/state", 10);
  ros::Publisher pub_team_index  = nh.advertise<std_msgs::UInt8>("/game_controller/team_index", 10);
  ros::Publisher pub_penalty     = nh.advertise<std_msgs::UInt8>("/heroehs/alice/penalty", 10);
  ros::Publisher pub_destination = nh.advertise<geometry_msgs::Vector3>("/heroehs/alice/global_destination", 10);

  alice.Init();

  while(ros::ok())
  {
    mtx.lock();
    alice.Update();
    mtx.unlock();

    pub_move_cmd.publish(alice.move_cmd);

    std_msgs::UInt8 game_state_msg;
    game_state_msg.data = (uint8_t)control_data.state;
    pub_game_state.publish(game_state_msg);

    std_msgs::UInt8 penalty_msg;
    penalty_msg.data = (uint8_t)control_data.state;
    pub_penalty.publish(penalty_msg);

    std_msgs::UInt8 team_index_msg;
    if(alice.team_index != -1)
    {
      team_index_msg.data = alice.team_index;
      pub_team_index.publish(team_index_msg);
    }

    geometry_msgs::Vector3 destination_msg;
    if(control_data.state == STATE_READY)
    {
      destination_msg.x = alice.destination.x;
      destination_msg.y = alice.destination.y;
      destination_msg.z = alice.destination.z;
      pub_destination.publish(destination_msg);
    }

    ros::spinOnce();
    usleep(100000);
  }
}



void VisionCallback(const alice_msgs::FoundObjectArray &msg)
{
  alice.ball_found = false;
  for(int i=0 ; i<msg.length ; i++)
  {
    if(msg.data[i].name.compare("ball") == 0)
    {
      alice.ball_found = true;
      ball.x = msg.data[i].pos.x;
      ball.y = msg.data[i].pos.y;
      ball.z = msg.data[i].pos.z;
      break;
    }
  }
}

void RobotPosCallback(const geometry_msgs::Pose2D &msg)
//void RobotPosCallback(const geometry_msgs::Vector3 &msg)
{
  alice.position.x = msg.x;
  alice.position.y = msg.y;
  //alice.position.z = msg.z;
  alice.position.z = msg.theta;
}

void RobotInit()
{
  passwd *user = getpwuid(getuid());
  string file_path = string(user->pw_dir) + "/config/robot.cfg";//"/catkin_ws/src/alice_operator/config/robot.cfg";

  string option[] = { 
    "player_number", 
    "team_number"};
  string value[2];

  ReadConf(file_path, value, option, 2);

  robot_data.player = atoi(value[0].c_str());
  robot_data.team   = atoi(value[1].c_str());
  // GAMECONTROLLER_RETURN_MSG_MAN_PENALISE   0
  // GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE 1
  // GAMECONTROLLER_RETURN_MSG_MAN_ALIVE      2
  robot_data.message = GAMECONTROLLER_RETURN_MSG_ALIVE;

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
    //    if(recv_len > 0)
    //    {
    //      mtx.lock();
    //      PrintControlData(control_data);
    //      mtx.unlock();
    //    }
    client_addr.sin_port = htons(GAMECONTROLLER_RETURN_PORT);
    client_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);  // set broadcast ip(0.0.0.0)
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


void GazeboCallback(const gazebo_msgs::ModelStates &msg)
{
  for(int i=0 ; i<msg.name.size() ; i++)
  {
    if(msg.name[i].compare("soccer_ball_0_0") == 0)
    {
      alice.ball_global.x = msg.pose[i].position.x;
      alice.ball_global.y = msg.pose[i].position.y;
    }
    if(msg.name[i].compare("alice_1_robot") == 0 || msg.name[i].compare("alice_2_robot") == 0)
    {
      alice.position.x = msg.pose[i].position.x;
      alice.position.y = msg.pose[i].position.y;
      float q_x = msg.pose[i].orientation.x;
      float q_y = msg.pose[i].orientation.y;
      float q_z = msg.pose[i].orientation.z;
      float q_w = msg.pose[i].orientation.w;
      //alice.position.z = atan2(2*(q_x*q_w + q_y*q_z), 1-2*(q_z*q_z + q_w*q_w)) + PI;
      //cout << "alice pos z : " << alice.position.z/PI*180 <<endl;
    }
  }
}

void PositionCallback(const geometry_msgs::Vector3 &msg)
{
  alice.position.z = msg.z;
}


