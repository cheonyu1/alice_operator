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
void RobotPosCallback(const geometry_msgs::Vector3 &msg);

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

  bool ball_found;
  float dist_to_ball;
  float angle_to_ball;
  Vector3 ball_global;
  uint8_t last_packet_num;

  Alice():state(Initial), last_packet_num(0), speed(3)
  {
  };

  void Init()
  {
    id = robot_data.player-1;
    map_size.x = 14;
    map_size.y = 9;
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
        set_point.x = -1;
      }
      else
      {
        set_point.x = -2;
      }
      shot_point.x = 6;
    }
    else if(robot_data.player == 2)
    {
      set_point.x = -6.5;
      shot_point.x = -6;
    }

    if(team_index == 1)
    {
      set_point.x = -set_point.x;
      set_point.z = 180;
      shot_point.x = -shot_point.x;
      shot_point.z = 180;
    }
  }

  void ReadControlData()
  {
    // read control command & check packet number & get team index. 
    if(last_packet_num != control_data.packetNumber)
    {
      last_packet_num = control_data.packetNumber;
      CheckTeamIndex();
    }

    // if team index is available, update local state. 
    if(team_index >= 0)
    {
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
        destination.z = 180;
      break;

    case Attack:
      UpdateBall();
      if( dist_to_ball  < 0.8 && 
          angle_to_ball < 10 && 
          angle_to_ball > -10 )
      {
        destination = shot_point;
      }
      else
      {
        destination = ball_global;
        speed = 3;
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
      dist_to_ball = sqrt(pow(ball.x, 2) + (pow(ball.y, 2)));
      angle_to_ball = atan2(ball.y, ball.x) * 180/PI;

      ball_global.x = position.x + (cos(position.z/180*PI)*ball.x) - (sin(position.z/180*PI)*ball.y);
      ball_global.y = position.y + (sin(position.z/180*PI)*ball.x) + (cos(position.z/180*PI)*ball.y);
    }
  }

  void Move()
  {
    float dist_to_dest = sqrt(pow(destination.x-position.x, 2) + (pow(destination.y-position.y, 2)));
    float angle_to_dest = atan2(destination.y-position.y, destination.x-position.x) * 180/PI - destination.z;

    char tmp[10];
    // if destination is not nearby.
    if( strategy != Stop && dist_to_dest > 0.2 )
    {
      // do something when destination is far enough.
      if( dist_to_dest > 1 )
      {
        // go straight if destination is in front of robot. 
        if( angle_to_dest < 30 && angle_to_dest > -30 )
        {
          move_cmd.key = "forward";
        }
        // step back when destination is at the back. 
        else if( angle_to_dest > 90 || angle_to_dest < -90 )
        {
          move_cmd.key = "backward";
        }
        // turn around to the destination, when it's at side. 
        else if(angle_to_dest > 0)
        {
          //move_cmd.key = "turn_left";
          move_cmd.key = "centered_left";
        }
        else if(angle_to_dest < 0)
        {
          //move_cmd.key = "turn_right";
          move_cmd.key = "centered_right";
        }
        sprintf(tmp, "%d", speed);
        move_cmd.value = tmp;
      }
      // do something when destination is close enough. 
      else
      {
        if( angle_to_dest > 10 )
        {
          move_cmd.key = "turn_left_precision";
          sprintf(tmp, "%.4f", angle_to_dest);
          move_cmd.value = tmp;
        }
        else if( angle_to_dest < -10 )
        {
          move_cmd.key = "turn_right_precision";
          sprintf(tmp, "%.4f", -angle_to_dest);
          move_cmd.value = tmp;
        }
        else if( dist_to_dest > 0.5 )
        {
          move_cmd.key = "forward_precision";
          sprintf(tmp, "%.4f", dist_to_dest);
          move_cmd.value = tmp;
        }
        else
        {
          move_cmd.key = "stop";
          move_cmd.value = "3";
        }
      }
    }
    else
    {
      move_cmd.key = "stop";
      move_cmd.value = "3";
    }
  }

  void CheckTeamIndex()
  {
    team_index = -1;
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

  ros::Subscriber sub_ball_pos = nh.subscribe("/heroehs/environment_detector", 10, VisionCallback);
  ros::Subscriber sub_robot_pos = nh.subscribe("/heroehs/alice/robot_state", 10, RobotPosCallback);

  ros::Publisher pub_move_cmd  = nh.advertise<diagnostic_msgs::KeyValue>("/heroehs/move_command", 10);

  diagnostic_msgs::KeyValue msg;

  alice.Init();

  while(ros::ok())
  {
    mtx.lock();
    alice.Update();
    mtx.unlock();

    msg = alice.move_cmd;
    pub_move_cmd.publish(msg);

    ros::spinOnce();
    usleep(10);
  }
}

void VisionCallback(const alice_msgs::FoundObjectArray &msg)
{
  for(int i=0 ; i<msg.length ; i++)
  {
    alice.ball_found = false;
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

void RobotPosCallback(const geometry_msgs::Vector3 &msg)
{
  alice.position.x = msg.x;
  alice.position.y = msg.y;
  alice.position.z = msg.z;
}

void RobotInit()
{
  passwd *user = getpwuid(getuid());
  string file_path = string(user->pw_dir) + "/catkin_ws/src/alice_operator/config/robot.cfg";

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
  controller_addr.sin_addr.s_addr = inet_addr("0.0.0.0");//htonl(INADDR_ANY);  // set broadcast ip(0.0.0.0)
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
      PrintControlData(control_data);
      mtx.unlock();
    }
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






