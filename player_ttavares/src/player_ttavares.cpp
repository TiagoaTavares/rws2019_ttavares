#include <iostream>
#include <ros/ros.h>
#include <vector>

#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace ros;
using namespace boost;


float randomizePosition()
{
  srand(4232*time(NULL)); // set initial seed value to 5323
  return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
}

namespace ttavares_ns // criamos o nosso namespace para nao aver conflitos
{

class Team // class Team
{
public:
  string team_name;
  vector<string> player_names; // ciar um vetor de strings chamado player_names

  NodeHandle n; // criate a node

  Team(string team_name_in) {
    team_name = team_name_in;

    n.getParam("/team_" + team_name, player_names);
  }

  void printInfo() {
    cout << "Team " << team_name << " has players: " << endl;

    for (size_t i = 0; i < player_names.size(); i++) {
      cout << player_names[i] << endl;
    }
  }

  bool playerBelongsToTeam(string player_name) 
  {
    for (size_t i = 0; i < player_names.size(); i++) 
    {
      if (player_name == player_names[i]) {
        return true;
      }
    }
    return false;
  }

private:
};

class Player {

public: // methods
  string player_name;

  Player(string player_name_in) // construtor
  {
    player_name = player_name_in;
  }

  void setTeamName(string team_name_in) {
    if (team_name_in == "red" || team_name_in == "green" || team_name_in == "blue") {
      team_name = team_name_in;
    } else {
      cout << "Hello Cannote set team name" << team_name_in << endl;
    }
  }
  void setTeamName(int team_index) // damos o overload do setTeamName, ele
                                   // escolhe qual usa consoante o argumento
  {
    if (team_index == 0)
      setTeamName("red");
    else if (team_index == 1)
      setTeamName("green");
    else if (team_index == 2)
      setTeamName("blue");
    else if (team_index == 1)
      setTeamName("red");
    else
      setTeamName("");
  }

  string getTeamName() { return team_name; };

private:
  string team_name;
};

class MyPlayer
    : public Player // declaro a classe e herdo as propriedades da classe Player
{
public:
  boost::shared_ptr<Team> team_red; // nomenclatura para um ponteiro shares_pointer declaração
  boost::shared_ptr<Team> team_green;
  boost::shared_ptr<Team> team_blue;

  boost::shared_ptr<Team> team_hunters;
  boost::shared_ptr<Team> team_mine;
  boost::shared_ptr<Team> team_preys;

  // para as tf's
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  //tf::Transform transform;

  MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in) {
    team_red = (boost::shared_ptr<Team>)new Team("red"); // do genero de um malloc, ou seja team_red aponta para um campo
                // q ja existe
    team_green = (boost::shared_ptr<Team>)new Team("green");
    team_blue = (boost::shared_ptr<Team>)new Team("blue");

    if (team_red->playerBelongsToTeam(player_name)) // se o
    {
      team_mine = team_red;
      team_preys = team_green;
      team_hunters = team_blue;
    } else if (team_green->playerBelongsToTeam(player_name)) {
      team_mine = team_green;
      team_preys = team_blue;
      team_hunters = team_red;
    } else if (team_blue->playerBelongsToTeam(player_name)) {
      team_mine = team_blue;
      team_preys = team_red;
      team_hunters = team_green;
    } else {
      cout << "something wrong in team parametrization" << endl;
    }
    setTeamName(team_mine->team_name);

    //DEFINE INTIAL POSTION:
    tf::Transform T1;
    T1.setOrigin(tf::Vector3(randomizePosition(), randomizePosition(), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, M_PI);
    T1.setRotation(q);


    br.sendTransform(tf::StampedTransform(T1, ros::Time::now(), "world", player_name));




    printInfo();
  }

  void printInfo() {
    ROS_INFO_STREAM("My name is " << player_name << " and my team is "
                                  << team_mine->team_name << endl);
  }

  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg) {
    ROS_INFO("received a new msg");

    // so podemos aplicar transformações quando jogamos
    // TF

    //STEP 1: Find out where i am
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform("/world", player_name, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.1).sleep();
    }

    //STEP 2: difine how i want to move
    float dx= 0.5;
    float angle=M_PI/6;


    //STEP 3: define local movement
    tf::Transform T1;
    T1.setOrigin(tf::Vector3(dx, 0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, angle);
    T1.setRotation(q);

    //STEP 4: define global movement
    tf::Transform Tglobal = T0 * T1 ;


    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));
  }

private:
};

}; // namespace ttavares_ns

main(int argc, char **argv) {

  init(argc, argv, "PlayerTiago");
  NodeHandle n;

  ttavares_ns::MyPlayer player("ttavares", "green");
  // Player player("tiagoTavares");
  // player.setTeamName("red");
  // player.setTeamName(0);

  // cout << "Hello World " << player.player_name << " of team " <<
  // player.getTeamName() << endl;

  // ttavares_ns::Team team_green("green");
  // team_green.player_names.push_back("moliveira");
  // team_green.player_names.push_back("blourenco");

  ros::Subscriber sub = n.subscribe("/make_a_play", 100, &ttavares_ns::MyPlayer::makeAPlayCallback, &player);

    player.printInfo();
    ros::Rate r(20);
  while (ok()) {
    // team_green.printInfo();
    // bool check_team = team_green.playerBelongsToTeam("moliveira");
    // cout << "o resultado do check é=" << check_team << endl;
    ros::spinOnce();
    r.sleep();
  }

  return 1;
}