#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>  // std::sort

// PCL specific includes
// #include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.h>
// #include <boost/foreach.hpp>

#include <rws2019_msgs/DoTheMath.h> //para o servico


using namespace std;
using namespace ros;
using namespace boost;

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;  // PCL

float randomizePosition()
{
  srand(4232 * time(NULL));  // set initial seed value to 5323
  return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
}

namespace ttavares_ns
{  // criamos o nosso namespace para nao haver conflitos

struct mysort
{
  bool operator()(int i, int j)
  {
    return (i < j);
  }
} myobject;

class Team  // class Team
{
public:
  string team_name;
  vector<string> player_names;  // ciar um vetor de strings chamado player_names

  NodeHandle n;  // criate a node

  Team(string team_name_in)
  {
    team_name = team_name_in;
    n.getParam("/team_" + team_name, player_names);
  }

  void printInfo()
  {
    cout << "Team " << team_name << " has players: " << endl;

    for (size_t i = 0; i < player_names.size(); i++)
    {
      cout << player_names[i] << endl;
    }
  }

  // funct returns True or False dependendo the player_name
  bool playerBelongsToTeam(string player_name)
  {
    for (size_t i = 0; i < player_names.size(); i++)
    {
      if (player_name == player_names[i])
      {
        return true;
      }
    }
    return false;
  }

private:
};

// class PLAYER
class Player
{
public:  // methods
  string player_name;

  Player(string player_name_in)  // construtor
  {
    player_name = player_name_in;
  }

  void setTeamName(string team_name_in)
  {
    if (team_name_in == "red" || team_name_in == "green" || team_name_in == "blue")
    {
      team_name = team_name_in;
    }
    else
    {
      cout << "Can not set team name: " << team_name_in << endl;
    }
  }
  void setTeamName(int team_index)  // damos o overload do setTeamName, ele
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

  string getTeamName()
  {
    return team_name;
  };

private:
  string team_name;
};

class MyPlayer : public Player
{  // declaro a classe e herdo as propriedades da
   // classe Player
public:
  boost::shared_ptr<Team> team_red;  // nomenclatura para um ponteiro shares_pointer declaração
  boost::shared_ptr<Team> team_green;
  boost::shared_ptr<Team> team_blue;

  boost::shared_ptr<Team> team_hunters;
  boost::shared_ptr<Team> team_mine;
  boost::shared_ptr<Team> team_preys;

  // para as tf's
  tf::TransformBroadcaster br;
  tf::TransformListener listener;

  boost::shared_ptr<ros::Publisher> vis_pub;

  string last_prey;
  string last_hunter;

  ros::Publisher pub;

  MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in)
  {  // contrutor

    team_red = (boost::shared_ptr<Team>)new Team("red");  // do genero de um malloc, ou seja team_red aponta para um
                                                          // campo q ja existe
    team_green = (boost::shared_ptr<Team>)new Team("green");
    team_blue = (boost::shared_ptr<Team>)new Team("blue");

    ros::NodeHandle n;
    vis_pub = (boost::shared_ptr<ros::Publisher>)new ros::Publisher;
    (*vis_pub) = n.advertise<visualization_msgs::Marker>("/bocas", 0);

    if (team_red->playerBelongsToTeam(player_name))  // se o
    {
      team_mine = team_red;
      team_preys = team_green;
      team_hunters = team_blue;
    }
    else if (team_green->playerBelongsToTeam(player_name))
    {
      team_mine = team_green;
      team_preys = team_blue;
      team_hunters = team_red;
    }
    else if (team_blue->playerBelongsToTeam(player_name))
    {
      team_mine = team_blue;
      team_preys = team_red;
      team_hunters = team_green;
    }
    else
    {
      cout << "something wrong in team parametrization" << endl;
    }

    setTeamName(team_mine->team_name);  // team_mine é um ponteiro = team_green p.ex... Na classe Team existe um campo
                                        // team_name

    // DEFINE INTIAL POSTION:
    tf::Transform T1;
    T1.setOrigin(tf::Vector3(randomizePosition(), randomizePosition(), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, M_PI);
    T1.setRotation(q);

    br.sendTransform(tf::StampedTransform(T1, ros::Time::now(), "world", player_name));
    ros::Duration(0.1).sleep();
    br.sendTransform(tf::StampedTransform(T1, ros::Time::now(), "world", player_name));

    printInfo();

    last_prey = "";
    last_hunter = "";
  }

  void printInfo()
  {
    ROS_INFO_STREAM("My name is " << player_name << " and my team is " << team_mine->team_name << endl);
  }

  std::tuple<float, float> getDistanceAndAngleToWorld()
  {
    return getDistanceAndAngleToPlayer("world");
  }

  std::tuple<float, float> getDistanceAndAngleToPlayer(string other_player)
  {
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform(player_name, other_player, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
      return { 1000, 1000 };
    }

    float d = sqrt(T0.getOrigin().x() * T0.getOrigin().x() + T0.getOrigin().y() * T0.getOrigin().y());
    float a = atan2(T0.getOrigin().y(), T0.getOrigin().x());

    return { d, a };
  }

  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg)
  {
    // ROS_INFO("received a new msg");

    visualization_msgs::Marker marker;

    // so podemos aplicar transformações quando jogamos
    // TF
    bool something_changed = false;

    // STEP 1: Find out where i am
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform("world", player_name, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
    }

    // STEP 2: difine how i want to move
    vector<float> distance_to_preys;
    vector<float> distance_sort_preys;
    vector<float> angle_to_preys;

    vector<float> distance_to_hunters;
    vector<float> distance_sort_hunters;
    vector<float> angle_to_hunters;

    // For each prey find the closest. Tenh follow her:
    for (size_t i = 0; i < team_preys->player_names.size(); i++)
    {
      // ROS_WARN_STREAM("team_preys =" << team_preys->player_names[i] << endl);

      std::tuple<float, float> t = getDistanceAndAngleToPlayer(team_preys->player_names[i]);
      distance_to_preys.push_back(std::get<0>(t));
      angle_to_preys.push_back(std::get<1>(t));
    }

    int idx_closest_prey = -1;
    float distance_closest_prey = 1000;

    for (size_t i = 0; i < distance_to_preys.size(); i++)
    {
      if (distance_to_preys[i] < distance_closest_prey)
      {
        idx_closest_prey = i;
        distance_closest_prey = distance_to_preys[i];
      }
    }

    // for each hunter find the closest, and run away
    for (size_t i = 0; i < team_hunters->player_names.size(); i++)
    {
      // ROS_WARN_STREAM("team_hunters =" << team_hunters->player_names[i] << endl);

      std::tuple<float, float> t = getDistanceAndAngleToPlayer(team_hunters->player_names[i]);
      distance_to_hunters.push_back(std::get<0>(t));
      angle_to_hunters.push_back(std::get<1>(t));
    }

    int idx_closest_hunter = -1;
    float distance_closest_hunter = 1000;

    for (size_t i = 0; i < distance_to_hunters.size(); i++)
    {
      if (distance_to_hunters[i] < distance_closest_hunter)
      {
        idx_closest_hunter = i;
        distance_closest_hunter = distance_to_hunters[i];
      }
    }

    // sort vectores
    distance_sort_hunters = distance_to_hunters;
    distance_sort_preys = distance_to_preys;

    std::sort(distance_sort_hunters.begin(), distance_sort_hunters.end(), myobject);
    std::sort(distance_sort_preys.begin(), distance_sort_preys.end(), myobject);

    // get world
    float distance_to_world = std::get<0>(getDistanceAndAngleToWorld());
    float angle_to_world = std::get<1>(getDistanceAndAngleToWorld());

    // Check if last_prey is different from prey
    // string prey = "";
    // if (idx_closest_prey != -1)
    // {
    //   prey = msg->blue_alive[idx_closest_prey];
    //   if (prey != last_prey)
    //   {
    //     something_changed = true;
    //     last_prey = prey;
    //   }
    // }

    // string hunter = "";
    // if (idx_closest_hunter != -1)
    // {
    //   hunter = msg->red_alive[idx_closest_hunter];
    //   if (hunter != last_hunter)
    //   {
    //     something_changed = true;
    //     last_hunter = hunter;
    //   }
    // }

    // check
    float max_distance_world = 7;
    float dx = 10;
    float a = M_PI / 30;

    if (distance_closest_hunter <= 1)
    {
      a = M_PI + angle_to_hunters[idx_closest_hunter];
      marker.text = "running away from my hunter " + team_hunters->player_names[idx_closest_hunter];
    }
    else
    {
      a = angle_to_preys[idx_closest_prey];
      marker.text = "i will catch you " + team_preys->player_names[idx_closest_prey];
    }
    if (distance_to_world >= max_distance_world)
    {
      a = angle_to_world + M_PI / 2;
      marker.text = "oh no! arena limits";

      if (distance_closest_hunter >= 2)
      {
        a = angle_to_preys[idx_closest_prey];
        marker.text = "i will catch you " + team_preys->player_names[idx_closest_prey];
      }
    }

    // STEP2.5: check values
    float dx_max = msg->turtle;
    dx > dx_max ? dx = dx_max : dx = dx;

    double amax = M_PI / 30;

    if (a != 0)
    {
      fabs(a) > fabs(amax) ? a = amax * a / fabs(a) : a = a;
    }
    // STEP 3: define local movement
    tf::Transform T1;
    T1.setOrigin(tf::Vector3(dx, 0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, a);
    T1.setRotation(q);

    // STEP 4: define global movement
    tf::Transform Tglobal = T0 * T1;
    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));

    // if (something_changed){
    // marker.header.frame_id = player_name;
    // marker.header.stamp = ros::Time();
    // marker.ns = player_name;
    // marker.id = 0;
    // marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.position.y = 0.5;
    // marker.scale.z = 0.4;
    // marker.color.a = 1.0;  // Don't forget to set the alpha!
    // marker.color.r = 0.0;
    // marker.color.g = 0.0;
    // marker.color.b = 0.0;
    // marker.lifetime = ros::Duration(2);
    // marker.frame_locked= 1; //marcador para mexer ao longo  do tempo
    // // only if using a MESH_RESOURCE marker type:
    // // marker.mesh_resource =
    // // "package://pr2_description/meshes/base_v0/base.dae";
    // vis_pub->publish(marker);
    // }
  }

  // void PCL_callback(const PointCloud::ConstPtr& msg)
  // {
  //   printf("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  //   BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  //     printf("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  // }

  bool DoOperation(rws2019_msgs::DoTheMath::Request &req, 
                  rws2019_msgs::DoTheMath::Response &res)
  {
    if (req.op=="+")
    res.result = req.a + req.b;
    else if (req.op=="-")
    res.result = req.a - req.b;
    else if (req.op=="/")
    res.result = req.a / req.b;
    else if (req.op=="*")
    res.result = req.a * req.b;
    else
    {
      res.result = -1;
      return true;
    }
    
    // ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    // ROS_INFO("sending back response: [%ld]", (long int)res.result);
    return true;
  }

private:
};

};  // namespace ttavares_ns

main(int argc, char **argv)
{
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

  //========================================== PCL ====================================
  // ros::init(argc, argv, "sub_pcl");
  // ros::NodeHandle nh;
  // ros::Subscriber sub_pcl = n.subscribe<PointCloud>("/object_point_cloud", 1, &ttavares_ns::MyPlayer::PCL_callback,
  // &player);

  //========================================================================================
  //===============SERVIDOR=================
  ros::ServiceServer service = n.advertiseService("do_the_math", &ttavares_ns::MyPlayer::DoOperation, &player);
  // ROS_INFO("Ready to operate");

  //=====================================================
  ros::Subscriber sub = n.subscribe("/make_a_play", 100, &ttavares_ns::MyPlayer::makeAPlayCallback, &player);
  // make_a_play is a topic
  player.printInfo();
  ros::Rate r(20);
  while (ok())
  {
    // team_green.printInfo();
    // bool check_team = team_green.playerBelongsToTeam("moliveira");
    // cout << "o resultado do check é=" << check_team << endl;
    ros::spinOnce();
    r.sleep();
  }

  return 1;
}