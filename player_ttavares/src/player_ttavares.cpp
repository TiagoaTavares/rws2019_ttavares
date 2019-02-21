#include <iostream>
#include <ros/ros.h>
#include <vector>

using namespace std;
using namespace ros;
using namespace boost;

namespace ttavares_ns
{

class Team
{
  public:
    string team_name;
    vector<string> player_names;

    NodeHandle n;

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

class Player
{

  public:
    // methods
    string player_name;
    Player(string player_name_in) // construtor
    {
        player_name = player_name_in;
    }

    void setTeamName(string team_name_in)
    {
        if (team_name_in == "red" || team_name_in == "green" ||
            team_name_in == "blue")
        {
            team_name = team_name_in;
        }
        else
        {
            cout << "Hello Cannote set team name" << team_name_in << endl;
        }
    }
    void setTeamName(int team_index)
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
    boost::shared_ptr<Team>
        team_red; // nomenclatura para um ponteiro shares_pointer
    boost::shared_ptr<Team> team_green;
    boost::shared_ptr<Team> team_blue;

    boost::shared_ptr<Team> team_hunters;
    boost::shared_ptr<Team> team_mine;
    boost::shared_ptr<Team> team_preys;

    MyPlayer(string player_name_in, string team_name_in)
        : Player(player_name_in)
    {
        team_red = (boost::shared_ptr<Team>)new Team(
            "red"); // do genero de um malloc, ou seja team_red aponta para um campo
                    // q ja existe
        team_green = (boost::shared_ptr<Team>)new Team("green");
        team_blue = (boost::shared_ptr<Team>)new Team("blue");

        if (team_red->playerBelongsToTeam(player_name))
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

        setTeamName(team_mine->team_name);

        printInfo();
    }

    void printInfo()
    {
        ROS_INFO_STREAM("My name is " << player_name << " and my team is "
                                      << team_mine->team_name << endl);
    }

  private:
};

}; // namespace ttavares_ns

main(int argc, char **argv)
{

    init(argc, argv, "PlayerTiago");
    NodeHandle n;

    ttavares_ns::MyPlayer player("talmeida", "green");
    // Player player("tiagoTavares");
    // player.setTeamName("red");
    // player.setTeamName(0);

    // cout << "Hello World " << player.player_name << " of team " <<
    // player.getTeamName() << endl;

    // ttavares_ns::Team team_green("green");
    // team_green.player_names.push_back("moliveira");
    // team_green.player_names.push_back("blourenco");

    while (ok())
    {
        // team_green.printInfo();
        // bool check_team = team_green.playerBelongsToTeam("moliveira");
        // cout << "o resultado do check é=" << check_team << endl;
        player.printInfo();

        Duration(1).sleep();
    }

    return 1;
}