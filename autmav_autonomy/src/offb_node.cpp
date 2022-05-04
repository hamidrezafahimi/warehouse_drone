#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
// #include <mavros_msgs/GlobalPositionTarget.h>
#include <std_msgs/Float64.h>
#include <float.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// #include <mavros_msgs/CommandTOL.h>
// #include <mavros_msgs/WaypointPush.h>
// #include <mavros_msgs/WaypointClear.h>
// #include <mavros_msgs/CommandHome.h>
#include <std_msgs/String.h>
#include <cstdlib>
// #include <mavros_msgs/Waypoint.h>
// #include <mavros_msgs/WaypointReached.h>
// #include <mavros_msgs/Altitude.h>
// #include <mavros_msgs/WaypointSetCurrent.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

using namespace std;

enum Action
{
    POSITION_HOLD,
    DELIVERY_ACTION,
    FRAGILE_DELIVERY_ACTION,
    SHOT_ACTION,
    SHOT_END_ACTION,
    LAND_ACTION,
    NONE
};

struct Stage {
    int wp_index;
    Action action;
};

class StageManager {
public:
    std::vector<Stage> seq;
    StageManager();
    bool advance(ros::ServiceClient wpSetClient);
};

bool isHomeLocationSet=false;
bool isHomeAltitudeSet=false;
bool isHoemHeadingSet=false;
float initHeading =0 ;
float shotInterval = 4;
int AeraLat=47,AeraLong=8;
//int AeraLat=35,AeraLong=51;//FOR UNIY
sensor_msgs::NavSatFix home ;
ros::ServiceClient set_mode_client;
ros::Timer shotTimer;
char ReadyForTakeOff ;


//ROS_CALLBACK_FUNCS
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

mavros_msgs::WaypointReached mission_reached;
void mission_r (const mavros_msgs::WaypointReached::ConstPtr& msg)
{
    mission_reached = *msg ;
}
sensor_msgs::NavSatFix global_pos ;
void gps_data(const sensor_msgs::NavSatFix::ConstPtr& msg){
    global_pos = *msg ;

    if (!isHomeLocationSet)
    {
        home = *msg;
        isHomeLocationSet = true;
    }
}


mavros_msgs::Altitude altitude;
void alt_cb(const mavros_msgs::Altitude::ConstPtr& msg){
    altitude = *msg;
    if (!isHomeAltitudeSet)
    {
        home.altitude = msg->amsl;
        isHomeAltitudeSet = true;
    }
}

std_msgs::Float64 Heading;
std_msgs::Float64 HomeHeading;
void hdg (const std_msgs::Float64::ConstPtr& msg)
{
    Heading = *msg ;

    if (!isHoemHeadingSet)
    {
        HomeHeading = *msg;
        isHoemHeadingSet = true;
    }
}


mavros_msgs::WaypointClear wp_clear_srv;
mavros_msgs::CommandHome set_home_srv;
mavros_msgs::Waypoint wps_msg;
mavros_msgs::WaypointSetCurrent current_wp;
geometry_msgs::PoseStamped pose;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::SetMode guided_set_mode;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::CommandTOL takeoff_cmd;
mavros_msgs::CommandTOL land_cmd;


ros::Time last_request;

int wp_number = 0 ;
int NumberOfMissions = 0;
float alt=0;


enum mission_enum {
    PREPRATION,
    AUTO_ARM,
    TAKEOFF_INIT,
    TAKEOFF_FINISH,
    MISSION_MODE,
    WPS,
    RTL,
    LAND_INIT,
    LAND_FINISH,
    FINISHED
} mission;

vector <mavros_msgs::Waypoint> new_wp;
vector <int> actions;

/////////////////////////////////////////////////////////////
//needed to be recersive
void clear_waypoints(mavros_msgs::WaypointClear wp_clear_srv,
     ros::ServiceClient wp_clear_client)
{
    wp_clear_srv.request = {};
    if (wp_clear_client.call(wp_clear_srv))
    {
        ROS_INFO("Waypoint list was cleared");
    }
    else
    {
        ROS_ERROR("Waypoint list couldn't been cleared");
    }
}
/////////////////////////////////////////////////////////////
void set_home (mavros_msgs::CommandHome set_home_srv,ros::ServiceClient set_home_client)
{
    set_home_srv.request.current_gps = false;
    set_home_srv.request.latitude = home.latitude;
    set_home_srv.request.longitude = home.longitude;
    set_home_srv.request.altitude = home.altitude;

    if (set_home_client.call(set_home_srv))
    {
        ROS_INFO("Home was set to new value ");
    }
    else
    {
        ROS_ERROR("Home position couldn't been changed");
    }
}
////////////////////////////////////////////////////////////
bool Auto_mode_arming (ros::ServiceClient set_mode_client,ros::ServiceClient arming_client)
{
        if( current_state.mode != "AUTO.MISSION" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("AUTO.MISSION enabled");
            }
            last_request = ros::Time::now();
            return true;
        }
        else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        return true ;
}

///////////////////////////////////////////////////////////////////////////////////////////

bool Guided_mode (ros::ServiceClient set_mode_client,ros::ServiceClient arming_client)
{
        if( current_state.mode != "Guided" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(guided_set_mode) &&
                guided_set_mode.response.mode_sent)
            {
                ROS_INFO("Guided enabled");
            }
            last_request = ros::Time::now();
            return true;
        }
        // else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0)))
                //{
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success)
        //         {
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }

        return true ;
}

///////////////////////////////////////////////////////////////////////////////////////////

void takeoff(ros::ServiceClient takeoff_client)
{
    takeoff_cmd.request.altitude = home.altitude+2.5;
    takeoff_cmd.request.latitude = home.latitude;
    takeoff_cmd.request.longitude = home.longitude;
    takeoff_cmd.request.min_pitch = 0;
    ROS_INFO("1-HomeHeading : %f & tak.request.yaw = %f" , HomeHeading,takeoff_cmd.request.yaw );
    takeoff_cmd.request.yaw = HomeHeading.data;
    ROS_INFO("2-HomeHeading : %f & tak.request.yaw = %f" , HomeHeading,takeoff_cmd.request.yaw );
    if(takeoff_client.call(takeoff_cmd))
    {
        ROS_INFO("Takeoff");
    }
    else
    {
        ROS_ERROR("Takeoff Failed");
    }
}

////////////////////////////////////////////////////////////

bool set_current_wp(ros::ServiceClient wp_set_current,int wpNumber)
{
    current_wp.request.wp_seq = wpNumber;
    if( wp_set_current.call(current_wp) && current_wp.response.success)
    {
        ROS_INFO("currect wp : %d ",wpNumber );
        return true ;
    }
    else
    {
        ROS_ERROR("Current state : %d ",wp_number);
        return false ;
    }
}

////////////////////////////////////////////////////////////
bool land(ros::ServiceClient land_client,mavros_msgs::CommandTOL land_cmd,std_msgs::Float64 last_heading)
{
    land_cmd.request.altitude = home.altitude;
    land_cmd.request.latitude = home.latitude;
    land_cmd.request.longitude = home.longitude;
    land_cmd.request.min_pitch = 0;
    land_cmd.request.yaw = last_heading.data;
    if(land_client.call(land_cmd))
    {
        ROS_INFO("Land");
        return true;
    }
    else
    {
        ROS_ERROR("Land Failed");
        return false;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////

bool pushback_waypoint(ros::ServiceClient,ros::ServiceClient,int);
ros::ServiceClient drop_client;
ros::ServiceClient capture_client;

///////////////////////////////////////////////////////////////////////////////////////////
void timerCallback(const ros::TimerEvent&)
{
  ROS_INFO("Camera capture triggered.");
  std_srvs::Empty msg;
  capture_client.call(msg);
}

void readWaypoints(string file)
{
    std::ifstream infile(file);
    std::string line;

    while(std::getline(infile, line))
    {
        if (line.length() > 0)
        {
            if (line.substr(0,1) == "#")
                continue;

            double lat;
            double lng;
            double alt;
            int type;

            std::string delimiter = ",";

            size_t pos = 0;
            std::string token;

            if ((pos = line.find(delimiter)) != std::string::npos) {
                token = line.substr(0, pos);

                if (token == "homeLat")
                    lat = home.latitude;
                else
                    lat = stof(token);

                line.erase(0, pos + delimiter.length());
            }
            else
            {
                continue;
            }

            if ((pos = line.find(delimiter)) != std::string::npos) {
                token = line.substr(0, pos);

                if (token == "homeLng")
                    lng = home.longitude;
                else
                    lng = stof(token);

                line.erase(0, pos + delimiter.length());
            }
            else
            {
                continue;
            }

            if ((pos = line.find(delimiter)) != std::string::npos) {
                token = line.substr(0, pos);
                alt = stof(token);
                line.erase(0, pos + delimiter.length());
            }
            else
            {
                continue;
            }

            type = stoi(line);

            ROS_INFO("wp %d: %.7f %.7f %.1f  action: %d", NumberOfMissions, lat, lng, alt, type);

            mavros_msgs::Waypoint new_waypoint;
            new_waypoint.frame = 3; // mavros_msgs::Waypoint::FRAME_GLOBAL;
            new_waypoint.command = 16;
            new_waypoint.is_current = false;
            new_waypoint.autocontinue = false;
            new_waypoint.param1 = 0;
            new_waypoint.param2 = 0;
            new_waypoint.param3 = 0;
            new_waypoint.param4 = NAN;
            new_waypoint.x_lat = lat;
            new_waypoint.y_long = lng;
            new_waypoint.z_alt = alt;

            new_wp.push_back(new_waypoint);
            actions.push_back(type);

            NumberOfMissions ++;
        }
    }

}



//START OF INT MAIN

int main(int argc, char **argv)
{
    string missionFile = argv[1];
    string missionPlatform = argv[2];
    bool isSim = (missionPlatform == "sim");

    string prefix = (isSim) ? "/uav0" : "";
//Init ROS
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
//Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            (prefix + "/mavros/state", 10, state_cb);
    ros::Subscriber altitude_sub = nh.subscribe<mavros_msgs::Altitude>
            (prefix + "/mavros/altitude", 10, alt_cb);
    ros::Subscriber check_mission = nh.subscribe<mavros_msgs::WaypointReached>
            (prefix + "/mavros/mission/reached", 10, mission_r);
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
            (prefix + "/mavros/global_position/global",10, gps_data);
    ros::Subscriber hdg_sub = nh.subscribe<std_msgs::Float64>
            (prefix + "/mavros/global_position/compass_hdg",10, hdg);
//Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            (prefix + "/mavros/setpoint_position/local", 10);

//Services
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        (prefix + "/mavros/cmd/arming");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
        (prefix + "/mavros/cmd/takeoff");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
        (prefix + "/mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        (prefix + "/mavros/set_mode");
    ros::ServiceClient wp_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>
        (prefix + "/mavros/mission/clear");
    ros::ServiceClient wp_srv_client = nh.serviceClient<mavros_msgs::WaypointPush>
        (prefix + "/mavros/mission/push");
    ros::ServiceClient wp_set_current = nh.serviceClient<mavros_msgs::WaypointSetCurrent>
        (prefix + "/mavros/mission/set_current");
    ros::ServiceClient set_home_client = nh.serviceClient<mavros_msgs::CommandHome>
        (prefix + "/mavros/cmd/set_home");
    drop_client = nh.serviceClient<std_srvs::SetBool>
        ("/drop_srv");
    capture_client = nh.serviceClient<std_srvs::Empty>
        ("/trigger_srv");


    shotTimer = nh.createTimer(ros::Duration(shotInterval), timerCallback);
    shotTimer.stop();

    ROS_INFO("initialized");

/////////////////////////////////////////////////////////////////////////////////////////////

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(5);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Trying to connect ... ");
    }
    ROS_INFO("connected");

    while(!isHomeAltitudeSet || !isHomeLocationSet)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for HOME ... ");
    }
/////////////////////////////////////////////////////////////////////////////////////////////

        offb_set_mode.request.custom_mode = "AUTO.MISSION";
        arm_cmd.request.value = true;

/////////////////////////////////////////////////////////////////////////////////////////////

//SET HOME
    last_request = ros::Time::now();
    clear_waypoints(wp_clear_srv,wp_clear_client);
    // set_home(set_home_srv,set_home_client);

/////////////////////////////////////////////////////////////////////////////////////////////
//SET WAYPOINTS
    ROS_INFO("HomeHeading : %f", HomeHeading.data);

    std::string path = ros::package::getPath("autmav_autonomy");
    readWaypoints(path + "/mission/" + missionFile + ".txt");

    wp_number = 0;

///////////////////////////////////////////////////////
    ros::spinOnce();
///////////////////////////////////////////////////////
    StageManager sm;

    while(ros::ok()){

        //ROS_INFO("****mission number is : %d ",(int)NumberOfMissions);

        switch (mission)
        {

            case PREPRATION :
            {
                std_srvs::SetBool msg;
                msg.request.data = true ;
                drop_client.call(msg);
                ros::Duration(0.5).sleep();
                drop_client.call(msg);
                ros::Duration(0.5).sleep();
                drop_client.call(msg);
                ros::Duration(2.0).sleep();

                ROS_INFO(" Preare for takeoff ? ");
                std::cin>>ReadyForTakeOff ;
                if (ReadyForTakeOff=='y' || ReadyForTakeOff=='Y')
                {
                        msg.request.data = false;
                        drop_client.call(msg);
                        ros::Duration(0.5).sleep();
                        drop_client.call(msg);
                        ros::Duration(2.0).sleep();

                    ros::spinOnce();

                        ROS_INFO(" Preare for takeoff ? ");
                        std::cin>>ReadyForTakeOff ;
                        if (ReadyForTakeOff=='y' || ReadyForTakeOff=='Y')
                        {
                            ros::spinOnce();

                            if(pushback_waypoint(wp_clear_client,wp_srv_client,NumberOfMissions))
                            {
                                    mission = AUTO_ARM ;
                                    ROS_INFO(" AUTO_ARMING_MODE ");
                            }
                        }
                }


            };
            break;

            /////////////////////////////////

            case AUTO_ARM :
            {

                Auto_mode_arming(set_mode_client,arming_client);
                if(current_state.mode == "AUTO.MISSION" && current_state.armed)
                {
                    mission = TAKEOFF_INIT ;
                }

            };
            break;

            /////////////////////////////////

            case TAKEOFF_INIT :
            {
                takeoff(takeoff_client);
                ROS_INFO(" TAKEOFF INIT ");
                mission= TAKEOFF_FINISH ;

            };
            break;

           /////////////////////////////////

            case TAKEOFF_FINISH :
            {

                    ROS_INFO("Take off .... ");
                    ROS_INFO("Altitude : %.2f %.2f", altitude.amsl,takeoff_cmd.request.altitude);
                    if (altitude.amsl >= takeoff_cmd.request.altitude - 0.2)
                    {
                        mission = MISSION_MODE ;
                        ROS_INFO("Take off complete ");

                    }


            };
            break;

            /////////////////////////////////

            case MISSION_MODE :
            {
                ROS_INFO(" MISSION_MODE ");
                Auto_mode_arming(set_mode_client,arming_client);
                if(current_state.mode == "AUTO.MISSION" && current_state.armed)
                {
                    mission = WPS ;
                    ros::Duration(1.0).sleep();
                    ros::spinOnce();
                }

            };
            break;

            ////////////////////////////////

            case WPS :
            {
                ROS_INFO(" WPS ");
                ros::Duration(3.0).sleep();

                sm.advance(wp_set_current);
                break ;

                ros::Duration(3.0).sleep();

            };
            break;

            ///////////////////////////////

            case RTL:
            {

                    set_current_wp(wp_set_current,0);
                    while(mission_reached.wp_seq != 0)
                    {
                        ROS_INFO("COMMING BACK HOME .... " );
                        ros::spinOnce();
                        rate.sleep();
                    }
                    ROS_INFO("Vehicle ARRIVED HOME !");
                    ros::Duration(7.0).sleep();

                    mission = LAND_INIT ;
            };
            break;

            //////////////////////////////////

            case LAND_INIT :
            {
                ROS_INFO(" LAND ");

                if (land(land_client,land_cmd,HomeHeading))
                {
                    mission = LAND_FINISH ;
                }

            };
            break;

            //////////////////////////////////

            case LAND_FINISH :
            {

                ROS_INFO("Landing .... ");
                ROS_INFO("reletive_alt : %.2f ", altitude.amsl);

                if((altitude.amsl <= home.altitude-0.08 || altitude.amsl <= home.altitude+0.08) &&
                    sm.seq.size()!=0)
                {
                    mission = PREPRATION ;
                }
                else if((altitude.amsl <= home.altitude-0.08 || altitude.amsl <= home.altitude+0.08) &&
                         sm.seq.size()==0)
                {
                    mission = FINISHED;
                }


            };
            break;

            /////////////////////////////////

            default :
            {
                ROS_INFO("Case Default ");
                ROS_INFO("MISSION FINISHED ");

            };
            break;

            /////////////////////////////////
        }


        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}

//End of int main
///////////////////////////////////////////////////////////////////////////////////////////

//Functions

bool pushback_waypoint(ros::ServiceClient wp_clear_client,
                       ros::ServiceClient wp_srv_client,int count)
{
    ROS_INFO("Pushback inited");
//Get wp message ready for Pub

    mavros_msgs::WaypointPush wp_push_srv;
    wp_push_srv.request.start_index = 0;

    for (int i=0 ; i<count ; i++)
    {
        wp_push_srv.request.waypoints.push_back(new_wp[i]);
    }


//Check if wp sent correctly
    if (wp_srv_client.call(wp_push_srv))
    {
        if (wp_push_srv.response.success)
        {
            ROS_INFO("Success:%d", (bool)wp_push_srv.response.success);
            return true ;
        }
        else
        {
            ROS_ERROR("No succsess response from autopilot");
            return false ;
        }

    }
    else
    {
        ROS_ERROR("Waypoint couldn't been sent");
        return false;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

StageManager::StageManager(){

    for(int i=0; i<new_wp.size(); i++)
    {
        Stage st;
        st.wp_index = i;
        st.action = static_cast<Action>(actions[i]);
        seq.push_back(st);
    }
}


bool StageManager::advance(ros::ServiceClient wpSetClient)
{
    set_current_wp(wpSetClient,seq.begin()->wp_index);
    ros::Rate rate(5);
    while(mission_reached.wp_seq != seq.begin()->wp_index)
    {
        ROS_INFO("Waitng For Approach WP number %d .... " , seq.begin()->wp_index);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("WP number %d Approached " , seq.begin()->wp_index );
    ROS_INFO("WP action is  %d",seq.begin()->action );
    ros::Duration(5.0).sleep();

    switch (seq.begin()->action)
    {
         case POSITION_HOLD:
            {

                mavros_msgs::SetMode offboard_mode;
                offboard_mode.request.custom_mode = "OFFBOARD";

                while ( !set_mode_client.call(offboard_mode) || !offboard_mode.response.mode_sent)
                {
                    ROS_INFO("Switching to OFFBOARD");
                    ros::Duration(0.5).sleep();
                }

                ros::Duration(60).sleep();

                seq.erase(seq.begin());
                mission = WPS;
            };
            break;

        case DELIVERY_ACTION:
            {
                seq.erase(seq.begin());
                std_srvs::SetBool msg;
                /////////////////
                msg.request.data = true ;
                drop_client.call(msg);
                ros::Duration(0.5).sleep();
                drop_client.call(msg);
                ros::Duration(0.5).sleep();
                drop_client.call(msg);
                ros::Duration(2.0).sleep();
                /////////////////
                msg.request.data = false;
                drop_client.call(msg);
                ros::Duration(0.5).sleep();
                drop_client.call(msg);
                ros::Duration(2.0).sleep();
                /////////////////

                mission = WPS;
            };
            break;
        case FRAGILE_DELIVERY_ACTION:
            {
                seq.erase(seq.begin());
                mission = WPS;
            };
            break;
        case SHOT_ACTION:
            {
                shotTimer.start();
                seq.erase(seq.begin());
                mission = WPS;
            };
            break;
        case SHOT_END_ACTION:
            {
                shotTimer.stop();
                seq.erase(seq.begin());
                mission = WPS;
            };
            break;
        case LAND_ACTION:
            {
                seq.erase(seq.begin());
                mission = LAND_INIT;
            };
            break;
        case NONE:
            {
                seq.erase(seq.begin());
                mission = WPS;
            };
            break;
        default:
            break;
    }
}
