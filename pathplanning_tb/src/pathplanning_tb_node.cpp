#include <ros/ros.h>

#include <pathplanning_tb/pathplanning_ros.hpp>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <rns_msgs/MoveToAction.h>

#include <pathplanning_tb/structs.h>
#include <pathplanning_tb/map.h>
#include <pathplanning_tb/theta.h>

int main( int argc, char **argv )
{
    
    ROS_INFO( "Initializing pathplanning_tb_node..." );
    ros::init( argc, argv, "pathplanning_tb" );
    
    ros::NodeHandle nh;
    try
    {
        PathplanningTB::PathplanningRos ppRos(nh);
        ppRos.getOccupancyMap( );
        ppRos.getPosition( );
        ppRos.getGoal( );
        ppRos.setAgentSize( 1.0 );
        ppRos.planPath();
        ppRos.drawPath();
        ppRos.execute();
    }
    catch( PathplanningTB::WaitForMessageTimeout& e )
    {
        ROS_ERROR( "%s", e.what() );
    }
    catch( PathplanningTB::AgentSizeLessThanZero& e )
    {
        ROS_ERROR( "%s", e.what() );
    }
    catch( PathplanningTB::PathNotFound& e )
    {
        ROS_WARN( "%s", e.what() );
    }
    catch( std::exception& e )
    {
        ROS_ERROR( "%s", e.what() );
    }
    //ppRos.execute();
    //ROS_INFO( "Starting pathplanning_tb node." );
    /*
    //Creating action client
    actionlib::SimpleActionClient< rns_msgs::MoveToAction > actionClient( "move_action", true );
    
    ROS_INFO( "Waiting for action server." );
    actionClient.waitForServer();
    
    ROS_INFO( "Connected to server. Sending goal." );
    //================== Forming action message with goal ============================
    
    //Goal coordinates
    geometry_msgs::PoseStamped   goalPose;
    
    //Header
    //PoseStamped sequnce order number (can be set to 0)
    goalPose.header.seq      = 0;
    //Timestamp
    goalPose.header.stamp    = ros::Time::now();
    //Frame ID of the map
    goalPose.header.frame_id = "/map";
    
    //Pose coordinates
    goalPose.pose.position.x    = 0.4; //Move 0.4 meters forward
    goalPose.pose.position.y    = 0.0;
    goalPose.pose.position.z    = 0.0;
    
    goalPose.pose.orientation.x = 0.0;
    goalPose.pose.orientation.y = 0.0;
    goalPose.pose.orientation.z = 0.0;
    goalPose.pose.orientation.w = 1.0;
    
    
    rns_msgs::MoveToGoal goalObjective;
    //Goal's data
    goalObjective.goal            = goalPose;
    //Maximum distance for path search
    goalObjective.maxDistance     = 0.0;
    //Minimum distance to be reached
    goalObjective.minDistance     = 0.0;
    //Set to true if target's orientation is necessery
    goalObjective.oriented        = false;
    //Set to true if search can enter unknown areas
    goalObjective.canVisitUnknown = false;
    //================================================================================

    //Sending goal
    actionClient.sendGoal( goalObjective );
    
    //Waiting for action to be finished
    bool isActionFinished = actionClient.waitForResult( ros::Duration( 30.0 ) );
    
    if( isActionFinished )
    {
        actionlib::SimpleClientGoalState goalState = actionClient.getState();
        ROS_INFO( "Action finished: %s", goalState.toString().c_str() );
    }
    else
    {
        ROS_INFO( "Action timed out" );
    }
    */
    //ros::shutdown();
    
    return 0;
}




































