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
    
    while( ros::ok() )
    {
        try
        {
            PathplanningTB::PathplanningRos ppRos(nh);
            ppRos.getOccupancyMap( );
            ppRos.getPosition( );
            ppRos.getGoal( );
            ppRos.setAgentSize( 0 );
            ppRos.planPath( true );
            ppRos.drawPath();
            //ppRos.executePath();
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
        catch( PathplanningTB::GoalExecutionFailed& e )
        {
            ROS_ERROR( "%s", e.what() );
        }
        catch( std::exception& e )
        {
            ROS_ERROR( "%s", e.what() );
        }
    }
   
    return 0;
}




































