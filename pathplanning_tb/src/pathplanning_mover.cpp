#include <pathplanning_tb/pathplanning_mover.hpp>

using namespace PathplanningTB;

RobotMover::RobotMover( )
: isGoalFinished( false )
, isPathExecuted( false )
{
    this->m_actionClient = std::make_shared<actionlib::SimpleActionClient<rns_msgs::MoveToAction>>( actionServerName.c_str(), true );
}

RobotMover::RobotMover(const std::list<goalPose> _goalPoses)
: RobotMover()
{
    this->m_goalPoses = _goalPoses;
}


RobotMover::~RobotMover()
{
    this->m_actionClient->cancelAllGoals();
    
}

int32_t RobotMover::executePath()
{
    ROS_INFO( "Executing path. Waiting for action server..." );
    this->m_actionClient->waitForServer();
    ROS_INFO( "Action server started, sending goals" );
    

    for( auto& poses : this->m_goalPoses )
    {
        //Converting goalPose to rns_msgs::MoveToGoal
        geometry_msgs::PoseStamped   goalPose;
    
        //Header
        //PoseStamped sequnce order number (can be set to 0)
        goalPose.header.seq      = 0;
        //Timestamp
        goalPose.header.stamp    = ros::Time::now();
        //Frame ID of the map
        goalPose.header.frame_id = "/map";
    
        //Pose coordinates
        goalPose.pose.position.x    = poses.pose.position.x;
        goalPose.pose.position.y    = poses.pose.position.y;
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
        
        //Sending goal
        ROS_INFO( "Moving to (%f, %f)...", poses.pose.position.x, poses.pose.position.y );
        this->m_actionClient->sendGoal( goalObjective );
        
        //Waiting for the goal to finish
        //WARNING: Move ros duration to params 
        bool finishedBeforeTimeout = this->m_actionClient->waitForResult( ros::Duration(60.0) );
        
        if( finishedBeforeTimeout )
        {
            actionlib::SimpleClientGoalState goalState = this->m_actionClient->getState();
            ROS_INFO( "Action finished: %s", goalState.toString().c_str() );
        }
        else
        {
            this->m_actionClient->cancelAllGoals();
            throw GoalExecutionFailed();
        }
    }
    ROS_INFO( "Path executed!" );
}

//TODO: Complete callbacks
void RobotMover::activeCallback()
{
}

void PathplanningTB::RobotMover::doneCallback(const actionlib::SimpleClientGoalState& _state, const rns_msgs::MoveToActionResultConstPtr& _result)
{
}

void PathplanningTB::RobotMover::feedbackCallback(const rns_msgs::MoveToActionFeedbackConstPtr& _feedback)
{

    
}
