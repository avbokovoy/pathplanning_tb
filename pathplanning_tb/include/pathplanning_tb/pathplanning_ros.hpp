#ifndef PTHPLNNNG_RS_HPP
#define PTHPLNNNG_RS_HPP

#include <string>
#include <exception>
#include <algorithm>

#include <pathplanning_tb/map.h>
#include <pathplanning_tb/theta.h>
#include <pathplanning_tb/structs.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rns_msgs/MoveToAction.h>

namespace PathplanningTB
{
    //** Topic names
    //const std::string mapMetaDataTopicName = "/map_metadata";
    const std::string mapDataTopicName                = "/map";
    const std::string goalMarkerTopicName             = "/move_base_simple/goal";
    const std::string robotPoseTopicName              = "/robot_pose";
    const std::string visualizationPublisherTopicName = "/pathplanning_tb/rviz_marker";  
    const std::string markerFrameId                   = "pathplanning_tb_path";
    
    class PathplanningRos
    {
        public:
            //Default constructor
            PathplanningRos();
            
            //Minimal constructor
            PathplanningRos( const ros::NodeHandle& _nodeHandler );
            
            //Main constructor
            //PathplanningRos( int _argc, char** _argv, const std::string& _nodeName );
            
            //Default destructor 
            ~PathplanningRos();
            
            // Gets current position and converts it to Pathplanning algorithm datatype
            int32_t getPosition( const std::string& _robotPoseTopicName = robotPoseTopicName
                               , const ros::Duration& _timeout = ros::Duration( 10.0 ) );
            // Gets goal from RViz. 
            int32_t getGoal( const std::string& _goalMarkerTopicName = goalMarkerTopicName
                           , const ros::Duration& _timeout           = ros::Duration( 0.0 ) );
            // Gets the map whenever it's available on topic. Default timeout is 10 seconds
            int32_t getOccupancyMap( const std::string& _mapDataTopicName = mapDataTopicName 
                                   , const ros::Duration& _timeout = ros::Duration( 10.0 ) );
            // Setting agent size
            int32_t setAgentSize( const float& _agentSize = 1.0 );
            // Executes pathplanning algorithm
            int32_t planPath();
            // Drawing path in RViz
            int32_t drawPath( const std::string& _frameId = markerFrameId )const;
            //Main function
            int32_t executePath( )const;
            
        private:
            // Pathplanning theta* related data
            std::shared_ptr<OccupancyGrid>   m_ppOccupancyGrid;
            std::shared_ptr<goalPose>        m_ppGoalPose;
            std::shared_ptr<goalPose>        m_ppStartPose;
            std::shared_ptr<Map>             m_ppMap;
            std::shared_ptr<Theta>           m_ppTheta;
            std::shared_ptr<SearchResult>    m_ppSearchResult;
            float                            m_ppAgentSize;
            std::list<goalPose>              m_ppPathPoints;
            
            // ROS nodehandler
            std::shared_ptr<ros::NodeHandle> m_nodeHandler;
            // ROS visualization topic 
            std::shared_ptr<ros::Publisher>  m_visualizationPublisher;
            
            // ROS topic names
            std::string                      m_mapDataTopicName;
            std::string                      m_goalMarkerTopicName;
            std::string                      m_robotPoseTopicName;
    };
    
    struct WaitForMessageTimeout : public std::exception
    {
        const char* what() const throw()
        {
            return "waitForMessage() timed out";
        }
    };
    
    struct AgentSizeLessThanZero : public std::exception
    {
        const char* what() const throw()
        {
            return "agentSize is less than zero";
        }
    };
    
    struct PathNotFound : public std::exception
    {
        const char* what() const throw()
        {
            return "path not found";
        }
        
    };
    
    namespace Utils
    {
        //Adds marker to marker array
        uint32_t addMarker( visualization_msgs::Marker& _inMarker
                          , const float&   _posX
                          , const float&   _posY );
    }//namespace Utils
    
}//namespace PathplanningTB

#endif
