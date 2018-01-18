#include <pathplanning_tb/pathplanning_ros.hpp>

using namespace PathplanningTB;

PathplanningRos::PathplanningRos()
: m_ppOccupancyGrid( new OccupancyGrid )
, m_ppGoalPose( new goalPose )
, m_ppStartPose( new goalPose )
, m_ppMap( new Map )
, m_ppTheta( new Theta )
, m_ppSearchResult( new SearchResult )
, m_nodeHandler( new ros::NodeHandle )
, m_visualizationPublisher( new ros::Publisher )
{
    //this->m_ppOccupancyGrid = std::shared_ptr<OccupancyGrid>( { 0 } );
}

PathplanningRos::PathplanningRos( const ros::NodeHandle& _nodeHandler )
: PathplanningRos()
{
    this->m_nodeHandler             = std::make_shared<ros::NodeHandle>( _nodeHandler );
    this->m_visualizationPublisher  = std::make_shared<ros::Publisher>( this->m_nodeHandler->advertise<visualization_msgs::Marker>
                                                                                    ( visualizationPublisherTopicName.c_str(), 50 ) );
}


PathplanningRos::~PathplanningRos()
{
    delete [] this->m_ppOccupancyGrid->data;
}

int32_t PathplanningRos::getPosition( const std::string& _robotPoseTopicName
                                    , const ros::Duration& _timeout )
{
    this->m_robotPoseTopicName = _robotPoseTopicName;
    ROS_INFO( "Waiting for robot ropisition (topic %s)...", this->m_robotPoseTopicName.c_str() );
    nav_msgs::OdometryConstPtr robotPose = ros::topic::waitForMessage<nav_msgs::Odometry>( this->m_robotPoseTopicName.c_str()
                                                          , _timeout );
    
    if( robotPose == nullptr )
    {
        ROS_ERROR( "Robot pose is not recieved (timed out)" );
        throw WaitForMessageTimeout();
    }
    ROS_INFO( "Robot position (global):" );
    ROS_INFO( "-- x: %f", ( *robotPose ).pose.pose.position.x );
    ROS_INFO( "-- y: %f", ( *robotPose ).pose.pose.position.y );
    
    //Converting to pathplanning coordinate system
    this->m_ppStartPose->pose.position.x = ( *robotPose ).pose.pose.position.x + this->m_ppOccupancyGrid->info.origin.position.x;
    this->m_ppStartPose->pose.position.y = ( *robotPose ).pose.pose.position.y + this->m_ppOccupancyGrid->info.origin.position.y;
    ROS_INFO( "Robot position (pathplanning): " );
    ROS_INFO( "-- x: %f", this->m_ppStartPose->pose.position.x );
    ROS_INFO( "-- y: %f", this->m_ppStartPose->pose.position.y );
}

int32_t PathplanningRos::getGoal( const std::string& _goalMarkerTopicName
                                , const ros::Duration& _timeout )
{
    this->m_goalMarkerTopicName = _goalMarkerTopicName;
    ROS_INFO( "Waiting for goal from RViz (topic %s)", this->m_goalMarkerTopicName.c_str() );
    geometry_msgs::PoseStampedConstPtr goalPose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>( this->m_goalMarkerTopicName.c_str()
                                                                                                        , _timeout );
    if( goalPose == nullptr )
    {
        ROS_ERROR( "Goal pose is not recieved (timed out)" );
        throw WaitForMessageTimeout();
    }
    ROS_INFO( "Goal position (global): " );
    ROS_INFO( "-- x: %f", ( *goalPose ).pose.position.x );
    ROS_INFO( "-- y: %f", ( *goalPose ).pose.position.y );
    
    // Converting goal position into pathplanning coordinate system
    this->m_ppGoalPose->pose.position.x = ( *goalPose ).pose.position.x + this->m_ppOccupancyGrid->info.origin.position.x;
    this->m_ppGoalPose->pose.position.y = ( *goalPose ).pose.position.y + this->m_ppOccupancyGrid->info.origin.position.y;
    ROS_INFO( "Goal position (pathplanning): " );
    ROS_INFO( "-- x: %f", this->m_ppGoalPose->pose.position.x );
    ROS_INFO( "-- y: %f", this->m_ppGoalPose->pose.position.y );
    
}


int32_t PathplanningRos::getOccupancyMap( const std::string& _mapDataTopicName
                                        , const ros::Duration& _timeout )
{
    this->m_mapDataTopicName = _mapDataTopicName;
    //Getting the map and the metadata from ros topic
    ROS_INFO( "Waiting for map..." );
    nav_msgs::OccupancyGridConstPtr mapData = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>( this->m_mapDataTopicName.c_str(), _timeout );
    
    if( mapData == nullptr )
    {
        ROS_ERROR( "Map data is not recieved (timed out)" );
        throw WaitForMessageTimeout();
    }
    ROS_INFO( "Map metadata: " );
    ROS_INFO( "-- Width  (cells)      : %d"        , ( *mapData ).info.width );
    ROS_INFO( "-- Height (cells)      : %d"        , ( *mapData ).info.height );
    ROS_INFO( "-- Resolution (m/cells): %f"        , ( *mapData ).info.resolution );
    ROS_INFO( "-- Origin (map)        : ( %f, %f )", ( *mapData ).info.origin.position.x
                                                   , ( *mapData ).info.origin.position.y );
    //Assigning map data to original pathplanotithm datatypes
    this->m_ppOccupancyGrid->info.width              = ( *mapData ).info.width;
    this->m_ppOccupancyGrid->info.height             = ( *mapData ).info.height;
    this->m_ppOccupancyGrid->info.resolution         = ( *mapData ).info.resolution;
    
    // If resolution is greater then 1 we multiply global origin by this resolution, else - we divide
    //WARNING: Check if resolution >= 1
    if( this->m_ppOccupancyGrid->info.resolution >= 1 )
    {
        this->m_ppOccupancyGrid->info.origin.position.x  = - ( *mapData ).info.origin.position.x * this->m_ppOccupancyGrid->info.resolution;
        this->m_ppOccupancyGrid->info.origin.position.y  = - ( *mapData ).info.origin.position.y * this->m_ppOccupancyGrid->info.resolution;
    }
    else
    {
        this->m_ppOccupancyGrid->info.origin.position.x  = - ( *mapData ).info.origin.position.x / this->m_ppOccupancyGrid->info.resolution;
        this->m_ppOccupancyGrid->info.origin.position.y  = - ( *mapData ).info.origin.position.y / this->m_ppOccupancyGrid->info.resolution;
    }
    ROS_INFO( "-- Origin              : ( %f, %f )", this->m_ppOccupancyGrid->info.origin.position.x
                                                   , this->m_ppOccupancyGrid->info.origin.position.y );
    //Allocating memory for map data
    //!!! WARNING !!! This code may not be safe due to int8_t to int conversation
    uint32_t mapDataSize    = ( *mapData ).info.height * ( *mapData ).info.width;
    this->m_ppOccupancyGrid->data                    = new int[ mapDataSize ];
    for( uint32_t i = 0; i < mapDataSize - 1; ++i )
    {
        this->m_ppOccupancyGrid->data[i] = ( *mapData ).data[i];
    }
    
    return 0;
}

int32_t PathplanningTB::PathplanningRos::setAgentSize(const float& _agentSize)
{
    if( _agentSize < 0 )
    {
        throw AgentSizeLessThanZero();
    }
    this->m_ppAgentSize = _agentSize;
    ROS_INFO( "Agent size is set to %f", this->m_ppAgentSize );
}


int32_t PathplanningRos::planPath()
{
    ROS_INFO( "Starting pathplanning algorithm" );
    ROS_INFO( "Initializing map..." );
    this->m_ppMap->initialize  ( ( *this->m_ppOccupancyGrid ) );
    this->m_ppMap->setStartPos ( this->m_ppStartPose->pose.position );
    this->m_ppMap->setGoalPos  ( this->m_ppGoalPose->pose.position );
    this->m_ppMap->setAgentSize( this->m_ppAgentSize );
    
    ROS_INFO( "Starting path search..." );
    this->m_ppSearchResult = std::make_shared<SearchResult>( this->m_ppTheta->startSearch( ( *this->m_ppMap ) ) );
    if( this->m_ppSearchResult->pathfound )
    {
        ROS_INFO( "Path found!" );
        ROS_INFO( "-- Path length: %f", this->m_ppSearchResult->pathlength );
    }
    else
    {
        throw PathNotFound();
    }
    
    this->m_ppPathPoints = this->m_ppMap->getGoalPoses( ( *this->m_ppSearchResult ) );
    ROS_INFO( "Path points are: " );
    for( auto& point : this->m_ppPathPoints )
    {
        ROS_INFO( "---" );
        ROS_INFO( "x: %f", point.pose.position.x );
        ROS_INFO( "y: %f", point.pose.position.y );
        ROS_INFO( "---" );
    }
}

int32_t PathplanningRos::drawPath( const std::string& _frameId )const
{
    visualization_msgs::Marker visMarker;
    
    visMarker.header.frame_id    = _frameId;
    visMarker.header.stamp       = ros::Time::now();
    visMarker.ns                 = "pathplanning_tb";
    visMarker.id                 = 0;
    visMarker.type               = visualization_msgs::Marker::LINE_STRIP;
    visMarker.action             = visualization_msgs::Marker::ADD;
    visMarker.scale.x            = 0.1;
    visMarker.color.a            = 1.0;
    visMarker.color.r            = 1.0;
    visMarker.color.g            = 0.0;
    visMarker.color.b            = 0.0;
    
    //Creating vis markers for all the path points and forming MarkerArray with it. Converting coordinate system as well
    // Adding start point first
    Utils::addMarker( visMarker
                    , this->m_ppStartPose->pose.position.x - this->m_ppOccupancyGrid->info.origin.position.x
                    , this->m_ppStartPose->pose.position.y - this->m_ppOccupancyGrid->info.origin.position.y );
    for( auto& midPoints : this->m_ppPathPoints )
    {
        Utils::addMarker( visMarker
                        , midPoints.pose.position.x - this->m_ppOccupancyGrid->info.origin.position.x
                        , midPoints.pose.position.y - this->m_ppOccupancyGrid->info.origin.position.y );
    }
    /*Utils::addMarker( visMarker
                    , this->m_ppGoalPose->pose.position.x - this->m_ppOccupancyGrid->info.origin.position.x
                    , this->m_ppGoalPose->pose.position.y - this->m_ppOccupancyGrid->info.origin.position.y );*/
    
    this->m_visualizationPublisher->publish( visMarker );
}

int32_t PathplanningRos::execute()const
{
    //TODO: Complete and/or rename to robotFollowPath
    
    return 0;
}

uint32_t PathplanningTB::Utils::addMarker( visualization_msgs::Marker& _inMarker, const float& _posX, const float& _posY )
{
    geometry_msgs::Point tmpPoint;
    
    tmpPoint.x = _posX;
    tmpPoint.y = _posY;
    tmpPoint.z = 0;
    
    
    _inMarker.points.push_back( tmpPoint );
}














































