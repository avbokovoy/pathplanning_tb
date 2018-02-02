#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

class ogToGmConverter
{
    public:
        ogToGmConverter( const ros::NodeHandle _nh )
        : m_nodeHandler( _nh )
        {
            this->m_gridMapPublisher        = this->m_nodeHandler.advertise< grid_map_msgs::GridMap >( "og_to_gm", 10 );
            this->m_occupancyGridSubscriber = this->m_nodeHandler.subscribe( "map", 10, &ogToGmConverter::occupancyGridCallback, this );
        
        };
        
        ~ogToGmConverter()
        {
            ROS_INFO( "ogToGmConverter destroyed" );
        };
        
        void occupancyGridCallback( const nav_msgs::OccupancyGrid &_msg )
        {
            ROS_INFO( "Map recieved" );
            //grid_map::GridMapRosConverter    tmpConverter;
            
            nav_msgs::OccupancyGrid tmpOccupancyGrid( _msg );
            
            //Scale down
            /*for( unsigned int i = 0; i < tmpOccupancyGrid.info.width * tmpOccupancyGrid.info.height - 1; ++i )
            {
                if( tmpOccupancyGrid.data[i] != -1 )
                {
                    tmpOccupancyGrid.data[i] = tmpOccupancyGrid.data[i] / 100.0;
                }
            }*/
            
            //Convert to GridMap
            grid_map::GridMap tmpGridMap;
            grid_map::GridMapRosConverter::fromOccupancyGrid( tmpOccupancyGrid, this->m_layer, tmpGridMap );
            
            //Scale down
            for( grid_map::GridMapIterator it( tmpGridMap ); !it.isPastEnd(); ++it )
            {
                tmpGridMap.at( this->m_layer, *it ) /= 200.0;
            }
            
            //Convert to GridMapMsg
            grid_map_msgs::GridMap tmpGridMsg;
            grid_map::GridMapRosConverter::toMessage( tmpGridMap, tmpGridMsg );
            
            
            
            
            
            
            this->m_gridMapPublisher.publish( tmpGridMsg );
        };
        
    private:
        ros::NodeHandle     m_nodeHandler;
        ros::Publisher      m_gridMapPublisher;
        ros::Subscriber     m_occupancyGridSubscriber;
        const std::string   m_layer = "elevation";
};

int main( int argc, char** argv )
{
    ROS_INFO( "Starting og_to_gm node" );
    ROS_INFO( "This node converts nav_msgs::OccupancyGrid to grid_map_msgs::GridMap for better visualization" );
    
    ros::init( argc, argv, "og_to_om" );
    
    ros::NodeHandle nodeHandler;
    
    ogToGmConverter converter( nodeHandler );
    
    ros::spin();
    
    return 0;
}
