#ifndef PTHPLNNNG_MVR_HPP
#define PTHPLNNNG_MVR_HPP

#include <string>
#include <exception>

#include <pathplanning_tb/structs.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <rns_msgs/MoveToAction.h>

namespace PathplanningTB
{
    class RobotMover
    {
        public:
            const std::string actionServerName = "move_action";
            
            //Default constructor
            RobotMover( );
            // Minimal constructor
            RobotMover( const std::list<goalPose> _goalPoses );
            
            //Default destructor
            ~RobotMover();
            //Execute path (move robot)
            int32_t executePath( );
            
            //Callbacks for actionClient
            //TODO: Rewrite main exec function to work with callbacks
            void doneCallback( const actionlib::SimpleClientGoalState& _state
                             , const rns_msgs::MoveToActionResultConstPtr& _result);
            
            void activeCallback(  );
            void feedbackCallback( const rns_msgs::MoveToActionFeedbackConstPtr& _feedback );
    
    
    
    
        private:
            //Action client
            std::shared_ptr< actionlib::SimpleActionClient< rns_msgs::MoveToAction > >  m_actionClient;
            //Goal positions
            std::list<goalPose>                                                         m_goalPoses;
            
            // Logical params
            bool        isGoalFinished;
            bool        isPathExecuted;
            
    };
    
       
    struct GoalExecutionFailed : public std::exception
    {
        const char* what() const throw()
        {
            return "Goal execution failed";
        }
    };
    
    
}//namespace PathplanningTB


#endif
