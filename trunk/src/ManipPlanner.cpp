#include "ManipPlanner.hpp"

ManipPlanner::ManipPlanner(ManipSimulator * const manipSimulator)
{
    m_manipSimulator = manipSimulator;   
}

ManipPlanner::~ManipPlanner(void)
{
    //do not delete m_simulator  
}

void ManipPlanner::ConfigurationMove(double &deltaTheta, double &baseDeltaX, double &baseDeltaY)
{

  // add your code here!!
	// final position of base_x, base_y: 5.6, 1.6
	// we are assuming the orientation of insertion is already optimal; baseDeltaY will probably never be used here
	// simulator keeps track of restriction on order of joint movement and joint limits
	// not exactly the same as retracting a stylus but a simplified(?) equivalent
	deltaTheta = -0.01;
   
}

