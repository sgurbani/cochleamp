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


/**
 * This function calculates the angle (in radians) from the cochlear tip 
 * position to the point P. The angle is defined from the horizontal x
 * x-axis, going counterclockwise.
 * Returns a value between 0 and 2PI.
 */
double ManipPlanner::GetAngleToPoint(Point p)
{
    
    //to compile
    return 0.0;
    
}

/**
* Returns a Point with the x and y coordinates of the electrode tip.
*/
Point ManipPlanner::GetElectrodeTip(void)
{
    //find the number of links
    int i = m_manipSimulator->GetNrLinks();
    
    //get the x and y positions of the (i-1)-th link
    double x = m_manipSimulator->GetLinkEndX(i - 1);
    double y = m_manipSimulator->GetLinkEndX(i - 1);
    
    //and package them into a Point and return
    Point p;
    p.m_x = x;
    p.m_y = y;
    
    return p;
}