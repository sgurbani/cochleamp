#include "ManipPlanner.hpp"
using namespace std;

ManipPlanner::ManipPlanner(ManipSimulator * const manipSimulator)
{
    m_manipSimulator = manipSimulator;   
    
    //initialize maxmimum imaging depth of our OCT probe
    MAX_OCT_DEPTH = 1;
    
    //initialize angle bandwidth of OCT probe
    ANGLE_BANDWIDTH = 1.0/180 * M_PI;       //have a sensitivity of +/- 1 deg
    
    //initialize retraction coefficient to 0 (ie: stylus fully inserted)
    retractionCoeff = 0;
    
    //intialize other vars
    sensedPoints.clear();
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
    ScanOCT();
	deltaTheta = -0.01;
    baseDeltaX = 0.01;
    
    cout << sensedPoints.size() << endl;
}


/**
 * This function calculates the angle (in radians) from the cochlear tip 
 * position to the point P. The angle is defined from the horizontal x
 * x-axis, going counterclockwise.
 * Returns a value between 0 and 2PI.
 */
double ManipPlanner::GetAngleToPoint(Point p)
{
    //get the electrode tip
    Point e = GetElectrodeTip();
    
    //calculate the angle between e and p using atan2
    double theta = atan2(p.m_y-e.m_y, p.m_x-e.m_x);
    
    //since atan2 returns between (-PI, PI)...if theta < 0, then add 2*PI
    if(theta < 0)
    {
        theta += 2*M_PI;
    }
    
    return theta;
    
}

/**
* Returns a Point with the x and y coordinates of the electrode tip.
*/
Point ManipPlanner::GetElectrodeTip(void)
{
    //find the number of links
    int i = m_manipSimulator->GetNrLinks();
    
    //get the x and y positions of the (i-1)-th link
    double x = m_manipSimulator->GetLinkEndX(i-1);
    double y = m_manipSimulator->GetLinkEndY(i-1);
    
    //and package them into a Point and return
    Point p;
    p.m_x = x;
    p.m_y = y;
    
    return p;
}

/**
 * Determines whether the i-th link is able to bend based on the
 * value of the retraction coefficient.
 *
 * Return true if retractionCoeff >= NrLinks - i
 */
bool ManipPlanner::CanLinkBend(int i)
{
    return retractionCoeff >= m_manipSimulator->GetNrLinks() - i;
}

/**
 * Returns the Euclidean distance between points a and b
 */
double ManipPlanner::DistanceBetweenPoints(Point a, Point b)
{
    return sqrt(pow(a.m_x-b.m_x,2) + pow(a.m_y-b.m_y,2));
    
}

/**
* This method mimics an "OCT sweep" around the electrode. In effect,
* it will provide depth and angle information for all objects that can
* be sensed from the electrode tip (where our imaging probe is located).
* It returns an OCTData struct consisting of all the points on obstacles
* that can be detected.
*/
OCTData ManipPlanner::ScanOCT(void)
{
    //initialize vars
    OCTData data;
    data.NrScans = 0;
    sensedPoints.clear();
    
    //get the electrode tip position
    Point e = GetElectrodeTip();
    double ex = e.m_x;
    double ey = e.m_y;
        
    //since the cochlea tissue is made up of lots and lots of tiny 
    //circular obstacles, we can basically get the closest point to all of
    //them, ignoring any whose closest point is greater than MAX_OCT_DEPTH
    int NrObs = m_manipSimulator->GetNrObstacles();
    
    for(int i=0;i<NrObs;i++)
    {
        //find the closest point to this obstacle, incorporating OCT depth
        Point p = m_manipSimulator->ClosestPointOnObstacleAtMaxDist(i, ex, ey, MAX_OCT_DEPTH);
        
        //check to see if it's within our sensing depth
        if(p.m_x < 0.5*HUGE_VAL && p.m_y < 0.5*HUGE_VAL)
        {
            //we're good!
            
            //now, check to see what angle it makes with our link
            //we can only use it if is in front of us or directly orthogonal to
            //our link (within a margin ANGLE_BANDWIDTH ~= 1/20*PI).
            
            //angle w.r.t. our link
            double phi = GetAngleToPoint(p) - GetAngleFromXAxis(m_manipSimulator->GetNrLinks()-1);
            
            cout << "angle: " << abs(phi) << endl;
            
            if(fabs(phi) < ANGLE_BANDWIDTH || fabs(phi-0.5*M_PI) < ANGLE_BANDWIDTH || fabs(phi+0.5*M_PI) < ANGLE_BANDWIDTH)  //it's directy in front of us OR orthogonal to our link
            {
                cout << "IN HERE" << endl;
                //add it to the OCTData
                data.NrScans++;
                data.depth.push_back(DistanceBetweenPoints(e, p));
                data.angle.push_back(GetAngleToPoint(p));
                
                //add to sensedPoints for debugging
                sensedPoints.push_back(i);
            }
        }
    }
    
    return data;
}

/**
 * This function returns the angle of joint i with respect to the horizontal axis.
 * It returns a value between 0 and 2 PI.
*/
double ManipPlanner::GetAngleFromXAxis(const int j)
{
    double angle = 0;   //will hold the sum of all joint angles up to i
    
    //sum over all joint angles
    for(int i=0; i<=j; i++)
    {
        angle += m_manipSimulator->GetLinkTheta(i);
    }
    
    //angle will always be negative
    
    //truncate to [0, 2+PI]
    while(angle < 0)
    {
        angle += 2*M_PI;
    }
    
    return angle;
}