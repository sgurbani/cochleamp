#include "ManipPlanner.hpp"
using namespace std;

ManipPlanner::ManipPlanner(ManipSimulator * const manipSimulator)
{
    m_manipSimulator = manipSimulator;   
    
    //initialize maxmimum imaging depth of our OCT probe
    MAX_OCT_DEPTH = 1.5;
    
    //initialize angle bandwidth of OCT probe
    ANGLE_BANDWIDTH = 1.0/36 * M_PI;       //have a sensitivity of +/- 5 deg
    
    //initialize retraction coefficient to 0 (ie: stylus fully inserted)
    retractionCoeff = 0;
    
    //initialize sensedObstacles, which lets us build our potential field
    sensedObstacles.clear();
    for(int i=0; i<m_manipSimulator->GetNrObstacles(); i++)
    {
        sensedObstacles.push_back(false);
    }
    
    //initialize our algorithm to stage 0
    stage = 0;
    
    //initialize repulsive force parameters
    alpha = 1;
    gamma = 10;
    Q = 2;
    
    //intialize other vars
    sensedPoints.clear();
}

ManipPlanner::~ManipPlanner(void)
{
    //do not delete m_simulator  
}

void ManipPlanner::ConfigurationMove(double &deltaTheta, double &baseDeltaX, double &baseDeltaY)
{
    //always get OCT data
    OCTData oct = ScanOCT();
    
    switch(stage)
    {
        case 0:
            //IF WE'RE IN STAGE 0, JUST MOVE FORWARD UNTIL WE SENSE SOMETHING IN FRONT OF US
            //check to see if something in front of us
            for(int i=0; i<oct.NrScans; i++)
            {
                double a = oct.angle[i];
                if(a == 0)
                {
                    //uh oh, something in front of us
                    //switch stage and wait until next iteration
                    stage = 1;
                    deltaTheta = 0;
                    baseDeltaX = 0;
                    baseDeltaY = 0;
                    return;
                }
            }
            
            //okay nothing in front of us
            //keep moving in the +x direction
            deltaTheta = 0;
            baseDeltaX = 0.01;
            baseDeltaY = 0;
            break;

       // case 1:
            //AT STAGE 1, WE HAVE SENSED TISSUE IN FRONT OF US, BUT HAVE NOT
            //STARTED TO BEND THE PROBE YET.
            
        //    break;
        
        default:
            deltaTheta = m_manipSimulator->GetLinkThetaLimit(0) / 75;
            baseDeltaX = 0.015;
            break;
    }
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
 * Return true if retractionCoeff == NrLinks - i
 */
bool ManipPlanner::CanLinkBend(int i)
{
    return retractionCoeff == m_manipSimulator->GetNrLinks() - i;
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
            //our link (within a margin ANGLE_BANDWIDTH).
            
            //angle w.r.t. our link
            double phi = GetAngleToPoint(p) - GetAngleFromXAxis(m_manipSimulator->GetNrLinks()-1);
                        
            if(fabs(phi) < ANGLE_BANDWIDTH || fabs(phi-0.5*M_PI) < ANGLE_BANDWIDTH || fabs(phi+0.5*M_PI) < ANGLE_BANDWIDTH)  //it's directy in front of us OR orthogonal to our link
            {
                //add it to the OCTData
                data.NrScans++;
                data.depth.push_back(DistanceBetweenPoints(e, p));
                
                //if we're in "front", push a 0 as the angle
                //if we're to the side, push -1 for "left", +1 for "right"
                if(fabs(phi) < ANGLE_BANDWIDTH)
                    data.angle.push_back(0);
                else if(fabs(phi-0.5*M_PI) < ANGLE_BANDWIDTH)
                    data.angle.push_back(-1);
                else
                    data.angle.push_back(1);
                
                //add to sensedPoints for debugging
                sensedPoints.push_back(i);
                
                //add to our sensedObstacles data, to build our potential field
                sensedObstacles[i] = true;
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



double* ManipPlanner::RepulsiveCSFAtLink(int j)
{
    //get the endpoints of link j
    double px = m_manipSimulator->GetLinkEndX(j);
    double py = m_manipSimulator->GetLinkEndY(j);
    
    //get the number of links
    int N = m_manipSimulator->GetNrLinks();
    
    //get number of obstacles
    int O = m_manipSimulator->GetNrObstacles();
    
    //initialize config space force variable
    double* totalCSF = new double[N+2];
    
    for(int k=0; k<N+2; k++)
    {
        totalCSF[k] = 0;
    }
    
    
    for(int i=0; i<O; i++)
    {
        //get the force acting on link j from obstacle i
        Point force = RepulsiveForceAtPointFromObstacle(px, py, i);
        
        //convert the workspace force into a cspace force
        //IMPLEMENT THIS
        double* csfI = WSF2CSF(force, j);
        
        //add to the total force
        for(int k=0; k<N+2; k++)
        {
            totalCSF[k] += csfI[k];
        }
    }
    
    return totalCSF;
}


Point ManipPlanner::RepulsiveForceAtPointFromObstacle(double x, double y, int i)
{
    //calculate the repulsive force to point px,py from obstacle i
    
    //get the obstacle closest point
    double ox = m_manipSimulator->ClosestPointOnObstacle(i, x, y).m_x;
    double oy = m_manipSimulator->ClosestPointOnObstacle(i, x, y).m_y;
    
    //get distance to obstacle
    double d = sqrt(pow(ox-x,2) + pow(oy-y,2));
    
    
    Point force;
    
    //Use an exponential repusulsive force gradient (in the workspace), based on
    //something mentioned by Volpe, et al.
    //The potential function we choose causes very high gradients near the obstacle,
    //because of a 1/d^2 term. Additionally, the force decays quite fast so that
    //the manipulator is not affected far away. We have more granular control because
    //we are able to tweak the overall force at dist~1 by scaling gamma, and the decay
    //rate by modifying alpha.
    //
    //The gradient I calculate is:
    //Force = -gamma * exp(-alpha * d) * (1/d^2 + alpha/d) * [pt - obs]
    //where d is the scalar distance between pt and obstacle
    //
    //if d > Q, then force is 0
    if(d > Q)
    {
        force.m_x = 0;
        force.m_y = 0;
    }
    else
    {
        double forceScale = -gamma * exp(-alpha * d) * (1/pow(d,2) + alpha/d);
        force.m_x = (x-ox) * forceScale;
        force.m_y = (y-oy) * forceScale;
    }
    
    return force;
}

double* ManipPlanner::WSF2CSF(Point force, int j)
{
    //get the number of links
    int N = m_manipSimulator->GetNrLinks();
    
    //get coordinates of the j-th link endpoint
    double jx = m_manipSimulator->GetLinkEndX(j);
    double jy = m_manipSimulator->GetLinkEndY(j);
    
    //prepare the Jacobian matrix
    //Jacobian is a 2 x (#links + 2) matrix
    //use 2 row vectors cuz 2D arrays are not fun :(
    double* jacX = new double[N+2];
    double* jacY = new double[N+2];
    
    //for the first two columns of the Jac, we are dealing with base parameters
    jacX[0] = 1;
    jacX[1] = 0;
    jacY[0] = 0;
    jacY[1] = 1;
    
    
    //for the rest of the links, calculate Jacobian using angle approximation
    //shown in class, times our logic function for whether or not a link can
    //bend.
    for(int i=2; i<N+2; i++)
    {
        //get start point of (i-2)th joint
        double px = m_manipSimulator->GetLinkStartX(i-2);
        double py = m_manipSimulator->GetLinkStartY(i-2);
        
        jacX[i] = (-1*jy+py)*CanLinkBend(i-2);
        jacY[i] = (jx-px)*CanLinkBend(i-2);
    }
    
    //now, calculate the CSF from the WST
    //csf = jac_transpose * wsf
    
    double* csf = new double[N+2];
    
    double fx = force.m_x;
    double fy = force.m_y;
    
    for(int i=0; i<N+2; i++)
    {
        csf[i] = jacX[i]*fx + jacY[i]*fy;
    }
    
    return csf;
}

