#ifndef MANIP_PLANNER_HPP_
#define MANIP_PLANNER_HPP_

#include "ManipSimulator.hpp"
#include <math.h>
#include <iostream>

#define _USE_MATH_DEFINES

using namespace std;

struct OCTData
{
    int NrScans;
    vector<double> depth;
    vector<double> angle;    
};

class ManipPlanner
{
public:
    ManipPlanner(ManipSimulator * const manipSimulator);
            
    ~ManipPlanner(void);

/*
 * This is the function that you should implement.
 * This function needs to compute by how much the link angles should change
 * so that the robot makes a small move toward the goal while avoiding
 * obstacles, as guided by the potential field.
 *
 * allLinksDeltaTheta(j) should contain the small delta change for the angle
 * associated with the j-th link.
 *
 * Note that the attractive potential should be defined only between the end
 * effector point on the manipulator and the goal center. 
 *
 * The repulsive potential on the other hand should be defined between each
 * obstacle and each link end.
 *
 * This will ensure that, when possible, the end effector moves toward the
 * goal while every link avoids collisions with obstacles.
 *
 * You have access to the simulator.
 * You can use the methods available in simulator to get all the information
 * you need to correctly implement this function
 *
 */
    void ConfigurationMove(double &deltaTheta, double &base_deltaX, double &base_deltaY);
    
        
protected:
    ManipSimulator  *m_manipSimulator;
    int retractionCoeff;
    
    double GetAngleToPoint(Point p);
    Point GetElectrodeTip(void);
    bool CanLinkBend(int i);
    
    double DistanceBetweenPoints(Point, Point);
    
    double GetAngleFromXAxis(const int i);
    
    OCTData ScanOCT(void);

    vector<int> sensedPoints;
    double MAX_OCT_DEPTH;
    double ANGLE_BANDWIDTH;
    vector<bool> sensedObstacles;
    
    //potential field functions
    Point RepulsiveForceAtPointFromObstacle(double, double, int);
    Point AttractiveForceToGoal();
    double* WSF2CSF(Point force, int j);
    double* RepulsiveCSFAtLink(int j);
    
    //repulsive force constants
    double alpha, gamma, Q;
    
    //attractive force constants
    double beta;
    
    //local minimum params
    
    int stage;
    
    friend class Graphics;
};

#endif
