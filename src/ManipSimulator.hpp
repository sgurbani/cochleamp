#ifndef MANIP_SIMULATOR_HPP_
#define MANIP_SIMULATOR_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

struct Point
{
    double m_x;
    double m_y;    
};
    

class ManipSimulator
{
public:    
    ManipSimulator(const char fname[]);
    
    ~ManipSimulator(void);
    
    double GetGoalCenterX(void) const
    {
	return m_circles[0];	
    }

    double GetGoalCenterY(void) const
    {
	return m_circles[1];	
    }

    int GetNrObstacles(void) const
    {
	return m_circles.size() / 3 - 1;
    }

    /**
     *@brief Returns closest point on the i-th circle obstacle to point [x, y]
     */
    Point ClosestPointOnObstacle(const int i, const double x, const double y);
    
    /**
     *@brief Returns closest point on the i-th circle obstacle to point [x, y] if the closest point
     *       is at distance <= dist
     */
    Point ClosestPointOnObstacleAtMaxDist(const int i, const double x, const double y, const double dist);

    int GetNrLinks(void) const
    {
	return m_joints.size();
    }

    double GetLinkStartX(const int i) const
    {
	return m_positions[2 * i];
    }

    double GetLinkStartY(const int i) const
    {
	return m_positions[2 * i + 1];
    }

    double GetLinkEndX(const int i) const
    {
	return GetLinkStartX(i + 1);
    }

    double GetLinkEndY(const int i) const
    {
	return GetLinkStartY(i + 1);
    }
    
    double GetLinkTheta(const int i) const
    {
        return m_joints[i];
    }

    double GetLinkThetaLimit(const int i) const
    {
        return theta_limits[i];
    }

    int GetCurrentLink(void) const
    {
	    //checks which link is currently active for bending; returns -1 if fully bent
	    for(int i=0; i<this->GetNrLinks(); i++)
		    if(this->GetLinkTheta(i) < this->GetLinkThetaLimit(i))
			    return i;
	    return -1;
    }

    bool HasRobotReachedGoal(void) const;

protected:

    double GetObstacleCenterX(const int i) const
    {
	return m_circles[3 * i + 3];
    }
    
    double GetObstacleCenterY(const int i) const
    {
	return m_circles[3 * i + 4];
    }
    
    double GetObstacleRadius(const int i) const
    {
	return m_circles[3 * i + 5];
    }

    double GetLinkLength(const int i) const
    {
	return m_lengths[i];
    }

    double GetGoalRadius(void) const
    {
	return m_circles[2];
    }
    
    void FK(void);


protected:

    void AddLink(const double length);
    
    void AddToLinkTheta(const int i, const double dtheta)
    {
	m_joints[i] += dtheta;
    }

    void AddToLinkTheta(const double dtheta);

    /**
     *@brief Read circular obstacles from input file
     *
     *@param fname name of file with obstacle information
     */
    void SetupFromFile(const char fname[]);

    std::vector<double> m_joints;
    std::vector<double> m_lengths;
    std::vector<double> m_positions;
    std::vector<double> m_circles;

    std::vector<double> theta_limits;

    double base_x;
    double base_y;
    
    friend class Graphics;
};

#endif
