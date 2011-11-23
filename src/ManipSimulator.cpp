#include "ManipSimulator.hpp"

ManipSimulator::ManipSimulator(const char fname[])
{
	base_x = -8;
	base_y = 2;
    m_positions.push_back(base_x);
    m_positions.push_back(base_y);

    m_circles.push_back(5);
    m_circles.push_back(0.6);
    m_circles.push_back(0.2);    

    SetupFromFile(fname);
}

ManipSimulator::~ManipSimulator(void)
{
}

bool ManipSimulator::HasRobotReachedGoal(void) const
{
    const double ex = GetLinkEndX(GetNrLinks() - 1);
    const double ey = GetLinkEndY(GetNrLinks() - 1);
    
    return
	sqrt((ex - GetGoalCenterX()) * (ex - GetGoalCenterX()) +
	     (ey - GetGoalCenterY()) * (ey - GetGoalCenterY())) < GetGoalRadius();
}

void ManipSimulator::AddLink(const double length)
{
    m_joints.push_back(0);
    m_lengths.push_back(length);
    m_positions.resize(m_positions.size() + 2);	
}

Point ManipSimulator::ClosestPointOnObstacle(const int i, const double x, const double y)
{
    const double cx = GetObstacleCenterX(i);
    const double cy = GetObstacleCenterY(i);
    const double r  = GetObstacleRadius(i);
    const double d  = sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y));

    Point p;
    
    p.m_x = cx + r * (x - cx) / d;
    p.m_y = cy + r * (y - cy) / d;

    return p;
    
}

void MatrixMultMatrix(const double M1[6], const double M2[6], double M[6])
{
    double Mresult[6];

    Mresult[0] = M1[0] * M2[0] + M1[1] * M2[3];
    Mresult[1] = M1[0] * M2[1] + M1[1] * M2[4];
    Mresult[2] = M1[0] * M2[2] + M1[1] * M2[5] + M1[2];

    Mresult[3] = M1[3] * M2[0] + M1[4] * M2[3];
    Mresult[4] = M1[3] * M2[1] + M1[4] * M2[4];
    Mresult[5] = M1[3] * M2[2] + M1[4] * M2[5] + M1[5];
    
    M[0] = Mresult[0];
    M[1] = Mresult[1];
    M[2] = Mresult[2];
    M[3] = Mresult[3];
    M[4] = Mresult[4];
    M[5] = Mresult[5];
}

void ManipSimulator::FK(void)
{
    const int n = GetNrLinks();
    
    double M[6];
    double Mall[6];
    double p[2] = {0, 0};
    double pnew[2];
    
    
    Mall[0] = Mall[4] = 1;
    Mall[1] = Mall[2] = Mall[3] = Mall[5] = 0;
    
    for(int i = 0; i < n; ++i)
    {
	const double ctheta = cos(GetLinkTheta(i));
	const double stheta = sin(GetLinkTheta(i));
	
	M[0] = ctheta;  M[1] = -stheta; M[2] = GetLinkLength(i) * ctheta;
	M[3] = stheta;  M[4] =  ctheta; M[5] = GetLinkLength(i) * stheta;

	MatrixMultMatrix(Mall, M, Mall);

	m_positions[2 * i + 2] = Mall[2] + base_x;
	m_positions[2 * i + 3] = Mall[5] + base_y;;
    }
}

void ManipSimulator::SetupFromFile(const char fname[])
{
	//file with obstacles (x y r)
	FILE *in = fopen(fname, "r");
	if(in)
	{
		int nrObstacles;
		double x; double y; double r;

		if(fscanf(in, "%d", &nrObstacles) != 1)
		{
			printf("error: expecting number of obstacles\n");
			fclose(in);
			return;
		}

		for(int i=0; i<nrObstacles; i++)
		{
			if(fscanf(in, "%lf %lf %lf", &x, &y, &r) != 3)
			{
				printf("invalid obstacle definition, expecting x y r\n");
				fclose(in);
				return;
			}
			m_circles.push_back(x);
			m_circles.push_back(y);
			m_circles.push_back(r);
		}
	}
}
