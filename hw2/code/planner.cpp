/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <unordered_set>
#include <queue>
#include <tuple>
#include <string>
#include <stdexcept>
#include <regex>	// For regex and split logic
#include <iostream> // cout, endl
#include <fstream>	// For reading/writing files
#include <assert.h>
#include <cstdlib>
// Include all the Planner header files

#include "RRTplan.h"
#include "PRM.h"
/* Input Arguments */
#define MAP_IN prhs[0]
#define ARMSTART_IN prhs[1]
#define ARMGOAL_IN prhs[2]
#define PLANNER_ID_IN prhs[3]

/* Planner Ids */
#define RRT 0
#define RRTCONNECT 1
#define RRTSTAR 2
#define PRM 3

/* Output Arguments */
#define PLAN_OUT plhs[0]
#define PLANLENGTH_OUT plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y * XSIZE + X)

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

// the length of each link in the arm
#define LINKLENGTH_CELLS 10

// Some potentially helpful imports
using namespace std;
using std::array;
using std::cout;
using std::endl;
using std::make_tuple;
using std::runtime_error;
using std::string;
using std::tie;
using std::tuple;
using std::vector;

/// @brief
/// @param filepath
/// @return map, x_size, y_size
tuple<double *, int, int> loadMap(string filepath)
{
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f)
	{
	}
	else
	{
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2)
	{
		throw runtime_error("Invalid loadMap parsing map metadata");
	}

	////// Go through file and add to m_occupancy
	double *map = new double[height * width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			char c;
			do
			{
				if (fscanf(f, "%c", &c) != 1)
				{
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0'))
			{
				map[y + x * width] = 1; // Note transposed from visual
			}
			else
			{
				map[y + x * width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string &str, const string &delim)
{
	// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
	const std::regex ws_re(delim);
	return {std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator()};
}

double *doubleArrayFromString(string str)
{
	vector<string> vals = split(str, ",");
	double *ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i)
	{
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double *v1, double *v2, int size)
{
	for (int i = 0; i < size; ++i)
	{
		if (abs(v1[i] - v2[i]) > 1e-3)
		{
			cout << endl;
			return false;
		}
	}
	return true;
}

typedef struct
{
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;

void ContXY2Cell(double x, double y, short unsigned int *pX, short unsigned int *pY, int x_size, int y_size)
{
	double cellsize = 1.0;
	// take the nearest cell
	*pX = (int)(x / (double)(cellsize));
	if (x < 0)
		*pX = 0;
	if (*pX >= x_size)
		*pX = x_size - 1;

	*pY = (int)(y / (double)(cellsize));
	if (y < 0)
		*pY = 0;
	if (*pY >= y_size)
		*pY = y_size - 1;
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
	params->UsingYIndex = 0;

	if (fabs((double)(p2y - p1y) / (double)(p2x - p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
	{
		params->Y1 = p1x;
		params->X1 = p1y;
		params->Y2 = p2x;
		params->X2 = p2y;
	}
	else
	{
		params->X1 = p1x;
		params->Y1 = p1y;
		params->X2 = p2x;
		params->Y2 = p2y;
	}

	if ((p2x - p1x) * (p2y - p1y) < 0)
	{
		params->Flipped = 1;
		params->Y1 = -params->Y1;
		params->Y2 = -params->Y2;
	}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX = params->X2 - params->X1;
	params->DeltaY = params->Y2 - params->Y1;

	params->IncrE = 2 * params->DeltaY * params->Increment;
	params->IncrNE = 2 * (params->DeltaY - params->DeltaX) * params->Increment;
	params->DTerm = (2 * params->DeltaY - params->DeltaX) * params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
	if (params->UsingYIndex)
	{
		*y = params->XIndex;
		*x = params->YIndex;
		if (params->Flipped)
			*x = -*x;
	}
	else
	{
		*x = params->XIndex;
		*y = params->YIndex;
		if (params->Flipped)
			*y = -*y;
	}
}

int get_next_point(bresenham_param_t *params)
{
	if (params->XIndex == params->X2)
	{
		return 0;
	}
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else
	{
		params->DTerm += params->IncrNE;
		params->YIndex += params->Increment;
	}
	return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double *map,
					   int x_size, int y_size)
{
	bresenham_param_t params;
	int nX, nY;
	short unsigned int nX0, nY0, nX1, nY1;

	// printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);

	// make sure the line segment is inside the environment
	if (x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	// printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	// iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do
	{
		get_current_point(&params, &nX, &nY);
		if (map[GETMAPINDEX(nX, nY, x_size, y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double *angles, int numofDOFs, double *map,
							int x_size, int y_size)
{
	double x0, y0, x1, y1;
	int i;

	// iterate through all the links starting with the base
	x1 = ((double)x_size) / 2.0;
	y1 = 0;
	for (i = 0; i < numofDOFs; i++)
	{
		// compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS * cos(2 * PI - angles[i]);
		y1 = y0 - LINKLENGTH_CELLS * sin(2 * PI - angles[i]);

		// check the validity of the corresponding line segment
		if (!IsValidLineSegment(x0, y0, x1, y1, map, x_size, y_size))
			return 0;
	}
	return 1;
}

double *RandomConfig(double *map, int numofDOFs, int x_size, int y_size)
{

	double *ans = new double[numofDOFs * sizeof(double)];
	bool flag = true;
	while (flag)
	{
		for (int i = 0; i < numofDOFs; i++)
		{
			double j = (double)(rand() % 360) / 360 * 2 * PI;
			ans[i] = j;
		}
		if (IsValidArmConfiguration(ans, numofDOFs, map, x_size, y_size))
		{
			flag = false;
			return ans;
			break;
		}
	}

	return ans;
}
double dist_angles(double *start, double *end, int numofDOFs)
{
	double dist = 0;
	for (int i = 0; i < numofDOFs; i++)
	{
		dist = dist + (start[i] - end[i]) * (start[i] - end[i]);
	}
	return sqrt(dist);
}
bool connect(double *v, double *q, double *map, int x_size, int y_size, int nDoF)
{
	// check if connection possible via interpolation
	// get vector interpolation b/w v and q
	// check if config is valid for each interpolated point
	double dist = 0;
	for (int i = 0; i < nDoF; i++)
	{
		if (dist < abs(v[i] - q[i]))
			dist = abs(v[i] - q[i]);
	}
	int k = (int)(dist / (PI / 40.0));
	double *temp = new double[nDoF];
	double *ret_angle = new double[nDoF];
	for (int i = 1; i < k; i++)
	{
		for (int j = 0; j < nDoF; j++)
		{
			temp[j] = v[j] + (q[j] - v[j]) * (((double)i) / (k - 1));
		}
		if (!IsValidArmConfiguration(temp, nDoF, map, x_size, y_size))
		{
			return false;
		}
	}
	return true;
}
double *extend(int n_id, double *random_config, double *map, int numofDOFs, int x_size, int y_size, double epsilon, vector<double *> vertices)
{
	double distance = 0;
	int i, j;
	double *start_angles = vertices[n_id];
	for (j = 0; j < numofDOFs; j++)
	{
		if (distance < fabs(start_angles[j] - random_config[j]))
			distance = fabs(start_angles[j] - random_config[j]);
	}
	int numofsamples = (int)(distance / (PI / 40));

	double *tmp_angles = new double[numofDOFs * sizeof(double)];
	double *saved_angles = NULL;

	for (i = 1; i < numofsamples; i++)
	{
		for (j = 0; j < numofDOFs; j++)
		{
			tmp_angles[j] = start_angles[j] + ((double)(i) / (numofsamples - 1)) * (random_config[j] - start_angles[j]);
		}
		if (IsValidArmConfiguration(tmp_angles, numofDOFs, map, x_size, y_size) &&
			dist_angles(tmp_angles, start_angles, numofDOFs) < epsilon)
		{
			if (i == 1)
			{
				saved_angles = new double[numofDOFs * sizeof(double)];
				;
			}
			memcpy(saved_angles, tmp_angles, numofDOFs * sizeof(double));
		}
		else
		{
			break;
		}
	}

	free(tmp_angles);
	return saved_angles;
}

bool goal_reached(double *start, double *end, int numofDOFs, int thres)
{
	double dist = dist_angles(start, end, numofDOFs);
	if (dist < thres)
	{
		return true;
	}
	return false;
}
bool angles_equal(double *angle1, double *angle2, int numofDOFs)
{
	if (angle1 == NULL || angle2 == NULL)
		return false;
	for (int i = 0; i < numofDOFs; i++)
	{
		if (angle1[i] != angle2[i])
		{
			return false;
			break;
		}
	}
	return true;
}
static void RRTplan(
	double *map,
	int x_size,
	int y_size,
	double *armstart_anglesV_rad,
	double *armgoal_anglesV_rad,
	int numofDOFs,
	double ***plan,
	int *planlength)
{
	*plan = NULL;
	*planlength = 0;

	RRTplan_C rrtplan;
	rrtplan.RRTplan_init(numofDOFs, armstart_anglesV_rad);
	int k = 0;

	bool goalfound = false;
	double *random_config = new double[numofDOFs * sizeof(double)];
	double epsilon = 1;
	srand(time(NULL));
	while (goalfound == false and k < 80000)
	{
		random_config = RandomConfig(map, numofDOFs, x_size, y_size);
		int n_id = rrtplan.nearest_id(random_config);
		double *extend_angles = extend(n_id, random_config, map, numofDOFs, x_size, y_size, epsilon, rrtplan.vertices);
		k++;
		if (extend_angles)
		{
			int new_id = rrtplan.vertices.size();
			rrtplan.vertices.push_back(extend_angles);

			rrtplan.edges[new_id] = n_id;

			extend_angles = extend(new_id, armgoal_anglesV_rad, map, numofDOFs, x_size, y_size, epsilon, rrtplan.vertices);
			//    Change the goal condition and try goal region
			if (angles_equal(extend_angles, armgoal_anglesV_rad, numofDOFs))
			{
				int g_id = rrtplan.vertices.size();
				rrtplan.vertices.push_back(extend_angles);

				rrtplan.edges[g_id] = new_id;
				goalfound = true;
				cout << "GOAL FOUND" << endl;
				break;
			}
		}
	}
	if (goalfound)
	{
		vector<int> path;
		int next_id = rrtplan.vertices.size() - 1;
		while (next_id != 0)
		{
			path.insert(path.begin(), next_id);
			next_id = rrtplan.edges[next_id];
		}
		path.insert(path.begin(), 0);

		*planlength = path.size();

		*plan = (double **)malloc(path.size() * sizeof(double *));

		for (int i = 0; i < path.size(); i++)
		{
			(*plan)[i] = (double *)malloc(numofDOFs * sizeof(double));
			memcpy((*plan)[i], rrtplan.vertices[path[i]], numofDOFs * sizeof(double));
		}
	}
	cout << k << endl;
}

static void RRTConnectplan(
	double *map,
	int x_size,
	int y_size,
	double *armstart_anglesV_rad,
	double *armgoal_anglesV_rad,
	int numofDOFs,
	double ***plan,
	int *planlength)
{
	*plan = NULL;
	*planlength = 0;
	RRTplan_C rrt_connect_start;
	RRTplan_C rrt_connect_goal;
	rrt_connect_start.RRTplan_init(numofDOFs, armstart_anglesV_rad);
	rrt_connect_goal.RRTplan_init(numofDOFs, armgoal_anglesV_rad);
	int k = 0;
	bool goalfound = false;
	double *random_config = new double[numofDOFs * sizeof(double)];
	// Remember to remove the seed
	// std::srand(10);
	double epsilon = 0.2;
	bool swap = true;
	srand(time(NULL));
	while (goalfound == false and k < 80000)
	{
		k++;
		if (swap)
		{
			random_config = RandomConfig(map, numofDOFs, x_size, y_size);
			int n_id = rrt_connect_start.nearest_id(random_config);
			double *extend_angles = extend(n_id, random_config, map, numofDOFs, x_size, y_size, epsilon, rrt_connect_start.vertices);
			if (!extend_angles)
			{
				continue;
			}
			int new_id = rrt_connect_start.vertices.size();
			rrt_connect_start.vertices.push_back(extend_angles);
			rrt_connect_start.edges[new_id] = n_id;

			int g_id = rrt_connect_goal.nearest_id(extend_angles);
			double *goal_extend_angles = extend(g_id, extend_angles, map, numofDOFs, x_size, y_size, epsilon, rrt_connect_goal.vertices);
			if (!goal_extend_angles)
			{
				continue;
			}
			int goal_new_id = rrt_connect_goal.vertices.size();
			rrt_connect_goal.vertices.push_back(goal_extend_angles);
			rrt_connect_goal.edges[goal_new_id] = g_id;
			if (angles_equal(goal_extend_angles, extend_angles, numofDOFs))
			{
				goalfound = true;
				break;
			}
		}

		else
		{
			random_config = RandomConfig(map, numofDOFs, x_size, y_size);
			int gn_id = rrt_connect_goal.nearest_id(random_config);
			double *g_extend_angles = extend(gn_id, random_config, map, numofDOFs, x_size, y_size, epsilon, rrt_connect_goal.vertices);

			if (!g_extend_angles)
			{
				continue;
			}
			int gnew_id = rrt_connect_goal.vertices.size();
			rrt_connect_goal.vertices.push_back(g_extend_angles);
			rrt_connect_goal.edges[gnew_id] = gn_id;

			int s_id = rrt_connect_start.nearest_id(g_extend_angles);
			double *s_extend_angles = extend(s_id, g_extend_angles, map, numofDOFs, x_size, y_size, epsilon, rrt_connect_start.vertices);
			if (!s_extend_angles)
			{
				continue;
			}
			int s_new_id = rrt_connect_start.vertices.size();
			rrt_connect_start.vertices.push_back(s_extend_angles);
			rrt_connect_start.edges[s_new_id] = s_id;
			if (angles_equal(s_extend_angles, g_extend_angles, numofDOFs))
			{
				goalfound = true;
				break;
			}
		}

		swap = !swap;
	}
	if (goalfound)
	{
		deque<int> tree;
		int next_id = rrt_connect_start.vertices.size() - 1;
		while (next_id != 0)
		{
			tree.push_front(next_id);
			next_id = rrt_connect_start.edges[next_id];
		}
		tree.push_front(0);

		int start_plan_size = tree.size();

		next_id = rrt_connect_goal.vertices.size() - 1;
		;

		while (next_id != 0)
		{
			next_id = rrt_connect_goal.edges[next_id];
			tree.push_back(next_id);
		}

		*planlength = tree.size();

		*plan = (double **)malloc(tree.size() * sizeof(double *));

		for (int i = 0; i < tree.size(); i++)
		{
			(*plan)[i] = (double *)malloc(numofDOFs * sizeof(double));
			if (i < start_plan_size)
				memcpy((*plan)[i], rrt_connect_start.vertices[tree[i]], numofDOFs * sizeof(double));
			else
				memcpy((*plan)[i], rrt_connect_goal.vertices[tree[i]], numofDOFs * sizeof(double));
		}
	}

	cout << k << endl;
}

static void RRTstarplan(
	double *map,
	int x_size,
	int y_size,
	double *armstart_anglesV_rad,
	double *armgoal_anglesV_rad,
	int numofDOFs,
	double ***plan,
	int *planlength)
{
	*plan = NULL;
	*planlength = 0;
	int k = 0;
	srand(time(NULL));

	RRTplan_C rrtstar;
	bool goalfound = false;
	double *random_config = new double[numofDOFs * sizeof(double)];
	rrtstar.RRTplan_init(numofDOFs, armstart_anglesV_rad);
	double epsilon = 1;
	double r = 1.5 * epsilon;
	rrtstar.cost[0] = 0;
	int goal_id;
	while (goalfound == false and k < 80000)
	{
		k++;

		random_config = RandomConfig(map, numofDOFs, x_size, y_size);
		int n_id = rrtstar.nearest_id(random_config);
		double *extend_angles = extend(n_id, random_config, map, numofDOFs, x_size, y_size, epsilon, rrtstar.vertices);
		if (extend_angles)
		{
			int new_id = rrtstar.vertices.size();
			rrtstar.vertices.push_back(extend_angles);
			vector<int> extend_nearest;
			double min_cost = rrtstar.cost[n_id] + dist_angles(rrtstar.vertices[n_id], rrtstar.vertices[new_id], numofDOFs);
			int parent = n_id;

			for (int i = 0; i < rrtstar.vertices.size() - 1; i++)
			{
				double dist_cal = dist_angles(rrtstar.vertices[new_id], rrtstar.vertices[i], numofDOFs);
				if (dist_cal < r)
				{
					extend_nearest.push_back(i);
					if ((rrtstar.cost[i] + dist_cal) < min_cost)
					{
						min_cost = rrtstar.cost[i] + dist_cal;
						parent = i;
					}
				}
			}

			rrtstar.cost[new_id] = min_cost;
			rrtstar.edges[new_id] = parent;

			for (int near : extend_nearest)
			{
				if (rrtstar.cost[near] > rrtstar.cost[new_id] + dist_angles(rrtstar.vertices[new_id], rrtstar.vertices[near], numofDOFs))
				{
					rrtstar.cost[near] = rrtstar.cost[new_id] + dist_angles(rrtstar.vertices[new_id], rrtstar.vertices[near], numofDOFs);
					rrtstar.edges[near] = new_id;
				}
			}

			if (!goalfound)
			{
				double *extend_angles = extend(new_id, armgoal_anglesV_rad, map, numofDOFs, x_size, y_size, epsilon, rrtstar.vertices);
				if (angles_equal(extend_angles, armgoal_anglesV_rad, numofDOFs))
				{

					goal_id = rrtstar.vertices.size();
					rrtstar.vertices.push_back(armgoal_anglesV_rad);
					rrtstar.edges[goal_id] = new_id;
					rrtstar.cost[goal_id] = rrtstar.cost[new_id] + dist_angles(rrtstar.vertices[new_id], rrtstar.vertices[goal_id], numofDOFs);
					cout << "Goal in star Found" << endl;
					goalfound = true;
					break;
				}
			}
		}
	}

	if (goalfound)
	{
		vector<int> path;
		int next_id = rrtstar.vertices.size() - 1;
		while (next_id != 0)
		{
			path.insert(path.begin(), next_id);
			next_id = rrtstar.edges[next_id];
		}
		path.insert(path.begin(), 0);

		*planlength = path.size();

		*plan = (double **)malloc(path.size() * sizeof(double *));

		for (int i = 0; i < path.size(); i++)
		{
			(*plan)[i] = (double *)malloc(numofDOFs * sizeof(double));
			memcpy((*plan)[i], rrtstar.vertices[path[i]], numofDOFs * sizeof(double));
		}
	}
	cout << k << endl;
}

static void PRMplan_f(
	double *map,
	int x_size,
	int y_size,
	double *armstart_anglesV_rad,
	double *armgoal_anglesV_rad,
	int numofDOFs,
	double ***plan,
	int *planlength)
{
	*plan = NULL;
	*planlength = 0;
	PRMplan prmplan;
	prmplan.vertices.push_back(armstart_anglesV_rad);
	int start_id = 0;
	vector<int> t;
	prmplan.edges.push_back(t);	
	int num_of_samples = 10000;
	int i = 0;
	double *random_config = new double[numofDOFs * sizeof(double)];
	double min_dist_start = 0;
	double min_dist_goal =std::numeric_limits<double>::max() ;
	double radius = 1.5;
	int min_dist_id = -1;
    srand(time(NULL));
	while (i < num_of_samples)
	{
		i++;
		random_config = RandomConfig(map, numofDOFs, x_size, y_size);
		int ind = prmplan.vertices.size();
        prmplan.g[ind] = std::numeric_limits<double>::max();
		prmplan.vertices.push_back(random_config);
		vector<int> t;
		prmplan.edges.push_back(t);
		int number_of_comp = 0;
		for (int k = 0; k < prmplan.vertices.size()-1; k++)
		{
			if (dist_angles(prmplan.vertices[k], random_config, numofDOFs) < radius)
			{

				double *extendangles = extend(k, random_config, map, numofDOFs, x_size, y_size, std::numeric_limits<double>::max(), prmplan.vertices);
				if (angles_equal(extendangles, random_config, numofDOFs))
				{
                    
					if(extend(ind,armgoal_anglesV_rad,map,numofDOFs,x_size,y_size,std::numeric_limits<double>::max(),prmplan.vertices)!=NULL)
					{double min_dist = dist_angles(random_config,armgoal_anglesV_rad,numofDOFs);
					if(min_dist<min_dist_goal){
						min_dist_goal = min_dist;
                        min_dist_id = ind;
					}}
					prmplan.edges[k].push_back(ind);
					prmplan.edges[ind].push_back(k);

					number_of_comp++;
					if (number_of_comp > 10)
					{
						break;
					}
				}
			}
		}
	}
    int goal_ind = -2;
    double* goal_angle_add = extend(min_dist_id,armgoal_anglesV_rad,map,numofDOFs,x_size,y_size,std::numeric_limits<double>::max(), prmplan.vertices);
	if(angles_equal(goal_angle_add, armgoal_anglesV_rad, numofDOFs)){
 		goal_ind = prmplan.vertices.size();

		prmplan.vertices.push_back(armgoal_anglesV_rad);
		vector<int> t;
		prmplan.edges.push_back(t);   
		prmplan.edges[goal_ind].push_back(min_dist_id);
		prmplan.edges[min_dist_id].push_back(goal_ind);

	}


  unordered_set<int> CloseSet;
  priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> openSet; 
  unordered_map<int, int> parent;

  bool goalfound = false;
//   prmplan.g[0] = 0;
  prmplan.f[0] = 0;
  openSet.push(make_pair(prmplan.f[0], 0));
  int goal_id = 1;

  int num_expanded = 0;

  while(!openSet.empty() && !goalfound)
  {
    unordered_set<int>::iterator itr;
    unordered_set<int>::iterator close_index;
    double min_f =numeric_limits<double>::infinity();

    num_expanded++;

    // for(itr=OpenSet.begin(); itr!=OpenSet.end(); ++itr)
    // {
    //   double f = prmplan.g[*itr]+ 3*dist_angles(prmplan.GetVertice(*itr), prmplan.GetVertice(goal_ind), numofDOFs);
    //   if (f<min_f)
    //   {
    //       min_f = f;
    //       close_index = itr;
    //   }
    // }
	pair<int, int> node = openSet.top();
    int current_id = node.second;
    //mexPrintf("test2: %d\n", current_id);

    CloseSet.insert(current_id);
    openSet.pop();

    vector<int> successors = prmplan.edges[current_id];
    for(auto id :successors)
    {
      if(id == goal_ind)
      {
        // mexPrintf("Found! \n");
        // mexPrintf("number of expanded node: %d\n", num_expanded);
        goalfound = true;
        parent[goal_ind] = current_id;
        break;
      }

      if(CloseSet.count(id) == 0)
      { 
		  

          if(prmplan.g[id]> prmplan.g[current_id] + dist_angles(prmplan.GetVertice(id), prmplan.GetVertice(current_id), numofDOFs))
          {
            prmplan.g[id] = prmplan.g[current_id] + dist_angles(prmplan.GetVertice(id), prmplan.GetVertice(current_id), numofDOFs);
			prmplan.f[id] = prmplan.g[id] + 3*dist_angles(prmplan.GetVertice(id), prmplan.GetVertice(goal_ind), numofDOFs);
			openSet.push(make_pair(prmplan.f[id],id));
            parent[id] = current_id;
          }
        
      }
    }

  }


	if (goalfound)
	{
		vector<int> path;
		int next_id = prmplan.vertices.size() - 1;
		while (next_id != 0)
		{
			path.insert(path.begin(), next_id);
			next_id = parent[next_id];
		}
		path.insert(path.begin(), 0);

		*planlength = path.size();

		*plan = (double **)malloc(path.size() * sizeof(double *));

		for (int i = 0; i < path.size(); i++)
		{
			(*plan)[i] = (double *)malloc(numofDOFs * sizeof(double));
			memcpy((*plan)[i], prmplan.vertices[path[i]], numofDOFs * sizeof(double));
		}
	}
}

static void planner(
	double *map,
	int x_size,
	int y_size,
	double *armstart_anglesV_rad,
	double *armgoal_anglesV_rad,
	int numofDOFs,
	double ***plan,
	int *planlength)
{
	// no plan by default
	*plan = NULL;
	*planlength = 0;

	// for now just do straight interpolation between start and goal checking for the validity of samples

	double distance = 0;
	int i, j;
	for (j = 0; j < numofDOFs; j++)
	{
		if (distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
			distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
	}
	int numofsamples = (int)(distance / (PI / 20));
	if (numofsamples < 2)
	{
		printf("The arm is already at the goal\n");
		return;
	}
	int countNumInvalid = 0;
	*plan = (double **)malloc(numofsamples * sizeof(double *));
	for (i = 0; i < numofsamples; i++)
	{
		(*plan)[i] = (double *)malloc(numofDOFs * sizeof(double));
		for (j = 0; j < numofDOFs; j++)
		{
			(*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i) / (numofsamples - 1)) * (armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
		}
		if (!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size))
		{
			++countNumInvalid;
		}
	}
	printf("Linear interpolation collided at %d instances across the path\n", countNumInvalid);
	*planlength = numofsamples;

	return;
}

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos,
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char **argv)
{
	double *map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double *startPos = doubleArrayFromString(argv[3]);
	double *goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if (!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size) ||
		!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size))
	{
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double **plan = NULL;
	int planlength = 0;
	// RRTplan(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	// RRTConnectplan(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	// planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	// RRTstarplan(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	PRMplan_f(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

	// Your solution's path should start with startPos and end with goalPos
	if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) ||
		!equalDoubleArrays(plan[planlength - 1], goalPos, numOfDOFs))
	{
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open())
	{
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i)
	{
		for (int k = 0; k < numOfDOFs; ++k)
		{
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}