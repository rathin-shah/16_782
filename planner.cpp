#include <math.h>
#include <mex.h>
#include <stdio.h>
#include <string.h>
#include <utility>
#include <stdio.h>
#include <string.h>
#include <queue>
#include <unordered_map>
#include <climits>
#include <iostream>
#include <ctime>
#include <cmath>
#include <stack>

#include "aStar.hpp"

using namespace std;

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]

/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

#define printfunc(...) { mexPrintf(__VA_ARGS__); mexEvalString("drawnow;");}
int dX[NUMOFDIRS] = { -1, -1, -1,  0,  0,  1, 1, 1 };
int dY[NUMOFDIRS] = { -1,  0,  1, -1,  1, -1, 0, 1 };
int runCount = 0;
int goalposeX = -1;
int goalposeY = -1;
pair<int, int> goal(goalposeX, goalposeY);
clock_t start;


void aStar::backTrack(pair<int,int> goal)
{
    goalposeX = goal.first;
    goalposeY = goal.second;
    int n= xyToIndex(goalposeX, goalposeY);     
    //int n = (int)map[GETMAPINDEX(goalposeX, goalposeY, x_size, y_size)];
    int p = cellInfo[n].parent;
    int r = xyToIndex(robotposeX, robotposeY);

    if (n == r) {
        returnPath.push(n);
        return;
    }
    else {
        while (cellInfo[n].parent != xyToIndex(robotposeX, robotposeY))
        {
            returnPath.push(n);
            n = cellInfo[n].parent;
        }
        returnPath.push(n);
    }

}



void aStar::computePath()
{
    int goalposeX_h = (int)target_traj[target_steps - 1];
    int goalposeY_h = (int)target_traj[target_steps - 1 + target_steps];
    int k, j;
    int newx;
    int newy;
    while(!this->openList.empty())
    {
       
        pair<int, int> node = this->openList.top();     
        k = indexToXY(node.second).first;
        j = indexToXY(node.second).second;
        this->openList.pop();

        if (closedList[node.second] == true)    
            continue;

        
        closedList[node.second] = true;     

        
        for (int i = 0; i < NUMOFDIRS; i++)
        {
            newx = k + dX[i];
            newy = j + dY[i];
            if ((newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && ((int)map[GETMAPINDEX(newx, newy, x_size, y_size)] < collision_thresh))) {

                int h_n = (int)sqrt(((newx - goalposeX_h) * (newx - goalposeX_h) + (newy - goalposeY_h) * (newy - goalposeY_h)));
                if (cellInfo[xyToIndex(newx,newy)].g > cellInfo[node.second].g + this->map[xyToIndex(newx, newy)]) //If g of new > g of curr + cost of new
                {
                    cellInfo[xyToIndex(newx, newy)].g = cellInfo[node.second].g + this->map[xyToIndex(newx, newy)];  //Set g of new = g of curr + cost of new
                    cellInfo[xyToIndex(newx, newy)].h = h_n;
                    cellInfo[xyToIndex(newx, newy)].f = cellInfo[xyToIndex(newx, newy)].g + cellInfo[xyToIndex(newx, newy)].h;
                    openList.push(make_pair(cellInfo[xyToIndex(newx, newy)].f, xyToIndex(newx, newy)));
                    cellInfo[xyToIndex(newx, newy)].parent = node.second;
                }
            }
            //else {
            //    continue;
            //}
        }
    }
}



pair<int,int> aStar::goalFinder()
{   

    int Time_till_now = (int)(clock() - start);
    int steps_moved = (int)ceil(Time_till_now / ((CLOCKS_PER_SEC)));
    pair<int, int> opt_goal(goalposeX, goalposeX);
    for (int i = 0; i < target_steps; i++) {
        int steps = 0;
        int cost = 0;
        goalposeX = (int)target_traj[i];
        goalposeY = (int)target_traj[i + target_steps];
        {
            int p = xyToIndex(goalposeX, goalposeY);     
               while (p != xyToIndex(robotposeX, robotposeY))
            {
                p = cellInfo[p].parent;
                steps++;
            }
        }
        int total_steps_taken = steps + steps_moved;
        if (total_steps_taken < i) {
            
        {
              int n = xyToIndex(goalposeX, goalposeY);
                   
                 while (n != xyToIndex(robotposeX, robotposeY))
                  {
                        cost += (int)map[n];
                        n = cellInfo[n].parent;
                  }
                   
        }
            
            current_cost = cost + (int)(i - steps) * map[xyToIndex(goalposeX, goalposeY)];
            if (current_cost < opt_cost) {
                opt_cost = current_cost;
                opt_goal = make_pair(goalposeX, goalposeY);

            }
        }
    }
 

    return opt_goal ;
}


pair<int, int> aStarSearch(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    static aStar a(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps,
            target_traj, targetposeX, targetposeY, curr_time, action_ptr); 

    if (runCount == 1)
    {    
        start = clock();

        a.initStartCell();
        a.computePath();
        goal = a.goalFinder();
        a.backTrack(goal);
    }


    if (robotposeX == goal.first && robotposeY == goal.second)
    {
        return make_pair(robotposeX, robotposeY);
    }

    int nextIndex = a.returnPath.top();
    a.returnPath.pop();

    pair<int, int> nextXY = a.indexToXY(nextIndex);
    return nextXY;
}


static void planner(
        double*	map,
        int collision_thresh,
        int x_size,     //Number of columns
        int y_size,     //Number of rows
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{

    runCount++;
    pair<int, int> nextXY = aStarSearch(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, targetposeX, targetposeY, curr_time, action_ptr);
    action_ptr[0] = nextXY.first;
    action_ptr[1] = nextXY.second;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//              MEX FUNCTION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )    
{
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}