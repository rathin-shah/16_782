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
#include <vector>
#include "aStar_3d.hpp"

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
int dX[NUMOFDIRS + 1] = {0, -1, -1, -1,  0,  0,  1, 1, 1 };
int dY[NUMOFDIRS + 1] = {0, -1,  0,  1, -1,  1, -1, 0, 1 };
int dt = 1;
int runCount = 0;
int goalposeX = -1;
int goalposeY = -1;
pair<int, int> goal(goalposeX, goalposeY);
clock_t start;


void aStar::backTrack()
{
    goalposeX = (int)target_traj[target_steps - 1];;
    goalposeY = (int)target_traj[target_steps - 1 + target_steps];
    int goalposeT = target_steps;
    vector<int> n{ goalposeX ,goalposeY, goalposeT };
    //int n = (int)map[GETMAPINDEX(goalposeX, goalposeY, x_size, y_size)];
    //int p = cellInfo[n].parent;
    //int r = xyToIndex(robotposeX, robotposeY);

    //if (n == r) {
    //    returnPath.push(n);
    //    return;
    //}
    int x_n = n[0];
    int y_n = n[1];
    int node = xyToIndex(x_n, y_n);
    int q = cellInfo[{ n[0], n[1], n[2] }].parent[0];
    int k = cellInfo[{ n[0], n[1], n[2] }].parent[1];
    int t = cellInfo[{ n[0], n[1], n[2] }].parent[2];
    int r = robotposeX;
    int s = robotposeY;
    bool cond = (cellInfo[{ n[0], n[1], n[2] }].parent[0] != robotposeX);
    bool cond1 = (cellInfo[{ n[0], n[1], n[2] }].parent[1] != robotposeY);
    bool cond2 = (cellInfo[{ n[0], n[1], n[2] }].parent[2] != 0);

       while((cellInfo[{ n[0], n[1], n[2] }].parent[0]==n[0]) && (cellInfo[{ n[0], n[1], n[2] }].parent[1] == n[1]) || (cellInfo[{ n[0], n[1], n[2] }].parent[2] == n[2])) {
            int x_n = n[0];
            int y_n = n[1];
            int t_n = n[2];
            node = xyToIndex(x_n, y_n);
            returnPath.push(node);
            n[0] = cellInfo[{x_n,y_n,t_n}].parent[0];
            n[1] = cellInfo[{x_n, y_n, t_n}].parent[1];
            n[2] = cellInfo[{x_n, y_n, t_n}].parent[2];
        }
        returnPath.push(node);
    

}
//
//
//
void aStar::computePath()
{
    //int goalposeX_h = (int)target_traj[target_steps - 1];
    //int goalposeY_h = (int)target_traj[target_steps - 1 + target_steps];
    int goalposeX = (int)target_traj[target_steps - 1];
    int goalposeY = (int)target_traj[target_steps - 1 + target_steps];
    int k,j,t;
    int newx;
    int newy;
    int newt;
    while (!this->openList.empty())
    {

        pair<int, vector<int>> node = this->openList.top();
        k = node.second[0];
        j = node.second[1];
        t = node.second[2];
        //mexPrintf("time: %i \n", t);
        int qwe_newewe = openList.size();
        int u = cellInfo[{k, j, t}].g;
        //*openList.erase(openList.begin());

        this->openList.pop();

        //mexPrintf("size: %i \n", this->openList.size());
 

        int qwe = openList.size();
        if (closedList[{k,j,t}] == true)
            continue;

        
        closedList[{k, j, t}] = true;

        bool cl = closedList[{k, j, t}];
        
        for (int i = 0; i < NUMOFDIRS +1 ; i++)
        {
            newx = k + dX[i];
            newy = j + dY[i];
            newt = t + dt;
            //mexPrintf("time: %i \n", newt);

            if ((newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && ((int)map[GETMAPINDEX(newx, newy, x_size, y_size)] < collision_thresh) && newt <=target_steps)) {
                if (newx == goalposeX && newy == goalposeY && newt == target_steps)  //if new pose is the goal pose at that time
                {

                    int curr_t = t;
                    int new_t = newt;
                    cellInfo[{newx, newy, newt}].parent = vector<int>{ k, j, t };
                    mexPrintf("found_path");
                    found_path = true;
                    break;
                   
                }
                int h_n = (int)sqrt(((newx - goalposeX) * (newx - goalposeX) + (newy - goalposeY) * (newy - goalposeY)));;
                int l = cellInfo[{newx, newy, newt}].g;
                int m = cellInfo[{k, j, t}].g;
                {
                    if (cellInfo[{newx, newy, newt}].g > cellInfo[{k, j, t}].g + this->map[xyToIndex(newx, newy)]) //If g of new > g of curr + cost of new
                    {
                        cellInfo[{newx, newy, newt}].g = cellInfo[{k, j, t}].g + this->map[xyToIndex(newx, newy)];  //Set g of new = g of curr + cost of new
                        
                        int old_info = cellInfo[{k, j, t}].g;
                        int cost_m = this->map[xyToIndex(newx, newy)];
                        int info = cellInfo[{newx, newy, newt}].g;
                        
                        cellInfo[{newx, newy, newt}].h = 100 * h_n;
                        cellInfo[{newx, newy, newt}].f = cellInfo[{newx, newy, newt}].g + cellInfo[{newx, newy, newt}].h;
                        openList.push(make_pair(cellInfo[{newx, newy, newt}].f, vector<int> {newx, newy, newt}));
                        cellInfo[{newx, newy, newt}].parent = vector<int> { k, j, t };

                        int a = cellInfo[{newx, newy, newt}].parent[0];
                        int b = cellInfo[{newx, newy, newt}].parent[1];
                        int c = cellInfo[{newx, newy, newt}].parent[2];

                    }
                }

            }
            //else {
            //    continue;
            //}

        }

        if (found_path) {
            break;
        }
        
    }
}



//pair<int,int> aStar::goalFinder()
//{   
//
//    int Time_till_now = (int)(clock() - start);
//    int steps_moved = (int)ceil(Time_till_now / ((CLOCKS_PER_SEC)));
//    pair<int, int> opt_goal(goalposeX, goalposeX);
//    for (int i = 0; i < target_steps; i++) {
//        int steps = 0;
//        int cost = 0;
//        goalposeX = (int)target_traj[i];
//        goalposeY = (int)target_traj[i + target_steps];
//        {
//            int p = xyToIndex(goalposeX, goalposeY);     
//               while (p != xyToIndex(robotposeX, robotposeY))
//            {
//                p = cellInfo[p].parent;
//                steps++;
//            }
//        }
//        int total_steps_taken = steps + steps_moved;
//        if (total_steps_taken < i) {
//            
//        {
//              int n = xyToIndex(goalposeX, goalposeY);
//                   
//                 while (n != xyToIndex(robotposeX, robotposeY))
//                  {
//                        cost += (int)map[n];
//                        n = cellInfo[n].parent;
//                  }
//                   
//        }
//            
//            current_cost = cost + (int)(i - steps) * map[xyToIndex(goalposeX, goalposeY)];
//            opt_goal = make_pair(goalposeX, goalposeY)
//            if (current_cost < opt_cost) {
//                opt_cost = current_cost;
//                opt_goal = make_pair(goalposeX, goalposeY);
//
//            }
//        }
//    }
// 
//
//    return opt_goal ;
//}


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
        //goal = a.goalFinder();
        a.backTrack();
    }

    //int goalposeX = (int)target_traj[target_steps - 1];
    //int goalposeY = (int)target_traj[target_steps - 1 + target_steps];
    //if (robotposeX == goalposeX && robotposeY == goalposeY)
    //{
    //    return make_pair(robotposeX, robotposeY);
    //}
    pair<int, int> nextXY;
    mexPrintf("Path: %i \n", a.returnPath.size());
    if (a.returnPath.size() > 1) {
        int nextIndex = a.returnPath.top();
        mexPrintf("Index: %i \n", nextIndex);
        a.returnPath.pop();
        nextXY = a.indexToXY(nextIndex);

    }
    else {
        nextXY = {robotposeX,robotposeY};
    }
    //int nextIndex = a.returnPath.top();
    //a.returnPath.pop();

    //pair<int, int> nextXY = a.indexToXY(nextIndex);
    //pair<int, int> nextXY = { robotposeX ,robotposeY };
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