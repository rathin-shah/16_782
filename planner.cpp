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
int dX[NUMOFDIRS + 1] = { 0, -1, -1, -1,  0,  0,  1, 1, 1 };
int dY[NUMOFDIRS + 1] = { 0, -1,  0,  1, -1,  1, -1, 0, 1 };
int dt = 1;
int runCount = 0;
int target_x = -1;
int target_y = -1;
int newt = 0;
int time_elapsed;
/*pair<int, int> goal(goalposeX, goalposeY)*/



void aStar::backTrack(int t_n)
{
    //goalposeX = (int)target_traj[target_steps - 1];;
    //goalposeY = (int)target_traj[target_steps - 1 + target_steps];
    //int goalposeT = target_steps;
    //vector<int> n{ goalposeX ,goalposeY, goalposeT };
    //int n = (int)map[GETMAPINDEX(goalposeX, goalposeY, x_size, y_size)];
    //int p = cellInfo[n].parent;
    //int r = xyToIndex(robotposeX, robotposeY);

    //if (n == r) {
    //    returnPath.push(n);
    //    return;
    //}
    int row = robotposeX;
    int col = robotposeY;
    //int time_elapsed = (int)((clock() - start) / CLOCKS_PER_SEC);


    int t_p = t_n;
    while ((cellInfo[{row, col, t_p}].parent[0] != 0
        && cellInfo[{row, col, t_p}].parent[1] != 0 && cellInfo[{row, col, t_p}].parent[2] != 0))
    {


        returnPath.push(make_pair(row, col));
        int temp_row = cellInfo[{row, col, t_p}].parent[0];
        int temp_col = cellInfo[{row, col, t_p}].parent[1];
        int temp_time = cellInfo[{row, col, t_p}].parent[2];
        row = temp_row;
        col = temp_col;
        t_p = temp_time;
    }
    returnPath.push(make_pair(row, col));




}
//
//
//






void aStar::compute2DPath()
{
    int m, n;
    int new_x, new_y;

    while (!this->openList2D.empty()) {
        pair<int, int> node2D = this->openList2D.top();
        m = indexToXY(node2D.second).first;
        n = indexToXY(node2D.second).second;
        this->openList2D.pop();

        if (closed2DList[node2D.second] == true)
            continue;

        closed2DList[node2D.second] = true;

        for (int i = 1; i < NUMOFDIRS; i++) {
            new_x = m + dX[i];
            new_y = n + dY[i];

            if ((new_x >= 1 && new_x <= x_size && new_y >= 1 && new_y <= y_size && ((int)map[GETMAPINDEX(new_x, new_y, x_size, y_size)] < collision_thresh))) {

                //int h_n = (int)sqrt(((newx - goalposeX_h) * (newx - goalposeX_h) + (newy - goalposeY_h) * (newy - goalposeY_h)));
                if (cellInfo2D[xyToIndex(new_x, new_y)].g > cellInfo2D[node2D.second].g + this->map[xyToIndex(new_x, new_y)]) //If g of new > g of curr + cost of new
                {
                    cellInfo2D[xyToIndex(new_x, new_y)].g = cellInfo2D[node2D.second].g + this->map[xyToIndex(new_x, new_y)];  //Set g of new = g of curr + cost of new
                    openList2D.push(make_pair(cellInfo2D[xyToIndex(new_x, new_y)].g, xyToIndex(new_x, new_y)));
                }
            }

        }
    }
}
void aStar::computePath()
{
    //int goalposeX_h = (int)target_traj[target_steps - 1];
    //int goalposeY_h = (int)target_traj[target_steps - 1 + target_steps];
    //int goalposeX = (int)target_traj[target_steps - 1];
    //int goalposeY = (int)target_traj[target_steps - 1 + target_steps];
    int k, j, t;
    int newx;
    int newy;

    while (!this->openList.empty())
    {


        pair<int, vector<int>> node = this->openList.top();
        k = node.second[0];
        j = node.second[1];
        t = node.second[2];
        //mexPrintf("%i %i %i \n", k, j, t);
        this->openList.pop();

        //mexPrintf("size: %i \n", this->openList.size());
        //mexPrintf("Map X size: %i ", x_size );
        //mexPrintf("Map Y size: %i ", y_size);
        //if (this->openList.size() > (x_size + y_size)) {
        //    break;
        //}


        if (closedList[{k, j, t}] == true)
            continue;


        closedList[{k, j, t}] = true;



        //int time_elapsed = (int)((clock() - start) / CLOCKS_PER_SEC);
        //target_x = (int)target_traj[MIN((curr_time + t + time_elapsed),(target_steps-1))];
        //target_y = (int)target_traj[MIN((curr_time + t + time_elapsed + target_steps),(2*target_steps-1))];


        // mexPrintf("target_x: %d, target_y: %d \n",target_x, target_y);
        // mexPrintf("time: %d \n",curr_time+k+time_elapsed);
        //if (k == target_x && j == target_y && curr_time + t + time_elapsed <= target_steps)                                       // if goal pose is expanded
        //{
        //    mexPrintf("target i: %d, j: %d \n", k, j);
        //    //new_pose = getPath(grid, target_x, target_y, k);
        //            this->found_path = true;
        //            break;
        //    //have_path = true;
        //}
        for (int i = 0; i < NUMOFDIRS + 1; i++)
        {
            newx = k + dX[i];
            newy = j + dY[i];
            newt = t - dt;
            //mexPrintf("time: %i \n", newt);
            //int time_elapsed_new = (int)((clock() - start) / CLOCKS_PER_SEC);
            //if (curr_time + time_elapsed_new + newt <= target_steps) {
            //    int time_step = (int)(time_elapsed_new + curr_time + newt);
            //    target_x = target_traj[time_step];
            //    target_y = target_traj[time_step + target_steps];
            //}


            time_elapsed = (int)((((clock() - start) / CLOCKS_PER_SEC)));

            if ((newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && ((int)map[GETMAPINDEX(newx, newy, x_size, y_size)] < collision_thresh)) && (newt - time_elapsed) >= 0) {


                //target_x = (int)target_traj[(curr_time + newt + time_elapsed-1)];
                //target_y = (int)target_traj[(curr_time + newt + time_elapsed + target_steps-1)];
                //if (newx== robotposeX && newy== robotposeY && ((newt-time_elapsed)>0))/* || (newx==target_traj[target_steps-1] && newy==target_traj[2*target_steps-1])*/ //if new pose is the goal pose at that time
                //{

                ////    mexPrintf("x: %i \n", target_x);
                ////    mexPrintf("y: %i \n", target_y);
                ////    mexPrintf("t: %i \n", newt);
                ////    mexPrintf("p_x: %i \n", k);
                ////    mexPrintf("p_y: %i \n", j);
                ////    mexPrintf("p_t: %i \n", t);
                //    cellInfo[{newx, newy, newt}].parent = vector<int>{ k, j, t };
                //        mexPrintf("found_path");
                //    this->found_path = true;
                //    break;

                //}
                time_elapsed = 1 + (int)((((clock() - start) / CLOCKS_PER_SEC)));
                if (newx == robotposeX && newy == robotposeY && (newt) == time_elapsed) {
                    //mexPrintf("Found Path");
                    //mexPrintf("Time: %i \n", time_elapsed);
                    //mexPrintf("Time: %i \n", target_steps);
                    cellInfo[{newx, newy, newt}].parent = vector<int>{ k, j, t };
                    this->found_path = true;
                    backTrack(time_elapsed);

                    break;
                    return;
                }


                //int h_n = 0 * (target_steps - (curr_time + time_elapsed + newt)) + (sqrt(2) * MIN(abs(newx - target_x), abs(newy - target_y)) + (MAX(abs(newx - target_x), abs(newy - target_y)) - MIN(abs(newx - target_x), abs(newy - target_y))));

                {
                    if (cellInfo[{newx, newy, newt}].g > cellInfo[{k, j, t}].g + this->map[xyToIndex(newx, newy)]) //If g of new > g of curr + cost of new
                    {
                        cellInfo[{newx, newy, newt}].g = cellInfo[{k, j, t}].g + this->map[xyToIndex(newx, newy)];  //Set g of new = g of curr + cost of new



                        cellInfo[{newx, newy, newt}].h = 50 * cellInfo2D[xyToIndex(newx, newy)].g;  //min(h_n,cellInfo2D[xyToIndex(newx,newy)].g);
                        cellInfo[{newx, newy, newt}].f = cellInfo[{newx, newy, newt}].g + cellInfo[{newx, newy, newt}].h;
                        openList.push(make_pair(cellInfo[{newx, newy, newt}].f, vector<int> {newx, newy, newt}));
                        cellInfo[{newx, newy, newt}].parent = vector<int>{ k, j, t };


                    }
                }

            }
            //else {
            //    continue;
            //}

        }

        if (this->found_path) {
            break;
        }

    }

    return;

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
    double* map,
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


        a.start = clock();
        a.initStart2DCell();
        a.compute2DPath();
        a.initStartCell();
        a.computePath();
        //goal = a.goalFinder();
        //a.backTrack();
    }

    //int goalposeX = (int)target_traj[target_steps - 1];
    //int goalposeY = (int)target_traj[target_steps - 1 + target_steps];
    //if (robotposeX == goalposeX && robotposeY == goalposeY)
    //{
    //    return make_pair(robotposeX, robotposeY);
    //}
    pair<int, int> nextXY;
    //int path = (int)a.found_path;
    //mexPrintf("found path :%i \n", path);

    if (a.returnPath.size() > 0 && a.found_path) {
        nextXY = a.returnPath.front();
        //mexPrintf("X: %i Y: %i \n", nextXY.first, nextXY.second);
        //mexPrintf("Path: %i \n", a.returnPath.size());
        //mexPrintf("X: Y: %i %i \n", nextXY.first, nextXY.second);
        a.returnPath.pop();


    }
    else {
        //mexPrintf("Path: %i \n", a.returnPath.size());
        mexPrintf("In Else X: Y: %i %i \n", robotposeX, robotposeY);
        nextXY = { robotposeX,robotposeY };
    }
    //int nextIndex = a.returnPath.top();
    //a.returnPath.pop();

    //pair<int, int> nextXY = a.indexToXY(nextIndex);
      //pair<int, int> nextXY = { robotposeX ,robotposeY };
    return nextXY;
}


static void planner(
    double* map,
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

    //int path = found_path;
    //mexPrintf("path bool is: %i", path);
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
void mexFunction(int nlhs, mxArray* plhs[],
    int nrhs, const mxArray* prhs[])
{
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidNumInputs",
            "Six input arguments required.");
    }
    else if (nlhs != 1) {
        mexErrMsgIdAndTxt("MATLAB:planner:maxlhs",
            "One output argument required.");
    }

    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);

    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if (robotpose_M != 1 || robotpose_N != 2) {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidrobotpose",
            "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];

    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);



    if (targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidtargettraj",
            "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;

    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if (targetpose_M != 1 || targetpose_N != 2) {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidtargetpose",
            "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];

    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);

    /* Create a matrix for the return action */
    ACTION_OUT = mxCreateNumericMatrix((mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL);
    double* action_ptr = (double*)mxGetData(ACTION_OUT);

    /* Get collision threshold for problem */
    int collision_thresh = (int)mxGetScalar(COLLISION_THRESH);

    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;
}