#pragma once

using namespace std;

class aStar
{
    double* map;
    int collision_thresh;
    int x_size;     //Number of columns
    int y_size;     //Number of rows
    int robotposeX;
    int robotposeY;
    int target_steps;
    double* target_traj;
    int targetposeX;
    int targetposeY;
    int curr_time;
    double* action_ptr;

    struct cell
    {
        int parent = -1;   //Parent's map index
        int g = INT_MAX;
        int h = INT_MAX;
        int f = INT_MAX;
    };

    unordered_map<int, bool> closedList;     //closedList of bool values for each cell
    unordered_map<int, cell> cellInfo;       //Stores info of each cell corresponding to the index of the cell

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> openList;   //f-value, cell index (sorted in increasing order of f-value)




public:


    bool destReached = false;
    stack <int> returnPath;

    aStar(
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
        this->map = map;
        this->collision_thresh = collision_thresh;
        this->x_size = x_size;
        this->y_size = y_size;
        this->robotposeX = robotposeX;
        this->robotposeY = robotposeY;
        this->target_steps = target_steps;
        this->target_traj = target_traj;
        this->targetposeX = targetposeX;
        this->targetposeY = targetposeY;
        this->curr_time = curr_time;
        this->action_ptr = action_ptr;
    }

    int xyToIndex(int x, int y)
    {
        return (y - 1) * x_size + (x - 1);
    }

    pair<int, int> indexToXY(int index)
    {
        return (make_pair((index % x_size) + 1, (index / x_size) + 1));
    }

    void initStartCell()
    {
        cellInfo[xyToIndex(robotposeX, robotposeY)].g = 0;
        cellInfo[xyToIndex(robotposeX, robotposeY)].h = 0;
        cellInfo[xyToIndex(robotposeX, robotposeY)].f = 0;
        cellInfo[xyToIndex(robotposeX, robotposeY)].parent = xyToIndex(robotposeX, robotposeY);
        this->openList.push(make_pair(0, xyToIndex(robotposeX, robotposeY)));
    }



    void computePath();
    void backTrack(pair<int, int>);
    int stepsToSeg();
    pair<int, int> goalFinder();
    int pathCost();
};