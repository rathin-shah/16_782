#pragma once

using namespace std;
#include <set> 
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

public:

    struct ArrayHasher {
        std::size_t operator()(const std::array<int, 3>& a) const {
            std::size_t h = 0;
            for (auto e : a) {
                h ^= std::hash<int>{}(e)+0x9e3779b9 + (h << 6) + (h >> 2);
            }
            return h;
        }
    };
    struct cell
    {
        vector<int> parent = {-1,-1,-1};   //Parent's map index
        int g = INT_MAX;
        int h = INT_MAX;
        int f = INT_MAX;
    };

    unordered_map<array<int,3>, bool,ArrayHasher> closedList;     //closedList of bool values for each cell
    unordered_map<array<int,3>, cell, ArrayHasher> cellInfo;       //Stores info of each cell corresponding to the index of the cell

    priority_queue<pair<int, vector<int>>, vector<pair<int, vector<int>>>, greater<pair<int, vector<int>>>> openList;   //f-value, cell index (sorted in increasing order of f-value)

    bool found_path = false;
    //set <pair<int, vector<int>>> openList;




    //bool destReached = false;
    stack <pair<int,int>> returnPath;
    int opt_cost = INT_MAX;
    int current_cost = 0;
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
        int h = robotposeX;
        int p = robotposeY;
        cellInfo[{robotposeX, robotposeY, 0}].g = 0;
        int m = cellInfo[{h, p, 0}].g;
        cellInfo[{robotposeX, robotposeY, 0}].h = 0;
        cellInfo[{robotposeX, robotposeY, 0}].f = 0;
        cellInfo[{robotposeX, robotposeY, 0}].parent = { robotposeX,robotposeY,0 };

        int par = cellInfo[{robotposeX, robotposeY, 0}].parent[0];
        openList.push(make_pair(0,vector<int>{ robotposeX ,robotposeY , 0}));
    }



    void computePath();
    void backTrack();
    pair<int, int> goalFinder();

};