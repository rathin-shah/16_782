#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <unordered_map>
#include<cstdlib>
#include<iostream>
#define PI 3.141592654
#define LINKLENGTH_CELLS 10
using namespace std;

class RRTplan_C{

public:

	vector<double* > vertices;
	unordered_map<int,int> edges;
    unordered_map<int,double> cost;
	int numofDOFs;

	void RRTplan_init(int nofDOFs, double* armstart_anglesV_rad)
	{
		numofDOFs = nofDOFs;
		vertices.clear();
		edges.clear();
		vertices.push_back(armstart_anglesV_rad);
	}

    int nearest_id(double* angles){
        double dist = std::numeric_limits<double>::max();
        int n_id = 0;
        for(int i = 0;i<vertices.size();i++){
            double n_dist = 0;

            for(int j=0;j<numofDOFs;j++){
                n_dist = n_dist + ((vertices[i][j]-angles[j])*(vertices[i][j]-angles[j]));
            }
            n_dist = sqrt(n_dist);
            
            if(n_dist < dist){
                dist = n_dist;
                n_id = i;
            }

        }

        return n_id;
        
    }

};


