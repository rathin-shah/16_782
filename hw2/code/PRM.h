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




class PRMplan{

public:

	vector<double* > vertices;
	vector<vector<int>> edges;
    unordered_map<int,double> cost;
    unordered_map<int, double> g;
    unordered_map<int, double> f;
	int numofDOFs;

	void PRMplan_init(int nofDOFs, double* armstart_anglesV_rad)
	{
		numofDOFs = nofDOFs;
		vertices.clear();
		edges.clear();
		vertices.push_back(armstart_anglesV_rad);
        g[0]=0;

	}

	double* GetVertice(int id)
	{
		return vertices[id];
	}
	vector<int> GetSuccessors(int id)
	{
		return edges[id];
	}

    int PRM_nearest_id(double* angles){
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

    // void prepocess_graph(int numofDOFs,double* armstart_anglesV_rad){

    //     vertices.push_back(armstart_anglesV_rad);
    //     int num_of_samples = 5000;
    //     int i = 0;
    //     double* random_config = new double[numofDOFs*sizeof(double)];
    //     while(i<num_of_samples){

    //         random_config = RandomConfig(map, numofDOFs, x_size, y_size);
            
    //     }

    // }

};