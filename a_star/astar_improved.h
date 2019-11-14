#ifndef ASTRAT_IMPROVED_H
#define ASTRAT_IMPROVED_H

#include <stack>
#include <vector>
#include <array>
#include <iostream>
#include <cmath>
#include <set>
#include <cstdlib>
#include <climits>
#include <cmath>
#include <cerrno>
#include <cfenv>
#include <cstring>

#include <cnpy.h>

#include "../common/search_space.h"



namespace kamaz {
namespace hagen {

class AStarImproved {
public:

	// represents a single pixel
	class Node {
		public:
			int idx;     // index in the flattened grid
			float cost;  // cost of traversing this pixel
			int x;
			int y;
			int z;
			Node(int i, float c) : idx(i),cost(c) {}
	};

	struct pose{
		int x;
		int y;
		int z;
	};
	
	struct Options{
		bool diagonal = false;
		std::string heuristic = "euclidean";
		float heightFactor =  1.0;
		float threshold_level_a_star = 0.0f;
	};

	SearchSpace search_space;
	int w;
    int h;
    int d;

	int threshold_level_a_star = 2;
	AStarImproved::Options _options;

	AStarImproved() = default;
	~AStarImproved() = default;

	// the top of the priority queue is the greatest element by default,
	// but we want the smallest, so flip the sign
	friend bool operator<(const Node &n1, const Node &n2) {
	return n1.cost > n2.cost;
	}

	friend  bool operator==(const Node &n1, const Node &n2) {
	return n1.idx == n2.idx;
	}

	// See for various grid heuristics:
	// http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#S7
	// L_\inf norm (diagonal distance)
	inline float linf_norm(int i0, int j0, int i1, int j1) {
		return std::max(std::abs(i0 - i1), std::abs(j0 - j1));
	}

	// L_1 norm (manhattan distance)
	inline float l1_norm(int i0, int j0, int i1, int j1) {
		return std::abs(i0 - i1) + std::abs(j0 - j1);
	}

	inline float linf_norm(int i0, int j0, int i1, int j1, int k0, int k1) {                     
		return std::max({ std::abs(i0 - i1), std::abs(j0 - j1), std::abs(j0 - j1) },
		[](const int s1, int s2) { return s1 < s2; } );
	}

	// L_1 norm (manhattan distance)
	float l1_norm(int i0, int j0, int i1, int j1, int k0, int k1) {
		return std::abs(i0 - i1) + std::abs(j0 - j1) + std::abs(k0 - k1);
	}
	
	std::array<int, 3> to3D( int idx);
	int to1D( int x, int y, int z);
	bool isValid(int x, int y, int z);
	std::vector<int> getNeighbors(int idx, bool diagonal);
	bool astar(const int start, const int goal, bool diag_ok, std::vector<int>& paths);
	float float_max;
	bool init_planner();
	float get_distance(std::vector<Eigen::VectorXf> trajectory_);
	void save_path(std::vector<Eigen::VectorXf> path);
	float get_cost_of_path(std::vector<Eigen::VectorXf> path1);

	std::vector<Eigen::VectorXf> astar_planner(SearchSpace X,  Eigen::VectorXf x_init, 
									Eigen::VectorXf x_goal);
	
	std::vector<Eigen::VectorXf> astar_planner_and_save(SearchSpace X,  Eigen::VectorXf x_init, 
									Eigen::VectorXf x_goal);
	int start_idx;
	int goal_idx;
};

}
}
#endif