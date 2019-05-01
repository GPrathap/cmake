#include "astar_improved.h"

namespace kamaz {
namespace hagen {

void AStarImproved::init_planner(SearchSpace X, AStarImproved::Options options){
	      float_max = 200000000.45;
        search_space = X;
        w = search_space.dim_lengths[1];
        h = search_space.dim_lengths[3];
		    d = search_space.dim_lengths[5];

        const int sz[] = {w, h, d}; 
        _options = options;
		    threshold_level_a_star = _options.threshold_level_a_star;
		    std::cout<< "rows: " << w << " cols:" << h << " depth: " << d << std::endl;
}

int AStarImproved::to1D( int x, int y, int z) {
    return (z * w * h) + (y * w) + x;
}

std::array<int, 3> AStarImproved::to3D(int idx) {
    std::array<int, 3> converted;
    int z = idx / (w * h);
    idx -= (z * w * h);
    int y = idx / w;
    int x = idx % w;
    converted[0] = x;
    converted[1] = y;
    converted[2] = z;
    return converted;
}

bool AStarImproved::isValid(int x, int y, int z) { 
	if (x < 0 || y < 0 || z < 0 || x >= w || y >= h || z >= d) {
		return false;
	}else{
		Eigen::VectorXd search_rect(3);
		search_rect << x, y, z;
		if(search_space.obstacle_free(search_rect)){
			return true;
		}
		std::cout<< "Point is too close to obstacle->>>>>>>>"<<  std::endl;
		return false;
	}
}

std::vector<int> AStarImproved::getNeighbors(int idx, bool diagonal) {
		
		std::vector<int> neighbors;
		auto current = to3D(idx);
		int x = current[0];
		int y = current[1];
		int z = current[2];

		// down
		if(isValid(x-1, y, z)) {
			neighbors.push_back(to1D(x-1, y, z));
		}else{
			neighbors.push_back(-1);
		}

		//up
		if(isValid(x+1, y, z)) {
			neighbors.push_back(to1D(x+1, y, z));
		}else{
			neighbors.push_back(-1);
		}

		//left
		if(isValid(x, y-1, z)) {
			neighbors.push_back(to1D(x, y-1, z));
		}else{
			neighbors.push_back(-1);
		}

		//right
		if(isValid(x, y+1, z)) {
			neighbors.push_back(to1D(x, y+1, z));
		}else{
      		neighbors.push_back(-1);
    	}

		//bottom down
		if(isValid(x-1, y, z-1)) {
			neighbors.push_back(to1D(x-1, y, z-1));
		}else{
      		neighbors.push_back(-1);
    	}

		//bottom up
		if(isValid(x+1, y, z-1)) {
			neighbors.push_back(to1D(x+1, y, z-1));
		}else{
      		neighbors.push_back(-1);
    	}

		//bottom left
		if(isValid(x, y-1, z-1)) {
			neighbors.push_back(to1D(x, y-1, z-1));
		}else{
      		neighbors.push_back(-1);
    	}

		//bottom right
		if(isValid(x, y+1, z-1)) {
			neighbors.push_back(to1D(x, y+1, z-1));
		}else{
      		neighbors.push_back(-1);
    	}

		//top down
		if(isValid(x-1, y, z+1)) {
			neighbors.push_back(to1D(x-1, y, z+1));
		}else{
      		neighbors.push_back(-1);
    	}

		//top up
		if(isValid(x+1, y, z+1)) {
			neighbors.push_back(to1D(x+1, y, z+1));
		}else{
      		neighbors.push_back(-1);
    	}

		//top left
		if(isValid(x, y-1, z+1)) {
			neighbors.push_back(to1D(x, y-1, z+1));
		}else{
      		neighbors.push_back(-1);
    	}

		//top right
		if(isValid(x, y+1, z+1)) {
			neighbors.push_back(to1D(x, y+1, z+1));
		}else{
      		neighbors.push_back(-1);
    	}

		//8-Way
		if (diagonal) {
			//left down
			if(isValid(x-1, y-1, z)) {
					neighbors.push_back(to1D(x-1, y-1, z));
			}
			
			//left up
			if(isValid(x+1, y-1, z)) {
				neighbors.push_back(to1D(x+1, y-1, z));
			}

			//right down
			if(isValid(x-1, y+1, z)) {
				neighbors.push_back(to1D(x-1, y+1, z));
			}

			//right up
			if(isValid(x+1, y+1, z)) {
				neighbors.push_back(to1D(x+1, y+1, z));
			}

			//bottom left down
			if(isValid(x-1, y-1, z-1)) {
					neighbors.push_back(to1D(x-1, y-1, z-1));
			}
			
			//bottom left up
			if(isValid(x+1, y-1, z-1)) {
				neighbors.push_back(to1D(x+1, y-1, z-1));
			}

			//bottom right down
			if(isValid(x-1, y+1, z-1)) {
				neighbors.push_back(to1D(x-1, y+1, z-1));
			}

			//bottom right up
			if(isValid(x+1, y+1, z-1)) {
				neighbors.push_back(to1D(x+1, y+1, z-1));
			}

			//top left down
			if(isValid(x-1, y-1, z+1)) {
				neighbors.push_back(to1D(x-1, y-1, z+1));
			}
			
			//top left up
			if(isValid(x+1, y-1, z+1)) {
				neighbors.push_back(to1D(x+1, y-1, z+1));
			}

			//top right down
			if(isValid(x-1, y+1, z+1)) {
				neighbors.push_back(to1D(x-1, y+1, z+1));
			}

			//top right up
			if(isValid(x+1, y+1, z+1)) {
				neighbors.push_back(to1D(x+1, y+1, z+1));
			}
		}
		return neighbors;
}

// weights:        flattened h x w grid of costs
// h, w:           height and width of grid
// start, goal:    index of start/goal in flattened grid
// diag_ok:        if true, allows diagonal moves (8-conn.)
// paths (output): for each node, stores previous node in path
bool AStarImproved::astar(const int start, const int goal, bool diag_ok, std::vector<int>& paths) {

  const float INF = std::numeric_limits<float>::infinity();
  AStarImproved::Node start_node(start, 0.);
  AStarImproved::Node goal_node(goal, 0.);

//   float* costs = new float[h * w * d];
  std::vector<float> costs(h*w*d);
  for (int i = 0; i < h * w * d; ++i)
    costs[i] = INF;
  costs[start] = 0.;
   
  std::priority_queue<AStarImproved::Node> nodes_to_visit;
  nodes_to_visit.push(start_node);
  bool solution_found = false;
  while (!nodes_to_visit.empty()) {
    AStarImproved::Node cur = nodes_to_visit.top();
    if (cur.idx == goal_node.idx) {
      solution_found = true;
      break;
    }
    nodes_to_visit.pop();
    auto nbrs = getNeighbors(cur.idx, false);
    float heuristic_cost;
    for (int i = 0; i < nbrs.size(); ++i) {
      if (nbrs[i] >= 0) {
        // the sum of the cost so far and the cost of this move
		auto tryy = to3D( nbrs[i]);
		float new_cost = costs[cur.idx] + 0.0;
		if (new_cost < costs[nbrs[i]]) {
          auto current_point = to3D( nbrs[i]);
          auto goal_point = to3D(goal);
          if (diag_ok) {
            heuristic_cost = linf_norm(current_point[0], current_point[1],
                                       goal_point[0],  goal_point[1], current_point[2],  goal_point[2]);
          }
          else {
            heuristic_cost = l1_norm(current_point[0], current_point[1],
                                       goal_point[0],  goal_point[1], current_point[2],  goal_point[2]);
          }
          float priority = new_cost + heuristic_cost;
          nodes_to_visit.push(AStarImproved::Node(nbrs[i], priority));
          costs[nbrs[i]] = new_cost;
          paths[nbrs[i]] = cur.idx;
        }
      }
    }
  }
  
  return solution_found;
}

	   void AStarImproved::save_path(std::vector<Eigen::VectorXd> path){
       std::vector<float> projected_path; 
       std::cout<< "Astar::save_path trajectory size: " << path.size()<< std::endl;
       for(auto const& way_point : path){
           projected_path.push_back(way_point[0]);
           projected_path.push_back(way_point[1]);
           projected_path.push_back(way_point[2]);
       }
       cnpy::npy_save("/dataset/a_star_path.npy",&projected_path[0],{path.size(), 3},"w");
    }
}
}