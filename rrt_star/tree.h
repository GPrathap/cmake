#ifndef RRT_TREE_TREE_H
#define RRT_TREE_TREE_H

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
#include "../common/search_space.h"

namespace kamaz {
namespace hagen {
        class Tree {
            public:
                Tree() = default;
                ~Tree() = default;
                SearchSpace V;
                int v_count = 0;
                std::map<std::tuple<float, float, float>, Eigen::VectorXd> E;
                void init(SearchSpace search_space);
        };
    }
}
#endif