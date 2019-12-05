#ifndef PATH_PLANNER_RRT_TREE_KINORRTSTAR_H_
#define PATH_PLANNER_RRT_TREE_KINORRTSTAR_H_

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
#include <limits>
#include <algorithm>
#include <functional>
#include <array>
#include "rrt.h"

namespace kamaz {
namespace hagen {
        class KinoRRTStar{
            public:
                KinoRRTStar(RRTPlannerOptions options);
                ~KinoRRTStar() = default;
                
        };
    }
}
#endif