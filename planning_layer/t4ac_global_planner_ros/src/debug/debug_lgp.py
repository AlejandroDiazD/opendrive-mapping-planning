"""
===================
Only for developers
===================
File to debug and test the LaneGraphPlanner (LGP)
"""

import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from lane_graph_planner import LaneGraphPlanner

LGP = LaneGraphPlanner("CampusUAH_v1_7", 0)

# route = LGP.calculate_global_route((216.91, -59.18, 0), (116.30, -59.36, 0))
# [print(x[0], x[1]) for x in route]

route = LGP.calculate_global_route((171.68, 167.53, 0.0), (185.87, 121.28, 0.0))
[print(x[0], x[1]) for x in route]


print(">>> OK")