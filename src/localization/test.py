from path_planning.global_path import GlobalPathPlanner

path_planner = GlobalPathPlanner()
steps = path_planner.direction(start='101 Rolling Green Dr Amherst, MA 01002', destination="94 Old Main St, Deerfield, MA 01342")
print(len(steps))
for i in steps:
    print(i)
