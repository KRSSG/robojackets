#include <planning/RRTPlanner.hpp>
#include <planning/MotionInstant.hpp>
#include <planning/MotionCommand.hpp>
#include <planning/MotionConstraints.hpp>
#include <Configuration.hpp>
#include <planning/Path.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include <iostream>

using namespace std;
int main(int argc, char const *argv[])
{
  using namespace Planning;
  std::shared_ptr<Configuration> config =
        Configuration::FromRegisteredConfigurables();
  MotionInstant mi({1,1},{0,0});
  std::unique_ptr<MotionCommand> mc(new PathTargetCommand(mi));
  std::unique_ptr<SingleRobotPathPlanner> planner = PlannerForCommandType(mc->getCommandType());
  Geometry2d::ShapeSet obstacles;
  std::unique_ptr<Path> path = planner->run(MotionInstant(), mc.get(), MotionConstraints(), &obstacles, std::move(nullptr));
  printf("path duration = %f\n", path->getDuration());
  int n = 10;
  float tMax = path->getDuration();
  for (float t = 0; t < tMax; t += tMax/n) {
    boost::optional<MotionInstant> mi = path->evaluate(t);
    if (mi) {
      cout << t << ", " << mi->pos << ", " << mi->vel << endl;
    }
  }
  printf("hello world!\n");
  return 0;
}