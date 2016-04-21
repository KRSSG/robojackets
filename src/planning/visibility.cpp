#include "robojackets/planning/visibility.hpp"  
#include "robojackets/planning/dijkstra.hpp"

#include <ros/console.h>
#include <Constants.hpp>
#include <Utils.hpp>
// #include <protobuf/LogFrame.pb.h>
#include "TrapezoidalMotion.hpp"
#include "planning/Util.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>

using namespace Geometry2d;
namespace Planning {

VisibilityGraph::VisibilityGraph() {

}
void VisibilityGraph::addBotObstacle(Point bot, float radius) {
  // add the bounding box of the bot as an obstacle.
  // form the set of line segments that are the bot.
  vector<Segment> botSegments;
  vector<Point> botPoints;
  int xArr[] = {-1,+1,+1,-1}, yArr[] = {+1, +1, -1, -1};
  Point prev = bot+Point(xArr[3], yArr[3])*radius;
  for (int i= 0; i < 4; i++) {
    Point p = bot+Point(xArr[i], yArr[i])*radius;    
    botSegments.push_back(Segment(p, prev));
    prev = p;
    botPoints.push_back(p);
  }
  // add all the bot's segments
  segments.insert(segments.end(), botSegments.begin(), botSegments.end());
  // add each vertex to the graph.
  int numOldVertices = vertices.size();
  for (int i = 0; i < botPoints.size(); i++) {
    addVertex(botPoints[i], numOldVertices);
  }
}

void VisibilityGraph::addVertex(Point p, int numOldVertices) {
  int me = vertices.size();
  graph.push_back(vector<int>());
  vertices.push_back(p);
  for (int j = 0; j < numOldVertices; j++) {
    // try edge from me to j
    Segment curSeg(vertices[me], vertices[j]);
    // make sure this edge intersects NONE of the segments, including bot's own segments.
    bool intersects = false;
    for (int k = 0; k < segments.size(); k++) {
      if (LineSegmentIntersection(curSeg, segments[k])) {
        intersects = true;
        break;
      }
    }
    if (!intersects) {
      graph[j].push_back(me);
      graph[me].push_back(j);
    }
  }
}

void VisibilityGraph::addBallObstacle(Point ball, float radius) {

}
void VisibilityGraph::addWallObstacle(Segment seg) {

}
std::list<vertex_t> VisibilityGraph::getPathUnoptimized(Point p1, Point p2) {
  // need to add p1 and p2 to the graph!
  // right now, NOT correcting the graph aftwerwards!
  // i.e. this function can be called only once, then VisibilityGraph object should not be used again.

 
  // add the 2 vertices;
  int idx1 = vertices.size(), idx2 = idx1+1;
  addVertex(p1, vertices.size());
  addVertex(p2, vertices.size());
  // construct dijkstra object.
  using namespace Dijkstra;
  assert(graph.size() == vertices.size());
  adjacency_list_t list(vertices.size());
  for (int i = 0; i < graph.size(); i++) {
    for (int j = 0; j < graph[i].size(); j++) {
      list[i].push_back(neighbor(graph[i][j], (vertices[i]-vertices[graph[i][j]]).mag()));
    }
  }
  // print the graph
  // for (int i = 0 ; i < graph.size(); i++) {
  //   printf("%d: ", i);
  //   for (int j = 0; j < graph[i].size(); j++) {
  //     printf("%d (%lf) ", graph[i][j], (vertices[i]-vertices[graph[i][j]]).mag());
  //   }
  //   printf("\n");
  // }
  std::vector<weight_t> min_distance;
  std::vector<vertex_t> previous;
  DijkstraComputePaths(idx1, list, min_distance, previous);
  // std::cout << "Distance from 0 to 4: " << min_distance[4] << std::endl;
  std::list<vertex_t> pth = DijkstraGetShortestPathTo(idx2, previous);
  cout << "total number of vertices: " << vertices.size();
  // std::cout << "Path : ";
  // std::copy(pth.begin(), pth.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
  

  return pth;
}

bool VisibilityGraph::LineSegmentIntersection(Segment s1, Segment s2) {
  // check if they share an end point; if they do, then no intersect4ion!
  if (s1.pt[0] == s2.pt[0] || s1.pt[0] == s2.pt[1] ||
      s1.pt[1] == s2.pt[0] || s1.pt[1] == s2.pt[1])
    return false;
  return s1.intersects(s2);
}

Planning::InterpolatedPath* VisibilityGraph::runVG(
        MotionInstant start, MotionInstant goal,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles) {
  // printf("run vg called!! <>< >< >< > <> < >< >< ><\n");
  InterpolatedPath *path = new InterpolatedPath();
  path->setStartTime(RJ::timestamp());
  // clean all data structures... probably should not have done this way
  segments.clear();
  graph.clear();
  vertices.clear();

  // HACK: assume each obstacle is a circle, and is a bot.
  for (const auto& shape : obstacles->shapes()) {
    Circle *bot=static_cast<Circle*>(shape.get());
    // increase radius a lil bit here so that replanning is not done that often.
    addBotObstacle(bot->center, bot->radius()*2);
  }
  // convert to robojeckets path object;
  // add usign this weird meethod, copied from addPath in Tree.cpp
  std::list<vertex_t> pth = getPathUnoptimized(start.pos, goal.pos);
  {
    path->waypoints.reserve(pth.size());
    for (auto v : pth) {
      path->waypoints.emplace_back(MotionInstant(vertices[v], Geometry2d::Point()),
                                    0);
    }    
  }
  ROS_INFO_NAMED("path", "non optimized path size = %lu", path->size());
  path = optimize(*path, obstacles, motionConstraints, start.vel, goal.vel);
  return path;
}

std::unique_ptr<Path> VisibilityGraph::run(
    MotionInstant start, const MotionCommand* cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, std::unique_ptr<Path> prevPath) {
    // This planner only works with commands of type 'PathTarget'
    assert(cmd->getCommandType() == Planning::MotionCommand::PathTarget);
    Planning::PathTargetCommand target =
        *static_cast<const Planning::PathTargetCommand*>(cmd);

    MotionInstant goal = target.pathGoal;

    // Simple case: no path
    if (start.pos == goal.pos) {
        InterpolatedPath* path = new InterpolatedPath();
        path->setStartTime(RJ::timestamp());
        path->waypoints.emplace_back(
            MotionInstant(start.pos, Geometry2d::Point()), 0);
        return unique_ptr<Path>(path);
    }

    // Locate a goal point that is obstacle-free
    // boost::optional<Geometry2d::Point> prevGoal;
    // if (prevPath) prevGoal = prevPath->end().pos;
    // goal.pos = EscapeObstaclesPathPlanner::findNonBlockedGoal(
    //     goal.pos, prevGoal, *obstacles);

    // Replan if needed, otherwise return the previous path unmodified
    if (SingleRobotPathPlanner::shouldReplan(start, motionConstraints,
                                             obstacles, prevPath.get())) {        
        // Run bi-directional RRT to generate a path.
        InterpolatedPath* path =
            runVG(start, goal, motionConstraints, obstacles);

        // If RRT failed, the path will be empty, so we need to add a single
        // point to make it valid.
        if (path && path->waypoints.empty()) {
            ROS_WARN("new path has no waypoints!!");
            path->waypoints.emplace_back(
                MotionInstant(start.pos, Geometry2d::Point()), 0);
        }
        return unique_ptr<Path>(path);
    } else {
        return prevPath;
    }
}


using namespace Eigen;











/// helpers


InterpolatedPath* VisibilityGraph::optimize(
    InterpolatedPath& path, const Geometry2d::ShapeSet* obstacles,
    const MotionConstraints& motionConstraints, Geometry2d::Point vi,
    Geometry2d::Point vf) {
    unsigned int start = 0;

    if (path.empty()) {
        delete &path;
        return nullptr;
    }

    vector<InterpolatedPath::Entry>& pts = path.waypoints;

    // The set of obstacles the starting point was inside of
    const auto startHitSet = obstacles->hitSet(pts[start].pos());
    int span = 2;
    while (span < pts.size()) {
        bool changed = false;
        for (int i = 0; i + span < pts.size(); i++) {
            bool transitionValid = true;
            const auto newHitSet = obstacles->hitSet(
                Geometry2d::Segment(pts[i].pos(), pts[i + span].pos()));
            if (!newHitSet.empty()) {
                for (std::shared_ptr<Geometry2d::Shape> hit : newHitSet) {
                    if (startHitSet.find(hit) == startHitSet.end()) {
                        transitionValid = false;
                        break;
                    }
                }
            }

            if (transitionValid) {
                for (int x = 1; x < span; x++) {
                    pts.erase(pts.begin() + i + 1);
                }
                changed = true;
            }
        }

        if (!changed) span++;
    }
    // Done with the path
    return cubicBezier(path, obstacles, motionConstraints, vi, vf);
}

// already defined in RRTPlanner, we just declare here...
float getTime(InterpolatedPath& path, int index,
              const MotionConstraints& motionConstraints, float startSpeed,
              float endSpeed);/* {
    return Trapezoidal::getTime(
        path.length(0, index), path.length(), motionConstraints.maxSpeed,
        motionConstraints.maxAcceleration, startSpeed, endSpeed);
}
*/
// TODO: Use targeted end velocity
InterpolatedPath* VisibilityGraph::cubicBezier(
    InterpolatedPath& path, const Geometry2d::ShapeSet* obstacles,
    const MotionConstraints& motionConstraints, Geometry2d::Point vi,
    Geometry2d::Point vf) {
    int length = path.waypoints.size();
    int curvesNum = length - 1;
    if (curvesNum <= 0) {
        delete &path;
        return nullptr;
    }

    // TODO: Get the actual values
    vector<double> pointsX(length);
    vector<double> pointsY(length);
    vector<double> ks(length - 1);
    vector<double> ks2(length - 1);

    for (int i = 0; i < length; i++) {
        pointsX[i] = path.waypoints[i].pos().x;
        pointsY[i] = path.waypoints[i].pos().y;
    }
    float startSpeed = vi.mag();

    // This is pretty hacky;
    Geometry2d::Point startDirection =
        (path.waypoints[1].pos() - path.waypoints[0].pos()).normalized();
    if (startSpeed < 0.3) {
        startSpeed = 0.3;
        vi = startDirection * startSpeed;
    } else {
        vi = vi.mag() * (startDirection + vi.normalized()) / 2.0 * 0.8;
    }

    const float endSpeed = vf.mag();

    for (int i = 0; i < curvesNum; i++) {
        ks[i] = 1.0 /
                (getTime(path, i + 1, motionConstraints, startSpeed, endSpeed) -
                 getTime(path, i, motionConstraints, startSpeed, endSpeed));
        ks2[i] = ks[i] * ks[i];
        if (std::isnan(ks[i])) {
            delete &path;
            return nullptr;
        }
    }

    VectorXd solutionX = cubicBezierCalc(vi.x, vf.x, pointsX, ks, ks2);
    VectorXd solutionY = cubicBezierCalc(vi.y, vf.y, pointsY, ks, ks2);

    Geometry2d::Point p0, p1, p2, p3;
    vector<InterpolatedPath::Entry> pts;
    const int interpolations = 10;
    double time = 0;

    for (int i = 0; i < curvesNum; i++) {
        p0 = path.waypoints[i].pos();
        p3 = path.waypoints[i + 1].pos();
        p1 = Geometry2d::Point(solutionX(i * 2), solutionY(i * 2));
        p2 = Geometry2d::Point(solutionX(i * 2 + 1), solutionY(i * 2 + 1));

        for (int j = 0; j < interpolations; j++) {
            double k = ks[i];
            float t = (((float)j / (float)(interpolations)));
            Geometry2d::Point pos =
                pow(1.0 - t, 3) * p0 + 3.0 * pow(1.0 - t, 2) * t * p1 +
                3 * (1.0 - t) * pow(t, 2) * p2 + pow(t, 3) * p3;
            t = ((float)j / (float)(interpolations)) / k;
            // 3 k (-(A (-1 + k t)^2) + k t (2 C - 3 C k t + D k t) + B (1 - 4 k
            // t + 3 k^2 t^2))
            Geometry2d::Point vel =
                3 * k * (-(p0 * pow(-1 + k * t, 2)) +
                         k * t * (2 * p2 - 3 * p2 * k * t + p3 * k * t) +
                         p1 * (1 - 4 * k * t + 3 * pow(k, 2) * pow(t, 2)));
            pts.emplace_back(MotionInstant(pos, vel), time + t);
        }
        time += 1.0 / ks[i];
    }
    pts.emplace_back(MotionInstant(path.waypoints[length - 1].pos(), vf), time);
    path.waypoints = pts;
    return &path;
}

VectorXd VisibilityGraph::cubicBezierCalc(double vi, double vf,
                                     vector<double>& points, vector<double>& ks,
                                     vector<double>& ks2) {
    int curvesNum = points.size() - 1;

    if (curvesNum == 1) {
        VectorXd vector(2);
        vector[0] = vi / (3.0 * ks[0]) + points[0];
        vector[1] = points[curvesNum] - vf / (3 * ks[curvesNum - 1]);
        return vector;
    } else {
        int matrixSize = curvesNum * 2;
        MatrixXd equations = MatrixXd::Zero(matrixSize, matrixSize);
        VectorXd answer(matrixSize);
        equations(0, 0) = 1;
        answer(0) = vi / (3.0 * ks[0]) + points[0];
        equations(1, matrixSize - 1) = 1;
        answer(1) = points[curvesNum] - vf / (3 * ks[curvesNum - 1]);

        int i = 2;
        for (int n = 0; n < curvesNum - 1; n++) {
            equations(i, n * 2 + 1) = ks[n];
            equations(i, n * 2 + 2) = ks[n + 1];
            answer(i) = (ks[n] + ks[n + 1]) * points[n + 1];
            i++;
        }

        for (int n = 0; n < curvesNum - 1; n++) {
            equations(i, n * 2) = ks2[n];
            equations(i, n * 2 + 1) = -2 * ks2[n];
            equations(i, n * 2 + 2) = 2 * ks2[n + 1];
            equations(i, n * 2 + 3) = -ks2[n + 1];
            answer(i) = points[n + 1] * (ks2[n + 1] - ks2[n]);
            i++;
        }

        ColPivHouseholderQR<MatrixXd> solver(equations);
        VectorXd solution = solver.solve(answer);
        return solution;
    }
}
}
