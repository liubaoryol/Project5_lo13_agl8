///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 5
// Authors: Alex Liapis & Liuba Orlov Savko
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "dRRT.h"
#include "CollisionChecking.h"

using namespace ompl;
namespace ob = ompl::base;
namespace og = ompl::geometric;

const double sideLen = 0.9;

bool isValidState(const ompl::base::State *state, double sideLength, const std::vector<Rectangle> &obstacles)
{
    // extract position space for collision detection
    const auto cstate = state->as<ob::CompoundState>();
    const auto pos = cstate->as<ob::SE2StateSpace::StateType>(0);

    return isValidStateSquare(pos, sideLength, obstacles);;
}

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    struct Rectangle r1 = {-2.5, 0, 2, 2};
    struct Rectangle r2 = {0.5, 0, 2, 2};

    obstacles.insert(obstacles.end(),  {r1, r2});
}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    struct Rectangle r1 = {-7.5, 5, 7, 2};
    struct Rectangle r2 = {-7.5, -7, 7, 2};
    struct Rectangle r3 = {-2.5, -5, 2, 10};
    struct Rectangle r4 = {1, -1, 5, 2};
    struct Rectangle r5 = {6, -4, 2, 8};

    obstacles.insert(obstacles.end(),  {r1, r2, r3, r4, r5});
}

void planMultipleRobots(std::vector<Rectangle> & obstacles)
{
    // Create state space for the system
    ob::StateSpacePtr se2;

    // Create R^2 component of the space
    auto r2 = std::make_shared<ob::RealVectorStateSpace>(2);

    // Set the bound for R^2
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);

    // Set the bounds on R^2
    r2->setBounds(bounds);

    // Create the SO(2) component of the state space
    auto so2 = std::make_shared<ob::SO2StateSpace>();

    // Create the compound state space
    se2 = r2 + so2;

    // Create simple setup container
    ompl::geometric::SimpleSetup ss(se2);

    // Setup the StateValidityChecker
    og::SpaceInformation *si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker(
        [si, obstacles](const ob::State *state) { return isStateValid(si, state, obstacles); });

    // Specify a planning algorithm to use
    auto planner = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // We have 4 robots and each has its own start and goal states
    int r1startX = -5;
    int r1startY = 4;
    int r1goalX = -3;
    int r1goalY = 4;

    int r2startX = -3;
    int r2startY = 4;
    int r2goalX = -5;
    int r2goalY = 4;

    int r3startX = 1;
    int r3startY = 4;
    int r3goalX = 3;
    int r3goalY = 4;

    int r4startX = 3;
    int r4startY = 4;
    int r4goalX = 1;
    int r4goalY = 4;

    // Specify the start and goal states for 1st robot
    std::cout << "Robot 1 PRM" << std::endl;
    ob::ScopedState<> r1Start(se2);
    r1Start[0] = r1startX;
    r1Start[1] = r1startY;

    ob::ScopedState<> r1goal(se2);
    r1goal[0] = r1goalX;
    r1goal[1] = r1goalY;

    // set the start and goal states
    ss.setStartAndGoalStates(r1Start, r1goal);

    // Attempt to solve the problem within the givin time
    ss.solve(1.0);
    planner->growRoadmap(5.0);
    planner->clearQuery();

    // Robot2
    std::cout << "Robot 2 PRM" << std::endl;
    ob::ScopedState<> r2Start(se2);
    r2Start[0] = r2startX;
    r2Start[1] = r2startY;

    ob::ScopedState<> r2goal(se2);
    r2goal[0] = r2goalX;
    r2goal[1] = r2goalY;

    // set the start and goal states
    ss.setStartAndGoalStates(r2Start, r2goal);

    // Attempt to solve the problem within the givin time
    ss.solve(1.0);
    planner->growRoadmap(5.0);
    planner->clearQuery();

    // Robot3
    std::cout << "Robot 3 PRM" << std::endl;
    ob::ScopedState<> r3Start(se2);
    r3Start[0] = r3startX;
    r3Start[1] = r3startY;

    ob::ScopedState<> r3goal(se2);
    r3goal[0] = r3goalX;
    r3goal[1] = r3goalY;

    // set the start and goal states
    ss.setStartAndGoalStates(r3Start, r3goal);

    // Attempt to solve the problem within the givin time
    ss.solve(1.0);
    planner->growRoadmap(5.0);
    planner->clearQuery();

    // Robot4
    std::cout << "Robot 4 PRM" << std::endl;
    ob::ScopedState<> r4Start(se2);
    r4Start[0] = r4startX;
    r4Start[1] = r4startY;

    ob::ScopedState<> r4goal(se2);
    r4goal[0] = r4goalX;
    r4goal[1] = r4goalY;

    // set the start and goal states
    ss.setStartAndGoalStates(r4Start, r4goal);

    // Attempt to solve the problem within the givin time
    ss.solve(10.0);
    planner->growRoadmap(5.0);
    planner->clearQuery();

    auto plannerData = std::make_shared<ob::PlannerData>(ss.getSpaceInformation());
    planner->getPlannerData(*plannerData);

    // Create composite roadmap
    std::vector<const ob::State*> roadMap;

    for(int i = 0; i < plannerData->numVertices(); i++) {
        const ob::State *st = plannerData->getVertex(i).getState();
        roadMap.push_back(st);

        const ob::CompoundStateSpace::StateType& cs = *st->as<ob::CompoundStateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType& pos = *cs.as<ob::RealVectorStateSpace::StateType>(0);
    }

    // We need create composite state space
    std::cout << "Create composite state space" << std::endl;
    // Create state space for the system
    ob::StateSpacePtr se8;

    // Create R^2 component of the space
    auto r8 = std::make_shared<ob::RealVectorStateSpace>(8);

    // Set the bound for R^2
    ob::RealVectorBounds compositeBounds(8);
    compositeBounds.setLow(-10);
    compositeBounds.setHigh(10);

    // Set the bounds on R^2
    r8->setBounds(compositeBounds);

    // Create the SO(2) component for each robot
    auto so2R1 = std::make_shared<ob::SO2StateSpace>();
    auto so2R2 = std::make_shared<ob::SO2StateSpace>();
    auto so2R3 = std::make_shared<ob::SO2StateSpace>();
    auto so2R4 = std::make_shared<ob::SO2StateSpace>();

    // Create the compound state space
    se8 = r8 + so2R1 + so2R2 + so2R3 + so2R4;

    // Create simple setup container
    ompl::geometric::SimpleSetup robotSetup(se8);

    // Setup the StateValidityChecker
    og::SpaceInformation *rsi = robotSetup->getSpaceInformation().get();
    robotSetup->setStateValidityChecker(
        [rsi, obstacles](const ob::State *state) { return isStateValid(si, state, obstacles); });

    // Set start and goal state for composite states
    ob::ScopedState<> compositeStart(se8);
    compositeStart[0] = r1startX;
    compositeStart[1] = r1startY;
    compositeStart[2] = r2startX;
    compositeStart[3] = r2startY;
    compositeStart[4] = r3startX;
    compositeStart[5] = r3startY;
    compositeStart[6] = r4startX;
    compositeStart[7] = r4startY;

    ob::ScopedState<> compositeGoal(se8);
    compositeGoal[0] = r1goalX;
    compositeGoal[1] = r1goalY;
    compositeGoal[2] = r2goalX;
    compositeGoal[3] = r2goalY;
    compositeGoal[4] = r3goalX;
    compositeGoal[5] = r3goalY;
    compositeGoal[6] = r4goalX;
    compositeGoal[7] = r4goalY;

    // set the start and goal states
    robotSetup.setStartAndGoalStates(compositeStart, compositeGoal);

    // Specify a planning algorithm to use
    auto robotPlanner = std::make_shared<ompl::geometric::dRRT>(robotSetup.getSpaceInformation());
    robotPlanner->setNumRobots(4);
    robotPlanner->setSideLen(sideLen);
    robotPlanner->setRoadmap(roadMap);
    robotPlanner->setRoadmapGraph(plannerData);

    robotSetup.setPlanner(robotPlanner);

    // Attempt to solve the problem within the givin time
    ob::PlannerStatus solved = robotSetup.solve(60.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;

        ss.simplifySolution();
        //ss.getSolutionPath().print(std::cout);
        og::PathGeometric &path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        std::ofstream fout("path.txt");
        fout << "R2" << std::endl;
        path.printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /* argc */, char ** /* argv */)
{
    // Make environment
    std::vector<Rectangle> obstacles;

    int env;
    do
    {
        std::cout << "Select an environment" << std::endl;
        std::cout << " (1) Environment 1,  Multi-robots using dRRT" << std::endl;
        std::cout << " (2) Environment 2,  Multi-robots using dRRT" << std::endl;

        std::cin >> env;
    } while (env != 1 && env != 2);

    if (env == 1)
    {
        makeEnvironment1(obstacles);
        planMultipleRobots(obstacles);
    }
    else if (env == 2)
    {
        makeEnvironment2(obstacles);
        planMultipleRobots(obstacles);
    }
    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;
    return 0;
}
