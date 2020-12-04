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
#include <ompl/base/SpaceInformation.h>
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

const double sideLen = 0.1;

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    struct Rectangle r1 = {-5.5, 5, 1, 5};
    struct Rectangle r2 = {-5.5, 5, 10.5, 1};
    struct Rectangle r3 = {5, -7.5, 1, 11.5};

    obstacles.insert(obstacles.end(),  {r1, r2, r3});

}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    struct Rectangle r1 = {-1, 1, 2, 9};
    struct Rectangle r2 = {-1, -10, 2, 9};

    obstacles.insert(obstacles.end(),  {r1, r2});
}

void planMultipleRobots(std::vector<Rectangle> & obstacles)
{
    /* Create R^2 space */
    ob::StateSpacePtr se2;
    auto r2 = std::make_shared<ob::RealVectorStateSpace>(2);

    /* Set x,y bounds */
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    r2->setBounds(bounds);

    /* Create SE2 space */
    auto so2 = std::make_shared<ob::SO2StateSpace>();
    se2 = r2 + so2;

    /* Simple setup for single robot */
    ompl::geometric::SimpleSetup ss(se2);
    ss.setStateValidityChecker(std::bind(isValidStateSquare, std::placeholders::_1, sideLen, obstacles));

    /* Set PRM to generate roadmap */
    auto planner = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    /* Goal & start states */
    int r1startX = -10.0;
    int r1startY = 5.0;
    int r1goalX = 8.0;
    int r1goalY = 5.0;

    int r2startX = -8.0;
    int r2startY = 5.0;
    int r2goalX = 6.0;
    int r2goalY = 5.0;

    int r3startX = 6.0;
    int r3startY = 5.0;
    int r3goalX = -10.0;
    int r3goalY = 5.0;

    int r4startX = 8.0;
    int r4startY = 5.0;
    int r4goalX = -8.0;
    int r4goalY = 5.0;

    /* Run the robot 1 PRM */
    ob::ScopedState<> r1Start(se2);
    r1Start[0] = r1startX;
    r1Start[1] = r1startY;
    ob::ScopedState<> r1goal(se2);
    r1goal[0] = r1goalX;
    r1goal[1] = r1goalY;
    ss.setStartAndGoalStates(r1Start, r1goal);
    /* Call solve to grow roadmap, clearQuery keeps the roadmap */
    ss.solve(1.0);
    //planner->growRoadmap(5.0);
    planner->clearQuery();

    /* Robot2 PRM */
    ob::ScopedState<> r2Start(se2);
    r2Start[0] = r2startX;
    r2Start[1] = r2startY;
    ob::ScopedState<> r2goal(se2);
    r2goal[0] = r2goalX;
    r2goal[1] = r2goalY;
    ss.setStartAndGoalStates(r2Start, r2goal);
    /* Call solve to grow roadmap, clearQuery keeps the roadmap */
    ss.solve(1.0);
    //planner->growRoadmap(5.0);
    planner->clearQuery();

    /* Robot3 PRM */
    ob::ScopedState<> r3Start(se2);
    r3Start[0] = r3startX;
    r3Start[1] = r3startY;
    ob::ScopedState<> r3goal(se2);
    r3goal[0] = r3goalX;
    r3goal[1] = r3goalY;
    ss.setStartAndGoalStates(r3Start, r3goal);
    /* Call solve to grow roadmap, clearQuery keeps the roadmap */
    ss.solve(1.0);
    //planner->growRoadmap(5.0);
    planner->clearQuery();

    /* Robot4 PRM */
    ob::ScopedState<> r4Start(se2);
    r4Start[0] = r4startX;
    r4Start[1] = r4startY;
    ob::ScopedState<> r4goal(se2);
    r4goal[0] = r4goalX;
    r4goal[1] = r4goalY;
    ss.setStartAndGoalStates(r4Start, r4goal);
    /* Call solve to grow roadmap, clearQuery keeps the roadmap */
    ss.solve(1.0);
    //planner->growRoadmap(5.0);
    planner->clearQuery();

    auto plannerData = std::make_shared<ob::PlannerData>(ss.getSpaceInformation());
    planner->getPlannerData(*plannerData);

    /* Create composite roadmap with PlannerData */
    std::vector<const ob::State*> roadmap;
    for(int i = 0; i < plannerData->numVertices(); i++) {
        const ob::State *roadState = plannerData->getVertex(i).getState();
        roadmap.push_back(roadState);
    }

    /* Now can start composite space */
    ob::StateSpacePtr se8;

    /* R^2 for each robot */
    auto r8 = std::make_shared<ob::RealVectorStateSpace>(8);
    ob::RealVectorBounds compositeBounds(8);
    compositeBounds.setLow(-10);
    compositeBounds.setHigh(10);
    r8->setBounds(compositeBounds);

    /* SO2 for each robot */
    auto so2r1 = std::make_shared<ob::SO2StateSpace>();
    auto so2r2 = std::make_shared<ob::SO2StateSpace>();
    auto so2r3 = std::make_shared<ob::SO2StateSpace>();
    auto so2r4 = std::make_shared<ob::SO2StateSpace>();

    /* Combine & SimpleSetup */
    se8 = r8 + so2r1 + so2r2 + so2r3 + so2r4;
    ompl::geometric::SimpleSetup compSS(se8);
    compSS.setStateValidityChecker(std::bind(isValidStateSquare, std::placeholders::_1, sideLen, obstacles));

    /* Composite goal & start states */
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
    compSS.setStartAndGoalStates(compositeStart, compositeGoal);

    /* Init dRRT */
    auto compPlanner = std::make_shared<ompl::geometric::dRRT>(compSS.getSpaceInformation());
    compPlanner->setNumRobots(4);
    compPlanner->setSideLen(sideLen);
    compPlanner->setRoadmap(roadmap);
    compPlanner->setRoadmapGraph(plannerData);

    compSS.setPlanner(compPlanner);

    /* Attempt to solve */
    ob::PlannerStatus solved = compSS.solve(60.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;

        ss.simplifySolution();
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
