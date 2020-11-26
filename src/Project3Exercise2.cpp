///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Robert Black and Liubove Orlov Savko
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include "CollisionChecking.h"
#include "RTP.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>

using namespace ompl;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void planPoint(const std::vector<Rectangle> &obstacles)
{
    int nbr_robots = 4;
    int dim = pow(2,4);
    //bool isStateValid(const ob::State *state)
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(dim));

    ob::RealVectorBounds bounds(dim);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);

    ss.setStateValidityChecker(std::bind(isValidStatePoint, std::placeholders::_1, obstacles));


    ob::ScopedState<> start(space);
    start.random();

    ob::ScopedState<> goal(space);
    goal.random()

    ss.setStartAndGoalStates(start, goal);

    //auto si = std::make_shared<ob::SpaceInformation>(space);
    auto rtp = std::make_shared<og::RTP>(ss.getSpaceInformation());
    ss.setPlanner(rtp);


    ob::PlannerStatus solved = ss.solve(5.0);
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
    {
        std::cout << "No solution found" << std::endl;
    }
}


/*
void planBox(const std::vector<Rectangle> &obstacles)
{
    //bool isStateValid(const ob::State *state)
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);

    double sideLen = 1.0;
    ss.setStateValidityChecker(std::bind(isValidStateSquare, std::placeholders::_1, sideLen, obstacles));

    ob::ScopedState<> start(space);
    start[0] = -5.4;
    start[1] = 2.4;
    start[2] = 0.45;
        
    ob::ScopedState<> goal(space);
    goal[0] = 5;
    goal[1] = -3;
    goal[2] = 0;

    ss.setStartAndGoalStates(start, goal);


    //auto si = std::make_shared<ob::SpaceInformation>(space);
    auto rtp = std::make_shared<og::RTP>(ss.getSpaceInformation());
    ss.setPlanner(rtp);


    ob::PlannerStatus solved = ss.solve(5.0);
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


*/

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    struct Rectangle r1 = {-10, -0.5, 1, 5};
    struct Rectangle r2 = {-12, -9, 2, 2};
    struct Rectangle r3 = {-3, 6, 3, 3};
    struct Rectangle r4 = {-7, -5, 10, 2};
    struct Rectangle r5 = {0, -1.75, 10, 4};

    obstacles.insert(obstacles.end(),  {r1, r2, r3, r4, r5});
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

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) Environment 1" << std::endl;
        std::cout << " (2) Environment 2" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
