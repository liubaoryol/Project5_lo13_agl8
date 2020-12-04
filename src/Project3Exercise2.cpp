///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 5
// Authors: Liubove Orlov Savko and Alex Liapis
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include "CollisionChecking.h"
#include "RTP.h"
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>

using namespace ompl;
namespace ob = ompl::base;
namespace og = ompl::geometric;


void planMultipleRobots(std::vector<Rectangle> & obstacles)
{    ''' 
    Create a composite roadmap for four square robots of length 0.9; a little smaller than the passages
    '''
    // Robot side length
    double sideLen = 1.5; 
    // Create workspace
    auto space(std::make_shared<ob::SE2StateSpace>());

    ob::RealVectorBounds bounds(2);
    bounds.setLow(10);
    bounds.setHigh(10);
    space->setBounds(bounds);
    
    og::SimpleSetup ss(space);

    ss.setStateValidityChecker(std::bind(isValidStateSquare, std::placeholders::_1, sideLen, obstacles));

    //ssptr->setStateValidityChecker(
      //  [&ss, &obstacles](const ob::State *state) { return isStateValid(ss.getSpaceInformation().get(), state, obstacles); });

    ob::ScopedState<> start(space);
    start[0] = -2.5;
    start[1] = 2;
    start[2] = 0;
     
    ob::ScopedState<> goal(space);
    goal[0] = -1.5;
    goal[1] = 2;
    goal[2] = 0;



    ss.setStartAndGoalStates(start, goal);

    // Since all robots are the same moving in the same workspace, we will use one PRM
    //auto si = std::make_shared<ob::SpaceInformation>(space);
    auto prm = std::make_shared<og::PRM>(ss.getSpaceInformation());
    ss.setPlanner(prm);


    ob::PlannerStatus solved = ss.solve(5.0);

    auto plannerData = std::make_shared<ob::PlannerData>(ss.getSpaceInformation());
    prm->getPlannerData(*plannerData);

    // Create composite roadmap
    std::vector<const ob::State*> roadmap;

    for ( int i = 0; i < plannerData->numVertices(); i++) {
        const ob::State *st = plannerData->getVertex(i).getState();
        roadmap.push_back(st);

        const ob::CompoundStateSpace::StateType& cs = *st->as<ob::CompoundStateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType& pos = *cs.as<ob::RealVectorStateSpace::StateType>(0);
    }                

    // We need create composite state space
    std::cout << "Creat composite state space" << std::endl;
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
    se8 = r8 + so2R1 + so2R + so2R3 + so2R4;

    // Create simple setup container
    ompl::geometric::SimpleSetup robotSetup(se8);

    // Setup the StateValidityChecker
    robotSetup.setStateValidityChecker(std::bind(isValidStateSquare, _1, 1, obstacles));

    // Set start and goal state for composite states
    ob::ScopedState<> compositeStart(se8);
    // Start position for R1
    compositeStart[0] = -5;
    compositeStart[1] = 4;
    // Start position for R2
    compositeStart[2] = -3;
    compositeStart[3] = 4;
    // Start position for R3 
    compositeStart[4] = 1;
    compositeStart[5] = 4;
    // start position for R4
    compositeStart[6] = 3;
    compositeStart[7] = 4;
    
    ob::ScopedState<> compositeGoal(se8);
    compositeGoal[0] = -3;
    compositeGoal[1] = 4;
    compositeGoal[2] = -5;
    compositeGoal[3] = 4;
    compositeGoal[4] = 3;
    compositeGoal[5] = 4;
    compositeGoal[6] = 1;
    compositeGoal[7] = 4;

    // set the start and goal states
    robotSetup.setStartAndGoalStates(compositeStart, compositeGoal);

    // Specify a planning algorithm to use
    auto robotPlanner = std::make_shared<ompl::geometric::dRRT>(robotSetup.getSpaceInformation());
    robotPlanner->getRoadMap(roadmap);
    robotPlanner->getRoadMapData(plannerData);

    robotSetup.setPlanner(robotPlanner);

    // Attempt to solve the problem within the givin time
    ob::PlannerStatus solved = robotSetup.solve(30.0);

    
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
    

void planSingleRobot(const std::vector<Rectangle> &obstacles)
{
    // Step 1) Create the state (configuration) space for your system
    ompl::base::StateSpacePtr se2;

    // Create the R^2 component of the space
    auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);

    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);  // x and y have a minimum of -2
    bounds.setHigh(10);  // x and y have a maximum of 2

    // Set the bounds on R^2
    r2->setBounds(bounds);

    // Create the SO(2) component of the state space
    auto so2 = std::make_shared<ompl::base::SO2StateSpace>();

    // Create the compound state space (R^2 X SO(2) = SE(2))
    se2 = r2 + so2;

    // Step 2) Create the SimpleSetup container for the motion planning problem.
    ompl::geometric::SimpleSetup ss(se2);

    // Step 3) Setup the StateValidityChecker
    ss.setStateValidityChecker(std::bind(isValidStateSquare, _1, 0.3, obstacles));

    // Step 4) Specify the start and goal states
    ompl::base::ScopedState<> start(se2);
    start[0] = -8.0;
    start[1] = 5.0;
    start[2] = 0.0;

    ompl::base::ScopedState<> goal(se2);
    goal[0] = 4.0;
    goal[1] = -6.0;
    goal[2] = 0.0;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Step 5) (optional) Specify a planning algorithm to use
    auto planner = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // Step 6) Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(20.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
        ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric &path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path.txt");
        fout << "SE2" << std::endl;
        path.printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}


void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    struct Rectangle r1 = {-10, -10, 20, 10};
    struct Rectangle r2 = {-10, 0, 9, 4};
    struct Rectangle r3 = {1, 0, 9, 4};
    struct Rectangle r4 = {-10, 6, 20, 4};

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


int main(int /* argc */, char ** /* argv */)
{
    // Make environment
    std::vector<Rectangle> obstacles;

    int env;
    do
    {
        std::cout << "Select an environment" << std::endl;
        std::cout << " (1) env1" << std::endl;
        std::cout << " (2) env2" << std::endl;

        std::cin >> env;
    } while (env != 1 && env != 2);
    
    if (env == 1)
    {
        makeEnvironment1(obstacles);
    }
    else if (env == 2)
    {
        makeEnvironment2(obstacles);
    }
    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;
    

    int choice;
    do
    {
        std::cout << "Select a plan" << std::endl;
        std::cout << " (1) Multi-robots using dRRT" << std::endl;
        std::cout << " (2) Single robot using PRM" << std::endl;

        std::cin >> choice;
    } while (choice != 1 && choice != 2);

    // Planning
    if (choice == 1)
    {
        planMultipleRobots(obstacles);
    }
    else if (choice == 2)
    {
        planSingleRobot(obstacles);
    }
    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}


