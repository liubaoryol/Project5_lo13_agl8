///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Liubove Orlov Savko and Robert Black
//////////////////////////////////////

#include <iostream>
#include "ompl/tools/benchmark/Benchmark.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>

#include <omplapp/config.h>

// Your random tree planner
#include "RTP.h"

using namespace ompl;



void benchmarkCubicles()
{
    // plan in SE3
    app::SE3RigidBodyPlanning setup;
	std::string benchmark_name;


    // load the robot and the environment
	benchmark_name = std::string("cubicles");
	std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
	std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
	setup.setRobotMesh(robot_fname);
	setup.setEnvironmentMesh(env_fname);

    // define start state
	base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
	start->setX(-4.96);
	start->setY(-40.62);
	start->setZ(70.57);
	start->rotation().setIdentity();

    // define goal state
	base::ScopedState<base::SE3StateSpace> goal(start);
	goal->setX(200.49);
	goal->setY(-40.62);
	goal->setZ(70.57);
	goal->rotation().setIdentity();

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // we call setup just so print() can show more information
    setup.setup();


	std::vector<double> cs(3);
	cs[0] = 35;
	cs[1] = 35;
	cs[2] = 35;
	setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);


    tools::Benchmark::Request request;
    request.maxTime = 5.0;
    request.maxMem = 10000.0;
    request.runCount = 50;
    request.displayProgress = true;

    tools::Benchmark b(setup, "Problem 3 - Cubicles");

    // Adding planners with default values for range
    b.addPlanner(std::make_shared<geometric::RTP>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));

    b.benchmark(request);
    b.saveResultsToFile();

}

void benchmarkTwistycool()
{
    // plan in SE3
    app::SE3RigidBodyPlanning setup;
	std::string benchmark_name;

    benchmark_name = std::string("Twistycool");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(270.);
    start->setY(160.);
    start->setZ(-200.);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(270.);
    goal->setY(160.);
    goal->setZ(-400.);
    goal->rotation().setIdentity();

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 350.);
    bounds.setHigh(1, 250.);
    bounds.setHigh(2, -150.);
    bounds.setLow(0, 200.);
    bounds.setLow(1, 75.);
    bounds.setLow(2, -450.);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    // we call setup just so print() can show more information
    setup.setup();


    tools::Benchmark::Request request;
    request.maxTime = 60.0;
    request.maxMem = 10000.0;
    request.runCount = 50;
    request.displayProgress = true;

    tools::Benchmark b(setup, "Problem 3 - Twistycool");
    
    // Adding planners with default values for range
    b.addPlanner(std::make_shared<geometric::RTP>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));

    b.benchmark(request);
    b.saveResultsToFile();

}

int main(int /* argc */, char ** /* argv */)
{
    int environment;

    do
    {
        std::cout << "Benchmark for: " << std::endl;
        std::cout << " (1) Cubicles" << std::endl;
        std::cout << " (2) Twistycool" << std::endl;

        std::cin >> environment;
    } while (environment < 1 || environment > 2);

    switch (environment)
    {
        case 1:
            benchmarkCubicles();
            break;
        case 2:
            benchmarkTwistycool();
            break;
        default:
            std::cerr << "Invalid Environment!" << std::endl;
            break;
    }

    return 0;
}
