///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 5
// Authors: Liubove Orlov Savko and Alex Liapis
//////////////////////////////////////

#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include "dRRT.h"
#include "math.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include <ompl/base/StateValidityChecker.h>
#include "CollisionChecking.h"

ompl::geometric::dRRT::dRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "dRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &dRRT::setRange, &dRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &dRRT::setGoalBias, &dRRT::getGoalBias, "0.:.05:1.");
}

ompl::geometric::dRRT::~dRRT()
{
    freeMemory();
}

void ompl::geometric::dRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
    roadmapGraph = nullptr;
}

void ompl::geometric::dRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::dRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::dRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;
///
        /* define State list for x,y,theta of each robot */
        ompl::base::ScopedState<> newState(si_);
        // std::vector<base::SE2StateSpace::StateType>(0)> robotStates;
        // for (int i = 0; i < numRobots_; i++) {
        //     robotStates[i]->setX(0.0);
        //     robotStates[i]->setY(0.0);
        //     robotStates[i]->setYaw(0.0);
        // }
        for (int i = 0; i < (numRobots_ * 3); i++) {
            newState[i] = 0;
        }
///
        // Get position coordinate for random state
        const ompl::base::CompoundStateSpace::StateType& rmotionCState = *rstate->as<ompl::base::CompoundStateSpace::StateType>();
        const ompl::base::RealVectorStateSpace::StateType& rmotionPosition = *rmotionCState.as<ompl::base::RealVectorStateSpace::StateType>(0);
///
        // Get position coordinate for nearest state
        const ompl::base::CompoundStateSpace::StateType& nmotionCState = *nmotion->state->as<ompl::base::CompoundStateSpace::StateType>();
        const ompl::base::RealVectorStateSpace::StateType& nmotionPosition = *nmotionCState.as<ompl::base::RealVectorStateSpace::StateType>(0);
///
        /* Create the list of robots as obstacles at their initial locations. */
        std::vector<Rectangle> robotInitialObstacles;
        for (int i = 0; i < numRobots_; i++) {
            Rectangle robotRect;
            // rectangle position at left corner
            robotRect.x = newState[i * 2] - sideLen;
            robotRect.y = newState[i * 2 + 1] - sideLen;
            robotRect.width = sideLen;
            robotRect.height = sideLen;

            robotInitialObstacles.push_back(robotRect);
        }

        for (int i = 0; i < numRobots_; i++) {

            // Find the index of the nearest state in the roadmap
            unsigned int stateIndex = -1;
            for (int j = 0; j < roadmap.size(); j++) {
                const ompl::base::State *roadmapState = roadmap[j];
                const ompl::base::CompoundStateSpace::StateType &roadmapStateCState = *roadmapState->as<ompl::base::CompoundStateSpace::StateType>();
                const ompl::base::RealVectorStateSpace::StateType &roadmapPosition = *roadmapStateCState.as<ompl::base::RealVectorStateSpace::StateType>(0);

                if (nmotionPosition[i * 2] == roadmapPosition[0] && nmotionPosition[2 * i + 1] == roadmapPosition[1]) {
                    stateIndex = j;
                    break;
                }
            }
///
            // Find connected vertices with nearest state
            // getEdges (unsigned int v, std::vector< unsigned int > &edgeList) const
            std::vector<unsigned int> edgeList;
            roadmapGraph->getEdges(stateIndex, edgeList);
            double max = -1;
            bool first = true;
///
            /* For each connected roadmap state */
            for (int k = 0; k < edgeList.size(); k++) {
                const ompl::base::State *state = roadmap[edgeList[k]];
                const ompl::base::CompoundStateSpace::StateType& stateCState = *state->as<ompl::base::CompoundStateSpace::StateType>();
                const ompl::base::RealVectorStateSpace::StateType &statePosition = *stateCState.as<ompl::base::RealVectorStateSpace::StateType>(0);
                const ompl::base::SO2StateSpace::StateType &stateTheta = *stateCState.as<ompl::base::SO2StateSpace::StateType>(1);

                const auto state2 = roadmap[edgeList[k]];
                const auto cstate = state2->as<ompl::base::CompoundState>();
                const auto stateSE2 = cstate->as<ompl::base::SE2StateSpace::StateType>(0);

                double randomX = rmotionPosition[i * 2];
                double randomY = rmotionPosition[i * 2 + 1];

                double nearestX = nmotionPosition[i * 2];
                double nearestY = nmotionPosition[i * 2 + 1];

                double roadmapX = statePosition[0];
                double roadmapY = statePosition[1];

                /* Find the angle between random & roadmap state, use the best roadmap state. */
                /* Side A: Random State to Nearest State */
                double sideA_X = randomX - nearestX;
                double sideA_Y = randomY - nearestY;

                /* Side B: Roadmap State to Nearest State */
                double sideB_X = roadmapX - nearestX;
                double sideB_Y = roadmapY - nearestY;

                double cosValue = -1;
                /* Ensure the sides are non-zero */
                if (!(sideA_X == 0 && sideA_Y == 0) && !(sideB_X == 0 && sideB_Y == 0)) {
                    double ADot = sideA_X * sideB_X;
                    double BDot = sideA_Y * sideB_Y;

                    double magA = sqrt(sideA_X * sideA_X + sideA_Y * sideA_Y);
                    double magB = sqrt(sideB_X * sideB_X + sideB_Y * sideB_Y);

                    cosValue = (ADot + BDot) / (magA * magB);
                }
///
                // /* Create the list of robots as obstacles at their final locations. */
                bool collide = false;
                std::vector<std::vector<int>> collisionList;

                std::vector<Rectangle> robotFinalObstacles;
                if (!first) {
                    for (int j = 0; j < i; j++) {
                        Rectangle robotRect;
                        // rectangle position at left corner
                        robotRect.x = newState[j * 2] - sideLen;
                        robotRect.y = newState[j * 2 + 1] - sideLen;
                        robotRect.width = sideLen;
                        robotRect.height = sideLen;

                        robotFinalObstacles.push_back(robotRect);
                    }
                    std::cout << "getX" << stateSE2->getX() << std::endl;
                    /* Then check if the robots are collision-free. */
                    if (robotFinalObstacles.size() > 0) {
                        if (!isValidStateSquare(stateSE2, sideLen, robotFinalObstacles)) {
                             collide = true;
                             break;
                        }
                    }
                }

                /* Check if any possible invalid initial-final location collisions occurs. */
                // for (int j = 0; j < robotInitialObstacles.size(); j++) {
                //     std::vector<Rectangle> initialObs = robotInitialObstacles;
                //     Rectangle rectInitial = initialObs[j];
                //     initialObs.erase(initialObs.begin() + j);
                //
                //     std::vector<int> coll;
                //     for (int k = 0; k < robotInitialObstacles.size(); k++) {
                //         if (k == j) {
                //           continue;
                //         }
                //         std::vector<Rectangle> robotObs;
                //         robotObs.push_back(initialObs[k]);
                //         if (!isValidStateSquare(stateSE2, sideLen, robotInitialObstacles)) {
                //             coll.push_back(k);
                //             coll.push_back(j);
                //             collisionList.push_back(coll);
                //             std::cout << "collisionList add: [" << k << ", " << j << "]" << std::endl;
                //             std::cout << "locs: [" << x2 << ", " << y2 << "], [" << robotObs[0].x << ", " << robotObs[0].y << "]" << std::endl;
                //             break;
                //         }
                //     }
                // }
                // std::cout << "end collisionList" << std::endl;
                //
                // /* Check initial-final collision tree & check for cycles. */
                // /* Size 2 loops are the problem, while size 3+ can be navigated. */
                // if (collisionList.size() > 1) {
                //     for (int j = 0; j < collisionList.size() - 1; j++) {
                //         for (int k = j; k < collisionList.size() - 1; k++) {
                //             if (collisionList[j][0] == collisionList[k][1] && collisionList[j][1] == collisionList[k][0]) {
                //                 std::cout << "initial-final loop collision caught" << std::endl;
                //                 collide = true;
                //                 break;
                //             }
                //         }
                //         if (collide) {
                //           break;
                //         }
                //    }
                // }

                /* We keep the largest cosine value as the closest direction of roadmap state to random state */
                if ((cosValue >= max) && (!collide || first)) {
                    first = false;
                    max = cosValue;
                    newState[i * 2] = roadmapX;
                    newState[i * 2 + 1] = roadmapY;
                    newState[i + (numRobots_ * 2)] = stateTheta.value;
                }
            }
        }
        /* Add motions to the tree. */
        auto *newMotion = new Motion(si_);
        si_->copyState(newMotion->state, newState.get());
        newMotion->parent = nmotion;
        nn_->add(newMotion);

        double dist = 0.0;
        bool sat = goal->isSatisfied(newMotion->state, &dist);
        if (sat) {
            approxdif = dist;
            solution = newMotion;
            break;
        }
        if (dist < approxdif) {
            approxdif = dist;
            approxsol = newMotion;
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::dRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

void ompl::geometric::dRRT::setRoadmap(std::vector<const ompl::base::State*> &roadmap)
{
    this->roadmap = roadmap;
}
