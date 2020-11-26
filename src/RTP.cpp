///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Robert Black and Liubove Orlov Savko
//////////////////////////////////////

#include "RTP.h"
#include "CollisionChecking.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <vector>
#include <ompl/geometric/PathGeometric.h>



ompl::geometric::RTP::RTP(const ompl::base::SpaceInformationPtr &si) : ompl::base::Planner(si, "RTP")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RTP::setRange, &RTP::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");

}

ompl::geometric::RTP::~RTP()
{
    freeMemory();
}

void ompl::geometric::RTP::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    lastGoalMotion_ = nullptr;

    for(auto &motion : tree)
    {
        if(motion->state != nullptr)
            si_->freeState(motion->state);
        delete motion;
    }
    tree.clear();
}


void ompl::geometric::RTP::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

}



void ompl::geometric::RTP::freeMemory()
{
    for(auto &motion : tree)
    {
        if(motion->state != nullptr)
            si_->freeState(motion->state);
        delete motion;
    }
    tree.clear();
}


ompl::base::PlannerStatus ompl::geometric::RTP::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    ompl::base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal);


    while (const ompl::base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        //nn_->add(motion);
        tree.push_back(motion);
    }

    
    if (tree.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_START;
    }
    

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), tree.size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    ompl::base::State *rstate = rmotion->state;


    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);


        /* find random state in the tree */
        Motion *nmotion = tree[rng_.uniformInt(0, tree.size()-1)];    

	    //add motion to tree if valid
        if(si_->checkMotion(nmotion->state, rstate)){
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, rstate);
            motion->parent = nmotion;
            tree.push_back(motion);

            nmotion = motion;

            //checks if goal condition is satisfied
            double dist = 0.0;
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
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

    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), tree.size());

    return {solved, approximate};
}

void ompl::geometric::RTP::getPlannerData(ompl::base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    motions = tree;

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(ompl::base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(ompl::base::PlannerDataVertex(motion->state));
        else
            data.addEdge(ompl::base::PlannerDataVertex(motion->parent->state), ompl::base::PlannerDataVertex(motion->state));
    }
}
