///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 5
// Authors: Liubove Orlov Savko and Alex Liapis
//////////////////////////////////////

#ifndef OMPL_GEOMETRIC_PLANNERS_dRRT_dRRT_
#define OMPL_GEOMETRIC_PLANNERS_dRRT_dRRT_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/goals/GoalState.h"

namespace ompl
{
    namespace geometric
    {
        class dRRT : public base::Planner
        {
        public:
            dRRT(const base::SpaceInformationPtr &si);

            ~dRRT() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            double getGoalBias() const
            {
                return goalBias_;
            }

            int getNumRobots() const
            {
                return numRobots_;
            }

            void setNumRobots(int numRobots)
            {
                this->numRobots_ = numRobots;
            }

            double getSideLen() const
            {
                return sideLen;
            }

            void setSideLen(double len)
            {
                this->sideLen = len;
            }

            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            double getRange() const
            {
                return maxDistance_;
            }

            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

            void setRoadmap(std::vector<const ompl::base::State*> &roadmap);

            void setRoadmapGraph(std::shared_ptr<ompl::base::PlannerData> roadmapGraph)
            {
                this->roadmapGraph = roadmapGraph;
            }

        protected:
            class Motion
            {
            public:
                Motion() = default;

                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                base::State *state{nullptr};

                Motion *parent{nullptr};
            };

            void freeMemory();

            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            base::StateSamplerPtr sampler_;

            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            double goalBias_{.05};

            double maxDistance_{0.};

            int numRobots_;

            double sideLen;

            RNG rng_;

            Motion *lastGoalMotion_{nullptr};

            std::vector<const ompl::base::State*> roadmap;

            std::shared_ptr<ompl::base::PlannerData> roadmapGraph;
        };
    }
}

#endif
