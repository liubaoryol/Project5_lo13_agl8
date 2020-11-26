\
///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 5
// Authors: Alex Liapis and Liubove Orlov Savko
//////////////////////////////////////
#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRT_
 #define OMPL_GEOMETRIC_PLANNERS_RRT_RRT_
  
 #include "ompl/datastructures/NearestNeighbors.h"
 #include "ompl/geometric/planners/PlannerIncludes.h"
  
 namespace ompl
 {
     namespace geometric
     {
         class RRT : public base::Planner
         {
         public:
             RRT(const base::SpaceInformationPtr &si, bool addIntermediateStates = false);
  
             ~RRT() override;
  
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
  
             bool getIntermediateStates() const
             {
                 return addIntermediateStates_;
             }
  
             void setIntermediateStates(bool addIntermediateStates)
             {
                 addIntermediateStates_ = addIntermediateStates;
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
  
             bool addIntermediateStates_;
  
             RNG rng_;
  
             Motion *lastGoalMotion_{nullptr};
         };
     }
 }
  
 #endif