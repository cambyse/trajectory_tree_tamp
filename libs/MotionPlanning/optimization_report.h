#pragma once

#include <string>
#include <unordered_map>
#include <set>

#include <skeleton.h>

#include <komo_factory.h>
#include <komo_planner_config.h>
#include <komo_wrapper.h>
#include <subtree_generators.h>

namespace mp
{

struct OptimizationReport
{
  void save(const std::string& filepath) const;

  struct TimeSliceInfo
  {
    arr q;
    std::map<std::string, double> objectivesResults;
  };

  uint stepsPerPhase{};
  double totalCost{};

  std::vector<TimeSliceInfo> slices;
  std::map<std::string, ObjectiveType> objectives;
  std::set<std::string> objectivesIrrelevantForCost; // some objectives are masked for the final costs
  std::vector<Vars> allVars;
};


}
