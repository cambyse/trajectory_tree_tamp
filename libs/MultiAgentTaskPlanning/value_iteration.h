#pragma once

#include <stdexcept>

#include <queue>
#include <string>
#include <unordered_map>

#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <skeleton.h>
#include <task_planner.h>

#include <logic_parser.h>
#include <decision_graph.h>
#include <mcts_decision_tree.h>

namespace matp
{
// This is value iteration for graphs, so it requires several iterations before convergence.
class ValueIterationAlgorithm
{
public:
  static std::vector< double > process( const DecisionGraph & decisionGraph, Rewards & rewards );
};

class ValueIterationOnTreeAlgorithm
{
public:
  static Values process( const MCTSDecisionTree & decisionGraph, Rewards & rewards );
};

} // namespace matp
