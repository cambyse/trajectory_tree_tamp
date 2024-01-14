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
#include <mcts_decision_graph.h>

#include <value_iteration.h>
#include <decide_on_graph.h>

namespace matp
{

class MCTSPlanner : public TaskPlanner
{
public:
  // modifiers
  virtual void setFol( const std::string & descrition ) override;
  virtual void solve() override;
  virtual void integrate( const Policy & policy ) override;

  // other setters
  void setR0( double r0 ) { rewards_.setR0(r0); }
  void setNIterMinMax( std::size_t nMin, std::size_t nMax ) { nIterMin_ = nMin; nIterMax_ = nMax; }
  void setRollOutMaxSteps( std::size_t nMaxSteps ) { rollOutMaxSteps_ = nMaxSteps; }
  void setExplorationTerm( double c ) { explorationTermC_ = c; }
  void setVerbose( bool verbose ) { verbose_ = true; };

  // getters
  virtual bool terminated() const override;
  Policy getPolicy() const override;

  void expandMCTS();

private:
  void buildPolicy();

private:
  LogicParser parser_;
  MCTSDecisionGraph tree_;
  Policy policy_;

  // value iteration
  mutable Rewards rewards_;// current state of rewards
  std::vector< double > values_;
  DecisionGraph decidedGraph_;

  // MCTS parameters
  std::size_t nIterMin_{0};
  std::size_t nIterMax_{50};
  std::size_t rollOutMaxSteps_{50};
  double explorationTermC_{1.0};

  // debug
  bool verbose_{false};
};

} // namespace matp
