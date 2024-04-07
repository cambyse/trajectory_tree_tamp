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
#include <mcts_decision_tree_bs.h>

#include <value_iteration.h>
#include <decide_on_graph.h>

namespace matp
{

// implemented based on https://papers.nips.cc/paper_files/paper/2010/file/edfbe1afcf9246bb0d40eb4d8027d90f-Paper.pdf
class MCTSPlannerBs : public TaskPlanner
{
public:
  // modifiers
  virtual void setFol( const std::string & descrition ) override;
  virtual void solve() override;
  virtual void integrate( const Policy & policy ) override;

  // other setters
  void setR0( double r0, double explorationScaling ) { rewards_.setR0(r0); explorationTermC_ = std::fabs(r0) * explorationScaling; } // force here the C and R0 to be in sync see (Alg 1. in https://papers.nips.cc/paper_files/paper/2010/file/edfbe1afcf9246bb0d40eb4d8027d90f-Paper.pdf)
  void setNIterMinMax( std::size_t nMin, std::size_t nMax ) { nIterMin_ = nMin; nIterMax_ = nMax; }
  void setNumberRollOutPerSimulation( std::size_t n ) { nRollOutsPerSimulation_ = n; }
  void setRollOutMaxSteps( std::size_t nMaxSteps ) { rollOutMaxSteps_ = nMaxSteps; }
  void setVerbose( bool verbose ) { verbose_ = verbose; };

  void saveMCTSGraphToFile( const std::string & filename ) const { tree_.saveMCTSTreeToFile( filename, "", rewards_, values_ ); }

  // getters
  virtual bool terminated() const override;
  Policy getPolicy() const override;
  Rewards& getRewards() { return rewards_; }; // for testing
  const Values& getValues() { return values_; }; // for testing

  void expandMCTS();

private:
  void buildPolicy();
  void valueIteration();

private:
  LogicParser parser_;
  MCTSDecisionTreeBs tree_;
  Policy policy_;
  std::unordered_map< std::size_t, std::weak_ptr< MCTSDecisionTree::GraphNodeType > > decisionTreeNodes_; // relevant node of the tree are indexed here (not done for all nodes to avoid the overhead)

  // value iteration
  Rewards rewards_;// current state of rewards. Rewards are set between the action and observation node
  Values values_;  // values from last value iteration pass

  // MCTS parameters
  std::size_t nIterMin_{0};
  std::size_t nIterMax_{50};
  std::size_t nRollOutsPerSimulation_{5};
  std::size_t rollOutMaxSteps_{30};
  double explorationTermC_{1.0};

  // debug
  bool verbose_{false};
};

} // namespace matp
