#pragma once

#include <set>
#include <queue>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <graph_node.h>
#include <logic_engine.h>
#include <utils.h>
#include <mcts_decision_tree_utils.h>

namespace matp
{
class MCTSDecisionTreeBs
{
public:
  using GraphNodeDataType = MCTSNodeData;
  using GraphNodeType = GraphNode< GraphNodeDataType >;

  MCTSDecisionTreeBs() = default;
  MCTSDecisionTreeBs( const LogicEngine &, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState );
  bool empty() const { return !root_ || root_->children().empty(); }

  // mcts building
  void expandMCTS( const double r0,
                   const std::size_t n_iter_min,
                   const std::size_t n_iter_max,
                   const std::size_t rolloutMaxSteps,
                   const std::size_t nRolloutsPerSimulation,
                   const double c,
                   const bool verbose );
  double simulate( const GraphNodeType::ptr& node,
                   const std::size_t depth,
                   const double r0,
                   const std::size_t rolloutMaxSteps,
                   const std::size_t nRolloutsPerSimulation,
                   const double c,
                   std::unordered_set< uint > & expandedNodesIds,
                   const bool verbose);
  double rollOutBs( const std::size_t node_h,
                          const double r0,
                          const std::size_t steps,
                          const std::size_t rolloutMaxSteps,
                          const std::size_t nRolloutsPerSimulation,
                          const bool verbose ) const;
  double rollOutBs( const std::size_t node_h,
                          const double r0,
                          const std::size_t steps,
                          const std::size_t rolloutMaxSteps,
                          const bool verbose ) const;

  // belief space extension
  std::vector< std::size_t > getCommonPossibleActions( const std::size_t node_h ) const; // using cache
  std::vector< std::size_t > getPossibleOutcomes( const std::size_t node_h, const std::size_t action_h ) const; // using cache

  // one world rollout
  const std::vector< std::size_t >& getPossibleActions( const std::size_t state_h ) const; // using cache
  std::vector< std::string > getPossibleActions( const std::set<std::string> & state, const std::size_t state_h ) const;
  std::tuple< std::size_t, bool > getOutcome( const std::size_t state_h, const std::size_t action_i ) const; // using cache
  std::tuple< std::set<std::string>, bool > getOutcome( const std::set<std::string>& state, const std::size_t state_h, const std::string& action ) const;

  void saveMCTSTreeToFile( const std::string & filename,
                           const std::string & mctsState,
                           const Rewards& rewards,
                           const Values& values ) const;

  // for printing
  MCTSDecisionTreeBs::GraphNodeType::ptr root() const { return root_; }
  const std::list< std::weak_ptr< MCTSDecisionTreeBs::GraphNodeType > >& terminalNodes() const { return terminalNodes_; }

  mutable LogicEngine engine_;
  MCTSDecisionTreeBs::GraphNodeType::ptr root_;
  std::list< std::weak_ptr< MCTSDecisionTreeBs::GraphNodeType > > terminalNodes_;
  std::vector<bool> terminality_; // false as long as the world indicated by the index has not been solved

  // upper level caching
  // basic states, actions, observations
  mutable std::unordered_map< std::size_t, std::set<std::string> > states_;                        // state_h -> states  (storage optimization, tree has ids only)
  mutable std::unordered_map< std::size_t, std::string > actions_;                                 // action_h -> action (storage optimization, tree has ids only)
  mutable std::unordered_map< std::size_t, std::vector< double > > beliefStates_;                  // belief_h -> belief ((storage optimization, tree has ids only))
  mutable std::unordered_map< std::size_t, GraphNodeDataType > nodesData_;                         // node_h -> node data (only action nodes)

  // transitions
  mutable std::unordered_map< std::size_t, std::vector< std::size_t > > stateToActionsH_;          // state_h -> actions_h (to speed up)
  mutable std::unordered_map< std::size_t, std::vector< std::size_t > > nodeHToActions_;           // id -> actions h      (to speed up)
  mutable std::unordered_map< std::pair< std::size_t, std::size_t >, std::size_t, StateHashActionHasher > stateActionHToNextState_; // state_h, action_h -> next_h (to speed up)
  mutable std::unordered_map< std::pair< std::size_t, std::size_t >, std::vector< std::size_t >, StateHashActionHasher > nodeHActionToNodesH_; // node_h, action_h -> next_node_h (to speed up)
  mutable std::unordered_map< std::size_t, bool > terminal_;

  // engine state
  mutable std::size_t lastSetStateEngine_;

  // debug data to measure cache efficiency
  struct CacheUsageDetails
  {
    std::size_t nQueries{};
    std::size_t nUsedCache{};
  };

  mutable CacheUsageDetails stateToActionsH_details_;
  mutable CacheUsageDetails nodeHToActions_details_;
  mutable CacheUsageDetails stateActionHToNextState_details_;
  mutable CacheUsageDetails nodeHActionToNodesH_details_;
};

std::string getObservation( const MCTSDecisionTreeBs::GraphNodeType::ptr & from, const MCTSDecisionTreeBs::GraphNodeType::ptr & to, const MCTSDecisionTreeBs & graph );

} // namespace matp
