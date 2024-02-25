#pragma once

#include <set>
#include <queue>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <graph_node.h>
#include <logic_engine.h>

namespace matp
{

struct MCTSNodeData
{
  enum class NodeType
  {
    ACTION = 0, // an action has to be taken at this node
    OBSERVATION
  };

  MCTSNodeData()
    : states_h( 0 )
    , beliefState_h( 0 )
    , node_h( 0 )
    , terminal( false )
    , nodeType( NodeType::ACTION )
    , leadingAction_h( 0 )
    , leadingObservation_h( 0 )
    , leadingProbability( 0.0 )
    , value{ std::numeric_limits<double>::lowest()} // expected reward to goal
    , n_rollouts( 0 )
    , isPotentialSymbolicSolution{false}
  {
  }

  MCTSNodeData( const std::vector< std::size_t > & states_h,
                const std::size_t beliefState_h,
                const std::size_t node_h,
                bool terminal,
                NodeType nodeType,
                const std::size_t leadingAction_h,
                const std::size_t leadingObservation_h,
                const double leadingProbability)
    : states_h( states_h )
    , beliefState_h( beliefState_h )
    , node_h( node_h )
    , terminal( terminal )
    , nodeType( nodeType )
    , leadingAction_h( leadingAction_h )
    , leadingObservation_h( leadingObservation_h )
    , leadingProbability( leadingProbability )
    , value{ std::numeric_limits<double>::lowest()} // expected reward to goal
    , n_rollouts( 0 )
    , isPotentialSymbolicSolution{false}
  {
  }
  std::vector< std::size_t > states_h;
  std::size_t beliefState_h;
  std::size_t node_h;
  bool terminal;
  NodeType nodeType;
  std::size_t leadingAction_h;
  std::size_t leadingObservation_h;
  double leadingProbability;

  // mcts
  double value;
  std::size_t n_rollouts;
  bool isPotentialSymbolicSolution; // indicates if node is on skeleton solving the pb
};

double priorityUCT( const GraphNode< MCTSNodeData >::ptr& node, const double c, bool verbose );
std::size_t sampleStateIndex( const std::vector< double >& bs );
std::string getNotObservableFact( const std::string& fullState );
GraphNode< MCTSNodeData >::ptr getMostPromisingChild( const GraphNode< MCTSNodeData >::ptr& node, const double c, const bool verbose );
std::size_t getHash( const std::set<std::string>& facts );
std::size_t getHash( const std::vector< std::size_t >& beliefState );
std::size_t getHash( const std::vector< std::size_t >& states_h, const std::size_t beliefState_h );

void backtrackIsPotentialSymbolicSolution( const GraphNode< MCTSNodeData >::ptr& node );

struct StateHashActionHasher
{
  std::size_t operator()(const std::pair<std::size_t, std::size_t> & state_action_pair) const
  {
    return state_action_pair.first << state_action_pair.second;
  }
};

class MCTSDecisionTree
{
public:
  using GraphNodeDataType = MCTSNodeData;
  using GraphNodeType = GraphNode< GraphNodeDataType >;

  MCTSDecisionTree() = default;
  MCTSDecisionTree( const LogicEngine &, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState );
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
                   const std::size_t stateIndex,
                   const std::size_t depth,
                   const double r0,
                   const std::size_t rolloutMaxSteps,
                   const std::size_t nRolloutsPerSimulation,
                   const double c,
                   std::unordered_set< uint > & expandedNodesIds,
                   const bool verbose);
  double rollOutOneWorld( const std::size_t state_h,
                          const double r0,
                          const std::size_t steps,
                          const std::size_t rolloutMaxSteps,
                          const std::size_t nRolloutsPerSimulation,
                          const bool verbose ) const;
  double rollOutOneWorld( const std::size_t state_h,
                          const double r0,
                          const std::size_t steps,
                          const std::size_t rolloutMaxSteps,
                          const bool verbose ) const;

  // belief space extension
  std::vector< std::size_t > getCommonPossibleActions( const std::size_t node_ ) const; // using cache
  std::vector< std::size_t > getPossibleOutcomes( const std::size_t node_h, const std::size_t action_h ) const; // using cache

  // one world rollout
  const std::vector< std::size_t >& getPossibleActions( const std::size_t state_h ) const; // using cache
  std::vector< std::string > getPossibleActions( const std::set<std::string> & state, const std::size_t state_h ) const;
  std::tuple< std::size_t, bool > getOutcome( const std::size_t state_h, const std::size_t action_i ) const; // using cache
  std::tuple< std::set<std::string>, bool > getOutcome( const std::set<std::string>& state, const std::size_t state_h, const std::string& action ) const;

  void saveMCTSTreeToFile( const std::string & filename, const std::string & mctsState ) const;

  // for printing
  MCTSDecisionTree::GraphNodeType::ptr root() const { return root_; }
  const std::list< std::weak_ptr< MCTSDecisionTree::GraphNodeType > >& terminalNodes() const { return terminalNodes_; }

  mutable LogicEngine engine_;
  MCTSDecisionTree::GraphNodeType::ptr root_;
  std::list< std::weak_ptr< MCTSDecisionTree::GraphNodeType > > terminalNodes_;

  // upper level caching
  // basic states, actions, observations
  mutable std::unordered_map< std::size_t, std::set<std::string> > states_;                        // state_h -> states  (storage optimization, tree has ids only)
  mutable std::unordered_map< std::size_t, std::string > actions_;                                 // action_h -> action (storage optimization, tree has ids only)
  mutable std::unordered_map< std::size_t, std::set<std::string> > observations_;                  // observation_h -> observation (storage optimization, tree has ids only)
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
} // namespace matp
