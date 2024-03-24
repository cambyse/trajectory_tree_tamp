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
    , leadingProbability( 0.0 )
    , mcts_q_value{ std::numeric_limits<double>::lowest()} // expected reward to goal given preceding action, this is a Q-Value!
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
                const double leadingProbability)
    : states_h( states_h )
    , beliefState_h( beliefState_h )
    , node_h( node_h )
    , terminal( terminal )
    , nodeType( nodeType )
    , leadingAction_h( leadingAction_h )
    , leadingProbability( leadingProbability )
    , mcts_q_value{ std::numeric_limits<double>::lowest()} // expected reward to goal
    , n_rollouts( 0 )
    , isPotentialSymbolicSolution{false}
  {
  }
  std::vector< std::size_t > states_h;
  std::size_t beliefState_h;
  std::size_t node_h;
  bool terminal;
  NodeType nodeType;

  // non-markovian -> shouldn't be cached!
  std::size_t leadingAction_h;
  double leadingProbability;

  // mcts
  double mcts_q_value;
  std::size_t n_rollouts;
  bool isPotentialSymbolicSolution; // indicates if node is on skeleton solving the pb
};

std::vector < double > normalizeBs( const std::vector < double > & bs );
double priorityUCT( const GraphNode< MCTSNodeData >::ptr& node, const double c, bool verbose );
std::size_t sampleStateIndex( const std::vector< double >& bs );
std::string getNotObservableFact( const std::string& fullState );
GraphNode< MCTSNodeData >::ptr getMostPromisingChild( const GraphNode< MCTSNodeData >::ptr& node, const double c, const bool verbose );
std::size_t getHash( const std::set<std::string>& facts );
std::size_t getHash( const std::vector< double >& beliefState );
std::size_t getHash( const std::vector< std::size_t >& states_h, const std::size_t beliefState_h ); // node_h

void backtrackIsPotentialSymbolicSolution( const GraphNode< MCTSNodeData >::ptr& node );

struct StateHashActionHasher
{
  std::size_t operator()(const std::pair<std::size_t, std::size_t> & state_action_pair) const
  {
    return state_action_pair.first << state_action_pair.second;
  }
};

} // namespace matp
