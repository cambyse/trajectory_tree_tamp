#include <mcts_decision_tree.h>

#include <set>
#include <algorithm>
#include <unordered_set>
#include <numeric>
#include <decision_graph_printer.h>
#include <belief_state.h>
#include <boost/functional/hash.hpp>
#include <mcts_decision_tree_utils.h>

namespace matp
{
std::vector < double > normalizeBs( const std::vector < double > & bs )
{
  std::vector < double > newBs = bs;

  auto sumfunc = []( const std::vector < double > bs ) -> double
  {
    double sum = 0;
    for( auto p : bs )
    {
      sum += p;
    }
    return sum;
  };

  const auto sum = sumfunc( bs );

  for( std::size_t w = 0; w < bs.size(); ++w )
  {
    newBs[ w ] = bs[ w ] / sum;
  }

  CHECK( fabs( sumfunc( newBs ) - 1.0 ) < 0.00001, "" );

  return newBs;
}

double priorityUCT( const GraphNode< MCTSNodeData >::ptr& node, const double c, bool verbose )
{
  const auto& node_data = node->data();

  std::size_t parent_n_rollouts{node_data.n_rollouts};
  if( node->parent() )
  {
    CHECK( node->parent() != nullptr, "node shouldn't be a root node!" );
    CHECK( node->parent()->data().nodeType == MCTSNodeData::NodeType::ACTION, "wrong node type!" );

    const auto& action_parent = node->parent();
    const auto& parent_node_data = action_parent->data();

    parent_n_rollouts = parent_node_data.n_rollouts;
  }

  if( node_data.n_rollouts == 0 )
  {
    if( verbose ) std::cout << "[priority] of " << node->id() << " = " << std::numeric_limits<double>::infinity() << std::endl;

    return std::numeric_limits<double>::infinity();
  }

  const double priority = node_data.mcts_q_value + c * sqrt( log( ( parent_n_rollouts ) / ( node_data.n_rollouts ) ) );

  if( verbose ) std::cout << "[priority] of " << node->id() << " = " << priority << std::endl;

  return priority;
}

std::size_t sampleStateIndex( const std::vector< double >& bs )
{
  // build integrated belief state, TODO: Compute once and for all and store instead of recomputing all the time
  std::vector<double> integratedBs;
  integratedBs.resize( bs.size() );

  double lastIntergatedProbability{0.0};
  for( auto w{0}; w < bs.size(); ++w )
  {
    integratedBs[w] = lastIntergatedProbability + bs[w];
    lastIntergatedProbability = integratedBs[w];
  }

  const auto v = ( ( double ) rand() / ( RAND_MAX ) );

  const auto s_it = std::lower_bound( integratedBs.begin(), integratedBs.end(), v );

  return std::distance( integratedBs.begin(), s_it );
}

GraphNode< MCTSNodeData >::ptr getMostPromisingChild( const GraphNode< MCTSNodeData >::ptr& node, const double c, const bool verbose )
{
  double max_prio{std::numeric_limits<double>::lowest()};
  GraphNode< MCTSNodeData >::ptr best_uct_child{};
  for( const auto& child : node->children() )
  {
    const auto prio = priorityUCT( child, c, verbose );

    if( prio > max_prio + 1.0e-5 ) // epsilon added to break tie and have fuly reproducilb esearch also when scaling r0
    {
      max_prio = prio;
      best_uct_child = child;
    }
  }

  return best_uct_child;
}

std::size_t getHash( const std::set<std::string>& facts )
{
  std::size_t hash{0};
  for(const auto& fact: facts)
  {
    hash += std::hash<std::string>()(fact);
  }

  return hash;
}

std::size_t getHash( const std::vector< double >& beliefState )
{
  std::size_t hash{0};

  for( auto w{0}; w < beliefState.size(); ++w )
  {
    boost::hash_combine( hash, beliefState[w] );
  }

  return hash;
}

std::size_t getHash( const std::vector< std::size_t >& states_h, const std::size_t beliefState_h )
{
  std::size_t hash{0};

  for( auto w{0}; w < states_h.size(); ++w )
  {
    boost::hash_combine( hash, states_h[w] );
  }

  boost::hash_combine( hash, beliefState_h );

  return hash;
}

void backtrackIsPotentialSymbolicSolution( const GraphNode< MCTSNodeData >::ptr& node )
{
  CHECK( node->data().nodeType == MCTSNodeData::NodeType::ACTION, "we should only backtrack from action nodes!" );

  if( node->parents().empty() )
  {
    return;
  }

  CHECK( node->parents().size() == 1, "MCTS works here for trees only!" );

  const auto observationParent = node->parents().front().lock();

  const auto observationParentIsSolved = std::accumulate(observationParent->children().begin(),
                                                         observationParent->children().end(),
                                                         true,
                                                         [](bool solved, const auto& n) { return solved && n->data().isPotentialSymbolicSolution; });

  observationParent->data().isPotentialSymbolicSolution = observationParentIsSolved;

  CHECK( observationParent->parents().size() == 1, "MCTS works here for trees only!" );

  if(observationParentIsSolved)
  {
    const auto actionParent =  observationParent->parents().front().lock();

    actionParent->data().isPotentialSymbolicSolution = true;

    //std::cout << "Set " << actionParent->id() << " is on potential solution!" << std::endl;

    backtrackIsPotentialSymbolicSolution( actionParent );
  }
}

} // namespace matp
