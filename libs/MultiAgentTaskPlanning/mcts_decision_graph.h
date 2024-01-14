#pragma once

#include <set>
#include <queue>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <decision_graph.h>

namespace matp
{

double priorityUCT( const GraphNode< NodeData >::ptr& node, const double c, bool verbose );
std::size_t sampleStateIndex( const std::vector< double >& bs );
std::string getNotObservableFact( const std::string& fullState );
GraphNode< NodeData >::ptr getMostPromisingChild( const GraphNode< NodeData >::ptr& node, const double c, const bool verbose );

class MCTSDecisionGraph : public DecisionGraph
{
public:
  MCTSDecisionGraph() = default;
  MCTSDecisionGraph( const LogicEngine &, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState );

  // mcts building
  void expandMCTS( const double r0, const std::size_t n_iter_min, const std::size_t n_iter_max, const std::size_t rolloutMaxSteps, const double c, const bool verbose );
  double simulate( const GraphNodeType::ptr& node,
                   const std::size_t stateIndex,
                   const std::size_t depth,
                   const double r0,
                   const std::size_t rolloutMaxSteps,
                   const double c,
                   std::unordered_set< uint > & expandedNodesIds,
                   const bool verbose);
  double rollOut( const std::vector< std::string > & states,
                  const std::vector< double >& bs,
                  const double r0,
                  const std::size_t steps,
                  const std::size_t rolloutMaxSteps,
                  const bool verbose ) const;
  void saveMCTSTreeToFile( const std::string & filename, const std::string & mctsState ) const;
};
} // namespace matp
