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

class MCTSDecisionGraph
{
public:
  MCTSDecisionGraph() = default;
  MCTSDecisionGraph( const LogicEngine &, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState );
  bool empty() const { return !root_ || root_->children().empty(); }

  // mcts building
  void expandMCTS( const double r0, const std::size_t n_iter_min, const std::size_t n_iter_max, const std::size_t rolloutMaxSteps, const double c, const bool verbose );
  double simulate( const DecisionGraph::GraphNodeType::ptr& node,
                   const std::size_t stateIndex,
                   const std::size_t depth,
                   const double r0,
                   const std::size_t rolloutMaxSteps,
                   const double c,
                   std::unordered_set< uint > & expandedNodesIds,
                   const bool verbose);
  double rollOutOneWorld( const std::string & state,
                          const double r0,
                          const std::size_t steps,
                          const std::size_t rolloutMaxSteps,
                          const bool verbose ) const;

  std::vector< std::string > getPossibleActions( const std::string & state ) const;
  std::tuple< std::string, bool > getOutcome( const std::string& state, const std::string& action ) const;
//  double rollOut( const std::vector< std::string > & states,
//                  const std::vector< double >& bs,
//                  const double r0,
//                  const std::size_t steps,
//                  const std::size_t rolloutMaxSteps,
//                  const bool verbose ) const;
  void saveMCTSTreeToFile( const std::string & filename, const std::string & mctsState ) const;

  // for printing
  DecisionGraph::GraphNodeType::ptr root() const { return root_; }
  const std::vector< std::weak_ptr< DecisionGraph::GraphNodeType > >& nodes() const { NIY; }
  const std::list< std::weak_ptr< DecisionGraph::GraphNodeType > >& terminalNodes() const { terminalNodes_; }
  const std::vector< DecisionGraph::EdgeDataType >& edges() const { NIY; }


private:
  mutable LogicEngine engine_;
  DecisionGraph::GraphNodeType::ptr root_;
  std::list< std::weak_ptr< DecisionGraph::GraphNodeType > > terminalNodes_;
};
} // namespace matp
