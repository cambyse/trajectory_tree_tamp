#pragma once

#include <fstream>

#include <decision_graph.h>
#include <mcts_decision_tree.h>

namespace matp
{
class GraphPrinter
{
public:
  GraphPrinter( std::ostream & ss )
    : ss_( ss )
  {

  }

  void print( const DecisionGraph & graph );

private:
  void saveGraphFrom( const DecisionGraph::GraphNodeType::ptr & node );
  std::string extractActionLabel( const std::string & leadingArtifact, uint agentId ) const;

private:
  std::ostream & ss_;
  std::set< uint > printedNodes_;

  std::vector< DecisionGraph::EdgeDataType > edges_; // store them here since they don't follow naturaly with the nodes
};

class MCTSTreePrinter
{
public:
  MCTSTreePrinter( std::ostream & ss, const std::string& state )
    : ss_( ss )
    , mctsState_( state )
  {

  }

  void print( const MCTSDecisionTree & graph );

private:
  void printNode( const MCTSDecisionTree::GraphNodeType::ptr & node, const MCTSDecisionTree & graph );
  void printEdge( const MCTSDecisionTree::GraphNodeType::ptr & node, const MCTSDecisionTree::GraphNodeType::ptr & c, const MCTSDecisionTree & graph );

  void saveTreeFrom( const MCTSDecisionTree::GraphNodeType::ptr & node, const MCTSDecisionTree & graph );

private:
  std::ostream & ss_;
  std::string mctsState_;
};

}
