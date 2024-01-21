#pragma once

#include <fstream>

#include <decision_graph.h>
#include <mcts_decision_graph.h>

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

  void print( const MCTSDecisionGraph & graph );

private:
  void printNode( const DecisionGraph::GraphNodeType::ptr & node );
  void printEdge( const DecisionGraph::GraphNodeType::ptr & node, const DecisionGraph::GraphNodeType::ptr & c );

  void saveTreeFrom( const DecisionGraph::GraphNodeType::ptr & node );
  std::string extractActionLabel( const std::string & leadingArtifact, uint agentId ) const;

private:
  std::ostream & ss_;
  std::string mctsState_;

  std::vector< DecisionGraph::EdgeDataType > edges_; // store them here since they don't follow naturaly with the nodes
};

}
