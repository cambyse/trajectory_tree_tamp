#pragma once

#include <fstream>

#include <decision_graph.h>
#include <mcts_decision_tree.h>
#include <decision_graph.h>

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
  MCTSTreePrinter( std::ostream & ss,
                   const std::string& state,
                   const Rewards& rewards,
                   const Values& values,
                   std::size_t maxDepth,
                   std::size_t fromNodeId
                  )
    : ss_( ss )
    , mctsState_( state )
    , rewards_( rewards )
    , values_( values )
    , maxDepth_( maxDepth )
    , fromNodeId_( fromNodeId )
  {

  }

  void print( const MCTSDecisionTree & graph );

private:
  void printNode( const MCTSDecisionTree::GraphNodeType::ptr & node, const MCTSDecisionTree & graph );
  void printEdge( const MCTSDecisionTree::GraphNodeType::ptr & from, const MCTSDecisionTree::GraphNodeType::ptr & to, const MCTSDecisionTree & graph );

  void saveTreeFrom( const MCTSDecisionTree::GraphNodeType::ptr & node, const MCTSDecisionTree & graph, std::size_t depth, bool printedUpToNode );

private:
  std::ostream & ss_;
  std::string mctsState_;
  std::size_t maxDepth_;
  std::size_t fromNodeId_;
  Values values_;
  Rewards rewards_;
};

}
