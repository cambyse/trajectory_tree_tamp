#pragma once

#include <fstream>
#include <belief_state.h>
#include <decision_graph.h>
#include <mcts_decision_tree.h>
#include <decision_graph.h>

#include <boost/algorithm/string/replace.hpp>

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

template <typename T>
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

  void print( const T & graph )
  {
    if( ! graph.root() )
    {
      return;
    }

    //edges_ = graph.edges();

    ss_ << "digraph g{" << std::endl;
    ss_ << "labelloc=\"t\"" << std::endl;
    ss_ << "label=\"" << mctsState_ << "\"" << std::endl;
    ss_ << "bgcolor=\"transparent\"";
    ss_ << "{" << std::endl;
    ss_ << graph.root()->id() << " [style=filled, fillcolor=blue]" << std::endl;

    ss_ << "}" << std::endl;

    saveTreeFrom( graph.root(), graph, 0, false );

    ss_ << "}" << std::endl;
  }

private:
  void printNode( const typename T::GraphNodeType::ptr & node, const T & graph )
  {
    ss_ << node->id() << " [shape=square, style=filled, fillcolor=" << ( node->id() == 0 ? "blue" : "cyan" ) << "]" << std::endl;

    if( node->data().nodeType == MCTSNodeData::NodeType::OBSERVATION )
    {
      ss_ << node->id() << " [shape=diamond]" << std::endl;
    }

    std::stringstream ss;
    ss << std::fixed;
    ss << std::setprecision(2);
    ss << "id: " << node->id() << std::endl;
    ss << node->data().n_rollouts << std::endl;

    if( true/*node->data().nodeType == MCTSNodeData::NodeType::OBSERVATION*/ )
    {
      for(const auto& value: {node->data().mcts_q_value, values_.getOrDefault(node->id())})
      {
        if( value >= -1000 ) // TODO change that
        {
          ss << value;
        }
        else
        {
          ss << "-inf";
        }

        ss << std::endl;
      }
    }

    ss_ << node->id() << " [label=\"" << ss.str() << "\"" << "]" << std::endl;

    if(node->data().terminal)
    {
      ss_ << node->id() << " [style=filled, fillcolor=green]" << std::endl;
    }
    else if(node->data().isPotentialSymbolicSolution)
    {
      ss_ << node->id() << " [style=filled, fillcolor=aquamarine]" << std::endl;
    }
  }

  void printEdge( const typename T::GraphNodeType::ptr & from, const typename T::GraphNodeType::ptr & to, const T & graph )
  {
    std::stringstream ss;
    std::string label;

    const auto p = transitionProbability( graph.beliefStates_.at(from->data().beliefState_h),
                                          graph.beliefStates_.at(to->data().beliefState_h)
                                         );

    if( from->data().nodeType == MCTSNodeData::NodeType::ACTION )
    {
      auto actionLabel = graph.actions_.at( to->data().leadingAction_h);

      boost::replace_all(actionLabel, "(", "");
      boost::replace_all(actionLabel, ")", "");

      ss << actionLabel << std::endl;
      ss << rewards_.get( from->id(), to->id() );

      label = ss.str();
      boost::replace_all(label, "{", "");
    }
    else
    {
      ss << getObservation( from, to, graph );
      ss << p;

      label = ss.str();
    }

    ss_ << from->id() << "->" << to->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;
  }

  void saveTreeFrom( const typename T::GraphNodeType::ptr & node, const T & graph, std::size_t depth, bool printedUpToNode )
  {
    if( depth > maxDepth_ )
    {
      return;
    }

    const auto printFromNode = printedUpToNode || ( node->id() == fromNodeId_ );

    if( printFromNode ) printNode( node, graph );

    // print edge
    for( const auto& c : node->children() )
    {
      if( printFromNode ) printEdge( node, c, graph );

      saveTreeFrom( c , graph, printFromNode ? depth + 1 : depth, printFromNode );
    }
  }
private:
  std::ostream & ss_;
  std::string mctsState_;
  std::size_t maxDepth_;
  std::size_t fromNodeId_;
  Values values_;
  Rewards rewards_;
};

}
