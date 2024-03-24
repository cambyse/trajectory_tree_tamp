#include <decision_graph_printer.h>
#include <belief_state.h>

#include <boost/algorithm/string/replace.hpp>

namespace matp
{

void GraphPrinter::print( const DecisionGraph & graph )
{
  if( ! graph.root() )
  {
    return;
  }

  edges_ = graph.edges();

  ss_ << "digraph g{" << std::endl;
  ss_ << "bgcolor=\"transparent\"";
  ss_ << "{" << std::endl;
  ss_ << graph.root()->id() << " [style=filled, fillcolor=blue]" << std::endl;
  for( auto weakN : graph.nodes() )
  {
    auto n = weakN.lock();

    if( n )
    {
      if( n->data().agentId == 0 )
      {
        ss_ << n->id() << " [shape=square, style=filled, fillcolor=" << ( n->id() == 0 ? "blue" : "cyan" ) << "]" << std::endl;
      }
      else
      {
        ss_ << n->id() << " [shape=circle]" << std::endl;
      }

      if( n->data().nodeType == NodeData::NodeType::OBSERVATION )
      {
        ss_ << n->id() << " [shape=diamond]" << std::endl;
      }
    }
  }
  for( auto weakN : graph.terminalNodes() )
  {
    auto n = weakN.lock();

    if( n )
    {
      ss_ << n->id() << " [style=filled, fillcolor=green]" << std::endl;
    }
  }
  ss_ << "}" << std::endl;

  saveGraphFrom( graph.root() );

  ss_ << "}" << std::endl;
}

void GraphPrinter::saveGraphFrom( const DecisionGraph::GraphNodeType::ptr & node )
{
  if( printedNodes_.count( node->id() ) != 0 )
  {
    return;
  }
  else
  {
    printedNodes_.insert( node->id() );
  }

  for( auto c : node->children() )
  {
    std::stringstream ss;
    std::string label;

    auto edge = edges_[ c->id()][ node->id() ];
    auto p = edge.first;
    auto leadingArtifact = edge.second;

    if( node->data().nodeType == NodeData::NodeType::ACTION )
    {
      auto agentLabel = agentPrefix_ + std::to_string( node->data().agentId ) + agentSuffix_;
      auto actionLabel = extractActionLabel( leadingArtifact, node->data().agentId );

      boost::replace_all(agentLabel, "__", "");
      boost::replace_all(actionLabel, "(", "");
      boost::replace_all(actionLabel, ")", "");

      ss << agentLabel << std::endl;
      ss << actionLabel;

      label = ss.str();
      boost::replace_all(label, "{", "");
    }
    else
    {
      if( ! leadingArtifact.empty() )
      {
        ss << leadingArtifact << std::endl;
      }
      ss << p;

      label = ss.str();
    }

    ss_ << node->id() << "->" << c->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;

    saveGraphFrom( c );
  }
}

std::string GraphPrinter::extractActionLabel( const std::string & leadingArtifact, uint agentId ) const
{
  auto agentLabel = agentPrefix_ + std::to_string( agentId ) + agentSuffix_;

  if( leadingArtifact.find( agentLabel ) != -1 )
  {
    return leadingArtifact.substr( agentLabel.size() + 1, leadingArtifact.size() );
  }
  else
  {
    return leadingArtifact;
  }
}


void MCTSTreePrinter::print( const MCTSDecisionTree & graph )
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

void MCTSTreePrinter::printNode( const MCTSDecisionTree::GraphNodeType::ptr & node,
                                 const MCTSDecisionTree & graph )
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

void MCTSTreePrinter::printEdge( const MCTSDecisionTree::GraphNodeType::ptr & from,
                                 const MCTSDecisionTree::GraphNodeType::ptr & to,
                                 const MCTSDecisionTree & graph)
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

void MCTSTreePrinter::saveTreeFrom( const MCTSDecisionTree::GraphNodeType::ptr & node,
                                    const MCTSDecisionTree & graph,
                                    const std::size_t depth,
                                    bool printedUpToNode )
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

std::string getObservation( const MCTSDecisionTree::GraphNodeType::ptr & from, const MCTSDecisionTree::GraphNodeType::ptr & to, const MCTSDecisionTree & graph )
{
  std::set< std::string > factIntersection;

  auto siblings = to->siblings();
  siblings.push_back( to );

  if( siblings.size() <= 1 )
  {
    return ""; // we consider that an observation is seen only where there are at least two possible outcomes
  }

  // compute the fact intersection between siblings
  bool firstFactSet{true};
  for( const auto& s: siblings )
  {
    const auto& beliefState = graph.beliefStates_.at( s->data().beliefState_h );
    for(auto w=0; w < beliefState.size(); ++w)
    {
      if( beliefState[w] > 0.0 )
      {
        const auto& facts = graph.states_.at( s->data().states_h[w] );

        if( firstFactSet )
        {
          factIntersection = facts;
        }
        else
        {
          auto currentIntersection = factIntersection;
          factIntersection.clear();
          std::set_intersection( facts.begin(), facts.end(), currentIntersection.begin(), currentIntersection.end(),
                                 std::inserter( factIntersection, factIntersection.begin() ) );
        }

        firstFactSet = false;
      }
    }
  }

  std::vector< std::set<std::string> > emergingFactsPerWorld;

  const auto& beliefState = graph.beliefStates_.at( to->data().beliefState_h );
  for( std::size_t w=0; w < beliefState.size(); ++w )
  {
    if( beliefState[w] > 0.0 )
    {
      const auto& facts = graph.states_.at( to->data().states_h[w] );
      emergingFactsPerWorld.push_back( getEmergingFacts( factIntersection, facts ) );
    }
  }

  std::set< std::string > observationIntersection = emergingFactsPerWorld.front();
  for( const auto& emergingFacts: emergingFactsPerWorld )
  {
    auto currentIntersection = observationIntersection;
    observationIntersection.clear();
    std::set_intersection( emergingFacts.begin(), emergingFacts.end(), currentIntersection.begin(), currentIntersection.end(),
                           std::inserter( observationIntersection, observationIntersection.begin() ) );
  }

  return concatenateFactsWithNewLine( observationIntersection );
}


}
