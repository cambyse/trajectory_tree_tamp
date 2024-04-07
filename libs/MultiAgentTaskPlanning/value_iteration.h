#pragma once

#include <stdexcept>

#include <queue>
#include <string>
#include <unordered_map>

#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <skeleton.h>
#include <task_planner.h>

#include <logic_parser.h>
#include <decision_graph.h>
#include <mcts_decision_tree.h>

namespace matp
{
// This is value iteration for graphs, so it requires several iterations before convergence.
class ValueIterationAlgorithm
{
public:
  static std::vector< double > process( const DecisionGraph & decisionGraph, Rewards & rewards );
};

class ValueIterationOnTreeAlgorithm
{
public:
  template<typename T>
  static Values process( const T & tree, Rewards & rewards )
  {
    Values values;

    // Dijkstra like
    using NodeTypePtr = std::weak_ptr< MCTSDecisionTree::GraphNodeType >;

    auto cmp = [&values]( const NodeTypePtr & _lhs, const NodeTypePtr & _rhs )
    {
      const auto& lhs = _lhs.lock();
      const auto& rhs = _rhs.lock();

      return values.getOrDefault( lhs->id() ) < values.getOrDefault( rhs->id() );
    };

    std::priority_queue< NodeTypePtr, std::vector< NodeTypePtr >,  decltype(cmp) > Q(cmp);

    // put initial terminal nodes
    for( const auto & m: tree.terminalNodes_ )
    {
      auto n = m.lock();
      values.set(n->id(), 0.0);
      Q.push( n );
    }

    while( ! Q.empty() )
    {
      const auto m = Q.top();
      Q.pop();
      const auto& n = m.lock();

      CHECK( n->data().nodeType == T::GraphNodeDataType::NodeType::ACTION, "Value iteration performed on action nodes only!" );

      // compute value of observation parent, compute value of parent taking different potenial observations into accound
      double value = 0.0;
      auto siblings = n->siblings();
      siblings.push_back( n );
      for( const auto& s: siblings )
      {
        if( s->data().leadingProbability > 0.0 && values.getOrDefault( s->id() ) == values.v0() )
        {
          value = values.v0(); // gentle minus infinity
          break;
        }
        value += s->data().leadingProbability * values.getOrDefault( s->id() );
      }

      // get action parent
      CHECK( n->parents().size() == 1, "Data structure should be a tree, so the nodes should have only one parent" );
      CHECK( n->parents().front().lock()->parents().size() == 1, "Data structure should be a tree, so the nodes should have only one parent" );
      const auto observationParent = n->parents().front().lock();
      const auto actionParent = observationParent->parents().front().lock();

      if( value == values.v0() ) // gentle minus infinity
      {
        // if value is minus inf, no need to update parent
        continue;
      }

      if( value > values.getOrDefault( observationParent->id() ) )
      {
        values.set( observationParent->id(), value );
      }

      const auto potentialNewValue = rewards.get( actionParent->id(), observationParent->id() ) + value;
      const auto previousValue = values.getOrDefault( actionParent->id() );
      if( potentialNewValue > previousValue )
      {
        values.set( actionParent->id(), potentialNewValue );

        CHECK_EQ( values.get( actionParent->id() ), potentialNewValue, "" );

        if( ! actionParent->parents().empty() )
        {
          Q.push( actionParent );
        }
      }
    }

    CHECK( values.values().find( 0 ) != values.values().end(), "No solution found for root!" ); //

    return values;
  }
};

} // namespace matp
