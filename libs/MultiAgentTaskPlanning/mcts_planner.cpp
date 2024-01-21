#include <mcts_planner.h>

#include <algorithm>    // std::random_shuffle
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <belief_state.h>
#include <utils.h>
#include <stack>

static double m_inf() { return std::numeric_limits< double >::lowest(); }

namespace matp
{

void MCTSPlanner::setFol( const std::string & descrition )
{
  if( ! boost::filesystem::exists( descrition ) ) throw FolFileNotFound();

  parser_.parse( descrition );
  tree_ = MCTSDecisionGraph( parser_.engine(), parser_.possibleStartStates(), parser_.egoBeliefState() );
}

void MCTSPlanner::solve()
{
  if( tree_.empty() )
  {
    expandMCTS();
  }

  buildPolicy();
}

void MCTSPlanner::integrate( const Policy & policy )
{

}

// getters
bool MCTSPlanner::terminated() const
{
  return false;
}

Policy MCTSPlanner::getPolicy() const
{
  return policy_;
}

void MCTSPlanner::expandMCTS()
{
  //void expandMCTS( const double r0, const std::size_t n_iter_min, const std::size_t n_iter_max, const std::size_t rolloutMaxSteps );

  tree_.expandMCTS( rewards_.R0(), nIterMin_, nIterMax_, rollOutMaxSteps_, explorationTermC_, verbose_ );
}

void MCTSPlanner::buildPolicy()
{
  std::cout << "MCTSPlanner::buildPolicy.." << std::endl;

  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  std::stack< std::pair< NodeTypePtr, Policy::GraphNodeTypePtr > > Q;
  //std::queue< std::pair< NodeTypePtr, Policy::GraphNodeTypePtr > > Q;

  // create policy root node from decision graph node
  const auto& root = tree_.root();
  PolicyNodeData rootData;
  rootData.beliefState = root->data().beliefState;

  const auto& policyRoot = GraphNode< PolicyNodeData >::root( rootData );

  Q.push( std::make_pair( tree_.root(), policyRoot ) );

  while( ! Q.empty() )
  {
    auto uPair = Q.top();
    Q.pop();

    const auto& u     = uPair.first;
    const auto& uSke = uPair.second;

    if( u->children().empty() )
      continue;

    const auto v = *std::max_element( u->children().cbegin(), u->children().cend(), []( const auto& lhs, const auto& rhs ) { return lhs->data().expectedRewardToGoal < rhs->data().expectedRewardToGoal; });

    PolicyNodeData data;// = decisionGraphtoPolicyData( v->data(), v->id() );
    data.beliefState = v->data().beliefState;
    data.markovianReturn = rewards_.R0();
    data.decisionGraphNodeId = v->id();
    data.leadingKomoArgs = decisionArtifactToKomoArgs( v->data().leadingAction );
    //data.markovianReturn = rewards_.get( fromToIndex( u->id(), v->id() ) );
    data.p = transitionProbability(uSke->data().beliefState, data.beliefState);
    auto vSke = uSke->makeChild( data );

//      //std::cout << "build ske from " << uSke->id() << "(" << u->id() << ") to " << vSke->id()<< " (" << v->id() << ") , p = " << data.p << std::endl;

    for( const auto& w : v->children() ) // skip obs nodes
    {
      Q.push( std::make_pair( w, vSke ) );
    }
  }

  policy_ = Policy( policyRoot );

  //policy_.setValue( values_[ decisionGraph().root()->id() ] );
  //CHECK(policy_.value() > std::numeric_limits<double>::lowest(), "extracted policy seems to be infeasible (infinite costs)!");

  std::cout << "MCTSPlanner::buildPolicy.. end (value=" << policy_.value() << ")" << std::endl;
}

}
