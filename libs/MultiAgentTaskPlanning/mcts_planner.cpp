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
namespace{
double getPolicyNodePriority( const MCTSDecisionTree::GraphNodeType::ptr& node )
{
  if( ! node->data().isPotentialSymbolicSolution )
  {
    return std::numeric_limits<double>::lowest();
  }

  return node->data().vi_value;
}
}

void MCTSPlanner::setFol( const std::string & descrition )
{
  if( ! boost::filesystem::exists( descrition ) ) throw FolFileNotFound();

  parser_.parse( descrition );
  tree_ = MCTSDecisionTree( parser_.engine(), parser_.possibleStartStates(), parser_.egoBeliefState() );
}

void MCTSPlanner::solve()
{
  if( tree_.empty() )
  {
    expandMCTS();
  }

  valueIteration();
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
  tree_.expandMCTS( rewards_.R0(),
                    nIterMin_,
                    nIterMax_,
                    rollOutMaxSteps_,
                    nRollOutsPerSimulation_,
                    explorationTermC_,
                    verbose_ );
}

void MCTSPlanner::buildPolicy()
{
  std::cout << "MCTSPlanner::buildPolicy.." << std::endl;

  using NodeTypePtr = std::shared_ptr< MCTSDecisionTree::GraphNodeType >;

  std::stack< std::pair< NodeTypePtr, Policy::GraphNodeTypePtr > > Q;

  // create policy root node from decision graph node
  const auto& root = tree_.root();
  PolicyNodeData rootData;
  rootData.beliefState = tree_.beliefStates_.at( root->data().beliefState_h );

  const auto& policyRoot = GraphNode< PolicyNodeData >::root( rootData );

  Q.push( std::make_pair( tree_.root(), policyRoot ) );

  while( ! Q.empty() )
  {
    auto uPair = Q.top();
    Q.pop();

    const auto& u     = uPair.first;
    const auto& uSke = uPair.second;

    if( u->children().empty() )
    {
      CHECK(u->data().terminal, "final policy node should be terminal!");
      uSke->data().terminal = u->data().terminal;
      continue;
    }

    const auto v = *std::max_element( u->children().cbegin(), u->children().cend(), []( const auto& lhs, const auto& rhs ) { return getPolicyNodePriority(lhs) < getPolicyNodePriority(rhs); });

    PolicyNodeData data;// = decisionGraphtoPolicyData( v->data(), v->id() );
    data.beliefState = tree_.beliefStates_.at( v->data().beliefState_h );
    data.markovianReturn = rewards_.R0();
    data.decisionGraphNodeId = v->id();
    data.leadingKomoArgs = decisionArtifactToKomoArgs( tree_.actions_.at( v->data().leadingAction_h ) );
    //data.markovianReturn = rewards_.get( fromToIndex( u->id(), v->id() ) );
    data.p = transitionProbability( uSke->data().beliefState, data.beliefState );
    auto vSke = uSke->makeChild( data );
    //std::cout << "build ske from " << uSke->id() << "(" << u->id() << ") to " << vSke->id()<< " (" << v->id() << ") , p = " << data.p << std::endl;

    if( v->children().empty() )
    {
      CHECK(v->data().terminal, "final policy node should be terminal!");
      uSke->data().terminal = v->data().terminal;
    }

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

void MCTSPlanner::valueIteration()
{
  values_ = ValueIterationOnTreeAlgorithm::process( tree_, rewards_ );
}

}
