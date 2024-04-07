#include <mcts_planner_bs.h>
#include <utils.h>

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

using namespace matp;

static void savePolicyToFile( const Policy & policy, const std::string & suffix = "" )
{
  std::stringstream namess, skenamess;

  namess << "policy-" << policy.id() << suffix;
  policy.save( namess.str() );

  namess << ".gv";
  policy.saveToGraphFile( namess.str() );
}

static void generatePngImage( const std::string & name )
{
  std::string nameCopy( name );
  const std::string ext( ".gv" );
  std::string newName = nameCopy.replace( nameCopy.find( ext ), ext.length(), ".png" );

  std::stringstream ss;
  ss << "dot"   << " ";
  ss << "-Tpng" << " ";
  ss << "-o"    << " ";
  ss << newName << " ";
  ss << name;

  system( ss.str().c_str() );
}

class MCTSPlannerBsTest : public ::testing::Test {
 protected:
  MCTSPlannerBs tp;
};


TEST_F(MCTSPlannerBsTest, MCTS_WhenPlanningTwoTimesWithSameParameters_ExpectSamePolicies)
{
  srand (1);

  MCTSPlannerBs tp_1;
  tp_1.setR0( -1.0, 15.0 );
  tp_1.setNIterMinMax( 200, 1000 );
  tp_1.setRollOutMaxSteps( 50 );
  tp_1.setNumberRollOutPerSimulation( 1 );
  tp_1.setVerbose( true );

  tp_1.setFol( "LGP-2-blocks-1-side-fol.g" );

  tp_1.solve();
  tp_1.saveMCTSGraphToFile( "decision_graph_mcts.gv" );
  generatePngImage( "decision_graph_mcts.gv" );

  const auto policy_1 = tp_1.getPolicy();

  srand (1);

  MCTSPlannerBs tp_2;
  tp_2.setR0( -1.0, 15.0 );
  tp_2.setNIterMinMax( 200, 1000 );
  tp_2.setRollOutMaxSteps( 50 );
  tp_2.setNumberRollOutPerSimulation( 1 );
  tp_2.setVerbose( false );

  tp_2.setFol( "LGP-2-blocks-1-side-fol.g" );

  tp_2.solve();
  const auto policy_2 = tp_2.getPolicy();

  EXPECT_EQ( policy_1, policy_2 );
}

TEST_F(MCTSPlannerBsTest, MCTS_WhenPlanningTwoTimesWithEquivalentParameters_ExpectSamePolicies)
{
  srand (1);

  MCTSPlannerBs tp_1;
  tp_1.setR0( -1.0, 15.0 );
  tp_1.setNIterMinMax( 50, 1000000 );
  tp_1.setRollOutMaxSteps( 50 );
  tp_1.setNumberRollOutPerSimulation( 1 );
  tp_1.setVerbose( true );

  tp_1.setFol( "LGP-2-blocks-1-side-fol.g" );

  tp_1.solve();
  const auto policy_1 = tp_1.getPolicy();

  srand (1);

  MCTSPlannerBs tp_2;
  tp_2.setR0( -0.1, 15.0 );
  tp_2.setNIterMinMax( 50, 1000000 );
  tp_2.setRollOutMaxSteps( 50 );
  tp_2.setNumberRollOutPerSimulation( 1 );
  tp_2.setVerbose( true );

  tp_2.setFol( "LGP-2-blocks-1-side-fol.g" );

  tp_2.solve();
  const auto policy_2 = tp_2.getPolicy();

  EXPECT_EQ( policy_1, policy_2 );
  EXPECT_EQ( tp_1.getValues().values().size(), tp_2.getValues().values().size() );
}

// Updating the internal rewards directly
TEST_F(MCTSPlannerBsTest, MCTS_WhenBuildingMCTSDecisionGraph_AndMakingAnEdgeWithInfiniteCosts_ExpectPolicyChanged)
{
  tp.setR0( -1.0, 15.0 );
  tp.setNIterMinMax( 1000, 1000000 );
  tp.setRollOutMaxSteps( 50 );
  tp.setNumberRollOutPerSimulation( 1 );
  tp.setVerbose( false );

  tp.setFol( "LGP-2-blocks-1-side-fol.g" );

  /// SIMULATE TAMP WITH ACTION INFEASIBLE

  // Step 1
  tp.solve();
  const auto policy = tp.getPolicy();

  tp.saveMCTSGraphToFile( "decision_graph_mcts.gv" );
  savePolicyToFile( policy, "-mcts" );

  EXPECT_EQ( policy.leaves().size(), 2 );

  // simulate intergation of a policy
  tp.getRewards().set( 0, policy.root()->children().front()->data().decisionGraphNodeId, -1000.0);

  // Step 2
  tp.solve();

  const auto policy_1 = tp.getPolicy();

  tp.saveMCTSGraphToFile( "decision_graph_mcts_1.gv" );
  savePolicyToFile( policy_1, "-mcts" );

  EXPECT_EQ( policy_1.leaves().size(), 2 );
  EXPECT_NE( policy.leaves().front()->data().decisionGraphNodeId, policy_1.leaves().front()->data().decisionGraphNodeId );
  EXPECT_NE( policy.leaves().back()->data().decisionGraphNodeId, policy_1.leaves().back()->data().decisionGraphNodeId );
  EXPECT_GE( policy.value(), policy_1.value() );

  // simulate intergation of a policy
  tp.getRewards().set( 0, policy_1.root()->children().front()->data().decisionGraphNodeId, -1000.0 );

  // Step 3
  tp.solve();

  const auto policy_2 = tp.getPolicy();

  tp.saveMCTSGraphToFile( "decision_graph_mcts_2.gv" );
  savePolicyToFile( policy_2, "-mcts" );

  EXPECT_EQ( policy_2.leaves().size(), 2 );
  EXPECT_NE( policy_1.leaves().front()->data().decisionGraphNodeId, policy_2.leaves().front()->data().decisionGraphNodeId );
  EXPECT_NE( policy_1.leaves().back()->data().decisionGraphNodeId, policy_2.leaves().back()->data().decisionGraphNodeId );
  EXPECT_GE( policy_1.value(), policy_2.value() );
}

TEST_F(MCTSPlannerBsTest, MCTS_WhenBuildingMCTSDecisionGraph_AndMakingAnEdgeLessCostlyThanR0_ExpectSamePolicyWithHigherValue)
{
  tp.setR0( -1.0, 15.0 );
  tp.setNIterMinMax( 1000, 1000000 );
  tp.setRollOutMaxSteps( 50 );
  tp.setNumberRollOutPerSimulation( 1 );
  tp.setVerbose( false );

  tp.setFol( "LGP-2-blocks-1-side-fol.g" );

  /// SIMULATE TAMP WITH ACTION INFEASIBLE

  // Step 1
  tp.solve();
  const auto policy = tp.getPolicy();

  tp.saveMCTSGraphToFile( "decision_graph_mcts.gv" );
  savePolicyToFile( policy, "-mcts" );

  EXPECT_EQ( policy.leaves().size(), 2 );
  //EXPECT_EQ( policy.leaves().front()->id(), 3 );
  //EXPECT_EQ( policy.leaves().back()->id(), 5 );

  // simulate intergation of a policy
  tp.getRewards().set( 0, policy.root()->children().front()->data().decisionGraphNodeId, -0.5 );

  // Step 2
  tp.solve();

  const auto policy_1 = tp.getPolicy();

  tp.saveMCTSGraphToFile( "decision_graph_mcts_1_bis.gv" );
  savePolicyToFile( policy_1, "-mcts" );

  EXPECT_EQ( policy_1.leaves().size(), 2 );
  //EXPECT_EQ( policy_1.leaves().front()->id(), 3 );
  //EXPECT_EQ( policy_1.leaves().back()->id(), 5 );
  EXPECT_EQ( policy.leaves().front()->data().decisionGraphNodeId, policy_1.leaves().front()->data().decisionGraphNodeId );
  EXPECT_EQ( policy.leaves().back()->data().decisionGraphNodeId, policy_1.leaves().back()->data().decisionGraphNodeId );
  EXPECT_LT( policy.value(), policy_1.value() );

  // simulate intergation of a policy
  tp.getRewards().set( 0, policy_1.root()->children().front()->data().decisionGraphNodeId, -0.25 );

  // Step 3
  tp.solve();

  const auto policy_2 = tp.getPolicy();

  tp.saveMCTSGraphToFile( "decision_graph_mcts_2_bis.gv" );
  savePolicyToFile( policy_2, "-mcts" );

  EXPECT_EQ( policy_2.leaves().size(), 2 );
  EXPECT_EQ( policy_1.leaves().front()->data().decisionGraphNodeId, policy_2.leaves().front()->data().decisionGraphNodeId );
  EXPECT_EQ( policy_1.leaves().back()->data().decisionGraphNodeId, policy_2.leaves().back()->data().decisionGraphNodeId );
  EXPECT_LT( policy_1.value(), policy_2.value() );
}

// Updating the policy
TEST_F(MCTSPlannerBsTest, MCTS_WhenBuildingMCTSDecisionGraph_AndSimulatingAnInfeasibleMotionPlanning_ExpectPolicyChanged)
{
  tp.setR0( -1.0, 15.0 );
  tp.setNIterMinMax( 1000, 1000000 );
  tp.setRollOutMaxSteps( 50 );
  tp.setNumberRollOutPerSimulation( 1 );
  tp.setVerbose( false );

  tp.setFol( "LGP-2-blocks-1-side-fol.g" );

  // Step 1
  tp.solve();
  const auto policy = tp.getPolicy();

  tp.saveMCTSGraphToFile( "decision_graph_mcts.gv" );
  savePolicyToFile( policy, "-mcts" );

  EXPECT_EQ( policy.leaves().size(), 2 );
  EXPECT_EQ( policy.leaves().front()->id(), 3 );
  EXPECT_EQ( policy.leaves().back()->id(), 5 );

  // simulate intergation of a policy
  //tp.getRewards().set( 0, policy.root()->children().front()->data().decisionGraphNodeId, -1000.0);
  policy.root()->children().front()->data().markovianReturn = -1000.0;
  tp.integrate( policy );

  // Step 2
  tp.solve();

  const auto policy_1 = tp.getPolicy();

  tp.saveMCTSGraphToFile( "decision_graph_mcts_1.gv" );
  savePolicyToFile( policy_1, "-mcts" );

  EXPECT_EQ( policy_1.leaves().size(), 2 );
  EXPECT_EQ( policy_1.leaves().front()->id(), 3 );
  EXPECT_EQ( policy_1.leaves().back()->id(), 5 );
  EXPECT_NE( policy.leaves().front()->data().decisionGraphNodeId, policy_1.leaves().front()->data().decisionGraphNodeId );
  EXPECT_NE( policy.leaves().back()->data().decisionGraphNodeId, policy_1.leaves().back()->data().decisionGraphNodeId );
  EXPECT_GE( policy.value(), policy_1.value() );
}

// simulate full tamp with mocked motion planning

struct MCTSPlannerBsTestWithMockedMotionPlanning : public ::testing::Test {
  void solveAndInform(Policy& policy)
  {
    std::queue< Policy::GraphNodeTypePtr > Q;
    Q.push( policy.root() );

    while( ! Q.empty() )
    {
      auto n = Q.front();
      Q.pop();

      // find parents in tree_
      if( n->id() % 3 == 0 ) n->data().markovianReturn = -10.0;
      if( n->id() % 3 == 1 ) n->data().markovianReturn = -1.0; // n->data().decisionGraphNodeId
      if( n->id() % 3 == 2 ) n->data().markovianReturn = -0.1;

      // push children on queue
      for( const auto & c : n->children() )
      {
        Q.push( c );
      }
    }
  }

  std::tuple<Policy, std::size_t> doTamp( const double r0 )
  {
    srand (1);

    tp = MCTSPlannerBs{};
    tp.setR0( r0, 30.0 );
    tp.setNIterMinMax( 5000, 1000000 );
    tp.setRollOutMaxSteps( 50 );
    tp.setNumberRollOutPerSimulation( 1 );
    tp.setVerbose( false );

    tp.setFol( "LGP-3-blocks-1-side-fol.g" );

    Policy policy, lastPolicy;

    tp.solve();
    policy = tp.getPolicy();

    uint nIt = 0;
    const uint maxIt = 1000;
    do
    {
      nIt++;

      savePolicyToFile( policy, "-candidate" );

      lastPolicy = policy;

      /// MOTION PLANNING
      solveAndInform( policy );
      savePolicyToFile( policy, "-informed" );

      /// TASK PLANNING
      tp.integrate( policy );
      tp.solve();

      policy = tp.getPolicy();
    }
    while( lastPolicy != policy && nIt != maxIt );

    savePolicyToFile( policy, "-final" );

    return std::make_tuple( policy, nIt );
  }

  MCTSPlannerBs tp;
};

TEST_F(MCTSPlannerBsTestWithMockedMotionPlanning, MCTS_WhenBuildingMCTSDecisionGraph_AndSimulatingAnInfeasibleMotionPlanning_ExpectPolicyChanged)
{
  const auto policy_and_it_0 = doTamp(-20.0);
  const auto policy_and_it_1 = doTamp(-1.0);
  const auto policy_and_it_2 = doTamp(-0.05);

  EXPECT_LE( std::get<0>(policy_and_it_0).value(), std::get<0>(policy_and_it_1).value() );
  EXPECT_LE( std::get<0>(policy_and_it_1).value(), std::get<0>(policy_and_it_2).value() );

  EXPECT_LE( std::get<1>(policy_and_it_0), std::get<1>(policy_and_it_1) );
  EXPECT_LE( std::get<1>(policy_and_it_1), std::get<1>(policy_and_it_2) );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
