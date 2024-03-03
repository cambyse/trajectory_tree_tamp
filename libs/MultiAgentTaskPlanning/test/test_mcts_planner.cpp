#include <mcts_planner.h>
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

class MCTSPlannerTest : public ::testing::Test {
 protected:
  MCTSPlanner tp;
};

TEST_F(MCTSPlannerTest, MCTS_WhenBuildingMCTSDecisionGraph_AndMakingAnEdgeWithInfiniteCosts_ExpectPolicyChanged)
{
  tp.setR0( -1.0 );
  tp.setNIterMinMax( 1000, 1000000 );
  tp.setRollOutMaxSteps( 50 );
  tp.setNumberRollOutPerSimulation( 1 );
  tp.setExplorationTerm( 15 ); // 20
  tp.setVerbose( false );

  tp.setFol( "LGP-2-blocks-1-side-fol.g" );

  /// SIMULATE TAMP WITH ACTION INFEASIBLE

  // Step 1
  tp.solve();
  const auto policy = tp.getPolicy();

  tp.saveMCTSGraphToFile( "decision_graph_mcts.gv" );
  savePolicyToFile( policy, "-mcts" );

  EXPECT_EQ( policy.leaves().size(), 2 );
  EXPECT_EQ( policy.leaves().front()->id(), 3 );
  EXPECT_EQ( policy.leaves().back()->id(), 5 );

  // simulate intergation of a policy
  tp.getRewards().set( 0, policy.root()->children().front()->data().decisionGraphNodeId, -1000.0);

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

TEST_F(MCTSPlannerTest, MCTS_WhenBuildingMCTSDecisionGraph_AndMakingAnEdgeLessCostlyThanR0_ExpectSamePolicyWithHigherValue)
{
  tp.setR0( -1.0 );
  tp.setNIterMinMax( 1000, 1000000 );
  tp.setRollOutMaxSteps( 50 );
  tp.setNumberRollOutPerSimulation( 1 );
  tp.setExplorationTerm( 15 ); // 20
  tp.setVerbose( false );

  tp.setFol( "LGP-2-blocks-1-side-fol.g" );

  /// SIMULATE TAMP WITH ACTION INFEASIBLE

  // Step 1
  tp.solve();
  const auto policy = tp.getPolicy();

  tp.saveMCTSGraphToFile( "decision_graph_mcts.gv" );
  savePolicyToFile( policy, "-mcts" );

  EXPECT_EQ( policy.leaves().size(), 2 );
  EXPECT_EQ( policy.leaves().front()->id(), 3 );
  EXPECT_EQ( policy.leaves().back()->id(), 5 );

  // simulate intergation of a policy
  tp.getRewards().set( 0, policy.root()->children().front()->data().decisionGraphNodeId, -0.5 );

  // Step 2
  tp.solve();

  const auto policy_1 = tp.getPolicy();

  tp.saveMCTSGraphToFile( "decision_graph_mcts_1_bis.gv" );
  savePolicyToFile( policy_1, "-mcts" );

  EXPECT_EQ( policy_1.leaves().size(), 2 );
  EXPECT_EQ( policy_1.leaves().front()->id(), 3 );
  EXPECT_EQ( policy_1.leaves().back()->id(), 5 );
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
//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
