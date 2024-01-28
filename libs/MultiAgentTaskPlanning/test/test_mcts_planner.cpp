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

TEST_F(MCTSPlannerTest, MCTS)
{
  tp.setR0( -1.0 );
  tp.setNIterMinMax( 1000, 1000000 ); //10 000
  tp.setRollOutMaxSteps( 50 );
  tp.setNumberRollOutPerSimulation( 1 );
  tp.setExplorationTerm( 15 ); // 20
  tp.setVerbose( false );


  tp.setFol( "LGP-2-blocks-1-side-fol.g" );
  tp.solve();
  const auto policy = tp.getPolicy();

  savePolicyToFile( policy, "-mcts" );

  EXPECT_EQ( policy.leaves().size(), 2 );
  EXPECT_EQ( policy.leaves().front()->id(), 3 );
  EXPECT_EQ( policy.leaves().back()->id(), 5 );
}
//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
