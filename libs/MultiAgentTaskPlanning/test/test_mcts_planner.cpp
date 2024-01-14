#include <mcts_planner.h>
#include <utils.h>

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

using namespace matp;

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
  tp.setFol( "data/LGP-2-blocks-1-side-fol.g" );
  tp.solve();
}
//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
