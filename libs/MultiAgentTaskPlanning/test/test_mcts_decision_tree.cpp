#include <mcts_decision_tree.h>
#include <logic_parser.h>

#include <gtest/gtest.h>

using namespace matp;

TEST( MCTSDecisionTree, sampleStateIndex )
{
  const std::vector<double> beliefState{0.5, 0.4, 0.1};

  for(auto i{0}; i < 50; ++i)
  {
    std::size_t stateIndex = sampleStateIndex( beliefState );

    EXPECT_LE( stateIndex, beliefState.size() );
  }
}

TEST( MCTSDecisionTree, testBeliefStateHash )
{
  std::vector<double> bs_1 = std::vector<double>(72);
  bs_1[38] = 0.2;
  bs_1[44] = 0.2;
  bs_1[56] = 0.2;
  bs_1[62] = 0.2;
  bs_1[68] = 0.2;

  std::vector<double> bs_2 = std::vector<double>(72);
  bs_2[53] = 1.0;

  const auto h_1 = getHash(bs_1);
  const auto h_2 = getHash(bs_2);

  EXPECT_NE( h_1, h_2 );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
