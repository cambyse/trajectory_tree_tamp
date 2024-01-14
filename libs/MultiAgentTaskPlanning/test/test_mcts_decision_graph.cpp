#include <mcts_decision_graph.h>
#include <logic_parser.h>

#include <gtest/gtest.h>

using namespace matp;

TEST( MCTSDecisionGraph, sampleStateIndex )
{
  const std::vector<double> beliefState{0.5, 0.4, 0.1};

  for(auto i{0}; i < 50; ++i)
  {
    std::size_t stateIndex = sampleStateIndex( beliefState );

    EXPECT_LE( stateIndex, beliefState.size() );
  }
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
