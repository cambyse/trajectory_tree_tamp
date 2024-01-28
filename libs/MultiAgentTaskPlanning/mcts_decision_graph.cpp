#include <mcts_decision_graph.h>

#include <set>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <numeric>
#include <decision_graph_printer.h>

namespace matp
{

double priorityUCT( const GraphNode< NodeData >::ptr& node, const double c, bool verbose )
{
  const auto& node_data = node->data();

  std::size_t parent_n_rollouts{node_data.n_rollouts};
  if( node->parent() )
  {
    CHECK( node->parent() != nullptr, "node shouldn't be a root node!" );
    CHECK( node->parent()->data().nodeType == NodeData::NodeType::ACTION, "wrong node type!" );

    const auto& action_parent = node->parent();
    const auto& parent_node_data = action_parent->data();

    parent_n_rollouts = parent_node_data.n_rollouts;
  }

  if( node_data.n_rollouts == 0 )
  {
    if( verbose ) std::cout << "[priority] of " << node->id() << " = " << std::numeric_limits<double>::infinity() << std::endl;

    return std::numeric_limits<double>::infinity();
  }

  const double priority = node_data.value + c * sqrt( log( ( parent_n_rollouts ) / ( node_data.n_rollouts ) ) );

  if( verbose ) std::cout << "[priority] of " << node->id() << " = " << priority << std::endl;

  return priority;
}

std::size_t sampleStateIndex( const std::vector< double >& bs )
{
  // build integrated belief state, TODO: Compute once and for all and store instead of recomputing all the time
  std::vector<double> integratedBs;
  integratedBs.resize( bs.size() );

  double lastIntergatedProbability{0.0};
  for( auto w{0}; w < bs.size(); ++w )
  {
    integratedBs[w] = lastIntergatedProbability + bs[w];
    lastIntergatedProbability = integratedBs[w];
  }

  const auto v = ( ( double ) rand() / ( RAND_MAX ) );

  const auto s_it = std::lower_bound( integratedBs.begin(), integratedBs.end(), v );

  return std::distance( integratedBs.begin(), s_it );
}

std::string getNotObservableFact( const std::string& fullState )
{
  std::istringstream ss( fullState );
  std::string chunk;

  while( std::getline(ss, chunk, ',' ) )
  {
    if( chunk.find("NOT_OBSERVABLE") != std::string::npos )
    {
      return chunk;
    }
  }
  return "";
}

GraphNode< NodeData >::ptr getMostPromisingChild( const GraphNode< NodeData >::ptr& node, const double c, const bool verbose )
{
  double max_prio{std::numeric_limits<double>::lowest()};
  GraphNode< NodeData >::ptr best_uct_child{};
  for( const auto& child : node->children() )
  {
    const auto prio = priorityUCT( child, c, verbose );

    if( prio > max_prio )
    {
      max_prio = prio;
      best_uct_child = child;
    }
  }

  return best_uct_child;
}

std::size_t getHash( const std::string& state )
{
  const auto facts = matp::getFilteredFacts(state);

  std::size_t hash{0};
  for(const auto& fact: facts)
  {
    hash += std::hash<std::string>()(fact);
  }

  return hash;
}

void backtrackIsPotentialSymbolicSolution( const GraphNode< NodeData >::ptr& node )
{
  CHECK( node->data().nodeType == NodeData::NodeType::ACTION, "we should only backtrack from action nodes!" );

  if( node->parents().empty() )
  {
    return;
  }

  CHECK( node->parents().size() == 1, "MCTS works here for trees only!" );

  const auto observationParent = node->parents().front().lock();

  const auto observationParentIsSolved = std::accumulate(observationParent->children().begin(),
                                                         observationParent->children().end(),
                                                         true,
                                                         [](bool solved, const GraphNode< NodeData >::ptr& n) { return solved && n->data().isPotentialSymbolicSolution; });

  observationParent->data().isPotentialSymbolicSolution = observationParentIsSolved;

  CHECK( observationParent->parents().size() == 1, "MCTS works here for trees only!" );

  if(observationParentIsSolved)
  {
    const auto actionParent =  observationParent->parents().front().lock();

    actionParent->data().isPotentialSymbolicSolution = true;

    //std::cout << "Set " << actionParent->id() << " is on potential solution!" << std::endl;

    backtrackIsPotentialSymbolicSolution( actionParent );
  }
}

MCTSDecisionGraph::MCTSDecisionGraph( const LogicEngine & engine, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState )
  : engine_( engine )
  , root_()
  , lastSetStateEngine_(0)
{
    // filter states before creating root
    std::vector< std::string > filteredStartStates = startStates;

    for( auto & s : filteredStartStates )
    {
      s = concatenateFacts( getFilteredFacts( s ) );
      const auto hash = getHash( s );
      states_[ hash ] = s;
    }

    root_ = GraphNode< NodeData >::root( NodeData( filteredStartStates, egoBeliefState, false, 0, NodeData::NodeType::ACTION ) );
}

void MCTSDecisionGraph::expandMCTS( const double r0,
                                const std::size_t n_iter_min,
                                const std::size_t n_iter_max,
                                const std::size_t rolloutMaxSteps,
                                const std::size_t nRolloutsPerSimulation,
                                const double c,
                                const bool verbose )
{
  // reference https://papers.nips.cc/paper_files/paper/2010/file/edfbe1afcf9246bb0d40eb4d8027d90f-Paper.pdf Alg 1.
  CHECK( engine_.agentNumber() == 1, "MCTS works here only for one agent" );

  std::size_t n_iter{0};

  std::unordered_set< uint > expandedNodesIds;

  while( (  !root_->data().isPotentialSymbolicSolution /*root_->data().expectedRewardToGoal <= std::numeric_limits<double>::lowest()*/ || n_iter < n_iter_min ) && n_iter < n_iter_max )
  {
    std::cout << "\n-------" << n_iter << "-------" << std::endl;

    // sample state
    const auto stateIndex = sampleStateIndex( root_->data().beliefState );

    simulate( root_, stateIndex, 0, r0, rolloutMaxSteps, nRolloutsPerSimulation, c, expandedNodesIds, verbose );

    // save current MCTS tree
    //std::stringstream ss;
    //ss << "decision_graph_" << n_it << ".gv";
    //saveMCTSTreeToFile( ss.str(), getNotObservableFact( root_->data().states[stateIndex] ) );

    n_iter++;
  }

  //std::stringstream ss;
  //ss << "decision_graph_final" << ".gv";
  //saveMCTSTreeToFile( ss.str(), "" );
}

double MCTSDecisionGraph::simulate( const DecisionGraph::GraphNodeType::ptr& node,
                                    const std::size_t stateIndex,
                                    const std::size_t depth,
                                    const double r0,
                                    const std::size_t rolloutMaxSteps,
                                    const std::size_t nRolloutsPerSimulation,
                                    const double c,
                                    std::unordered_set< uint > & expandedNodesIds,
                                    const bool verbose )
{
  /// Note: Here we enter with action nodes only! leafs are always action nodes
  if(verbose) std::cout << "[simulate] Simulate from node: " << node->id() << std::endl;

  const auto node_id_it = expandedNodesIds.find( node->id() );

  if( node->data().terminal )
  {
    if(verbose) std::cout << "[simulate] Already terminal! no costs" << std::endl;

    //std::cout << "stateIndex:" << stateIndex << " node:" << node->id() << std::endl;

    if( ! node->data().isPotentialSymbolicSolution )
    {
      node->data().isPotentialSymbolicSolution = true;
      backtrackIsPotentialSymbolicSolution( node );
      node->data().n_rollouts = std::numeric_limits<std::size_t>::max(); // no need for additional rollouts to estimate reward from this node!
    }

    return 0.0;
  }

  // 1. EXPAND to create new node (observation) nodes corresponding to the outocome after each possible actions
  if( node_id_it == expandedNodesIds.end() )
  {
    if(verbose) std::cout << "[simulate] not expanded, so expand.." << std::endl;

    const auto& states = node->data().states;
    const auto& beliefState = node->data().beliefState;

    for( const auto& state: states )
    {
      states_[ getHash( state ) ] = state;
    }

    // expand node (using bs and not sampled state)
    const auto& actions = getCommonPossibleActions( states, beliefState, 0, engine_ );

    for( const auto& action : actions )
    {
      // node after action, will receive one or several observations
      auto child = node->makeChild( DecisionGraph::GraphNodeDataType( states, beliefState, false, 0, NodeData::NodeType::OBSERVATION ) );
      child->data().leadingAction = action;

      if(verbose) std::cout << "[simulate] created " << child->id() << std::endl;
    }

    std::vector<double> rollOutBeliefState;
    rollOutBeliefState.resize( beliefState.size() );
    rollOutBeliefState[ stateIndex ] = 1.0;

    expandedNodesIds.insert( node->id() );

    if(verbose) std::cout << "[simulate] compute rollout from " << node->id() << std::endl;

    return rollOutOneWorld( node->data().states[stateIndex] , r0, 0, rolloutMaxSteps, nRolloutsPerSimulation, verbose );
  }

  if( node->children().empty() )
  {
    return 0.0;
  }

  // 2. Choose best UCT action based on UCB
  const auto best_uct_child = getMostPromisingChild( node, c, verbose );

  if(verbose) std::cout << "[simulate] node " << best_uct_child->id() << " is most promising so far.." << std::endl;

  // 3. Expand from the chosen node by simulating all possible received observations
  if( expandedNodesIds.find( best_uct_child->id() ) == expandedNodesIds.end() )
  {
    if(verbose) std::cout << "[simulate] not expanded, so expand based on possible observations.." << std::endl;

    CHECK( best_uct_child->children().empty(), "A non-expanded node should not have children!" );
    // expand observations - we expand just once
    // check outcomes after taking best action
    const auto outcomes = matp::getPossibleOutcomes( node->data().states, node->data().beliefState, best_uct_child->data().leadingAction, 0, engine_ );

    for(const auto& outcome: outcomes)
    {
      const auto& p    = std::get<0>(outcome);
      const auto& data = std::get<1>(outcome);
      const auto& observation = std::get<2>(outcome);
      const auto childChildData = DecisionGraph::GraphNodeDataType( data.states, data.beliefState, data.terminal, 0, NodeData::NodeType::ACTION );

      const auto childChild = best_uct_child->makeChild( childChildData );

      if(verbose) std::cout << "[simulate] create " << childChild->id() << " (observation: " << observation << ")" << std::endl;

      if( data.terminal )
      {
        terminalNodes_.push_back( childChild );
      }

      for( const auto& state: data.states )
      {
        states_[ getHash( state ) ] = state;
      }
    }

    expandedNodesIds.insert( best_uct_child->id() );
  }

  CHECK( !best_uct_child->children().empty(), "Should at least have one outcome!" );

  // 4. Get observation based on sampled state. Since we sample one state only there should be one possible outcome only
  std::vector< std::pair< DecisionGraph::GraphNodeType::ptr, double > > candidates; // node after observation and probability of bein in state indicated by stateIndex
  for( auto& child_after_observation : best_uct_child->children() )
  {
    double p = child_after_observation->data().beliefState[stateIndex];

    if( p > 0.0 )
    {
      candidates.push_back( std::make_pair( child_after_observation, p ) );
    }
  }
  CHECK( candidates.size() == 1, "We don't have a probabilistic observation model!" );

  auto& action_node_after_observation = candidates.front().first;

  // 5. Rollout after from chosen action + received observation
  if(verbose) std::cout << "[simulation] based on sample world(" << stateIndex << "), the corresponding child is:" << action_node_after_observation->id() << std::endl;

  const double reward = r0 + simulate( action_node_after_observation,
                                       stateIndex,
                                       depth + 1,
                                       r0,
                                       rolloutMaxSteps,
                                       nRolloutsPerSimulation,
                                       c,
                                       expandedNodesIds,
                                       verbose );

  // increments mc counters
  node->data().n_rollouts++;
  best_uct_child->data().n_rollouts++;

  if(best_uct_child->data().n_rollouts == 1)
  {
    best_uct_child->data().value = reward;
  }
  else
  {
    best_uct_child->data().value = best_uct_child->data().value + (reward - best_uct_child->data().value) / best_uct_child->data().n_rollouts;
  }

  if(verbose) std::cout << "[simulation] simulation counter of " << node->id() << ": " << node->data().n_rollouts << std::endl;
  if(verbose) std::cout << "[simulation] simulation counter of " << best_uct_child->id() << ": " << best_uct_child->data().n_rollouts << std::endl;
  if(verbose) std::cout << "[simulation] reward of " << best_uct_child->id() << ": " << best_uct_child->data().value << std::endl;

  return reward;
}

double MCTSDecisionGraph::rollOutOneWorld( const std::string & state,
                                           const double r0,
                                           const std::size_t steps,
                                           const std::size_t rolloutMaxSteps,
                                           const std::size_t nRolloutsPerSimulation,
                                           const bool verbose ) const
{
  const auto state_h = getHash( state );

  double maxRewards = std::numeric_limits<double>::lowest();
  for(std::size_t i{0}; i < nRolloutsPerSimulation; ++i)
  {
    maxRewards = std::max(maxRewards,
                          rollOutOneWorld( state_h, r0, steps, rolloutMaxSteps, verbose ));
  }

  return maxRewards;
}

double MCTSDecisionGraph::rollOutOneWorld( const std::size_t state_h,
                        const double r0,
                        const std::size_t steps,
                        const std::size_t rolloutMaxSteps,
                        const bool verbose ) const
{
  if( steps == rolloutMaxSteps )
  {
    return 0.0; // TODO, find some heuristic here?
  }

  const auto numberOfActions = getNumberOfPossibleActions( state_h );

  if( numberOfActions == 0 )
  {
    return 0.0;
  }

  // choose action
  const auto action_i = rand() %  numberOfActions;

  if(verbose) std::cout << "  [rollOut] depth: " << steps << ", " << numberOfActions << " possible actions, choosing: " << action_i << " : " << stateToActions_.at( state_h ).at( action_i ) << std::endl;

  const auto outcome = getOutcome( state_h, action_i );

  double simulatedReward{ r0 };

  const auto& next_h = std::get<0>(outcome);
  const auto terminal = std::get<1>(outcome);

  if(!terminal)
  {
    simulatedReward += rollOutOneWorld( next_h, r0, steps + 1, rolloutMaxSteps, verbose );
  }
  else
  {
    if(verbose) std::cout << "  [rollOut] depth: " << steps << " reached a TERMINAL STATE!" << std::endl;
  }

  if(verbose) std::cout << "  [rollOut] depth: " << steps << " simulatedReward: " << simulatedReward << std::endl;

  return simulatedReward;
}

std::size_t MCTSDecisionGraph::getNumberOfPossibleActions( const std::size_t state_h ) const
{
  const auto actions_it = stateToActions_.find( state_h );
  if( actions_it == stateToActions_.end() )
  {
    const auto& state = states_.at( state_h );

    stateToActions_[ state_h ] = getPossibleActions( state, state_h );
  }

  return stateToActions_.at( state_h ).size();
}

std::vector< std::string > MCTSDecisionGraph::getPossibleActions( const std::string & state, const std::size_t state_h ) const
{
  LogicEngine & engine = engine_;

  engine.setState( state );
  lastSetStateEngine_ = state_h;

  return engine.getPossibleActions( 0 );
}

std::tuple< std::size_t, bool > MCTSDecisionGraph::getOutcome( const std::size_t state_h, const std::size_t action_i ) const
{
  const auto stateActionKey = std::make_pair( state_h, action_i );

  const auto stateAction_it = stateActionToNextState_.find( stateActionKey );

  if( stateAction_it == stateActionToNextState_.end() )
  {
    const auto& state = states_.at( state_h );
    const auto& action = stateToActions_.at( state_h ).at( action_i );
    const auto outcome = getOutcome( state, state_h, action );
    lastSetStateEngine_ = state_h;

    const auto nextState = std::get<0>(outcome);
    const auto terminal = std::get<1>(outcome);

    const auto next_h = getHash( nextState );

    states_[ next_h ] = nextState;
    stateActionToNextState_[ stateActionKey ] = next_h;
    terminal_[ next_h ] = terminal;
  }

  return std::make_tuple( stateActionToNextState_.at( stateActionKey ), // state
                          terminal_.at( stateActionToNextState_.at( stateActionKey ) ) // terminality
                        );
}

std::tuple< std::string, bool > MCTSDecisionGraph::getOutcome( const std::string & state, const std::size_t state_h, const std::string& action ) const
{
  LogicEngine & engine = engine_;

  if( state_h != lastSetStateEngine_ )
  {
    engine.setState( state );
  }
  engine.transition( action );

  return std::make_tuple( engine.getState(), engine.isTerminal() );
}

//double MCTSDecisionGraph::rollOut( const std::vector< std::string > & states, const std::vector< double >& bs, const double r0, const std::size_t steps, const std::size_t rolloutMaxSteps, const bool verbose ) const
//{
//  CHECK( engine_.agentNumber() == 1, "not supported for multi agent");

//  if( steps == rolloutMaxSteps )
//  {
//    return 0.0; // TODO, find some heuristic here?
//  }

//  const auto& actions = getCommonPossibleActions( states, bs, 0 );

//  if( actions.empty() )
//  {
//    return 0.0;
//  }

//  // choose action
//  const auto action_index = rand() %  actions.size();
//  const auto& action = actions[action_index];

//  if(verbose) std::cout << "  [rollOut] depth: " << steps << ", " << actions.size() << " possible actions, choosing: " << action_index << " : " << action << std::endl;

//  const auto outcomes = getPossibleOutcomes( states, bs, action, 0 );

//  double simulatedReward{ r0 };

//  if(verbose) std::cout << "  [rollOut] depth: " << steps << ", " << " outcomes.size(): " << outcomes.size() << std::endl;

//  for(const auto &outcome: outcomes)
//  {
//    const auto p = std::get<0>(outcome);
//    const auto& nodeData = std::get<1>(outcome);

//    if(!nodeData.terminal)
//    {
//      simulatedReward += p * rollOut( nodeData.states, nodeData.beliefState, r0, steps + 1, rolloutMaxSteps, verbose );
//    }
//    else
//    {
//      if(verbose) std::cout << "  [rollOut] depth: " << steps << " reached a TERMINAL STATE!" << std::endl;
//    }
//  }

//  if(verbose) std::cout << "  [rollOut] depth: " << steps << " simulatedReward: " << simulatedReward << std::endl;

//  return simulatedReward;
//}

void MCTSDecisionGraph::saveMCTSTreeToFile( const std::string & filename, const std::string & mctsState ) const
{
  if( ! root_ )
  {
    return;
  }

  std::ofstream file;
  file.open( filename );

  MCTSTreePrinter printer( file, mctsState );
  printer.print( *this );

  file.close();

  // png
  std::string nameCopy( filename );
  const std::string ext( ".gv" );
  std::string newName = nameCopy.replace( nameCopy.find( ext ), ext.length(), ".png" );

  std::stringstream ss;
  ss << "dot"   << " ";
  ss << "-Tpng" << " ";
  ss << "-o"    << " ";
  ss << newName << " ";
  ss << filename;

  system( ss.str().c_str() );
}
///

} // namespace matp
