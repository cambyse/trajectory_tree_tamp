#include <mcts_decision_tree.h>

#include <set>
#include <algorithm>
#include <unordered_set>
#include <numeric>
#include <decision_graph_printer.h>
#include <belief_state.h>

namespace matp
{

MCTSDecisionTree::MCTSDecisionTree( const LogicEngine & engine, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState )
  : engine_( engine )
  , root_()
  , lastSetStateEngine_(0)
{
    // filter states before creating root
    std::vector< std::set<std::string> > filteredStartStates;
    std::vector< std::size_t > states_h;
    states_h.reserve( startStates.size() );
    for( const auto & s : startStates )
    {
      const auto facts = getFilteredFacts( s );
      const auto hash = getHash( facts );
      states_[ hash ] = facts;
      states_h.push_back( hash );
    }

    const auto bs_h = getHash( egoBeliefState );
    beliefStates_[ bs_h ] = egoBeliefState;

    const auto node_h = getHash( states_h, bs_h );

    root_ = GraphNode< MCTSNodeData >::root( GraphNodeDataType( states_h, bs_h, node_h, false, MCTSNodeData::NodeType::ACTION, 0, 1.0 ) );

    nodesData_[ node_h ] = root_->data();

    terminality_ = std::vector<bool>(egoBeliefState.size(), false);
}

void MCTSDecisionTree::expandMCTS( const double r0,
                                const std::size_t n_iter_min,
                                const std::size_t n_iter_max,
                                const std::size_t rolloutMaxSteps,
                                const std::size_t nRolloutsPerSimulation,
                                const double c,
                                const bool verbose )
{
  const auto log = [this](std::size_t n_iter, std::size_t stateIndex)
  {
    std::cout << "\n-------" << n_iter << "-------(stateIndex=" << stateIndex << ")" << std::endl;

    for(std::size_t w = 0; w < terminality_.size(); ++w)
    {
      std::cout << (terminality_[w] == true ? "x" : "-");
    }
    std::cout << std::endl;
  };

  // See Alg 1. in https://papers.nips.cc/paper_files/paper/2010/file/edfbe1afcf9246bb0d40eb4d8027d90f-Paper.pdf
  CHECK( engine_.agentNumber() == 1, "MCTS works here only for one agent" );

  std::size_t n_iter{0};

  std::unordered_set< uint > expandedNodesIds;

  const auto& beliefState = beliefStates_.at( root_->data().beliefState_h );

  while( ( ! root_->data().isPotentialSymbolicSolution || n_iter < n_iter_min ) && n_iter < n_iter_max )
  {
    // sample state
    const auto stateIndex = sampleStateIndex( beliefState );

    if(n_iter % 10 == 0) log(n_iter, stateIndex);//std::cout << "\n-------" << n_iter << "-------(stateIndex=" << stateIndex << ")" << std::endl;

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

double MCTSDecisionTree::simulate( const MCTSDecisionTree::GraphNodeType::ptr& node,
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

    return 0.0;
  }

  // 1. EXPAND to create new node (observation) nodes corresponding to the outocome after each possible actions
  if( node_id_it == expandedNodesIds.end() )
  {
    if(verbose) std::cout << "[simulate] not expanded, so expand.." << std::endl;

    const auto& states_h = node->data().states_h;
    const auto& beliefState_h = node->data().beliefState_h;
    const auto node_h = node->data().node_h;

    // expand node (using bs and not sampled state)
    const auto& actions_h = getCommonPossibleActions( node_h );

    for( const auto& action_h : actions_h )
    {
      // node after action, will receive one or several observations
      const auto child = node->makeChild( GraphNodeDataType( states_h, beliefState_h, node_h, false, MCTSNodeData::NodeType::OBSERVATION, action_h, 1.0 ) );

      if(verbose) std::cout << "[simulate] created " << child->id() << std::endl;
    }

    std::vector<double> rollOutBeliefState;
    rollOutBeliefState.resize( beliefStates_.at( beliefState_h ).size() );
    rollOutBeliefState[ stateIndex ] = 1.0;

    expandedNodesIds.insert( node->id() );

    if(verbose) std::cout << "[simulate] compute rollout from " << node->id() << " depth: " << depth << std::endl;

    const auto& rollOutState_h = node->data().states_h[ stateIndex ];
    const auto rolloutResult = rollOutOneWorld( rollOutState_h, r0, 0, rolloutMaxSteps, nRolloutsPerSimulation, verbose );

    if(verbose) std::cout << "[simulate] received rolloutResult: " << rolloutResult << std::endl;

    return rolloutResult;
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
    // check outcomes after taking best actionbeliefStates_

    const auto outcomes_h = getPossibleOutcomes( node->data().node_h, best_uct_child->data().leadingAction_h );

    for( const auto& outcome_h: outcomes_h )
    {
      const auto& outcome = nodesData_.at( outcome_h );
      const auto childChild = best_uct_child->makeChild( outcome );

      // assign non-markovian fields
      childChild->data().leadingProbability = transitionProbability( beliefStates_.at( node->data().beliefState_h ),
                                                                     beliefStates_.at( childChild->data().beliefState_h ) );
      childChild->data().leadingAction_h = best_uct_child->data().leadingAction_h;
      //

      if( verbose ) std::cout << "[simulate] create " << childChild->id() << std::endl; //<< " (observation: " << concatenateFacts( observations_.at( outcome.leadingObservation_h )) << ")" << std::endl;

      if( outcome.terminal )
      {
        terminalNodes_.push_back( childChild );
        childChild->data().isPotentialSymbolicSolution = true;

        const auto& bs = beliefStates_.at(childChild->data().beliefState_h);
        for(std::size_t w = 0; w < bs.size(); ++w)
        {
          terminality_[w] = terminality_[w] || (bs[w] > 0.0);
        }
      }
    }

    // backtracking must be done after adding all nodes
    for(const auto& c: best_uct_child->children() )
    {
      if(c->data().terminal)
      {
        backtrackIsPotentialSymbolicSolution( c );
      }
    }

    expandedNodesIds.insert( best_uct_child->id() );
  }

  CHECK( !best_uct_child->children().empty(), "Should at least have one outcome!" );

  // 4. Get observation based on sampled state. Since we sample one state only there should be one possible outcome only
  std::vector< std::pair< GraphNodeType::ptr, double > > candidates; // node after observation and probability of bein in state indicated by stateIndex
  for( auto& child_after_observation : best_uct_child->children() )
  {
    const auto& childBeliefState = beliefStates_.at( child_after_observation->data().beliefState_h );
    double p = childBeliefState[stateIndex];

    if( p > 0.0 )
    {
      candidates.push_back( std::make_pair( child_after_observation, p ) );
    }
  }
  CHECK( candidates.size() == 1, "We don't have a probabilistic observation model!" );

  auto& action_node_after_observation = candidates.front().first;

  // 5. Rollout after from chosen action + received observation
  if(verbose) std::cout << "[simulation] based on sample world(" << stateIndex << "), the corresponding child is:" << action_node_after_observation->id() << std::endl;

  const double q_value = r0 + simulate( action_node_after_observation,
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
    best_uct_child->data().mcts_q_value = q_value;
  }
  else
  {
    best_uct_child->data().mcts_q_value = best_uct_child->data().mcts_q_value + (q_value - best_uct_child->data().mcts_q_value) / best_uct_child->data().n_rollouts;
  }

  if(verbose) std::cout << "[simulation] simulation counter of " << node->id() << ": " << node->data().n_rollouts << std::endl;
  if(verbose) std::cout << "[simulation] simulation counter of " << best_uct_child->id() << ": " << best_uct_child->data().n_rollouts << std::endl;
  if(verbose) std::cout << "[simulation] q value of " << best_uct_child->id() << ": " << best_uct_child->data().mcts_q_value << std::endl;

  return q_value;
}

double MCTSDecisionTree::rollOutOneWorld( const std::size_t state_h,
                                           const double r0,
                                           const std::size_t steps,
                                           const std::size_t rolloutMaxSteps,
                                           const std::size_t nRolloutsPerSimulation,
                                           const bool verbose ) const
{
  double maxRewards = std::numeric_limits<double>::lowest();
  for(std::size_t i{0}; i < nRolloutsPerSimulation; ++i)
  {
    std::set<std::size_t> visitedStates;
    maxRewards = std::max(maxRewards,
                          rollOutOneWorld( state_h, r0, steps, rolloutMaxSteps, verbose ));
  }

  return maxRewards;
}

double MCTSDecisionTree::rollOutOneWorld( const std::size_t state_h,
                        const double r0,
                        const std::size_t steps,
                        const std::size_t rolloutMaxSteps,
                        const bool verbose ) const
{
  // Note: might be interesting to have an heuristic, but this didn't prove very efficient.
  // Preventing double states is too consraining, and doesn't allow the rollouts to be informative
  if( steps == rolloutMaxSteps )
  {
    return 0.0; // TODO, find some heuristic here?
  }

  const auto& actions_h = getPossibleActions( state_h );

  if( actions_h.size() == 0 )
  {
    CHECK(false, "node should be identified as terminal before, so that we don't end up here");
    return 0.0; // should be terminal before?
  }

  // choose action
  const auto action_i = rand() %  actions_h.size();
  const auto action_h = actions_h[ action_i ];

  if(verbose) std::cout << "  [rollOut] depth: " << steps << ", " << actions_h.size() << " possible actions, trying: " << action_i << " : " << actions_.at( action_h ) << std::endl;

  const auto outcome = getOutcome( state_h, action_h );

  const std::size_t next_h = std::get<0>(outcome);;
  const bool terminal = std::get<1>(outcome);

  if( terminal )
  {
    if(verbose) std::cout << "  [rollOut] depth: " << steps << " reached a TERMINAL STATE!" << std::endl;
    return r0;
  }

  const double simulatedReward = r0 + rollOutOneWorld( next_h, r0, steps + 1, rolloutMaxSteps, verbose );

  if(verbose) std::cout << "  [rollOut] depth: " << steps << " simulatedReward: " << simulatedReward << std::endl;

  return simulatedReward;
}

std::vector< std::size_t > MCTSDecisionTree::getCommonPossibleActions( const std::size_t node_h ) const
{
  CHECK( nodesData_.find( node_h ) != nodesData_.end(), "" );

  // if in cache we return cahce content
  const auto node_cache_it = nodeHToActions_.find( node_h );
  nodeHToActions_details_.nQueries++;

  if( node_cache_it != nodeHToActions_.end() )
  {
    nodeHToActions_details_.nUsedCache++;
    return nodeHToActions_.at( node_h );
  }

  const auto& nodeData = nodesData_.at( node_h );
  const auto& beliefState = beliefStates_.at( nodeData.beliefState_h );

  std::vector< std::size_t > actions_h;
  for( auto w{0}; w < nodeData.states_h.size(); w++ )
  {
    const auto p = beliefState[w];

    if(p < 1.0e-6)
    {
        continue;
    }

    const auto state_h = nodeData.states_h[w];
    const auto actions_it = stateToActionsH_.find( state_h );
    stateToActionsH_details_.nQueries++;

    // if actions are in cache, simply lookup
    if( actions_it != stateToActionsH_.end() )
    {
      stateToActionsH_details_.nUsedCache++;
      for(const auto action_h: actions_it->second)
      {
        // use set?
        if( std::find(actions_h.begin(), actions_h.end(), action_h) == actions_h.end()  )
        {
          actions_h.push_back( action_h );
        }
      }
    }
    else
    {
      const auto& state = states_.at( state_h );

      const auto actions = getPossibleActions( state, state_h );

      for( const auto& action: actions )
      {
        const auto action_h = std::hash<std::string>()(action);
        // use set?
        if( std::find( actions_h.begin(), actions_h.end(), action_h ) == actions_h.end() )
        {
          actions_h.push_back( action_h );
          stateToActionsH_[ state_h ].push_back( action_h );
        }

        if( actions_.find( action_h ) == actions_.end() )
        {
          actions_[ action_h ] = action;
        }
      }
    }
  }

  nodeHToActions_[node_h] = actions_h;

  return actions_h;
}

std::vector< std::size_t > MCTSDecisionTree::getPossibleOutcomes( const std::size_t node_h,
                                                                  const std::size_t action_h ) const
{
  std::vector< std::size_t > outcomes;

  const auto key = std::make_pair( node_h, action_h );
  const auto outcomeCache_it = nodeHActionToNodesH_.find( key );
  nodeHActionToNodesH_details_.nQueries++;

  if( outcomeCache_it != nodeHActionToNodesH_.end() )
  {
    nodeHActionToNodesH_details_.nUsedCache++;
    return outcomeCache_it->second;
  }

  const auto& nodeData = nodesData_.at( node_h );
  const auto& beliefState = beliefStates_.at( nodeData.beliefState_h );

  //        observable facts                                    world state
  std::map< std::set< std::string >, std::vector< std::pair< uint, std::size_t > > > observableStatesToStates;
  std::map< std::set< std::string >, bool > terminalOutcome;
  std::set< std::string > factIntersection;

  for( auto w = 0; w < beliefState.size(); ++w )
  {
    if( beliefState[ w ] > 1.0e-6 )
    {
      const auto state_h = nodeData.states_h[w];

      std::size_t nextState_h{0};

      const auto stateActionKey = std::make_pair( state_h, action_h );
      const auto c_it = stateActionHToNextState_.find( stateActionKey );
      stateActionHToNextState_details_.nQueries++;

      if( c_it != stateActionHToNextState_.end() )
      {
        stateActionHToNextState_details_.nUsedCache++;
        nextState_h = c_it->second;
      }
      else
      {
        const auto& state = states_.at( state_h );

        if( lastSetStateEngine_ != state_h )
        {
          engine_.setFacts( state );
        }

        const auto& action = actions_.at( action_h );
        engine_.transition( action );
        const auto nextState = engine_.getFacts();
        nextState_h = getHash( nextState );
        lastSetStateEngine_ = nextState_h;

        // update cache
        if( states_.find(nextState_h) == states_.end() )
        {
          states_[ nextState_h ] = nextState;
        }

        if( terminal_.find(nextState_h) == terminal_.end() )
        {
          terminal_[nextState_h] = engine_.isTerminal();
        }

        stateActionHToNextState_[ stateActionKey ] = nextState_h;
      }

      const auto& nextState = states_.at( nextState_h );
      const auto observableFacts  = getObservableFacts( nextState );// filtering
      const auto terminal         = terminal_.at( nextState_h );

      std::set< std::string > newIntersection;

      if( factIntersection.empty() )
      {
        newIntersection = nextState;
      }
      else
      {
        std::set_intersection( nextState.begin(), nextState.end(), factIntersection.begin(), factIntersection.end(),
                               std::inserter( newIntersection, newIntersection.begin() ) );

      }

      // store results
      factIntersection = newIntersection;
      observableStatesToStates[ observableFacts ].push_back( std::make_pair( w, nextState_h ) );
      terminalOutcome         [ observableFacts ] = terminal;
    }
  }

  for( const auto& observableResultPair : observableStatesToStates )
  {
    std::vector< std::size_t > states_h( beliefState.size(), 0 );
    std::vector< double > newBs( beliefState.size(), 0 );

    const auto & worldToOutcomes = observableResultPair.second;
    const auto terminal          = terminalOutcome[ observableResultPair.first ];

    for( const auto & worldOutcome :  worldToOutcomes )
    {
      auto w = worldOutcome.first;
      auto state = worldOutcome.second;

      states_h[ w ] = state;
      newBs[ w ] = beliefState[ w ];
    }

    newBs = normalizeBs( newBs );

    const auto beliefState_h = getHash( newBs );
    const auto node_h = getHash( states_h, beliefState_h );

    if( beliefStates_.find( beliefState_h ) == beliefStates_.end() )
    {
      beliefStates_[ beliefState_h ] = newBs;
    }

    if( nodesData_.find( node_h ) == nodesData_.end() )
    {
      MCTSNodeData nodeData( states_h, beliefState_h, node_h, terminal, MCTSNodeData::NodeType::ACTION, 0, 0.0 ); // default action and leading probabilities (since non cachable)!
      nodesData_[ node_h ] = nodeData;
    }

    outcomes.push_back( node_h );
  }

  // update final cache
  nodeHActionToNodesH_[ key ] = outcomes;

  return outcomes;
}

const std::vector< std::size_t >& MCTSDecisionTree::getPossibleActions( const std::size_t state_h ) const
{
  const auto actions_it = stateToActionsH_.find( state_h );
  stateToActionsH_details_.nQueries++;

  if( actions_it == stateToActionsH_.end() )
  {
    const auto& state = states_.at( state_h );
    const auto actions = getPossibleActions( state, state_h );

    std::vector< std::size_t > actions_h;
    for(const auto& action: actions)
    {
      const auto action_h = std::hash< std::string >()( action );
      actions_h.push_back(action_h);
      if(actions_.find(action_h) == actions_.end())
      {
        actions_[action_h] = action;
      }
    }

    stateToActionsH_[ state_h ] = actions_h;
  }
  else
  {
    stateToActionsH_details_.nUsedCache++;
  }

  return stateToActionsH_.at( state_h );
}

std::vector< std::string > MCTSDecisionTree::getPossibleActions( const std::set<std::string> & state, const std::size_t state_h ) const
{
  engine_.setFacts( state );
  lastSetStateEngine_ = state_h;

  return engine_.getPossibleActions( 0 );
}

std::tuple< std::size_t, bool > MCTSDecisionTree::getOutcome( const std::size_t state_h, const std::size_t action_h ) const
{
  const auto stateActionKey = std::make_pair( state_h, action_h );

  const auto stateAction_it = stateActionHToNextState_.find( stateActionKey );
  stateActionHToNextState_details_.nQueries++;

  if( stateAction_it == stateActionHToNextState_.end() )
  {
    const auto& state = states_.at( state_h );
    const auto& action = actions_.at( action_h );
    const auto outcome = getOutcome( state, state_h, action );

    const auto nextState = std::get<0>(outcome);
    const auto terminal = std::get<1>(outcome);

    const auto next_h = getHash( nextState );

    lastSetStateEngine_ = next_h;
    states_[ next_h ] = nextState;
    stateActionHToNextState_[ stateActionKey ] = next_h;
    terminal_[ next_h ] = terminal;
  }
  else
  {
    stateActionHToNextState_details_.nUsedCache++;
  }

  return std::make_tuple( stateActionHToNextState_.at( stateActionKey ), // state
                          terminal_.at( stateActionHToNextState_.at( stateActionKey ) ) // terminality
                        );
}

std::tuple< std::set<std::string>, bool > MCTSDecisionTree::getOutcome( const std::set<std::string> & state, const std::size_t state_h, const std::string& action ) const
{
  LogicEngine & engine = engine_;

  if( state_h != lastSetStateEngine_ )
  {
    engine.setFacts( state );
  }
  engine.transition( action );

  return std::make_tuple( engine.getFacts(), engine.isTerminal() );
}

void MCTSDecisionTree::saveMCTSTreeToFile( const std::string & filename,
                                           const std::string & mctsState,
                                           const Rewards& rewards,
                                           const Values& values ) const
{
  if( ! root_ )
  {
    return;
  }

  std::ofstream file;
  file.open( filename );

  MCTSTreePrinter printer( file, mctsState, rewards, values, 3, 0 );
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


std::string getObservation( const MCTSDecisionTree::GraphNodeType::ptr & from, const MCTSDecisionTree::GraphNodeType::ptr & to, const MCTSDecisionTree & graph )
{
  std::set< std::string > factIntersection;

  auto siblings = to->siblings();
  siblings.push_back( to );

  if( siblings.size() <= 1 )
  {
    return ""; // we consider that an observation is seen only where there are at least two possible outcomes
  }

  // compute the fact intersection between siblings
  bool firstFactSet{true};
  for( const auto& s: siblings )
  {
    const auto& beliefState = graph.beliefStates_.at( s->data().beliefState_h );
    for(auto w=0; w < beliefState.size(); ++w)
    {
      if( beliefState[w] > 0.0 )
      {
        const auto& facts = getObservableFacts( graph.states_.at( s->data().states_h[w] ) );

        if( firstFactSet )
        {
          factIntersection = facts;
        }
        else
        {
          auto currentIntersection = factIntersection;
          factIntersection.clear();
          std::set_intersection( facts.begin(), facts.end(), currentIntersection.begin(), currentIntersection.end(),
                                 std::inserter( factIntersection, factIntersection.begin() ) );
        }

        firstFactSet = false;
      }
    }
  }

  std::vector< std::set<std::string> > emergingFactsPerWorld;

  const auto& beliefState = graph.beliefStates_.at( to->data().beliefState_h );
  for( std::size_t w=0; w < beliefState.size(); ++w )
  {
    if( beliefState[w] > 0.0 )
    {
      const auto& facts = getObservableFacts( graph.states_.at( to->data().states_h[w] ) );
      emergingFactsPerWorld.push_back( getEmergingFacts( factIntersection, facts ) );
    }
  }

  std::set< std::string > observationIntersection = emergingFactsPerWorld.front();
  for( const auto& emergingFacts: emergingFactsPerWorld )
  {
    auto currentIntersection = observationIntersection;
    observationIntersection.clear();
    std::set_intersection( emergingFacts.begin(), emergingFacts.end(), currentIntersection.begin(), currentIntersection.end(),
                           std::inserter( observationIntersection, observationIntersection.begin() ) );
  }

  return concatenateFactsWithNewLine( observationIntersection );
}

} // namespace matp
