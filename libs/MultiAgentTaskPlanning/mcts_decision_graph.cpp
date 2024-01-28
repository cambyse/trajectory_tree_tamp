#include <mcts_decision_graph.h>

#include <set>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <numeric>
#include <decision_graph_printer.h>

namespace matp
{
namespace
{
std::vector < double > normalizeBs( const std::vector < double > & bs )
{
  std::vector < double > newBs = bs;

  auto sumfunc = []( const std::vector < double > bs ) -> double
  {
    double sum = 0;
    for( auto p : bs )
    {
      sum += p;
    }
    return sum;
  };

  const auto sum = sumfunc( bs );

  for( auto w = 0; w < bs.size(); ++w )
  {
    newBs[ w ] = bs[ w ] / sum;
  }

  CHECK( fabs( sumfunc( newBs ) - 1.0 ) < 0.00001, "" );

  return newBs;
}
}

double priorityUCT( const GraphNode< MCTSNodeData >::ptr& node, const double c, bool verbose )
{
  const auto& node_data = node->data();

  std::size_t parent_n_rollouts{node_data.n_rollouts};
  if( node->parent() )
  {
    CHECK( node->parent() != nullptr, "node shouldn't be a root node!" );
    CHECK( node->parent()->data().nodeType == MCTSNodeData::NodeType::ACTION, "wrong node type!" );

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

GraphNode< MCTSNodeData >::ptr getMostPromisingChild( const GraphNode< MCTSNodeData >::ptr& node, const double c, const bool verbose )
{
  double max_prio{std::numeric_limits<double>::lowest()};
  GraphNode< MCTSNodeData >::ptr best_uct_child{};
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

std::size_t getHash( const std::vector< double >& beliefState )
{
  std::size_t hash{0};

  for( auto w{0}; w < beliefState.size(); ++w )
  {
    hash += std::hash<double>()(beliefState[w]) << w;
  }

  return hash;
}

std::size_t getHash( const std::vector< std::size_t >& states_h, const std::size_t beliefState_h )
{
  std::size_t hash{0};

  for( auto w{0}; w < states_h.size(); ++w )
  {
    hash += states_h[w] << w;
  }

  return hash + beliefState_h;
}

void backtrackIsPotentialSymbolicSolution( const GraphNode< MCTSNodeData >::ptr& node )
{
  CHECK( node->data().nodeType == MCTSNodeData::NodeType::ACTION, "we should only backtrack from action nodes!" );

  if( node->parents().empty() )
  {
    return;
  }

  CHECK( node->parents().size() == 1, "MCTS works here for trees only!" );

  const auto observationParent = node->parents().front().lock();

  const auto observationParentIsSolved = std::accumulate(observationParent->children().begin(),
                                                         observationParent->children().end(),
                                                         true,
                                                         [](bool solved, const auto& n) { return solved && n->data().isPotentialSymbolicSolution; });

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
    std::vector< std::size_t > states_h;
    states_h.reserve( startStates.size() );
    for( auto & s : filteredStartStates )
    {
      s = concatenateFacts( getFilteredFacts( s ) );
      const auto hash = getHash( s );
      states_[ hash ] = s;
      states_h.push_back( hash );
    }

    const auto bs_h = getHash( egoBeliefState );
    beliefStates_[ bs_h ] = egoBeliefState;

    const auto node_h = getHash( states_h, bs_h );

    root_ = GraphNode< MCTSNodeData >::root( GraphNodeDataType( states_h, bs_h, node_h, false, MCTSNodeData::NodeType::ACTION ) );

    nodesData_[ node_h ] = root_->data();
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

  while( (  !root_->data().isPotentialSymbolicSolution || n_iter < n_iter_min ) && n_iter < n_iter_max )
  {
    std::cout << "\n-------" << n_iter << "-------" << std::endl;

    // sample state
    const auto& beliefState = beliefStates_.at( root_->data().beliefState_h );
    const auto stateIndex = sampleStateIndex( beliefState );

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

double MCTSDecisionGraph::simulate( const MCTSDecisionGraph::GraphNodeType::ptr& node,
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

    const auto& states_h = node->data().states_h;
    const auto& beliefState_h = node->data().beliefState_h;
    const auto node_h = node->data().node_h;

    // expand node (using bs and not sampled state)
    const auto& actions_h = getCommonPossibleActions( node_h );

    for( const auto& action_h : actions_h )
    {
      // node after action, will receive one or several observations
      auto child = node->makeChild( GraphNodeDataType( states_h, beliefState_h, node_h, false, MCTSNodeData::NodeType::OBSERVATION ) );
      child->data().leadingAction_h = action_h;
      child->data().leadingProbability = 1.0;

      if(verbose) std::cout << "[simulate] created " << child->id() << std::endl;
    }

    std::vector<double> rollOutBeliefState;
    rollOutBeliefState.resize( beliefStates_.at( beliefState_h ).size() );
    rollOutBeliefState[ stateIndex ] = 1.0;

    expandedNodesIds.insert( node->id() );

    if(verbose) std::cout << "[simulate] compute rollout from " << node->id() << std::endl;

    const auto& rollOutState_h = nodesData_.at( node->data().node_h ).states_h[ stateIndex ];
    return rollOutOneWorld( rollOutState_h, r0, 0, rolloutMaxSteps, nRolloutsPerSimulation, verbose );
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
    const auto outcomes_h = getPossibleOutcomes( node->data().node_h, best_uct_child->data().leadingAction_h );

    for( const auto& outcome_h: outcomes_h )
    {
      const auto& outcome = nodesData_.at( outcome_h );
      const auto childChild = best_uct_child->makeChild( outcome );

      if(verbose) std::cout << "[simulate] create " << childChild->id() << " (observation: " << observations_.at( outcome.leadingObservation_h ) << ")" << std::endl;

      if( outcome.terminal )
      {
        terminalNodes_.push_back( childChild );
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

double MCTSDecisionGraph::rollOutOneWorld( const std::size_t state_h,
                                           const double r0,
                                           const std::size_t steps,
                                           const std::size_t rolloutMaxSteps,
                                           const std::size_t nRolloutsPerSimulation,
                                           const bool verbose ) const
{
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

std::vector< std::size_t > MCTSDecisionGraph::getCommonPossibleActions( const std::size_t node_h ) const
{
  CHECK(nodesData_.find( node_h ) != nodesData_.end(), "");

  // if in cache we return cahce content
  const auto node_cache_it = nodeHToActions_.find( node_h );
  if( node_cache_it != nodeHToActions_.end() )
  {
    return nodeHToActions_.at( node_h );
  }

  const auto& nodeData = nodesData_.at( node_h );

  std::vector< std::size_t > actions_h;
  for( auto w{0}; w < nodeData.states_h.size(); w++ )
  {
    const auto p = beliefStates_.at( nodeData.beliefState_h )[w];

    if(p < 1.0e-6)
    {
        continue;
    }

    const auto state_h = nodeData.states_h[w];
    const auto actions_it = stateToActionsH_.find( state_h );

    // if actions are in cache, simply lookup
    if( actions_it != stateToActionsH_.end() )
    {
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

std::vector< std::size_t > MCTSDecisionGraph::getPossibleOutcomes( const std::size_t node_h,
                                                                   const std::size_t action_h ) const
{
  std::vector< std::size_t > outcomes;

  const auto key = std::make_pair( node_h, action_h );
  const auto outcomeCache_it = nodeHActionToNodesH_.find( key );

  if( outcomeCache_it != nodeHActionToNodesH_.end() )
  {
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
    if( beliefState[ w ] > 0 )
    {
      const auto state_h = nodeData.states_h[w];

      std::size_t nextState_h{0};

      const auto stateActionKey = std::make_pair( state_h, action_h );
      const auto c_it = stateActionHToNextState_.find( stateActionKey );
      if( false && c_it != stateActionHToNextState_.end() )
      {
        nextState_h = c_it->second;
      }
      else
      {
        const auto& state = states_.at( state_h );

        if( lastSetStateEngine_ != state_h )
        {
          engine_.setState( state );
        }

        const auto& action = actions_.at( action_h );
        engine_.transition( action );
        const auto _result = engine_.getState();                  // concatenation

        const auto facts            = getFilteredFacts( _result );// split, filtering (without komo and decision tags)
        const auto observableFacts  = getObservableFacts( facts );// filtering
        const auto result           = concatenateFacts( facts );  // concatenation (filtered)
        const auto terminal         = engine_.isTerminal();

        std::set< std::string > newIntersection;

        if( factIntersection.empty() )
        {
          newIntersection = facts;
        }
        else
        {
          std::set_intersection( facts.begin(), facts.end(), factIntersection.begin(), factIntersection.end(),
                                 std::inserter( newIntersection, newIntersection.begin() ) );

        }

        // store results
        const auto nextState_h = getHash( result ); // no decision // split, filtering
        factIntersection = newIntersection;
        observableStatesToStates[ observableFacts ].push_back( std::make_pair( w, nextState_h ) );
        terminalOutcome         [ observableFacts ] = terminal;

        // cache action transition
        if( states_.find( nextState_h ) == states_.end() )
        {
          states_[ nextState_h ] = result;
        }
        stateActionHToNextState_[ stateActionKey ] = nextState_h;

        lastSetStateEngine_ = nextState_h;
      }
    }
  }

  for( const auto& observableResultPair : observableStatesToStates )
  {
    std::vector< std::size_t > states_h( beliefState.size(), 0 );
    std::vector< double > newBs( beliefState.size(), 0 );

    const auto & worldToOutcomes = observableResultPair.second;
    const auto observationFacts = getEmergingFacts( factIntersection, observableResultPair.first );
    const auto observation      = concatenateFacts( observationFacts );
    const auto terminal         = terminalOutcome[ observableResultPair.first ];

    double p = 0;
    for( const auto & worldOutcome :  worldToOutcomes )
    {
      auto w = worldOutcome.first;
      auto state = worldOutcome.second;

      p += beliefState[ w ];
      states_h[ w ] = state;
      newBs[ w ] = beliefState[ w ];
    }

    newBs = normalizeBs( newBs );

    const auto observation_h = getHash( observation );

    if( observations_.find( observation_h ) == observations_.end() )
    {
      observations_[ observation_h ] = observation;
    }

    const auto beliefState_h = getHash( newBs );
    const auto node_h = getHash( states_h, beliefState_h );

    if( beliefStates_.find( beliefState_h ) == beliefStates_.end() )
    {
      beliefStates_[ beliefState_h ] = newBs;
    }

    if( nodesData_.find( node_h ) == nodesData_.end() )
    {
      MCTSNodeData nodeData( states_h, beliefState_h, node_h, terminal, MCTSNodeData::NodeType::ACTION );
      nodeData.leadingObservation_h = observation_h;
      nodeData.leadingProbability = p;
      nodesData_[ node_h ] = nodeData;
    }

    outcomes.push_back( node_h );
  }

  if(outcomes.empty())
  {
    int a{0};
  }

  return outcomes;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  //        observable facts                                    world state
//  std::map< std::set< std::string >, std::vector< std::pair< uint, std::string > > > observableStatesToStates;
//  std::map< std::set< std::string >, bool > terminalOutcome;
//  std::set< std::string > factIntersection;

//  for( auto w = 0; w < bs.size(); ++w )
//  {
//    if( bs[ w ] > 0 )
//    {
//      const auto& startState = states[ w ];
//      engine.setState( startState );

//      //std::cout << "start state:" << startState << std::endl; // tmp camille
//      //std::cout << "action:" << action << std::endl; // tmp camille

//      engine.transition( action );

//      const auto _result          = engine.getState();
//      const auto facts            = getFilteredFacts( _result );// without komo and action tags
//      const auto result           = concatenateFacts( facts );
//      const auto observableFacts  = getObservableFacts( facts );
//      const auto terminal         = engine.isTerminal();

//      //std::cout << "result:" << result << std::endl; // tmp camille

//      std::set< std::string > newIntersection;

//      if( factIntersection.empty() )
//      {
//        newIntersection = facts;
//      }
//      else
//      {
//        std::set_intersection( facts.begin(), facts.end(), factIntersection.begin(), factIntersection.end(),
//                               std::inserter( newIntersection, newIntersection.begin() ) );
//      }

//      // store results
//      factIntersection = newIntersection;
//      observableStatesToStates[ observableFacts ].push_back( std::make_pair( w, result ) );
//      terminalOutcome         [ observableFacts ] = terminal;
//    }
//  }

//  //
//  for( auto observableResultPair : observableStatesToStates )
//  {
//    std::vector< std::string > states( bs.size(), "" );
//    std::vector< double > newBs( bs.size(), 0 );

//    const auto & worldToOutcomes = observableResultPair.second;
//    auto observationFacts = getEmergingFacts( factIntersection, observableResultPair.first );
//    auto observation      = concatenateFacts( observationFacts );
//    auto terminal         = terminalOutcome[ observableResultPair.first ];

//    double p = 0;
//    for( const auto & worldOutcome :  worldToOutcomes )
//    {
//      auto w = worldOutcome.first;
//      auto state = worldOutcome.second;

//      p += bs[ w ];
//      states[ w ] = state;
//      newBs[ w ] = bs[ w ];
//    }

//    newBs = normalizeBs( newBs );

//    outcomes.push_back( std::make_tuple( p, NodeData( states, newBs, terminal, agentId, NodeData::NodeType::OBSERVATION ), observation ) );
//  }
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

    const auto next_h = getHash( nextState ); // with decision?

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

} // namespace matp
