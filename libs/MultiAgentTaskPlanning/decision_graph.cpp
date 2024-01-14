#include <decision_graph.h>

#include <set>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <decision_graph_printer.h>

namespace matp
{

bool sameState ( const NodeData & a, const NodeData & b )
{
  return a.hash() == b.hash();
}

std::ostream& operator<<(std::ostream& stream, NodeData const& data)
{
  stream << "states:" << std::endl;
  for( auto s : data.states )
  {
    stream << s << std::endl;
  }

  stream << "belief state:" << std::endl;
  for( auto p : data.beliefState )
  {
    stream << p << std::endl;
  }

  return stream;
}

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

void DecisionGraph::reset()
{
  //root_.reset();
  edges_.clear();
  nodes_.clear();
  hash_to_id_.clear();
  terminalNodes_.clear();
}

// copy
DecisionGraph::DecisionGraph( const DecisionGraph & graph ) // copy ctor
{
  copy( graph );
}

DecisionGraph& DecisionGraph::operator= ( const DecisionGraph & graph ) // assignment operator
{
  copy( graph );

  return *this;
}

// DecisionGraph
DecisionGraph::DecisionGraph( const LogicEngine & engine, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState )
  : engine_( engine )
  , root_()
{
  // filter states before creating root
  std::vector< std::string > filteredStartStates = startStates;

  for( auto & s : filteredStartStates )
  {
    s = concatenateFacts( getFilteredFacts( s ) );
  }

  root_ = GraphNode< NodeData >::root( NodeData( filteredStartStates, egoBeliefState, false, 0, NodeData::NodeType::ACTION ) );
  hash_to_id_[ root_->data().hash() ].push_back( 0 );

  nodes_.push_back( root_ );
  edges_.push_back( EdgeDataType() ); // dummy edge coming to root
}

void DecisionGraph::build( int maxSteps, bool graph )
{
  std::queue< GraphNode< NodeData >::ptr > queue;
  queue.push( root_ );
  isGraph_ = graph;

  uint step = 0;
  while( ! queue.empty() )
  {
    auto node = queue.front();
    queue.pop();

    step = node->depth() / 2 / engine_.agentNumber();

    if( step < maxSteps )
    {
      auto queueExtension = expand( node );

      while( ! queueExtension.empty() )
      {
        queue.push( std::move( queueExtension.front() ) );
        queueExtension.pop();
      }
    }
    else
    {
      break;
    }
  }
}

std::queue< GraphNode< NodeData >::ptr > DecisionGraph::expand( const GraphNode< NodeData >::ptr & node )
{
  const auto& bs     = node->data().beliefState;
  const auto& states = node->data().states;

  std::queue< GraphNode< NodeData >::ptr > nextQueue;

  nextQueue.push( node );

  // for each agent
  for( uint agentId = 0; agentId < engine_.agentNumber(); ++agentId )
  {
    auto queue = nextQueue; // copy
    nextQueue = std::queue< GraphNode< NodeData >::ptr >();

    while( ! queue.empty() )
    {
      const auto& node = queue.front();
      queue.pop();

      // for each agent
      const auto& actions = getCommonPossibleActions( node, agentId );

      for( const auto& action : actions )
      {
        // node after action, will receive one or several observations
        auto child = node->makeChild( GraphNodeDataType( states, bs, false, agentId, NodeData::NodeType::OBSERVATION ) );
        nodes_.push_back( child );
        edges_.push_back( { { node->id(), std::make_pair( 1.0, action ) } } );
        CHECK_EQ( edges_.size() - 1, child->id(), "corruption in edge data structure" );

        // for each outcome
        auto outcomes = getPossibleOutcomes( node, action );
        for( const auto& outcome : outcomes )
        {
          CHECK( node->data().agentId == agentId, "Corruption in the queue!" );
          auto nextAgentId = ( agentId + 1 ) % engine_.agentNumber();

          // outcome.p
          // node after observation, will have to choose between several actions
          const auto& p    = std::get<0>(outcome);
          const auto& data = std::get<1>(outcome);
          const auto& observation = std::get<2>(outcome);
          const auto childChildData = GraphNodeDataType( data.states, data.beliefState, data.terminal, nextAgentId, NodeData::NodeType::ACTION );

          bool nodeNeedsToBeCreated = true;
          if( isGraph_ )
          {
            if( hash_to_id_.count( childChildData.hash() ) )
            {
              const auto& childChildId = hash_to_id_[ childChildData.hash() ].front();
              const auto& childChild = nodes_[ childChildId ];

              child->addExistingChild( childChild.lock() );
              edges_[ childChildId ][ child->id() ] = std::make_pair( p, observation );

              nodeNeedsToBeCreated = false;
            }
          }
          // tree case
          if( nodeNeedsToBeCreated )
          {
            const auto childChild = child->makeChild( childChildData );

            hash_to_id_[ childChild->data().hash() ].push_back( childChild->id() );
            nodes_.push_back( childChild );
            edges_.push_back( { { child->id(), std::make_pair( p, observation ) } } );
            CHECK_EQ( edges_.size() - 1, childChild->id(), "corruption in edge data structure" );

            if( data.terminal )
            {
              terminalNodes_.push_back( childChild );
            }
            else
            {
              nextQueue.push( childChild );
            }
          }
        }
      }
    }
  }

  return nextQueue;
}

///
static double transitionProbability( const std::vector< double >& parent, const std::vector< double >& child )
{
  CHECK( parent.size() == child.size(), "incompatible belief states" );

  double p = 0.0;

  double sum = 0.0;
  for( std::size_t w{0}; w < parent.size(); ++w )
  {
    p += parent[w] * child [w];
    sum += parent[w];
  }

  return p / sum;
}

double priorityV0(const GraphNode< NodeData >::ptr& node)
{
  const auto& node_data = node->data();

  std::size_t parent_n_rollouts{node_data.n_rollouts};
  if(node->parent())
  {
    CHECK( node->parent() != nullptr, "node shouldn't be a root node!" );
    CHECK( node->parent()->data().nodeType == NodeData::NodeType::OBSERVATION, "wrong node type!" );
    CHECK( node->parent()->parent() != nullptr, "node shouldn't be a root node!" );
    CHECK( node->parent()->parent()->data().nodeType == NodeData::NodeType::ACTION, "wrong node type!" );

    const auto& observation_parent = node->parent();
    const auto& action_parent = node->parent()->parent();
    const auto& parent_node_data = action_parent->data();

    parent_n_rollouts = parent_node_data.n_rollouts;
  }

  const double priority = node_data.expectedRewardToGoal + 1.0 * sqrt( log( ( parent_n_rollouts ) / ( node_data.n_rollouts + 1 ) ) ); // TODO: Use constant for explo

  std::cout << "[priority] of " << node->id() << " = " << priority << std::endl;

  return priority;
}


//
double priority( const GraphNode< NodeData >::ptr& node, const double c )
{
  const auto& node_data = node->data();

  std::size_t parent_n_rollouts{node_data.n_rollouts};
  if(node->parent())
  {
    CHECK( node->parent() != nullptr, "node shouldn't be a root node!" );
    CHECK( node->parent()->data().nodeType == NodeData::NodeType::ACTION, "wrong node type!" );

    const auto& action_parent = node->parent();
    const auto& parent_node_data = action_parent->data();

    parent_n_rollouts = parent_node_data.n_rollouts;
  }

  if( node_data.n_rollouts == 0 )
  {
    std::cout << "[priority] of " << node->id() << " = " << std::numeric_limits<double>::infinity() << std::endl;

    return std::numeric_limits<double>::infinity();
  }

  const double priority = node_data.expectedRewardToGoal + c * sqrt( log( ( parent_n_rollouts ) / ( node_data.n_rollouts ) ) ); // TODO: Use constant for explo

  std::cout << "[priority] of " << node->id() << " = " << priority << std::endl;

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

GraphNode< NodeData >::ptr getMostPromisingChild( const GraphNode< NodeData >::ptr& node, const double c )
{
  double max_prio{std::numeric_limits<double>::lowest()};
  GraphNode< NodeData >::ptr best_uct_child{};
  for( const auto& child : node->children() )
  {
    const auto prio = priority( child, c );

    if( prio > max_prio )
    {
      max_prio = prio;
      best_uct_child = child;
    }
  }

  return best_uct_child;
}

void DecisionGraph::expandMCTS( const double r0, const std::size_t n_iter_min, const std::size_t n_iter_max, const std::size_t rolloutMaxSteps, const double c )
{
  // reference https://papers.nips.cc/paper_files/paper/2010/file/edfbe1afcf9246bb0d40eb4d8027d90f-Paper.pdf Alg 1.
  CHECK( engine_.agentNumber() == 1, "MCTS works here only for one agent" );

  std::size_t n_iter{0};

  std::unordered_set< uint > expandedNodesIds;

  while( ( root_->data().expectedRewardToGoal <= std::numeric_limits<double>::lowest() || n_iter < n_iter_min ) && n_iter < n_iter_max )
  {
    std::cout << "\n-------" << n_iter << "-------" << std::endl;

    // sample state
    const auto stateIndex = sampleStateIndex( root_->data().beliefState );

    simulate( root_, stateIndex, 0, r0, rolloutMaxSteps, c, expandedNodesIds );

    const auto max_reward_child_it = std::max_element( root_->children().cbegin(), root_->children().cend(), []( const auto& lhs, const auto& rhs ) { return lhs->data().expectedRewardToGoal < rhs->data().expectedRewardToGoal; });
    root_->data().expectedRewardToGoal = (*max_reward_child_it)->data().expectedRewardToGoal;

    // save current MCTS tree
    //std::stringstream ss;
    //ss << "decision_graph_" << n_it << ".gv";
    //saveMCTSTreeToFile( ss.str(), getNotObservableFact( root_->data().states[stateIndex] ) );

    n_iter++;
  }

  std::stringstream ss;
  ss << "decision_graph_final" << ".gv";
  saveMCTSTreeToFile( ss.str(), "" );
}

double DecisionGraph::simulate( const GraphNodeType::ptr& node,
                                const std::size_t stateIndex,
                                const std::size_t depth,
                                const double r0,
                                const std::size_t rolloutMaxSteps,
                                const double c,
                                std::unordered_set< uint > & expandedNodesIds )
{
  std::cout << "[simulate] Simulate from node: " << node->id() << std::endl;

  const auto node_id_it = expandedNodesIds.find( node->id() );

  if( node->data().terminal )
  {
    std::cout << "[simulate] Already terminal! no costs" << std::endl;

    return 0.0;
  }

  if( node_id_it == expandedNodesIds.end() )
  {
    std::cout << "[simulate] not expanded, so expand.." << std::endl;

    const auto& states = node->data().states;
    const auto& beliefState = node->data().beliefState;

    // expand node (using bs and not sampled state)
    const auto& actions = getCommonPossibleActions( states, beliefState, 0 );

    for( const auto& action : actions )
    {
      // node after action, will receive one or several observations
      auto child = node->makeChild( GraphNodeDataType( states, beliefState, false, 0, NodeData::NodeType::OBSERVATION ) );
      child->data().leadingAction = action;
      nodes_.push_back( child );
      edges_.push_back( { { node->id(), std::make_pair( 1.0, action ) } } );
      CHECK_EQ( edges_.size() - 1, child->id(), "corruption in edge data structure" );

      std::cout << "[simulate] created " << child->id() << std::endl;
    }

    std::vector<double> rollOutBeliefState;
    rollOutBeliefState.resize( beliefState.size() );
    rollOutBeliefState[ stateIndex ] = 1.0;

    expandedNodesIds.insert( node->id() );

    std::cout << "[simulate] compute rollout from " << node->id() << std::endl;

    return rollOut( states, rollOutBeliefState, 0, rolloutMaxSteps );
  }

  if( node->children().empty() )
  {
    return 0.0;
  }

  // Choose best UCT action based on UCB
  const auto best_uct_child = getMostPromisingChild( node, c );

  std::cout << "[simulate] node " << best_uct_child->id() << " is most promising so far.." << std::endl;

  if( expandedNodesIds.find( best_uct_child->id() ) == expandedNodesIds.end() )
  {
    std::cout << "[simulate] not expanded, so expand based on possible observations.." << std::endl;

    CHECK( best_uct_child->children().empty(), "A non-expanded node should not have children!" );
    // expand observations - we expand just once
    // check outcomes after taking best action
    auto outcomes = getPossibleOutcomes( node, best_uct_child->data().leadingAction );

    for(const auto& outcome: outcomes)
    {
      const auto& p    = std::get<0>(outcome);
      const auto& data = std::get<1>(outcome);
      const auto& observation = std::get<2>(outcome);
      const auto childChildData = GraphNodeDataType( data.states, data.beliefState, data.terminal, 0, NodeData::NodeType::ACTION );

      const auto childChild = best_uct_child->makeChild( childChildData );

      hash_to_id_[ childChild->data().hash() ].push_back( childChild->id() );
      nodes_.push_back( childChild );
      edges_.push_back( { { best_uct_child->id(), std::make_pair( p, observation ) } } );
      CHECK_EQ( edges_.size() - 1, childChild->id(), "corruption in edge data structure" );

      std::cout << "[simulate] create " << childChild->id() << " (observation: " << observation << ")" << std::endl;

      if( data.terminal )
      {
        terminalNodes_.push_back( childChild );
      }
    }

    expandedNodesIds.insert( best_uct_child->id() );
  }

  CHECK( !best_uct_child->children().empty(), "Should at least have one outcome!" );

  // Get observation based on sampled state! in our logic, there should be one possible action only!
  std::vector< std::pair< GraphNodeType::ptr, double > > candidates; // node after observation and probability of bein in state indicated by stateIndex
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

  std::cout << "[simulation] based on sample world(" << stateIndex << "), the corresponding child is:" << action_node_after_observation->id() << std::endl;

  double reward = r0 + simulate( action_node_after_observation, stateIndex, depth + 1, r0, rolloutMaxSteps, c, expandedNodesIds );

  // increments mc counters
  node->data().n_rollouts++;
  best_uct_child->data().n_rollouts++;

  if(best_uct_child->data().n_rollouts == 1)
  {
    best_uct_child->data().expectedRewardToGoal = reward;
  }
  else
  {
    best_uct_child->data().expectedRewardToGoal = best_uct_child->data().expectedRewardToGoal + (reward - best_uct_child->data().expectedRewardToGoal) / best_uct_child->data().n_rollouts;
  }

  std::cout << "[simulation] simulation counter of " << node->id() << ": " << node->data().n_rollouts << std::endl;
  std::cout << "[simulation] simulation counter of " << best_uct_child->id() << ": " << best_uct_child->data().n_rollouts << std::endl;
  std::cout << "[simulation] reward of " << best_uct_child->id() << ": " << best_uct_child->data().expectedRewardToGoal << std::endl;

  return reward;
}

void DecisionGraph::saveMCTSTreeToFile( const std::string & filename, const std::string & mctsState ) const
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
//void DecisionGraph::expandMCTSv0()
//{
//  std::unordered_set< GraphNode< NodeData >::ptr > nodes;
//  std::unordered_set< uint > expandedNodesIds;
//  nodes.insert( root_ );

//  const std::size_t n_it_min{4};
//  std::size_t n_it{0};
//  while( root_->data().expectedRewardToGoal <= std::numeric_limits<double>::lowest() || n_it < n_it_min )
//  {
//    std::cout << "iteration " << n_it << std::endl;

//    const auto& node = *std::max_element( nodes.begin(), nodes.end(), [](const auto& lhs, const auto& rhs) { return priority(lhs) < priority(rhs);} ); // TODO: chnge by traversal from root? uing a chosen with UCT

//    // expand node if it is on the fringe
//    const auto node_it = expandedNodesIds.find(node->id());

//    if( node_it == expandedNodesIds.cend() )
//    {
//      std::cout << "expand " << node->id() <<  std::endl;

//      auto extension = expand( node );
//      expandedNodesIds.insert( node->id() );

//      std::cout << extension.size() << " children!" << std::endl;

//      while(!extension.empty())
//      {
//        // alternative -> perform rollouts on only one child?
//        auto child = extension.front();

//        std::cout << "simulate rollout from " << child->id() <<  std::endl;

//        const auto simulated_reward = rollOut( child->data().states, child->data().beliefState, 10 ); // TODO, config

//        backtrack( child, simulated_reward );

//        std::cout << child->id() << " simulated reward:" << simulated_reward << " n rollouts:" << child->data().n_rollouts << std::endl;

//        nodes.insert( child );
//        extension.pop();
//      }
//    }
//    else
//    {
//      // additional rollouts on existing node

//      std::cout << "additional rollouts on: " << node->id() << std::endl;

//      NIY
//    }

//    n_it++;
//  }
//}

double DecisionGraph::rollOut( const std::vector< std::string > & states, const std::vector< double >& bs, const std::size_t steps, const std::size_t rolloutMaxSteps )
{
  std::stringstream ss;
  for(auto i = 0; i < steps; ++i)
  {
    ss << "-";
  }
  ss << " ";
  const std::string prefix = ss.str();

  CHECK( engine_.agentNumber() == 1, "not supported for multi agent");

  if( steps == rolloutMaxSteps )
  {
    return 0.0; // TODO, find some heuristic here?
  }

  const auto& actions = getCommonPossibleActions( states, bs, 0 );

  if( actions.empty() )
  {
    return 0.0;
  }

  // choose action
  const auto action_index = rand() %  actions.size();
  const auto& action = actions[action_index];

  std::cerr << "  [rollOut]" << prefix << actions.size() << " possible actions, choosing: " << action_index << " : " << action << std::endl;

  const auto outcomes = getPossibleOutcomes( states, bs, action, 0 );

  double simulatedReward{ -1.0 }; // TODO: put R0?

  std::cout << "  [rollOut]" << prefix << "outcomes.size(): " << outcomes.size() << std::endl;

  for(const auto &outcome: outcomes)
  {
    const auto p = std::get<0>(outcome);
    const auto& nodeData = std::get<1>(outcome);

    if(!nodeData.terminal)
    {
      simulatedReward += p * rollOut(nodeData.states, nodeData.beliefState, steps + 1, rolloutMaxSteps);
    }
    else
    {
      std::cout << "  [rollOut] reached a TERMINAL STATE!" << std::endl;
    }
  }

  std::cout << "  [rollOut]" << prefix << "simulatedReward: " << simulatedReward << std::endl;

  return simulatedReward;
}

//void DecisionGraph::backtrack( const GraphNode< NodeData >::ptr& node, const double expectedReward )
//{
//  auto& node_data = node->data();
//  //node_data.expectedRewardToGoal = ( node_data.expectedRewardToGoal * node_data.n_rollouts + expectedReward ) / ( node_data.n_rollouts + 1 );
//  node_data.expectedRewardToGoal = std::max( node_data.expectedRewardToGoal,  expectedReward );

//  std::cout << "[backtrack] " << node->id() << " reward :  " << node_data.expectedRewardToGoal << std::endl;

//  if(!node->parent())
//  {
//    node_data.n_rollouts++;

//    std::cout << "[backtrack] " << node->id() << " has now : " << node_data.n_rollouts << " rollouts" << std::endl;

//    return;
//  }

//  CHECK( node->parent() != nullptr, "node shouldn't be a root node!" );
//  CHECK( node->parent()->data().nodeType == NodeData::NodeType::OBSERVATION, "wrong node type!" );
//  CHECK( node->parent()->parent() != nullptr, "node shouldn't be a root node!" );
//  CHECK( node->parent()->parent()->data().nodeType == NodeData::NodeType::ACTION, "wrong node type!" );

//  const auto& observation_parent = node->parent();
//  const auto& action_parent = node->parent()->parent();
//  const auto& parent_node_data = action_parent->data();

//  //queue.erase( node );
//  //node_data.priority = node_data.expectedRewardToGoal + 1.0 * sqrt( log( ( parent_node_data.n_rollouts + 1 ) / ( node_data.n_rollouts + 1 ) ) ); // TODO: Use constant for explo
//  //std::cout << "[backtrack] " << node->id() << " priotrity : " << node_data.priority << std::endl;

//  //queue.insert( node );

//  node_data.n_rollouts++;

//  std::cout << "[backtrack] " << node->id() << " has now : " << node_data.n_rollouts << " rollouts" << std::endl;

//  double parentExpectedReward = -1.0;
//  for(const auto& action_sibling: observation_parent->children() )
//  {
//    const double p = transitionProbability( parent_node_data.beliefState, action_sibling->data().beliefState );
//    parentExpectedReward += p * action_sibling->data().expectedRewardToGoal;
//  }

//  backtrack( action_parent, parentExpectedReward );
//}

///

void DecisionGraph::_addNode( const std::weak_ptr< GraphNodeType > & _node )
{
  nodes_.push_back( _node );

  auto node = _node.lock();
  if( node->data().nodeType ==  DecisionGraph::GraphNodeDataType::NodeType::ACTION )
  {
    hash_to_id_[ node->data().hash() ].push_back( node->id() );

    if( node->data().terminal )
    {
      terminalNodes_.push_back( node );
    }
  }
}

void DecisionGraph::_addEdge( uint child, uint parent, double p, const std::string & artifact )
{
  if( child == edges_.size() )
  {
    edges_.push_back( { { parent, std::make_pair( p, artifact ) } } );
    CHECK_EQ( edges_.size()-1, child, "edges should be added in the right order!!" );
  }
  else
  {
    edges_[ child ][ parent ] = std::make_pair( p, artifact );
  }
}

void DecisionGraph::removeNode( const std::weak_ptr< GraphNodeType > & _node )
{
  auto node = _node.lock();

  // check in terminals
  for( auto tIt = terminalNodes_.begin(); tIt != terminalNodes_.end(); ++tIt )
  {
    if( tIt->lock()->id() == node->id() )
    {
      terminalNodes_.erase( tIt );
    }
  }

  // check in nodes
  for( auto tIt = nodes_.begin(); tIt != nodes_.end(); ++tIt )
  {
    if( tIt->lock()->id() == node->id() )
    {
      nodes_.erase( tIt );
    }
  }

  // inform parents
  for( auto _p : node->parents() )
  {
    auto p = _p.lock();
    if( p )
    {
      p->removeChild( node );
      nodes_[ node->id() ].reset();
    }
  }
}

void DecisionGraph::purgeNodes( const std::vector< bool > & toKeep ) // remove nodes from nodes_ and terminalNodes_ that are not valid anymore
{
  auto to_keep_lambda = [&] ( const std::weak_ptr< GraphNodeType > & node ) -> bool
  {
    return ! toKeep[ node.lock()->id() ];
  };
  nodes_.erase( std::remove_if(nodes_.begin(), nodes_.end(), to_keep_lambda), nodes_.end() );
  terminalNodes_.erase( std::remove_if(terminalNodes_.begin(), terminalNodes_.end(), to_keep_lambda), terminalNodes_.end() );
}

void DecisionGraph::saveGraphToFile( const std::string & filename ) const
{
  if( ! root_ )
  {
    return;
  }

  std::ofstream file;
  file.open( filename );

  GraphPrinter printer( file );
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

std::vector< std::string > DecisionGraph::getCommonPossibleActions( const GraphNode< NodeData >::ptr & node, uint agentId ) const
{
  const auto& states = node->data().states;
  const auto& bs     = node->data().beliefState;

  return getCommonPossibleActions( states, bs, agentId );
}

std::vector< std::string > DecisionGraph::getCommonPossibleActions( const std::vector< std::string > & states, const std::vector< double >& bs, uint agentId ) const
{
  LogicEngine & engine = engine_;

  /// get possible actions
  std::vector< std::string > possibleActions;
  for( auto w = 0; w < bs.size(); ++w )
  {
    if( bs[ w ] > 0 )
    {
      auto startState = states[ w ];
      engine.setState( startState );

      auto newActions = engine.getPossibleActions( agentId );

      if( possibleActions.empty() )
      {
        possibleActions = newActions;
      }
      else
      {
        std::vector< std::string > newPossibleActions;

        // each action in the new possible actions is has to be previously in the previous possible actions
        std::set_intersection( possibleActions.begin(), possibleActions.end(),
                               newActions.begin(), newActions.end(),
                               std::back_inserter( newPossibleActions ) );

        possibleActions = newPossibleActions;
      }
    }
  }

  return possibleActions;
}

std::vector< std::tuple< double, NodeData, std::string > > DecisionGraph::getPossibleOutcomes( const GraphNode< NodeData >::ptr & node, const std::string & action ) const
{
  const auto& bs     = node->data().beliefState;
  const auto& states = node->data().states;

  return getPossibleOutcomes( states, bs, action, node->data().agentId );
}

std::vector< std::tuple< double, NodeData, std::string > > DecisionGraph::getPossibleOutcomes( const std::vector< std::string > & states, const std::vector< double >& bs, const std::string & action, const uint agentId ) const
{
  std::vector< std::tuple< double, NodeData, std::string > > outcomes;

  LogicEngine & engine = engine_; // copy to be const

  //        observable facts                                    world state
  std::map< std::set< std::string >, std::vector< std::pair< uint, std::string > > > observableStatesToStates;
  std::map< std::set< std::string >, bool > terminalOutcome;
  std::set< std::string > factIntersection;

  for( auto w = 0; w < bs.size(); ++w )
  {
    if( bs[ w ] > 0 )
    {
      const auto& startState = states[ w ];
      engine.setState( startState );

      //std::cout << "start state:" << startState << std::endl; // tmp camille
      //std::cout << "action:" << action << std::endl; // tmp camille

      engine.transition( action );

      const auto _result          = engine.getState();
      const auto facts            = getFilteredFacts( _result );// without komo and action tags
      const auto result           = concatenateFacts( facts );
      const auto observableFacts  = getObservableFacts( facts );
      const auto terminal         = engine.isTerminal();

      //std::cout << "result:" << result << std::endl; // tmp camille

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
      factIntersection = newIntersection;
      observableStatesToStates[ observableFacts ].push_back( std::make_pair( w, result ) );
      terminalOutcome         [ observableFacts ] = terminal;
    }
  }

  //
  for( auto observableResultPair : observableStatesToStates )
  {
    std::vector< std::string > states( bs.size(), "" );
    std::vector< double > newBs( bs.size(), 0 );

    const auto & worldToOutcomes = observableResultPair.second;
    auto observationFacts = getEmergingFacts( factIntersection, observableResultPair.first );
    auto observation      = concatenateFacts( observationFacts );
    auto terminal         = terminalOutcome[ observableResultPair.first ];

    double p = 0;
    for( const auto & worldOutcome :  worldToOutcomes )
    {
      auto w = worldOutcome.first;
      auto state = worldOutcome.second;

      p += bs[ w ];
      states[ w ] = state;
      newBs[ w ] = bs[ w ];
    }

    newBs = normalizeBs( newBs );

    outcomes.push_back( std::make_tuple( p, NodeData( states, newBs, terminal, agentId, NodeData::NodeType::OBSERVATION ), observation ) );
  }

  return outcomes;
}

void DecisionGraph::copy( const DecisionGraph & graph )
{
  reset();

  if( graph.root() )
  {
    engine_ = graph.engine_;
    isGraph_ = graph.isGraph_;
    edges_ = graph.edges_;

    auto rootData = graph.root()->data();

    root_ = GraphNodeType::root( rootData );
    hash_to_id_[ root_->data().hash() ].push_back( 0 );
    nodes_.push_back( root_ );

    std::queue< std::pair < GraphNodeType::ptr, GraphNodeType::ptr > > Q;

    Q.push( std::make_pair( graph.root(), root_ ) ); // original - copy

    while( ! Q.empty() )
    {
      auto u = Q.front();
      Q.pop();

      auto uOriginal = u.first;
      auto uCopy     = u.second;

      CHECK( uCopy->data().nodeType == GraphNodeDataType::NodeType::ACTION, "wrong node type" );

      for( auto vOriginal : uOriginal->children() )
      {        
        auto vCopy = uCopy->makeChild( vOriginal->data() );
        vCopy->setId( vOriginal->id() );
        nodes_.push_back( vCopy );

        //std::cout << "copy " << vCopy->id() << std::endl;

        CHECK( ! vCopy->data().terminal, "termination can appear only after the observation!" );
        CHECK( vCopy->data().nodeType == GraphNodeDataType::NodeType::OBSERVATION, "wrong node type" );

        for( auto wOriginal : vOriginal->children() )
        {
          CHECK( wOriginal->data().nodeType == GraphNodeDataType::NodeType::ACTION, "wrong node type" );

          if( isGraph_ && hash_to_id_.count( wOriginal->data().hash() ) != 0 )
          {
            //std::cout << "rewire to " << wOriginal->id() << std::endl;

            CHECK( hash_to_id_.at( wOriginal->data().hash() ).size() == 1, "datastructure corruption" );
            CHECK( hash_to_id_.at( wOriginal->data().hash() ).front() == wOriginal->id(), "datastructure corruption" );
            auto wCopy = nodes_[ wOriginal->id() ].lock();
            vCopy->addExistingChild( wCopy );
          }
          else
          {
            auto wCopy = vCopy->makeChild( wOriginal->data() );
            wCopy->setId( wOriginal->id() );
            nodes_.push_back( wCopy );
            hash_to_id_[ wCopy->data().hash() ].push_back( wCopy->id() );

            //std::cout << "copy " << wCopy->id() << std::endl;

            if( wCopy->data().terminal )
            {
              terminalNodes_.push_back( wCopy );
            }
            else
            {
              Q.push( std::make_pair( wOriginal, wCopy ) );
            }
          }
        }
      }
    }
  }
}

} // namespace matp
