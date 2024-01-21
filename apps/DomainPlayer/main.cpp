#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>
#include <logic_parser.h>
#include <unordered_map>

//===========================================================================

void TEST(FOL_World){
  FOL_World world("boxes_new.kvg");

  auto actions = world.get_actions();
  for(auto& a:actions){ cout <<"DECISION: " <<*a <<endl; }

  for(uint k=0;k<10;k++){
    auto res=world.transition_randomly();
    cout <<"RND TRANSITION: obs=" <<*res.observation <<" r=" <<res.reward <<endl;
  }

  world.get_actions();

  world.make_current_state_new_start();

  world.reset_state();
  world.get_actions();

}

//===========================================================================

void TEST(PlayFOL_World){
  const char *file = "../lgp-learn-self-driving/LGP-merging-1w.g";
  if(rai::argc>1) file = rai::argv[1];

  FOL_World world(file);
  world.verbose = rai::getParameter<int>("verbose", 2);

  for(bool go=true;go;){
    bool terminal = world.is_terminal_state();
    auto actions = world.get_actions();
    cout <<"********************" <<endl;
    cout <<"STATE: ";
    world.get_info(MCTS_Environment::writeState);
    cout <<"\nCHOICES:" <<endl;
    cout <<"(q) quit" <<endl;
    cout <<"(r) reset_state" <<endl;
    cout <<"(m) make_current_initial" <<endl;
    uint c=0;
    if(!terminal) for(auto& a:actions){ cout <<"(" <<c++ <<") DECISION: " <<*a <<endl; }

    char cmd='1';
    std::cin >>cmd;
    cout <<"COMMAND: '" <<cmd <<"'" <<endl;

    if(!terminal && cmd>='0' && cmd<='9'){
      auto &a = actions[int(cmd-'0')];
      cout <<"executing decision " <<*a <<endl;
      auto res=world.transition(a);
      cout <<"->  result: obs=" <<*res.observation <<" reward=" <<res.reward <<endl;
    }else switch(cmd){
      case 'q': go=false; break;
      case 'r': world.reset_state(); break;
      case 'm': world.make_current_state_new_start(); break;
      default: LOG(-1) <<"command '" <<c <<"' not known";
    }
  }
}

void testWithLogicParserManual()
{
  matp::LogicParser w;

  w.parse( "LGP-3-blocks-1-side-fol.g" );

  std::cout << "Possible start states:" << std::endl;

  for(auto i = 0; i < w.possibleStartStates().size(); ++i )
  {
    const auto& state = w.possibleStartStates()[i];
    std::cout << i << "." << state << std::endl << std::endl;
  }

  std::cout << "Choose a start state (between [0 and " << w.possibleStartStates().size() << "[)" << std::endl;

  std::size_t startStateIndex{};
  std::cin >> startStateIndex;

  const auto& startState = w.possibleStartStates()[startStateIndex];

  std::cout << "Start state:" << std::endl<< startState << std::endl;

  auto& engine = w.engine();

  engine.setState(startState);

  while(true)
  {
    const auto actions = engine.getPossibleActions(0);

    std::cout << std::endl << "Possible actions:" << std::endl;
    for(auto i = 0; i < actions.size(); ++i )
    {
      std::cout << i << "." << actions[i] << std::endl;
    }

    std::cout << "Choose an action (between [0 and " << actions.size() << "[)" << std::endl;

    std::size_t actionIndex{};
    std::cin >> actionIndex;
    const auto& action = actions[actionIndex];

    engine.transition(action);

    const auto state = engine.getState();
    const auto facts = matp::getFilteredFacts(state);

    std::cout << "State:" << std::endl;

    std::size_t hash{0};
    for(const auto& fact: facts)
    {
      std::cout << fact << std::endl;
      hash += std::hash<std::string>()(fact);
    }
    std::cout << hash << std::endl;
  }
}

void testWithLogicParserRandomized(const std::size_t n)
{
  //
  struct StateHashActionHasher
  {
    std::size_t operator()(const std::pair<std::size_t, std::size_t> & state_action_pair) const
    {
      return state_action_pair.first << state_action_pair.second;
    }
  };

  std::unordered_map< std::size_t, std::string > states; // hash -> state
  std::unordered_map< std::pair< std::size_t, std::size_t >, std::size_t, StateHashActionHasher > stateActionToSuccessorState;// (hash, action_index) -> hash
  std::unordered_map< std::size_t, std::vector< std::string > > stateToActions;
  std::unordered_map< std::size_t, bool > terminal; // hash to terminal state

  const auto getHash = [](const std::string& state)
  {
    const auto facts = matp::getFilteredFacts(state);

    std::size_t hash{0};
    for(const auto& fact: facts)
    {
      hash += std::hash<std::string>()(fact);
    }

    return hash;
  };

  //

  matp::LogicParser w;

  w.parse( "LGP-3-blocks-1-side-fol.g" );

  std::cout << "Possible start states:" << std::endl;

  for(auto i = 0; i < w.possibleStartStates().size(); ++i )
  {
    const auto& state = w.possibleStartStates()[i];
    std::cout << i << "." << state << std::endl << std::endl;
  }

  std::cout << "Choose a start state (between [0 and " << w.possibleStartStates().size() << "[)" << std::endl;

  std::size_t startStateIndex{0};

  const auto& startState = w.possibleStartStates()[startStateIndex];

  std::cout << "Start state:" << std::endl<< startState << std::endl;

  std::size_t current_h = getHash(startState);
  states[current_h] = startState;
  terminal[current_h] = false;

  const auto start_h = current_h;

  auto& engine = w.engine();

  static constexpr bool useCaching{true};
  for(auto i{0}; i < n; ++i)
  {
    const auto cacheAction_it = stateToActions.find(current_h);
    if( (cacheAction_it == stateToActions.end() ) || (!useCaching) )
    {
      engine.setState(states.at(current_h));
      auto actions = engine.getPossibleActions(0);
      stateToActions[current_h] = actions;
    }

    // draw action
    const std::size_t actionIndex = rand() % stateToActions.at(current_h).size();
    //

    std::size_t next_h{0};
    const auto stateActionKey = std::make_pair( current_h, actionIndex );
    const auto cache_it = stateActionToSuccessorState.find( stateActionKey );
    if( ( cache_it != stateActionToSuccessorState.end() ) && useCaching )
    {
      next_h = cache_it->second;
    }
    else
    {
      // load state
      const auto& state = states.at(current_h);

      // load action
      const auto& action = stateToActions.at(current_h).at(actionIndex);

      if( cacheAction_it != stateToActions.end() )
      {
        engine.setState(state); // if the action has been retrieved from emgine, we don't need to set the state again
      }
      engine.transition(action);

      const auto nextState = engine.getState();
      next_h = getHash(nextState);

      // add to cache
      if( states.find(next_h) == states.end() )
      {
        states[ next_h ] = nextState;
      }

      stateActionToSuccessorState[ stateActionKey ] = next_h;

      terminal[ next_h ] = engine.isTerminal();
    }

    if( terminal[ next_h ] )
    {
      next_h = start_h;
    }

    current_h = next_h;
  }

  std::cout << "Number of states: " << states.size() << std::endl;
}

//===========================================================================

int main(int argn, char** argv){
  rai::initCmdLine(argn, argv);
  rnd.clockSeed();

//  testMCTS();

//  testPlayFOL_World();

//  testWithLogicParserManual();

  testWithLogicParserRandomized(500000);
}
