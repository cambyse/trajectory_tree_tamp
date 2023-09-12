#include <functional>
#include <list>
#include <chrono>

#include <boost/filesystem.hpp>
#include <Kin/kinViewer.h>

#include <graph_planner.h>

#include <komo_planner.h>
#include <approx_shape_to_sphere.h>
#include <observation_tasks.h>

#include "komo_tree_groundings.h"


//===========================================================================

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

static void savePolicyToFile( const Policy & policy, const std::string & suffix = "" )
{
  std::stringstream namess, skenamess;

  namess << "policy-" << policy.id() << suffix;
  policy.save( namess.str() );

  namess << ".gv";
  policy.saveToGraphFile( namess.str() );
}

//===========================================================================

void display_robot()
{
    rai::KinematicWorld kin;

    kin.init( "LGP-1-block-6-sides-kin.g" );
    //kin.setJointState({0.0, 3.0, 0.0, 0.65, 0.0, -1.0, -0.78});

    std::cout << "q:" << kin.q << std::endl;

//    kin.init( "LGP-blocks-kin-unified-b6.g" );

////    const double zf = 1.47;
////    const double s = 0.55;
////    kin.gl().camera.setPosition(s * 10., s * 4.5, zf + s * ( 3.5 - zf ));

//    const double zf = 1.0;
//    const double s = 0.35;
//    kin.gl().camera.setPosition(s * 10., s * 0, zf + s * ( 1.5 - zf ));

//    kin.gl().camera.focus(0.5, 0, zf);
//    kin.gl().camera.upright();

    kin.watch(100);
//    kin.write( std::cout );

//    rai::wait( 300, true );
}

void plan()
{
  matp::GraphPlanner tp;
  mp::KOMOPlanner mp;

  // set planning parameters
  tp.setR0( -0.00001 ); //-0.25//-0.1//-0.015 ); for blocks one side
  tp.setMaxDepth( 10 );
  mp.setNSteps( 20 );
  mp.setMinMarkovianCost( 0.00 );

  // set problem
  //tp.setFol( "LGP-1-block-1-side-fol.g" );
  //mp.setKin( "LGP-1-block-1-side-kin.g" );

  //tp.setFol( "LGP-2-blocks-1-side-fol.g" );
  //tp.setFol( "LGP-2-blocks-1-side-kin.g" );

  tp.setFol( "LGP-1-block-6-sides-fol.g" );
  mp.setKin( "LGP-1-block-6-sides-kin.g" );

  // register symbols
  mp.registerInit( groundTreeInit );
  mp.registerTask( "pick-up"      , groundTreePickUp );
  mp.registerTask( "put-down"     , groundTreePutDown );
  mp.registerTask( "check"        , groundTreeCheck );
  mp.registerTask( "stack"        , groundTreeStack );
  mp.registerTask( "unstack"      , groundTreeUnStack );

  /// BUILD DECISION GRAPH
  tp.buildGraph(true);
  //tp.saveGraphToFile( "decision_graph.gv" );
  //generatePngImage( "decision_graph.gv" );

#if 0
  /// POLICY SEARCH
  Policy policy, lastPolicy;

  tp.solve();
  policy = tp.getPolicy();

  uint nIt = 0;
  const uint maxIt = 1000;
  do
  {
    nIt++;

    savePolicyToFile( policy );

    lastPolicy = policy;

    /// MOTION PLANNING

    tp.solve();
    policy = tp.getPolicy();
  }
  while( lastPolicy != policy && nIt != maxIt );

  savePolicyToFile( policy, "-final" );
#else
  Policy policy;
  policy.load("policy-0"); //policy-check_pickup_stack_linear
#endif
  // default method
  //mp.display(policy, 200);

  /// JOINT OPTIMIZATION
  // single joint optimization
//  {
//    auto po     = MotionPlanningParameters( policy.id() );
//    po.setParam( "type", "jointSparse" );
//    mp.solveAndInform( po, policy ); // it displays
//  }

  /// JOINT OPTIMIZATION
  // adsm
  {
    auto po     = MotionPlanningParameters( policy.id() );
    po.setParam( "type", "ADMMCompressed" ); //ADMMSparse, ADMMCompressed
    po.setParam( "decompositionStrategy", "Identity" ); // SubTreesAfterFirstBranching, BranchGen, Identity
    po.setParam( "nJobs", "8" );
    mp.solveAndInform( po, policy ); // it displays
  }
}

//===========================================================================

int main(int argc,char **argv)
{
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  //display_robot();

  plan();

  return 0;
}
