#include <functional>
#include <list>
#include <chrono>

#include <boost/filesystem.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>

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
  ///
  std::ofstream candidate, results, best_policy;
  candidate.open( "policy-candidates.data" );
  results.open( "policy-results.data" );
  best_policy.open( "policy-best_policy.data" );
  double graph_building_s = 0;
  double task_planning_s = 0;
  double motion_planning_s = 0;
  double joint_motion_planning_s = 0;

  namespace ba = boost::accumulators;
  boost::accumulators::accumulator_set<double, ba::features< ba::tag::variance, ba::tag::mean, ba::tag::min, ba::tag::max > > acc_length;
  boost::accumulators::accumulator_set<double, ba::features< ba::tag::variance, ba::tag::mean, ba::tag::min, ba::tag::max > > acc_acc_cost;
  ///

  matp::GraphPlanner tp;
  mp::KOMOPlanner mp;

  // set planning parameters
  tp.setR0( -.25 ); //-0.25//-0.1//-0.015 ); for blocks one side
  tp.setMaxDepth( 10 );
  mp.setNSteps( 20 );
  mp.setMinMarkovianCost( 0.00 );
  mp.setMaxConstraint( 15.0 );
  mp.addCostIrrelevantTask( "SensorDistanceToObject" );

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
  mp.registerTask( "put-down-flipped", groundTreePutDownFlipped );
  mp.registerTask( "check"        , groundTreeCheck );
  mp.registerTask( "stack"        , groundTreeStack );
  mp.registerTask( "unstack"      , groundTreeUnStack );

  /// BUILD DECISION GRAPH
  {
    auto start = std::chrono::high_resolution_clock::now();
    /// GRAPH BUILDING
    tp.buildGraph(true);
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    graph_building_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
  }
  //tp.saveGraphToFile( "decision_graph.gv" );
  //generatePngImage( "decision_graph.gv" );

#if 1
  /// POLICY SEARCH
  Policy policy, lastPolicy;
  double best_value{std::numeric_limits<double>::lowest()};
  {
    auto start = std::chrono::high_resolution_clock::now();
    tp.solve();
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    task_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
  }

  policy = tp.getPolicy();

  uint nIt = 0;
  const uint maxIt = 1000;
  do
  {
    nIt++;

    ///
    savePolicyToFile( policy );
    candidate << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;
    ///

    lastPolicy = policy;

    {
      /// MOTION PLANNING
      auto start = std::chrono::high_resolution_clock::now();
      auto po     = MotionPlanningParameters( policy.id() );
      po.setParam( "type", "markovJointPath" );
      mp.solveAndInform( po, policy );
      auto elapsed = std::chrono::high_resolution_clock::now() - start;
      motion_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;

      ///
      best_value = std::max(policy.value(), best_value);
      results << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;
      best_policy << policy.id() << "," << best_value << std::endl;
      ///
    }

    {
      /// TASK PLANNING
      auto start = std::chrono::high_resolution_clock::now();
      tp.integrate( policy );
      tp.solve();
      auto elapsed = std::chrono::high_resolution_clock::now() - start;
      task_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
    }
    policy = tp.getPolicy();
  }
  while( lastPolicy != policy && nIt != maxIt );

  savePolicyToFile( policy, "-final" );
#else
  Policy policy;
  policy.load("policy-0");
#endif
  ///
  savePolicyToFile( policy, "-final" );
  candidate << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;
  results << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;

  candidate.close();
  results.close();
  ///

  // default method
//  mp.display(policy, 200);

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
    po.setParam( "decompositionStrategy", "SubTreesAfterFirstBranching" ); // SubTreesAfterFirstBranching, BranchGen, Identity
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
