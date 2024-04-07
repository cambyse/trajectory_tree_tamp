#include <object_manipulation_tamp_controller.h>
#include <chrono>

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

  namess << "results/policy-" << policy.id() << suffix;
  policy.save( namess.str() );

  namess << ".gv";
  policy.saveToGraphFile( namess.str() );
}

Policy ObjectManipulationTAMPController::plan( const TAMPlanningConfiguration & config )
{
  /////////////////////
  /// POLICY SEARCH ///
  /////////////////////
  Policy policy, lastPolicy;

  /// TASK PLANNING
  { // build decision tree and get first policy
    const auto start = std::chrono::high_resolution_clock::now();

    tp_.solve();
    policy = tp_.getPolicy();

    const auto elapsed = std::chrono::high_resolution_clock::now() - start;
    decision_tree_building_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
    candidate << policy.id() << "," << std::min( 100.0, -policy.value() ) << std::endl;
 }

  uint nIt = 0;
  const uint maxIt = 1000;
  do
  {
    nIt++;

    savePolicyToFile( policy, "-candidate" );

    lastPolicy = policy;

    /// MOTION PLANNING
    {
      const auto start = std::chrono::high_resolution_clock::now();

      auto po     = MotionPlanningParameters( policy.id() );
      po.setParam( "type", "markovJointPath" );
      mp_.solveAndInform( po, policy );

      const auto elapsed = std::chrono::high_resolution_clock::now() - start;
      motion_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
      results << policy.id() << "," << std::min( 100.0, -policy.value() ) << std::endl;
    }

    savePolicyToFile( policy, "-informed" );

    /// TASK PLANNING
    {
      const auto start = std::chrono::high_resolution_clock::now();

      tp_.integrate( policy );
      tp_.solve();

      const auto elapsed = std::chrono::high_resolution_clock::now() - start;
      task_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
      policy = tp_.getPolicy();
      candidate << policy.id() << "," << std::min( 100.0, -policy.value() ) << std::endl;
    }
  }
  while( lastPolicy != policy && nIt != maxIt );

  savePolicyToFile( policy, "-final" );

  { // evaluate and display markovian -> output in optimizationReportMarkovianPathTree.re
    auto po     = MotionPlanningParameters( policy.id() );
    po.setParam( "type", "EvaluateMarkovianCosts" );
    mp_.solveAndInform( po, policy, config.watchMarkovianOptimizationResults );
  }

  /////////////////////////
  /// POLICY REFINEMENT ///
  /////////////////////////

  /// DECOMPOSED SPARSE OPTIMIZATION
  // adsm
  {
    const auto start = std::chrono::high_resolution_clock::now();

    auto po     = MotionPlanningParameters( policy.id() );
    po.setParam( "type", "ADMMCompressed" ); //ADMMSparse, ADMMCompressed
    po.setParam( "decompositionStrategy", "Identity" ); // SubTreesAfterFirstBranching, BranchGen, Identity
    po.setParam( "nJobs", "8" );
    mp_.solveAndInform( po, policy, config.watchJointOptimizationResults );   // optimize and displays joint optimization -> output in optimizationReportAdmmCompressed.re

    const auto elapsed = std::chrono::high_resolution_clock::now() - start;
    joint_motion_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
  }

  //////////////////////////
  /// SAVE RESULTS       ///
  //////////////////////////
  timings << "decision_tree_building_s="<< decision_tree_building_s << std::endl;
  timings << "task_planning_s="<< task_planning_s << std::endl;
  timings << "motion_planning_s="<< motion_planning_s << std::endl;
  timings << "joint_motion_planning_s="<< joint_motion_planning_s << std::endl;
  timings << "total_s="<< decision_tree_building_s + task_planning_s + motion_planning_s + joint_motion_planning_s << std::endl;

  timings.close();

  return policy;
}
