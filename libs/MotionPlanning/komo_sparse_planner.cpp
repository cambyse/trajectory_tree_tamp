#include <komo_sparse_planner.h>
#include <tree_builder.h>
#include <subtree_generators.h>
#include <komo_wrapper.h>
#include <trajectory_tree_visualizer.h>
#include <komo_planner_utils.h>

#include <utils.h>
#include <decentralized_optimizer.h>

#include <Kin/kin.h>
#include <Kin/switch.h>
#include <Kin/TM_transition.h>
#include <Kin/TM_FlagConstraints.h>
#include <Kin/TM_FixSwitchedObjects.h>

#include <unordered_set>

using T = ConstrainedProblem;
using U = AverageUpdater;

namespace mp
{

static double TM_FixSwichedObjects_scale{3.0e1};


std::shared_ptr< ExtensibleKOMO > KOMOSparsePlanner::intializeKOMO( const TreeBuilder & tree, const std::shared_ptr< const rai::KinematicWorld > & startKinematic ) const
{
  auto komo = komoFactory_.createKomo();
  komo->setModel(*startKinematic);
  komo->sparseOptimization = true;

  CHECK(tree.d() == 0, "support for free prefix komo deactivated!");
  //komo->freePrefix = !(tree.d() == 0); // free prefix is used when decomposing the trajectory in several pieces and optimizing them independantly, this is NOT efficient

  const auto nPhases = tree.n_nodes() - 1;
  komo->setTiming(nPhases, config_.microSteps_, config_.secPerPhase_, 2);

  return komo;
}

std::vector<Vars> KOMOSparsePlanner::getSubProblems( const TreeBuilder & tree, Policy & policy ) const
{
  std::vector<Vars> allVars;
  allVars.reserve(tree.get_leaves().size());

  for(const auto& l: policy.sleaves())
  {
    auto vars0 = tree.get_vars({0, 1.0 * l->depth()}, l->id(), 0, config_.microSteps_);
    auto vars1 = tree.get_vars({0, 1.0 * l->depth()}, l->id(), 1, config_.microSteps_);
    auto vars2 = tree.get_vars({0, 1.0 * l->depth()}, l->id(), 2, config_.microSteps_);
    Vars branch{vars0, vars1, vars2, config_.microSteps_};
    allVars.push_back(branch);
  }

  return allVars;
}

std::vector<intA> KOMOSparsePlanner::getSubProblemMasks( const std::vector<Vars> & allVars, uint T ) const
{
  std::vector<intA> masks(allVars.size());
  for(auto w = 0; w < allVars.size(); ++w)
  {
    auto & mask = masks[w];
    auto & vars = allVars[w][0];

    mask = intA(T);

    for(auto i: vars)
    {
      mask(i) = 1;
    }
  }

  return masks;
}

void KOMOSparsePlanner::groundPolicyActionsJoint( const TreeBuilder & tree,
                               Policy & policy,
                               const std::shared_ptr< ExtensibleKOMO > & komo ) const
{
  // traverse tree and ground symbols
  std::unordered_set<uint> visited;

  komo->groundInit(tree);

  for(const auto& l: policy.sleaves())
  {
    auto q = l;
    auto p = q->parent();

    while(p)
    {
      if(visited.find(q->id()) == visited.end())
      {
        double start = p->depth();
        double end = q->depth();

        Interval interval;
        interval.time = {start, start + 1.0};
        interval.edge = {p->id(), q->id()};

        // square acc + fix switched objects
        W(komo.get()).addObjective(interval, tree, new TM_Transition(komo->world), OT_sos, NoArr, 1.0, 2);
        W(komo.get()).addObjective(interval, tree, new TM_FixSwichedObjects(), OT_eq, NoArr, TM_FixSwichedObjects_scale, 2);

        // ground other tasks
        komo->groundTasks(interval, tree, q->data().leadingKomoArgs, 1);

        visited.insert(q->id());
      }
      q = p;
      p = q->parent();
    }
  }
}

void KOMOSparsePlanner::watch( const std::shared_ptr< ExtensibleKOMO > & komo ) const
{
  Var<WorldL> configs;
  auto v = std::make_shared<KinPathViewer>(configs,  0.2, -0 );
  v->setConfigurations(komo->configurations);
  rai::wait();
}

void KOMOSparsePlanner::watch( const std::shared_ptr< ExtensibleKOMO > & komo, const TreeBuilder & tree ) const
{
  rai::Array< rai::Array< rai::Array< rai::KinematicWorld > > > frames;
  frames.resize(1);
  const auto leaves = tree.get_leaves();

  frames(0).resize(leaves.size());

  for(auto l = 0; l < leaves.size(); ++l)
  {
    const auto leaf = leaves[l];

    const auto branch = tree.get_branch(leaf);

    const auto vars0 = branch.get_vars0({0.0, branch.n_nodes() - 1}, tree._get_branch(leaf), komo->stepsPerPhase);

    frames(0)(l).resize(vars0.size());

    for(auto s{0}; s < vars0.size(); ++s)
    {
      auto global = vars0(s);
      frames(0)(l)(s) = *komo->configurations(global + komo->k_order);
    }
  }

  TrajectoryTreeVisualizer viz( frames, "policy", komo->stepsPerPhase * 0.5);

  rai::wait();
}

void KOMOSparsePlanner::watch( const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics,
                               const rai::Array<rai::KinematicSwitch*> switches,
                               const Policy & policy, const TreeBuilder & tree,
                               const arr& x,
                               const uint stepsPerPhase,
                               const uint k_order ) const
{
  rai::Array< rai::Array< rai::Array< rai::KinematicWorld > > > frames;
  frames.resize(1);

  const auto policy_leaves = policy.leaves();

  const auto get_leaf_for_world = [](const uint w, const std::list<Policy::GraphNodeTypePtr>& leaves ) -> uint
  {
    const auto leaf_it = std::find_if(leaves.cbegin(), leaves.cend(), [&w](const auto& leaf) { return leaf->data().beliefState[w] > 0.0; } );

    CHECK(leaf_it != leaves.cend(), "policy doesn't seem to solve all belief states!");

    return (*leaf_it)->id();
  };

  frames(0).resize(startKinematics.size());
  for(auto w{0}; w < startKinematics.size(); ++w)
  {
    const auto leaf = get_leaf_for_world(w, policy_leaves);
    const auto branch = tree.get_branch(leaf);
    const auto vars0 = branch.get_vars0({0.0, branch.n_nodes() - 1}, tree._get_branch(leaf), stepsPerPhase);
    const auto qDim = startKinematics(w)->q.d0;

    frames(0)(w).resize(vars0.d0 + k_order);
    frames(0)(w)(0).copy(*startKinematics(w), true);

    // copy prefix
    for(auto s{1}; s < k_order; ++s)
    {
      frames(0)(w)(s).copy(frames(0)(w)(s - 1), true);
    }

    // copy frames and apply q
    for(auto s{0}; s < vars0.d0; ++s)
    {
      const auto global = vars0(s);
      const auto x_start{global * qDim};
      const auto q = x({x_start, x_start + qDim - 1});

      auto& K = frames(0)(w)(s + k_order);
      K.copy(frames(0)(w)(s + k_order - 1), true);
      K.setJointState(q);

      //apply potential graph switches
      for(auto *sw:switches)
      {
        if(sw->timeOfApplication == global)
        {
          sw->apply(K);
        }
      }
    }
  }

  TrajectoryTreeVisualizer viz( frames, "policy", stepsPerPhase * 0.5);

  rai::wait();
}

OptimizationReport KOMOSparsePlanner::getOptimizationReport(const std::shared_ptr< ExtensibleKOMO > & komo, const std::vector<Vars>& allVars ) const
{
  OptimizationReport report;
  report.slices.resize(komo->T);
  report.allVars = allVars;

  for(uint i=0; i<komo->objectives.N; i++)
  {
    Objective *task = komo->objectives(i);
    report.objectives[std::string(task->name.p)] = task->type;

    WorldL Ktuple;
    Ktuple.resize(task->vars.d1);

    double task_cost{0.0};

    /// DEBUG
    //const std::string task_name_str( task->name.p );
    //bool verbose = (task_name_str.find("TargetPosition") != std::string::npos);
    ///

    const double global_scale = 1.0; //task->map->scale.N ? task->map->scale(0) : 1.0; already applied by phi!

    for(uint t=0;t<task->vars.d0;t++)
    {
      CHECK(task->map->order + 1 == task->vars.d1, "inconsistent tm order!");

      for( uint s=0; s < task->vars.d1; ++s )
      {
        const auto global = task->vars(t, s) + komo->k_order;

        CHECK(global >= 0 && global < komo->configurations.d0, "");

        Ktuple(s) = komo->configurations(global);
      }

      uint d=task->map->__dim_phi(Ktuple);
      arr y;
      arr J;
      task->map->__phi(y, J, Ktuple);
      CHECK(y.d0 == d, "wrong tm dimensionality");

      CHECK(t < task->scales.d0, "");

      const double scale = task->scales(t) * global_scale;

      const auto global_task_time = task->vars(t, -1);

      report.slices[global_task_time].q = komo->configurations(task->vars(t, -1) + komo->k_order)->q;

      if(task->type==OT_sos)
      {
        const auto slice_cost = scale * sumOfSqr(y);
        report.slices[global_task_time].objectivesResults[std::string(task->name.p)] = slice_cost;
        task_cost += slice_cost;
      }
      else if(task->type==OT_eq)
      {
        // TODO: sumOfSqr ? or max abs?
      }
      else if(task->type==OT_ineq)
      {
        // TODO: max? or sumOfSqr of violations?
      }
    }

    if(!isTaskIrrelevant(task->name, config_.taskIrrelevantForPolicyCost))
    {
      report.totalCost += task_cost;

      //std::cerr << task->name << " scale:" << global_scale * task->scales(0) << " from:" << task->vars(0, -1) + komo->k_order << " to: " << task->vars(-1, 0) + komo->k_order  << " cost:" << task_cost << std::endl;
    }
    else
    {
      report.objectivesIrrelevantForCost.insert(std::string(task->name.p));
    }
  }

  std::cout << "total_cost : " << report.totalCost << std::endl;

  report.stepsPerPhase = config_.microSteps_;
  report.allVars = allVars;

  return report;
}

/// JOINT
void JointPlanner::optimize( Policy & policy, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics ) const
{
  using W = KomoWrapper;

  arr x;

  //for(auto i{0}; i < 2; ++i)
  {
  // build tree
  const auto tree = buildTree(policy);

  // prepare komo
  auto komo = intializeKOMO(tree, startKinematics.front());

  // ground policy actions
  komo->groundInit(tree);
  const auto allVars = getSubProblems(tree, policy);
  groundPolicyActionsJoint(tree, policy, komo);

  // run optimization
  komo->verbose = 1;
  W(komo.get()).reset(allVars);

  // initialize
  if(x.d0)
  {
    komo->set_x( x );
  }

  const auto start = std::chrono::high_resolution_clock::now();

  komo->run();

  const auto elapsed = std::chrono::high_resolution_clock::now() - start;
  const auto optimizationTime = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;

  // LOGS
  const auto log = true;
//  if(log) {
//    cout <<"** Hessian size.[" << komo->opt->newton.Hx.d0 << "] sparsity=" << sparsity(komo->opt->newton.Hx);
//    cout <<endl;
//  }

  if(log) {
    cout <<"** optimization time=" << optimizationTime
         <<" setJointStateCount=" << rai::KinematicWorld::setJointStateCount <<endl;
    cout <<" (kin:" << komo->timeKinematics <<" coll:" << komo->timeCollisions << " feat:" << komo->timeFeatures <<" newton: " << komo->timeNewton <<")" << std::endl;
  }

  //
  komo->getReport(true);

  x = komo->x;

  watch(komo, tree);
  }
}

/// ADMM SPARSE
void ADMMSParsePlanner::optimize( Policy & policy, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics ) const
{
  // build tree
  auto tree = buildTree(policy);

  // prepare komos
  std::vector< std::shared_ptr< ExtensibleKOMO > > komos;
  for(auto w = 0; w < policy.leaves().size(); ++w)
  {
    auto komo = intializeKOMO(tree, startKinematics.front());
    komos.push_back(komo);
  }

  // ground each komo
  for(uint w = 0; w < policy.leaves().size(); ++w)
  {
    groundPolicyActionsJoint(tree, policy, komos[w]);
  }

  // reset each komo
  auto allVars = getSubProblems(tree, policy);   // get subproblems
  auto allTMasks = getSubProblemMasks(allVars, komos.front()->T);

  for(auto & komo: komos)
  {
    W(komo.get()).reset(allVars, 0);
  }

  // ADMM
  std::vector<std::shared_ptr<GraphProblem>> converters;
  std::vector<std::shared_ptr<ConstrainedProblem>> constrained_problems;
  std::vector<arr> xmasks;
  converters.reserve(policy.leaves().size());
  constrained_problems.reserve(policy.leaves().size());
  xmasks.reserve(policy.leaves().size());

  for(auto w = 0; w < policy.leaves().size(); ++w)
  {
    auto& komo = *komos[w];
    auto& tmask = allTMasks[w];

    auto gp = std::make_shared<ADMM_MotionProblem_GraphProblem>(komo);
    gp->setSubProblem(tmask);
    arr xmask;
    gp->getXMask(xmask);//, false);

    auto pb = std::make_shared<Conv_Graph_ConstrainedProblem>(*gp, komo.logFile);

    converters.emplace_back(gp);
    constrained_problems.push_back(pb);
    xmasks.push_back(xmask);
  }

  // RUN
  auto start = std::chrono::high_resolution_clock::now();

  auto x = komos.front()->x;

  DecOptConstrained<T, U> opt(x, constrained_problems, xmasks, U(), DecOptConfig(PARALLEL, false));
  opt.run();

  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  double optimizationTime=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;


  // LOGS
  if(true) {
    cout <<"** optimization time=" << optimizationTime
         <<" setJointStateCount=" << rai::KinematicWorld::setJointStateCount <<endl;
  }

  auto & komo = komos.back();
  komo->set_x(x);
  watch(komo, tree);
}

/// ADMM COMPRESSED
void ADMMCompressedPlanner::setDecompositionStrategy( const std::string& strategy, const std::string& nJobs )
{
  decompositionStrategy_ = strategy;

  try
  {
    nJobs_ = std::stoi(nJobs);
  }
  catch (const std::invalid_argument& ia)
  {
    CHECK(false, "wrong argument for the number of jobs");
  }
}

void ADMMCompressedPlanner::groundPolicyActionsCompressed( const TreeBuilder & fullTree,
                                                           const TreeBuilder & uncompressed,
                                                           const TreeBuilder & compressed,
                                                           const Mapping & mapping,
                                                           Policy & policy,
                                                           const std::shared_ptr< ExtensibleKOMO > & komo ) const
{
  komo->groundInit(compressed);

  const auto edges = interactingEdges(fullTree, uncompressed); // interacting edges and the closest edge on subtree
  // interating edges are all the edges on which groundings have an influence on the subtree

  // traverse tree and ground symbols
  std::unordered_set<uint> visited;
  for(const auto& l: policy.sleaves())
  {
    auto q = l;
    auto p = q->parent();

    while(p)
    {
      // check if p-q is part of the interacting edges
      auto it = std::find_if(edges.begin(), edges.end(), [&](const std::pair<Edge, Edge> & es)
      {
        return es.first == Edge({p->id(), q->id()});
      });
      if(visited.find(q->id()) == visited.end()
         && it != edges.end())
      {
        double start = p->depth();
        double start_offset = uncompressed.d();
        double end = q->depth();

        const auto& local_edge = it->second;

        auto pid = mapping.orig_to_compressed(local_edge.from);
        auto qid = mapping.orig_to_compressed(local_edge.to);

        Interval interval;
        interval.time = {start - start_offset, start - start_offset + 1.0};
        interval.edge = {pid, qid};

        // square acc + fix switch objects
        W(komo.get()).addObjective(interval, compressed, new TM_Transition(komo->world), OT_sos, NoArr, 1.0, 2);
        W(komo.get()).addObjective(interval, compressed, new TM_FixSwichedObjects(), OT_eq, NoArr, TM_FixSwichedObjects_scale, 2);

        // ground other tasks
        komo->groundTasks(interval, compressed, q->data().leadingKomoArgs, 1);

        visited.insert(q->id());
      }
      q = p;
      p = q->parent();
    }
  }
}

void ADMMCompressedPlanner::optimize( Policy & policy, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics ) const
{
  using W = KomoWrapper;

  // build tree
  const auto tree = buildTree(policy);

  // 0 - SPLIT INTO SUBTREES
  const auto gen = generatorFactory_.create(decompositionStrategy_, nJobs_, tree);
  const auto subproblems = get_subproblems(gen); // uncompressed pb, compressed pb, mapping
  const auto allVars = get_all_vars(subproblems, config_.microSteps_);   // get subproblem vars (local)

  // 1 - PREPARE KOMOS
  const auto witness = intializeKOMO(tree, startKinematics.front());

  groundPolicyActionsJoint(tree, policy, witness);

  // 1.1 - init and ground each komo
  std::vector< std::shared_ptr< ExtensibleKOMO > > komos;
  for(auto w = 0; w < subproblems.size(); ++w)
  {
    std::cout << "GROUND " << w << std::endl;

    const auto& sub = subproblems[w];
    const auto& uncompressed = std::get<0>(sub);
    const auto& compressed = std::get<1>(sub);
    const auto& mapping = std::get<2>(sub);

    auto komo = intializeKOMO(compressed, startKinematics(0));

    komos.push_back(komo);
    groundPolicyActionsCompressed(tree, uncompressed, compressed, mapping, policy, komos[w]);
  }

  // 1.2 - reset komos
  const auto& witnessVars = std::get<2>(allVars); // fused
  W(witness.get()).reset(std::get<0>(allVars), 0); // uncompressed -> used fused!
  for(auto w = 0; w < komos.size(); ++w)
  {
    const std::vector<Vars>& uncompressed{std::get<0>(allVars)[w]};
    const std::vector<Vars>& compressed{std::get<1>(allVars)[w]};

    const auto firstIndex = uncompressed.front().order0.front();

    if(firstIndex>0)
    {
      const auto prev = witnessVars.getPreviousStep(firstIndex);
      komos[w]->world.copy(*witness->configurations(prev + witness->k_order));
    }
    //else // if first index == 0, the start configuration (world) is already well configured from the komo init

    W(komos[w].get()).reset(compressed);
  }

  // 2 - CREATE SUB-OPTIMIZATION-PROBLEMS
  auto uncompressedTMasks = getSubProblemMasks(std::get<0>(allVars), witness->T); // use uncompressed

  // 2.1 - create xmasks
  std::vector<arr> xmasks;
  xmasks.reserve(policy.leaves().size());
  auto gp = std::make_shared<ADMM_MotionProblem_GraphProblem>(*witness);
  for(auto w = 0; w < komos.size(); ++w)
  {
    auto& tmask = uncompressedTMasks[w];
    gp->setSubProblem(tmask);
    arr xmask;
    gp->getXMask(xmask);//, komos[w]->freePrefix);
    xmasks.push_back(xmask);
  }

  // 2.2 - create sub-opt-problems
  std::vector<std::shared_ptr<GraphProblem>> converters;
  std::vector<std::shared_ptr<ConstrainedProblem>> constrained_problems;
  converters.reserve(policy.leaves().size());
  constrained_problems.reserve(policy.leaves().size());
  for(auto w = 0; w < komos.size(); ++w)
  {
    auto& komo = *komos[w];
    auto gp = std::make_shared<KOMO::Conv_MotionProblem_GraphProblem>(komo);
    auto pb = std::make_shared<Conv_Graph_ConstrainedProblem>(*gp, komo.logFile);

    converters.push_back(gp);
    constrained_problems.push_back(pb);
  }

  // 3 - RUN
  auto start = std::chrono::high_resolution_clock::now();

  auto x = x_.d0 ? x_ : witness->x;

  OptOptions options;
  options.verbose = 1;
//  options.stopTolerance = 0; // avoid ADMM to return too early if it seems stuck
//  options.stopEvals = 20000; // avoid ADMM and optconstraintd solver to stop too early
//  options.stopIters = 20000; // ..
//  options.stopOuters = 20000; // ..

  DecOptConfig decOptConfig(PARALLEL, true, options, false);
  decOptConfig.scheduling = PARALLEL;
  decOptConfig.compressed = true;
  decOptConfig.checkGradients = false;

  DecOptConstrained<T, U> opt(x, constrained_problems, xmasks, U(), decOptConfig);
  opt.run();

  const auto elapsed = std::chrono::high_resolution_clock::now() - start;
  double optimizationTime=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;

  //4 - LOGS + PRINTS
  if(true)
  {
    cout <<"** optimization time=" << optimizationTime
         <<" setJointStateCount=" << rai::KinematicWorld::setJointStateCount <<endl;

    for(auto w = 0; w < opt.newtons.size(); ++w)
    {
      const auto& komo = *komos[w];
      auto timeNewton = opt.newtons[w]->timeNewton;
      std::cout <<" (kin:" << komo.timeKinematics <<" coll:" << komo.timeCollisions << " feat:" << komo.timeFeatures <<" newton: " << timeNewton <<")" << std::endl;
    }
  }
  //
  witness->set_x(x);
  witness->x = x;

  EvaluationPlanner(config_,
                    komoFactory_,
                    x,
                    "results/optimizationReportAdmmCompressed.re").optimize(policy, startKinematics);
  //const auto report = getOptimizationReport(witness, std::get<0>(allVars));
  //report.save("optimizationReportAdmmCompressed.re");
  //watch( startKinematics, witness->switches, policy, tree, x, witness->stepsPerPhase, witness->k_order );
}

void EvaluationPlanner::optimize( Policy & policy, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics ) const
{
  using W = KomoWrapper;

  // build tree
  const auto tree = buildTree(policy);

  // prepare komo
  auto komo = intializeKOMO(tree, startKinematics.front());

  // ground policy actions
  komo->groundInit(tree);
  const auto allVars = getSubProblems(tree, policy);
  groundPolicyActionsJoint(tree, policy, komo);

  // run optimization
  W(komo.get()).reset(allVars);

  // initialize
  CHECK(x_.d0 != 0, "invalid x")

  komo->set_x( x_ );

  const auto report = getOptimizationReport(komo, allVars);
  report.save( reportFile_ );
  watch( startKinematics, komo->switches, policy, tree, x_, komo->stepsPerPhase, komo->k_order );
}
}
