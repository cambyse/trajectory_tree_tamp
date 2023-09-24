#include <komo_wrapper.h>

#include "komo_tree_groundings.h"

#include <Kin/switch.h>
#include <Kin/TM_angVel.h>
#include <Kin/TM_default.h>
#include <Kin/TM_gravity.h>
#include <Kin/TM_InsideBox.h>
#include <Kin/TM_AboveBox.h>
#include <Kin/TM_qLimits.h>

#include <observation_tasks.h>
#include <axis_alignment.h>

using namespace rai;
using W = mp::KomoWrapper;

constexpr bool activateObjectives{true};

double shapeSize(const KinematicWorld& K, const char* name, uint i=2);

void groundTreeInit( const mp::TreeBuilder& tb, KOMO_ext* komo, int verbose )
{

}

void groundTreePickUp(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  groundTreeUnStack(it, tb, facts, komo, verbose);
}

void groundTreeUnStack(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  const auto& eff = "franka_hand";
  const auto& object = facts[0].c_str();
  const bool flipped = !strcmp(facts[2].c_str(), "TRUE");

  // approach
  mp::Interval before{{it.time.to - 0.3, it.time.to - 0.3}, it.edge};
  const auto approach_position = flipped ? ARR( -0.1, 0.0, 0.0 ) : ARR( 0.0, 0.0, 0.1 );
  if(activateObjectives) W(komo).addObjective( before, tb, new TargetPosition( eff, object, approach_position ), OT_sos, NoArr, 1e2, 0 ); // coming from above

  // switch
  mp::Interval st{{it.time.to, it.time.to}, it.edge};
  Transformation rel{0};
  rel.rot.setRpy(0.0, (flipped ? 3.145 * 0.5 : 0.0), 0.0);
  rel.pos.set(0, 0, 0);
  W(komo).addSwitch(st, tb, new KinematicSwitch(SW_effJoint, JT_rigid, eff, object, komo->world, SWInit_zero, 0, rel));

  mp::Interval end{{it.time.to, it.time.to}, it.edge};
  if(komo->k_order > 1)
  {
    mp::Interval just_after{{it.time.to, it.time.to + 0.2}, it.edge};
    if(activateObjectives) W(komo).addObjective( just_after, tb, new ZeroVelocity( object ), OT_eq, NoArr, 1e2, 1 ); // force the object not to move when starting to pick (mainly to force it not to go under the table)
  }

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << "(" << it.edge.from << ")" << " -> " << it.time.to << "(" << it.edge.to << ")" <<  " : unstack " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundTreePutDown(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  const auto& object = facts[0].c_str();
  const auto& place = facts[1].c_str();
  const bool flipped = !strcmp(facts[2].c_str(), "TRUE");

  mp::Interval before{{it.time.to - 0.3, it.time.to - 0.3}, it.edge};

  // approach
  if(activateObjectives) W(komo).addObjective( before, tb, new TargetPosition( object, place, ARR( 0.0, 0.0, 0.1 ) ), OT_sos, NoArr, 1e2, 0 ); // coming from above

  const auto axis = flipped ? ARR( -1.0, 0.0, 0.0 ) : ARR( 0.0, 0.0, 1.0 );
  mp::Interval just_before{{it.time.to - 0.2, it.time.to - 0.001}, it.edge};
  if(activateObjectives) W(komo).addObjective( just_before, tb, new AxisAlignment( object, axis, ARR( 0.0, 0.0, 1.0 ) ), OT_sos, NoArr, 1e2, 0 );

  mp::Interval end{{it.time.to, it.time.to}, it.edge};
  if(activateObjectives) W(komo).addObjective(end, tb, new TM_AboveBox(komo->world, object, place), OT_ineq, NoArr, 1e1, 0);

  // switch
  mp::Interval st{{it.time.to, it.time.to}, it.edge};
  Transformation rel{0};
  rel.rot.setRpy(0.0, (flipped ? 3.145 * 0.5 : 0.0), 0.0);
  rel.pos.set(0,0, .5*(shapeSize(komo->world, place) + shapeSize(komo->world, object)));
  W(komo).addSwitch(st, tb, new KinematicSwitch(SW_effJoint, JT_rigid, place, object, komo->world, SWInit_zero, 0, rel));

  if(komo->k_order > 1)
  {
    mp::Interval just_after{{it.time.to, it.time.to + 0.2}, it.edge};
    if(activateObjectives) W(komo).addObjective( just_after, tb, new ZeroVelocity( object ), OT_eq, NoArr, 1e2, 1 ); // force the object not to move when starting to pick (mainly to force it not to go under the table)
  }

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << "(" << it.edge.from << ")" << " -> " << it.time.to << "(" << it.edge.to << ")" << " : put down " << facts[0] << " at " << facts[1] << std::endl;
  }
}


void groundTreePutDownFlipped(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  const auto& object = facts[0].c_str();
  const auto& place = facts[1].c_str();

  mp::Interval before{{it.time.to - 0.3, it.time.to - 0.3}, it.edge};
  // approach
  if(activateObjectives) W(komo).addObjective( before, tb, new TargetPosition( object, place, ARR( 0.0, 0.0, 0.1 ) ), OT_sos, NoArr, 1e2, 0 ); // coming from above

  mp::Interval end{{it.time.to, it.time.to}, it.edge};
  if(activateObjectives) W(komo).addObjective(end, tb, new TM_AboveBox(komo->world, object, place), OT_ineq, NoArr, 1e1, 0);

  // switch
  mp::Interval st{{it.time.to, it.time.to}, it.edge};
  Transformation rel{0};
  rel.rot.setRpy(0.0, 3.145 * 0.5, 0.0);
  rel.pos.set(0,0, .5*(shapeSize(komo->world, place) + shapeSize(komo->world, object)));
  W(komo).addSwitch(st, tb, new KinematicSwitch(SW_effJoint, JT_rigid, place, object, komo->world, SWInit_zero, 0, rel));

  if(komo->k_order > 1)
  {
    mp::Interval just_after{{it.time.to, it.time.to + 0.2}, it.edge};
    if(activateObjectives) W(komo).addObjective( just_after, tb, new ZeroVelocity( object ), OT_eq, NoArr, 1e2, 1 ); // force the object not to move when starting to pick (mainly to force it not to go under the table)
  }

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << "(" << it.edge.from << ")" << " -> " << it.time.to << "(" << it.edge.to << ")" << " : put down flipped" << facts[0] << " at " << facts[1] << std::endl;
  }
}


void groundTreeCheck(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  std::map<std::string, arr> sideToPivot{
    {"side_0", ARR( -0.05, 0.0, 0.0 )},
    {"side_1", ARR( 0.00, 0.05, 0.0 )},
    {"side_2", ARR( 0.05, 0.00, 0.0 )},
    {"side_3", ARR( 0.00, -0.05, 0.0 )},
    {"side_4", ARR( 0.00, 0.00, 0.05 )},
    {"side_5", ARR( 0.00, 0.00, -0.05 )}
  };

  mp::Interval end{{it.time.to - 0.2, it.time.to}, it.edge};
  if(activateObjectives) W(komo).addObjective(end, tb, new SensorAimAtObjectCenter( "franka_hand", facts[0].c_str(), ARR( 0, 0, -1 ) ), OT_eq, NoArr, 1e2, 0 );
  if(activateObjectives) W(komo).addObjective(end, tb, new SensorAlignsWithPivot( "franka_hand", facts[0].c_str(), sideToPivot[facts[1]], 45.0 * 3.1415 / 180.0 ), OT_ineq, NoArr, 1e2, 0 );
  if(activateObjectives) W(komo).addObjective(end, tb, new SensorDistanceToObject( "franka_hand", facts[0].c_str(), 0.2 ), OT_sos, NoArr, 5e1, 0 );

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << "(" << it.edge.from << ")" << " -> " << it.time.to << "(" << it.edge.to << ")" << " : check " << facts[0] << std::endl;
  }
}

void groundTreeStack(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  groundTreePutDown(it, tb, facts, komo, verbose);
}
