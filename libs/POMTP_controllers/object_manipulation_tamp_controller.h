/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#pragma once

#include <tamp_controller.h>
#include <boost/filesystem.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

class ObjectManipulationTAMPController : public TAMPController
{
public:
  ObjectManipulationTAMPController( TaskPlanner & tp, MotionPlanner & mp )
    : TAMPController( tp, mp )
  {
    namespace fs = boost::filesystem;
    fs::path resultDir( fs::current_path() / "results" );
    if( fs::exists( resultDir ) )
    {
      boost::uuids::random_generator gen;
      const auto id = gen();

      const auto archivedResultDir( fs::current_path() / std::string{"results_"}.append( boost::uuids::to_string(id) ) );
      fs::rename( resultDir, archivedResultDir );
    }

    fs::create_directory( resultDir );

    candidate.open( "results/policy-candidates.data" );
    results.open( "results/policy-results.data" );
    timings.open( "results/timings.data" );
  }

  Policy plan( const TAMPlanningConfiguration & ) override;

  std::ofstream candidate, results, timings;

  double decision_tree_building_s = 0;
  double task_planning_s = 0;
  double motion_planning_s = 0;
  double joint_motion_planning_s = 0;
};
