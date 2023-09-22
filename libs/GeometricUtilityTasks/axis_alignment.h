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

#include <math_utility.h>

#include <Kin/feature.h>
#include <Kin/taskMaps.h>

// alignment to one world axis
struct AxisAlignment:Feature
{
  AxisAlignment( const char* bobyName, const arr & axis )
    : bobyName_     ( bobyName )
    , axis_( axis )
  {

  }

  virtual void phi( arr& y, arr& J, const rai::KinematicWorld& G );

  uint dim_phi( const rai::KinematicWorld& G ) { return 1; }

  rai::String shortTag(const rai::KinematicWorld& G){ return STRING("AxisAlignment"); }

  private:
    const rai::String bobyName_;
    const arr axis_;
};

// orthogonal to one world axis
struct AxisOrthogonal:Feature
{
  AxisOrthogonal( const char* bobyName, const arr & bodyAxis, const arr & worldAxis )
    : bobyName_     ( bobyName )
    , bodyAxis_( bodyAxis )
    , worldAxis_( worldAxis )
  {

  }

  virtual void phi( arr& y, arr& J, const rai::KinematicWorld& G );

  uint dim_phi( const rai::KinematicWorld& G ) { return 1; }

  rai::String shortTag(const rai::KinematicWorld& G ){ return STRING("AxisOrthogonal"); }

  private:
    const rai::String bobyName_;
    const arr bodyAxis_; // typically effector
    const arr worldAxis_;
};

// aligned to axis of box
struct BoxAxisAligned:Feature
{
  BoxAxisAligned( const char* effectorName, const arr & effectorAxis, const char * boxName, const arr & boxAxis )
    : effectorName_( effectorName )
    , effectorAxis_( effectorAxis )
    , boxName_( boxName )
    , boxAxis_( boxAxis )
  {

  }

  virtual void phi( arr& y, arr& J, const rai::KinematicWorld& G );

  uint dim_phi( const rai::KinematicWorld& G ) { return 1; }

  rai::String shortTag(const rai::KinematicWorld& G ){ return STRING("BoxAxisAligned"); }

  private:
    const rai::String effectorName_;
    const arr effectorAxis_;
    const rai::String boxName_;
    const arr boxAxis_;
};

// Position target position
struct TargetPosition:Feature
{
  TargetPosition( const char* effectorName, const char * boxName, const arr & targetPosition )
    : effectorName_( effectorName )
    , boxName_( boxName )
    , targetPosition_( targetPosition )
  {

  }

  virtual void phi( arr& y, arr& J, const rai::KinematicWorld& G );

  uint dim_phi( const rai::KinematicWorld& G ) { return 3; }

  rai::String shortTag(const rai::KinematicWorld& G ){ return STRING("TargetPosition"); }

  private:
    const rai::String effectorName_;
    const rai::String boxName_;
    const arr targetPosition_; // in box's frame
};

// Zero velocity (to be used in 1st order to force a zero velocity)
struct ZeroVelocity:Feature
{
    ZeroVelocity( const char* objectName )
      : objectName_( objectName )
    {

    }

    virtual void phi( arr& y, arr& J, const rai::KinematicWorld& G );

    uint dim_phi( const rai::KinematicWorld& G ) { return 3; }

    rai::String shortTag(const rai::KinematicWorld& G ){ return STRING("ZeroVelocity"); }

    private:
      const rai::String objectName_;
};

// Zero velocity (to be used in 1st order to force a zero velocity)
struct ZeroRelativeRotationVel:Feature
{
    ZeroRelativeRotationVel( const char* objectName )
      : objectName_( objectName )
    {

    }

    virtual void phi( arr& y, arr& J, const rai::KinematicWorld& G );

    uint dim_phi( const rai::KinematicWorld& G ) { return 4; }

    rai::String shortTag(const rai::KinematicWorld& G ){ return STRING("ZeroRelativeRotationVel"); }

    private:
      const rai::String objectName_;
};

// Zero velocity (to be used in 1st order to force a zero velocity)
struct ZeroRelativeVel:Feature
{
    ZeroRelativeVel( const char* objectName )
      : objectName_( objectName )
    {

    }

    virtual void phi( arr& y, arr& J, const rai::KinematicWorld& G );

    uint dim_phi( const rai::KinematicWorld& G ) { return 3; }

    rai::String shortTag(const rai::KinematicWorld& G ){ return STRING("ZeroRelativeVel"); }

   private:
    const rai::String objectName_;
};

