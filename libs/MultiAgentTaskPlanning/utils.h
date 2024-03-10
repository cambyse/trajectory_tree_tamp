#pragma once

#include <stdexcept>

#include <queue>
#include <string>
#include <unordered_map>

#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <skeleton.h>
#include <task_planner.h>

#include <logic_parser.h>

namespace matp
{

std::vector< std::string > decisionArtifactToKomoArgs( const std::string & artifact );


class Rewards
{
public:
  void setR0(double r0) { r0_ = r0; }
  double R0() const { return r0_; }

  double get(uint from, uint to)
  {
    return get((from << 10 ) + to);
  }

  double get(uint key)
  {
    if(rewards_.find(key) == rewards_.end())
      rewards_[key] = r0_;

    return rewards_[key];
  }

  void set(uint from, uint to, double r)
  {
    set( (from << 10 ) + to, r );
  }

  void set(uint key, double r)
  {
    rewards_[key] = r;
  }

private:
  std::unordered_map< uint, double > rewards_;
  double r0_{-1.0};
};

class Values
{
public:
  double v0() const { return v0_; }

  double get(uint nodeId) const
  {
    const auto vIt = values_.find(nodeId);

    CHECK( vIt != values_.end(), "" );

    return vIt->second;
  }

  double getOrDefault(uint nodeId)
  {
    if(values_.find(nodeId) == values_.end())
      values_[nodeId] = std::numeric_limits< double >::lowest();

    return values_[nodeId];
  }

  void set(uint nodeId, double v)
  {
    values_[nodeId] = v;
  }

  const std::unordered_map< uint, double >& values() const { return values_; }

private:
  double v0_{std::numeric_limits< double >::lowest()};
  std::unordered_map< uint, double > values_;
};

} // namespace matp
