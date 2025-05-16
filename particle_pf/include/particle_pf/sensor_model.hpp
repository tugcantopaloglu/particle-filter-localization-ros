#pragma once
#include "particle.hpp"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <particle_pf/reflector_map.hpp>

namespace pf
{
  struct Observation
  {
    double r;
    double b;
    double intensity;
  };

  class SensorModel
  {
  public:
    SensorModel(double sr, double sb) : sr_(sr), sb_(sb) {}

    std::vector<Observation> extract(const sensor_msgs::LaserScan &) const;
    double likelihood(const Particle &, const std::vector<Observation> &) const;

  private:
    double sr_, sb_;
  };
}
