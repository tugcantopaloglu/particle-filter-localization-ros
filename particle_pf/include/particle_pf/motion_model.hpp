#pragma once
#include "particle.hpp"
#include <geometry_msgs/Twist.h>
namespace pf{
struct MotionParams{double alpha1,alpha2,alpha3,alpha4,alpha5,alpha6;};
class MotionModel{
public:
  explicit MotionModel(const MotionParams&p):p_(p){}
  Particle sample(const Particle&,const geometry_msgs::Twist&,double) const;
private: MotionParams p_;
};
}
