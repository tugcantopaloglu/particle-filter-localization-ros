#include <particle_pf/motion_model.hpp>
#include <particle_pf/utils.hpp>
#include <cmath>

using namespace pf;

static const double PF_DT_SCALE = 1;
static const double OMEGA_SCALE = 1;
Particle MotionModel::sample(const Particle &p,
                             const geometry_msgs::Twist &u,
                             double dt) const
{
  dt *= PF_DT_SCALE;

  double v = u.linear.x + sampleNormal(p_.alpha1 * fabs(u.linear.x) + p_.alpha2 * fabs(u.angular.z));
  double w = u.angular.z * OMEGA_SCALE + sampleNormal(p_.alpha3 * fabs(u.linear.x) + p_.alpha4 * fabs(u.angular.z));
  double gamma = sampleNormal(p_.alpha5 * fabs(u.linear.x) + p_.alpha6 * fabs(u.angular.z));

  Particle n;
  if (fabs(w) > 1e-5)
  {
    n.x = p.x - (v / w) * sin(p.theta) + (v / w) * sin(p.theta + w * dt);
    n.y = p.y + (v / w) * cos(p.theta) - (v / w) * cos(p.theta + w * dt);
  }
  else
  {
    n.x = p.x + v * dt * cos(p.theta);
    n.y = p.y + v * dt * sin(p.theta);
  }

  n.theta = p.theta + w * dt + gamma * dt;
  n.weight = p.weight;
  return n;
}
