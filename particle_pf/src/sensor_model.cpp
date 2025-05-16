#include <particle_pf/sensor_model.hpp>
#include <cmath>

using namespace pf;

static double gauss(double x, double s)
{
  return std::exp(-0.5 * x * x / (s * s)) / (s * std::sqrt(2.0 * M_PI));
}

constexpr double BEARING_OFF = -0.0175;

std::vector<Observation> SensorModel::extract(const sensor_msgs::LaserScan &scan) const
{
  std::vector<Observation> obs;
  for (size_t i = 0; i < scan.ranges.size(); ++i)
  {
    if (std::isfinite(scan.ranges[i]) && scan.intensities[i] > 1000)
    {
      Observation o;
      o.r = scan.ranges[i];
      o.b = scan.angle_min + i * scan.angle_increment;
      o.intensity = scan.intensities[i];
      obs.push_back(o);
    }
  }
  return obs;
}

double SensorModel::likelihood(const Particle &p,
                               const std::vector<Observation> &obs) const
{
  double w = 1e-12;

  for (const auto &o : obs)
  {
    double best = 1e-12;

    for (const auto &lm : reflectors())
    {
      double dx = lm.x - p.x;
      double dy = lm.y - p.y;

      double r_pred = std::hypot(dx, dy);
      double b_pred = std::atan2(dy, dx) - p.theta + BEARING_OFF;

      double r_err = o.r - r_pred;
      double b_err = std::atan2(std::sin(o.b - b_pred), std::cos(o.b - b_pred));

      double prob = gauss(r_err, sr_) * gauss(b_err, sb_);

      double refl = std::exp(-std::pow(o.intensity / 4096.0 - lm.reflect, 2) /
                             (2 * 0.05 * 0.05));
      prob *= refl;

      if (prob > best)
        best = prob;
    }
    w += best;
  }
  return w;
}
