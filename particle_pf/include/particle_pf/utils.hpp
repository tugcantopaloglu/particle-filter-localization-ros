#pragma once
#include <random>
namespace pf{
inline double sampleNormal(double s){
  static thread_local std::mt19937 g{std::random_device{}()};
  std::normal_distribution<double> d(0.0,s);
  return d(g);
}}
