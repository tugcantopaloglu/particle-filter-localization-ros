#include <particle_pf/low_variance_sampler.hpp>
#include <cstdlib>
using namespace pf;
std::vector<Particle> pf::resample(const std::vector<Particle>&in){
  size_t N=in.size();
  std::vector<Particle> out; out.reserve(N);
  double r=((double)rand()/RAND_MAX)/N;
  double c=in[0].weight;
  size_t i=0;
  for(size_t m=0;m<N;++m){
    double U=r+m*1.0/N;
    while(U>c && i+1<N){++i; c+=in[i].weight;}
    out.push_back(in[i]); out.back().weight=1.0/N;
  }
  return out;
}
