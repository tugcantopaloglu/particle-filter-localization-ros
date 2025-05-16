#pragma once
#include <fstream>
#include <filesystem>
#include <cmath>
namespace pf{
class CSVLogger{
public:
  CSVLogger(const std::string&dir){
    std::filesystem::create_directories(dir);
    trace_.open(dir+"/trace.csv"); err_.open(dir+"/errors.csv");
    trace_<<"t,x_gt,y_gt,x_est,y_est\n";
    err_<<"t,trans_err,rot_err\n";
  }
  void log(double t,double xg,double yg,double yawg,double xe,double ye,double yawe){
    trace_<<t<<','<<xg<<','<<yg<<','<<xe<<','<<ye<<'\n';
    double terr=hypot(xg-xe, yg-ye);
    double aerr=fabs(atan2(sin(yawg-yawe),cos(yawg-yawe)));
    err_<<t<<','<<terr<<','<<aerr<<'\n';
  }
private:
  std::ofstream trace_,err_;
};
}
