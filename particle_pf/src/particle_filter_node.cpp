#include <ros/ros.h>
#include <particle_pf/motion_model.hpp>
#include <particle_pf/sensor_model.hpp>
#include <particle_pf/low_variance_sampler.hpp>
#include <particle_pf/logger.hpp>
#include <particle_pf/particle.hpp>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>

using namespace pf;

class PF
{
public:
  PF(ros::NodeHandle &nh) : nh_(nh), pnh_("~") { init(); }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_scan_, sub_cmd_, sub_gt_;
  ros::Publisher pub_path_, pub_mark_;
  MotionModel *motion_;
  SensorModel *sensor_;
  CSVLogger *log_;
  std::vector<Particle> ps_;
  MotionParams mp_;
  geometry_msgs::Twist last_cmd_;
  ros::Time last_stamp_;
  nav_msgs::Path path_;
  std::string gt_topic_;
  bool have_gt_ = false;
  geometry_msgs::PoseStamped gt_;
  void init()
  {
    pnh_.param("gt_topic", gt_topic_, std::string("/gazebo/model_states"));
    pnh_.param("alpha1", mp_.alpha1, 0.1);
    pnh_.param("alpha2", mp_.alpha2, 0.1);
    pnh_.param("alpha3", mp_.alpha3, 0.05);
    pnh_.param("alpha4", mp_.alpha4, 0.05);
    pnh_.param("alpha5", mp_.alpha5, 0.01);
    pnh_.param("alpha6", mp_.alpha6, 0.01);
    int n;
    pnh_.param("num_particles", n, 500);
    double sr, sb;
    pnh_.param("sigma_r", sr, 0.05);
    pnh_.param("sigma_b", sb, 0.02);
    motion_ = new MotionModel(mp_);
    sensor_ = new SensorModel(sr, sb);
    ps_.resize(n);
    for (auto &pp : ps_)
      pp.weight = 1.0 / n;
    std::string dir = ros::package::getPath("particle_pf") + "/results";
    log_ = new CSVLogger(dir);
    sub_scan_ = nh_.subscribe("scan", 1, &PF::scanCb, this);
    sub_cmd_ = nh_.subscribe("cmd_vel", 1, &PF::cmdCb, this);
    sub_gt_ = nh_.subscribe(gt_topic_, 1, &PF::gtCb, this);
    pub_path_ = nh_.advertise<nav_msgs::Path>("pf_path", 1);
    pub_mark_ = nh_.advertise<visualization_msgs::MarkerArray>("pf_particles", 1);
    path_.header.frame_id = "map";
  }
  void cmdCb(const geometry_msgs::Twist::ConstPtr &msg) { last_cmd_ = *msg; }
  void gtCb(const gazebo_msgs::ModelStates::ConstPtr &msg)
  {
    auto it = std::find(msg->name.begin(), msg->name.end(), "husky");
    if (it == msg->name.end())
      return;
    size_t idx = std::distance(msg->name.begin(), it);

    gt_.header.stamp = ros::Time::now();
    gt_.header.frame_id = "map";
    gt_.pose = msg->pose[idx];
    have_gt_ = true;
  }

  void scanCb(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    if (last_stamp_.isZero())
    {
      last_stamp_ = msg->header.stamp;
      return;
    }
    double dt = (msg->header.stamp - last_stamp_).toSec();
    last_stamp_ = msg->header.stamp;
    for (auto &pp : ps_)
      pp = motion_->sample(pp, last_cmd_, dt);
    auto obs = sensor_->extract(*msg);
    double sum = 0;
    for (auto &pp : ps_)
    {
      pp.weight = sensor_->likelihood(pp, obs);
      sum += pp.weight;
    }
    for (auto &pp : ps_)
      pp.weight /= sum;
    ps_ = resample(ps_);
    Particle mean = getMean();
    publishPath(mean);
    publishMarkers();
    if (have_gt_)
    {
      double t = msg->header.stamp.toSec();
      log_->log(t,
                gt_.pose.position.x, gt_.pose.position.y,
                tf::getYaw(gt_.pose.orientation),
                mean.x, mean.y, mean.theta);
    }
    ROS_INFO_THROTTLE(1.0, "dt=%.3f", dt);
  }
  Particle getMean()
  {
    Particle m;
    m.x = m.y = m.theta = 0;
    for (const auto &pp : ps_)
    {
      m.x += pp.x;
      m.y += pp.y;
      m.theta += pp.theta;
    }
    m.x /= ps_.size();
    m.y /= ps_.size();
    m.theta /= ps_.size();
    return m;
  }
  void publishPath(const Particle &mean)
  {
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = ros::Time::now();
    ps.pose.position.x = mean.x;
    ps.pose.position.y = mean.y;
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(mean.theta);
    path_.poses.push_back(ps);
    path_.header.stamp = ps.header.stamp;
    pub_path_.publish(path_);
  }
  void publishMarkers()
  {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.ns = "particles";
    m.id = 0;
    m.type = visualization_msgs::Marker::POINTS;
    m.scale.x = m.scale.y = 0.02;
    m.color.g = 1.0;
    m.color.a = 0.7;
    for (const auto &pp : ps_)
    {
      geometry_msgs::Point pt;
      pt.x = pp.x;
      pt.y = pp.y;
      m.points.push_back(pt);
    }
    arr.markers.push_back(m);
    pub_mark_.publish(arr);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "particle_filter_node");
  ros::NodeHandle nh;
  PF node(nh);
  ros::spin();
  return 0;
}
