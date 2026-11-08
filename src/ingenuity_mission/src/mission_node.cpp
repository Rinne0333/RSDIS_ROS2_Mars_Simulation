#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.81;
double clamp(double v, double lo, double hi) { return std::clamp(v, lo, hi); }
struct Vec { double x=0, y=0, z=0; };
Vec operator+(Vec a, Vec b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
Vec operator-(Vec a, Vec b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
Vec operator*(Vec a, double s) { return {a.x*s,a.y*s,a.z*s}; }
struct Quat { double x=0,y=0,z=0,w=1; };
Quat conjugate(Quat q) { return {-q.x,-q.y,-q.z,q.w}; }
Quat product(Quat a, Quat b) {
  return {a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
          a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
          a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w,
          a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z};
}
Quat normalized(Quat q) {
  double n=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
  if (!std::isfinite(n)||n<1e-9) throw std::runtime_error("Invalid odometry quaternion");
  return {q.x/n,q.y/n,q.z/n,q.w/n};
}
Vec rotate(Quat q, Vec v) {
  const Quat r=product(product(q,{v.x,v.y,v.z,0}),conjugate(q));
  return {r.x,r.y,r.z};
}
Quat axisTilt(double roll, double pitch) {
  const double sr=std::sin(roll/2),cr=std::cos(roll/2);
  const double sp=std::sin(pitch/2),cp=std::cos(pitch/2);
  return product({sr,0,0,cr},{0,sp,0,cp});
}
double xyNorm(Vec v) { return std::hypot(v.x,v.y); }
}

class MissionNode : public rclcpp::Node {
public:
  MissionNode() : Node("ingenuity_mission") {
    arm_=declare_parameter<bool>("arm",false);
    dx_=declare_parameter<double>("goal_dx",3.0);
    dy_=declare_parameter<double>("goal_dy",0.0);
    rise_=declare_parameter<double>("flight_height",2.0);
    mass_=declare_parameter<double>("mass_kg",2.0);
    gravity_=declare_parameter<double>("gravity",kGravity);
    hover_total_=declare_parameter<double>("hover_total",19.62);
    top_fraction_=declare_parameter<double>("top_fraction",0.5);
    top_sign_=declare_parameter<double>("top_sign",1.0);
    bottom_sign_=declare_parameter<double>("bottom_sign",1.0);
    sw1_sign_=declare_parameter<double>("sw1_sign",1.0);
    sw2_sign_=declare_parameter<double>("sw2_sign",1.0);
    sw1_axis_sign_=declare_parameter<double>("sw1_axis_sign",1.0);
    sw2_axis_sign_=declare_parameter<double>("sw2_axis_sign",1.0);
    max_thrust_=declare_parameter<double>("max_thrust_per_rotor",35.0);
    max_tilt_=declare_parameter<double>("max_tilt_rad",0.10);
    max_swash_=declare_parameter<double>("max_swash_rad",0.05);
    kp_z_=declare_parameter<double>("kp_z",4.0);
    kd_z_=declare_parameter<double>("kd_z",5.0);
    kp_xy_=declare_parameter<double>("kp_xy",0.6);
    kd_xy_=declare_parameter<double>("kd_xy",1.1);
    kp_att_=declare_parameter<double>("kp_att",0.5);
    kd_att_=declare_parameter<double>("kd_att",0.15);
    arrival_radius_=declare_parameter<double>("arrival_radius",0.30);
    max_stage_s_=declare_parameter<double>("max_stage_seconds",35.0);
    hover_hold_s_=declare_parameter<double>("hover_hold_seconds",2.0);

    auto unitSign=[](double s){return s==1.0||s==-1.0;};
    if (!unitSign(top_sign_)||!unitSign(bottom_sign_)||!unitSign(sw1_sign_)||
        !unitSign(sw2_sign_)||!unitSign(sw1_axis_sign_)||!unitSign(sw2_axis_sign_)||
        !(mass_>0&&gravity_>0&&hover_total_>0&&top_fraction_>0&&top_fraction_<1&&
          max_thrust_>0&&max_thrust_<=100&&max_tilt_>0&&max_tilt_<=0.25&&
          max_swash_>0&&max_swash_<=0.15&&rise_>0&&rise_<=10&&
          arrival_radius_>0&&max_stage_s_>5&&hover_hold_s_>=0) ||
        !std::isfinite(dx_)||!std::isfinite(dy_)||!std::isfinite(hover_total_))
      throw std::invalid_argument("Invalid mission configuration");

    pub_top_=create_publisher<std_msgs::msg::Float64>("/ingenuity/top_blades/thrust",10);
    pub_bottom_=create_publisher<std_msgs::msg::Float64>("/ingenuity/bottom_blades/thrust",10);
    pub_sw1_=create_publisher<std_msgs::msg::Float64>("/ingenuity/swashplate_1/joint",10);
    pub_sw2_=create_publisher<std_msgs::msg::Float64>("/ingenuity/swashplate_2/joint",10);
    sub_=create_subscription<nav_msgs::msg::Odometry>(
      "/ingenuity/odometry",10,
      [this](nav_msgs::msg::Odometry::SharedPtr m){this->onOdom(*m);});
    timer_=create_wall_timer(50ms,[this](){tick();});
    RCLCPP_INFO(get_logger(),"A->B mission ready; arm=%s, goal offset=(%.2f,%.2f)m, height=%.2fm",
                arm_?"true":"false",dx_,dy_,rise_);
  }
  ~MissionNode() override { if(pub_top_&&pub_bottom_&&pub_sw1_&&pub_sw2_) neutral(); }
private:
  enum class State { WAIT, TAKEOFF, TRAVEL, HOLD, LAND, DONE, FAULT };
  const char *name(State s) const {
    switch(s){case State::WAIT:return "WAIT";case State::TAKEOFF:return "TAKEOFF";
    case State::TRAVEL:return "TRAVEL";case State::HOLD:return "HOLD";
    case State::LAND:return "LAND";case State::DONE:return "DONE";
    case State::FAULT:return "FAULT";} return "?";
  }
  void change(State s) {
    state_=s; stage_time_=std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(),"MISSION STATE: %s",name(s));
  }
  void command(double top, double bottom, double sw1, double sw2) {
    std_msgs::msg::Float64 m;
    m.data=clamp(top,0,max_thrust_);pub_top_->publish(m);
    m.data=clamp(bottom,0,max_thrust_);pub_bottom_->publish(m);
    m.data=clamp(sw1,-max_swash_,max_swash_);pub_sw1_->publish(m);
    m.data=clamp(sw2,-max_swash_,max_swash_);pub_sw2_->publish(m);
  }
  void neutral() {command(0,0,0,0);}
  void fault(const char *reason) {
    if(state_!=State::FAULT){RCLCPP_ERROR(get_logger(),"MISSION FAULT: %s",reason);change(State::FAULT);} neutral();
  }
  void onOdom(const nav_msgs::msg::Odometry &m) {
    auto p=m.pose.pose.position;auto q=m.pose.pose.orientation;
    auto v=m.twist.twist.linear;auto w=m.twist.twist.angular;
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
       !std::isfinite(v.x)||!std::isfinite(v.y)||!std::isfinite(v.z)||
       !std::isfinite(w.x)||!std::isfinite(w.y)||!std::isfinite(w.z)) {
      odom_ok_=false;return;
    }
    try{ q_=normalized({q.x,q.y,q.z,q.w}); }
    catch(const std::exception &){odom_ok_=false;return;}
    p_={p.x,p.y,p.z};
    // nav_msgs/Odometry twist is expressed in child_frame_id.
    v_world_=rotate(q_,{v.x,v.y,v.z});
    omega_={w.x,w.y,w.z};
    odom_ok_=true;last_odom_=std::chrono::steady_clock::now();
  }
  void tick() {
    const auto now=std::chrono::steady_clock::now();
    if(!arm_){neutral();return;}
    if(state_==State::DONE||state_==State::FAULT){neutral();return;}
    if(!odom_ok_) {if(state_!=State::WAIT)fault("invalid odometry");else neutral();return;}
    const double age=std::chrono::duration<double>(now-last_odom_).count();
    if(age>0.5){fault("odometry timeout");return;}
    if(state_==State::WAIT) {
      start_=p_;target_={start_.x+dx_,start_.y+dy_,start_.z+rise_};
      reference_=q_;
      change(State::TAKEOFF);neutral();return;
    }
    const double elapsed=std::chrono::duration<double>(now-stage_time_).count();
    if(elapsed>max_stage_s_){fault("stage timeout");return;}
    if(p_.z<start_.z-0.8||p_.z>target_.z+3.0||
       xyNorm(p_-start_)>std::max(15.0,xyNorm(target_-start_)+8.0)) {
      fault("position outside mission envelope");return;
    }

    Vec desired=start_;
    if(state_==State::TAKEOFF) desired.z=target_.z;
    if(state_==State::TRAVEL||state_==State::HOLD) desired=target_;
    if(state_==State::LAND) desired={target_.x,target_.y,start_.z};

    // Position PD (in world frame) produces desired horizontal acceleration.
    Vec error=desired-p_;
    Vec accel_xy={kp_xy_*error.x-kd_xy_*v_world_.x,
                  kp_xy_*error.y-kd_xy_*v_world_.y,0};
    double accel_norm=xyNorm(accel_xy);
    const double max_accel=gravity_*std::tan(max_tilt_);
    if(accel_norm>max_accel){accel_xy=accel_xy*(max_accel/accel_norm);}

    // World-frame acceleration -> reference body-frame tilt request.
    // Signs are configurable because the Gazebo rotor / swashplate mechanics are unverified.
    Vec local=rotate(conjugate(reference_),accel_xy);
    const double roll_cmd=sw2_axis_sign_*clamp(-local.y/gravity_,-max_tilt_,max_tilt_);
    const double pitch_cmd=sw1_axis_sign_*clamp(local.x/gravity_,-max_tilt_,max_tilt_);
    Quat target_att=product(reference_,axisTilt(roll_cmd,pitch_cmd));
    Quat qe=product(conjugate(q_),target_att);
    if(qe.w<0)qe={-qe.x,-qe.y,-qe.z,-qe.w};
    const double en=std::sqrt(qe.x*qe.x+qe.y*qe.y+qe.z*qe.z);
    const double scale=en>1e-9?2*std::atan2(en,qe.w)/en:2.0;
    const double sw1=sw1_sign_*(kp_att_*qe.y*scale-kd_att_*omega_.y);
    const double sw2=sw2_sign_*(kp_att_*qe.x*scale-kd_att_*omega_.x);

    // Height PD. The vertical velocity is explicitly transformed into world frame.
    double total=hover_total_+kp_z_*error.z-kd_z_*v_world_.z;
    total=clamp(total,0.0,2*max_thrust_);
    command(top_sign_*total*top_fraction_,
            bottom_sign_*total*(1.0-top_fraction_),sw1,sw2);

    if(state_==State::TAKEOFF&&p_.z>=target_.z-0.25&&std::abs(v_world_.z)<0.35)
      change(State::TRAVEL);
    if(state_==State::TRAVEL&&xyNorm(target_-p_)<arrival_radius_&&
       xyNorm(v_world_)<0.4) change(State::HOLD);
    if(state_==State::HOLD&&elapsed>=hover_hold_s_) change(State::LAND);
    if(state_==State::LAND&&p_.z<=start_.z+0.12&&std::abs(v_world_.z)<0.3) {
      neutral();change(State::DONE);return;
    }
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
      "%s | XYZ=(%.2f,%.2f,%.2f) goal=(%.2f,%.2f,%.2f) thrust=%.2f SW=(%.3f,%.3f)",
      name(state_),p_.x,p_.y,p_.z,desired.x,desired.y,desired.z,total,sw1,sw2);
  }
  bool arm_=false,odom_ok_=false;
  double dx_,dy_,rise_,mass_,gravity_,hover_total_,top_fraction_;
  double top_sign_,bottom_sign_,sw1_sign_,sw2_sign_,sw1_axis_sign_,sw2_axis_sign_;
  double max_thrust_,max_tilt_,max_swash_,kp_z_,kd_z_,kp_xy_,kd_xy_,kp_att_,kd_att_;
  double arrival_radius_,max_stage_s_,hover_hold_s_;
  State state_=State::WAIT;
  Vec p_,v_world_,omega_,start_,target_;
  Quat q_,reference_;
  std::chrono::steady_clock::time_point last_odom_{},stage_time_{};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_top_,pub_bottom_,pub_sw1_,pub_sw2_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<MissionNode>()); }
  catch(const std::exception &e){RCLCPP_ERROR(rclcpp::get_logger("ingenuity_mission"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
