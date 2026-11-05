// Servo physics, effort limits and independent truth instrumentation.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/Utility.hh>
#include <array>
#include <algorithm>
#include <mutex>
#include <sstream>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace dragonfly {
using namespace gz::sim;
class Physics : public System, public ISystemConfigure, public ISystemPreUpdate, public ISystemPostUpdate {
  Model model{kNullEntity}; Link base;
  std::array<Entity,6> tilt{}; std::vector<Entity> arms, links, rotors;
  std::array<double,6> command{}, target{}, integral{}, tiltApplied{};
  std::vector<double> applied;
  std::vector<double> armEfforts;
  double rate{}, effort{}, kp{}, kd{}, lastPublish{-1}; bool ideal{};
  gz::transport::Node node; gz::transport::Node::Publisher truth, momentum, torques, rotorState, tiltTorques;
  std::mutex mutex; gz::msgs::Wrench wrench;
 public:
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &ecm, EventManager &) override {
    model = Model(entity); base = Link(model.LinkByName(ecm, "uav_base_link"));
    rate=sdf->Get<double>("tilt_rate"); effort=sdf->Get<double>("tilt_effort");
    kp=sdf->Get<double>("tilt_kp"); kd=sdf->Get<double>("tilt_kd");
    ideal=sdf->Get<bool>("ideal");
    int k=0;
    for (auto unit : {"a","b","c"}) for (auto axis : {"outer","inner"}) {
      auto e=model.JointByName(ecm,std::string("unit_")+unit+"_"+axis+"_joint"); tilt[k]=e;
      enableComponent<components::JointPosition>(ecm,e); enableComponent<components::JointVelocity>(ecm,e);
      node.Subscribe<gz::msgs::Double>("/model/dragonfly/servo_"+std::to_string(k), [this,k](const gz::msgs::Double &m) {
        std::lock_guard<std::mutex> lock(mutex); command[k]=std::clamp(m.data(),-1.57079632679,1.57079632679);
      }); ++k;
    }
    std::unordered_map<std::string,double> limits;
    if (!sdf->HasElement("arm_effort_limit"))
      throw std::runtime_error("Missing named arm_effort_limit entries");
    for (auto item=sdf->FindElement("arm_effort_limit"); item;
         item=item->GetNextElement("arm_effort_limit")) {
      auto name=item->Get<std::string>("joint");
      double limit=item->Get<double>();
      if (!std::isfinite(limit) || limit<=0 || !limits.emplace(name,limit).second)
        throw std::runtime_error("Invalid / duplicate arm effort limit: "+name);
    }
    if (limits.size()!=10) throw std::runtime_error("Expected ten named arm effort limits");
    for (auto side : {"left","right"}) for (int i=1;i<=5;++i) {
      auto name=std::string(side)+"_joint_"+std::to_string(i);
      auto joint=model.JointByName(ecm,name);
      if (joint==kNullEntity || !limits.count(name))
        throw std::runtime_error("Missing arm joint / effort limit: "+name);
      arms.push_back(joint); armEfforts.push_back(limits.at(name));
    }
    for(auto unit:{"a","b","c"}) for(auto side:{"left","right"}) {
      auto e=model.JointByName(ecm,std::string("unit_")+unit+"_rotor_"+side+"_joint");
      rotors.push_back(e); enableComponent<components::JointVelocity>(ecm,e);
    }
    ecm.Each<components::Link,components::ParentEntity>([&](const Entity &e,const auto*,const auto *p) {
      if (p->Data()==entity) { links.push_back(e); Link(e).EnableVelocityChecks(ecm); } return true;
    });
    truth=node.Advertise<gz::msgs::Odometry>("/dragonfly/truth");
    momentum=node.Advertise<gz::msgs::Wrench>("/dragonfly/momentum");
    torques=node.Advertise<gz::msgs::StringMsg>("/dragonfly/efforts");
    rotorState=node.Advertise<gz::msgs::StringMsg>("/dragonfly/rotor_speeds");
    tiltTorques=node.Advertise<gz::msgs::StringMsg>("/dragonfly/tilt_efforts");
    if (ideal) node.Subscribe<gz::msgs::Wrench>("/dragonfly/ideal_wrench",[this](const auto &msg){
      std::lock_guard<std::mutex> lock(mutex); wrench=msg;
    });
  }
  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override {
    if (info.paused) return;
    double dt=std::chrono::duration<double>(info.dt).count();
    std::lock_guard<std::mutex> lock(mutex);
    for (int k=0;k<6;++k) {
      auto q=ecm.Component<components::JointPosition>(tilt[k]);
      auto v=ecm.Component<components::JointVelocity>(tilt[k]);
      if (!q || !v || q->Data().empty() || v->Data().empty()) continue;
      target[k]+=std::clamp(command[k]-target[k],-rate*dt,rate*dt);
      double error=target[k]-q->Data()[0];
      integral[k]=std::clamp(integral[k]+20.*error*dt,-2.,2.);
      double u=std::clamp(kp*error+integral[k]-kd*v->Data()[0],-effort,effort);
      tiltApplied[k]=u;
      ecm.SetComponentData<components::JointForceCmd>(tilt[k],{u});
    }
    // Runs after ros2_control (priority 10): clamp the actual applied effort.
    applied.clear();
    for (size_t index=0;index<arms.size();++index) {
      auto e=arms[index]; double armEffort=armEfforts[index];
      auto u=ecm.Component<components::JointForceCmd>(e);
      if(u && !u->Data().empty()) u->Data()[0]=std::clamp(u->Data()[0],-armEffort,armEffort);
      applied.push_back(u && !u->Data().empty()?u->Data()[0]:0.);
    }
    if(ideal) {
      auto r=worldPose(base.Entity(),ecm).Rot();
      auto f=gz::msgs::Convert(wrench.force());
      // AddWorldWrench uses the link origin (unlike AddWorldForce).
      base.AddWorldWrench(ecm,r*f,r*gz::msgs::Convert(wrench.torque()));
    }
  }
  void PostUpdate(const UpdateInfo &info,const EntityComponentManager &ecm) override {
    double time=std::chrono::duration<double>(info.simTime).count();
    if(info.paused || time-lastPublish<.0099) return; lastPublish=time;
    gz::msgs::Odometry msg;
    *msg.mutable_header()->mutable_stamp()=gz::msgs::Convert(info.simTime);
    auto p=worldPose(base.Entity(),ecm);
    gz::msgs::Set(msg.mutable_pose(),p);
    gz::msgs::Set(msg.mutable_twist()->mutable_linear(),base.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero));
    gz::msgs::Set(msg.mutable_twist()->mutable_angular(),base.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero));
    truth.Publish(msg); // Both velocities here are world ENU (documented adapter contract).
    gz::math::Vector3d linear= gz::math::Vector3d::Zero, angular=linear;
    for(auto e:links) {
      auto inertial=ecm.Component<components::Inertial>(e); if(!inertial) continue;
      auto in=inertial->Data(); auto pose=worldPose(e,ecm); auto center=pose*in.Pose();
      auto omega=Link(e).WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
      auto vel=Link(e).WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero)+omega.Cross(center.Pos()-pose.Pos());
      auto pm=vel*in.MassMatrix().Mass(); linear+=pm;
      auto r=gz::math::Matrix3d(center.Rot()); angular+=center.Pos().Cross(pm)+r*in.MassMatrix().Moi()*r.Transposed()*omega;
    }
    gz::msgs::Wrench mom; *mom.mutable_header()->mutable_stamp()=gz::msgs::Convert(info.simTime);
    gz::msgs::Set(mom.mutable_force(),linear); gz::msgs::Set(mom.mutable_torque(),angular); momentum.Publish(mom);
    gz::msgs::StringMsg forces; std::ostringstream values;
    for(size_t i=0;i<applied.size();++i) { if(i) values<<','; values<<applied[i]; }
    forces.set_data(values.str());
    torques.Publish(forces);
    gz::msgs::StringMsg speeds; std::ostringstream velocity;
    for(size_t i=0;i<rotors.size();++i) {
      auto v=ecm.Component<components::JointVelocity>(rotors[i]);
      if(i) velocity<<','; velocity<<(v && !v->Data().empty()?10.*v->Data()[0]:0.);
    }
    speeds.set_data(velocity.str()); rotorState.Publish(speeds);
    gz::msgs::StringMsg tiltForces; std::ostringstream tiltValues;
    for(size_t i=0;i<tiltApplied.size();++i) { if(i) tiltValues<<','; tiltValues<<tiltApplied[i]; }
    tiltForces.set_data(tiltValues.str()); tiltTorques.Publish(tiltForces);
  }
};
}
GZ_ADD_PLUGIN(dragonfly::Physics,gz::sim::System,dragonfly::Physics::ISystemConfigure,dragonfly::Physics::ISystemPreUpdate,dragonfly::Physics::ISystemPostUpdate)
