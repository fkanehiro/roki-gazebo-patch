#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/JointController.hh>

#include <map>
#include <queue>
#include <sstream>
#include <fstream>
#include <iostream>

#define DEBUG_PRINT(...) printf(__VA_ARGS__)

namespace gazebo
{
  class TimeCode {
    public:
      TimeCode() : t(0.0) {}
      TimeCode(const double &_t) : t(_t) {}

      std::string ToString() const {
        std::stringstream ss;

        ss << "t=" << t << ",";
        std::map<std::string, double>::const_iterator it;
        for (it = map.begin(); it != map.end(); ++it) {
          ss << it->first << "=" << it->second << ", ";
        }
        return ss.str();
      }

      double t;
      std::map<std::string, double> map;
  };
  using TimeLine = std::deque<TimeCode>;

  class JointInfo {
    public: JointInfo() : is_init(false)
    {
    }

    public: JointInfo(const std::string &_name, const double &_p, const double &_i, const double &_d, const double &_initial_value, physics::ModelPtr _model)
      : name(_name), pid(_p, _i, _d), value(_initial_value), model(_model), is_init(false)
    {
      controller = physics::JointControllerPtr(new physics::JointController(_model));
    }

    private: void init_joint_controller_()
    {
      if (is_init == true) return;

      joint = model->GetJoint(name);

      if (!joint) {
        printf("JointInfo::init_joint_controller() : cannot find joint...name=%s\n", name.c_str());
        return;
      }

      controller->AddJoint(joint);
      controller->SetPositionPID(joint->GetScopedName(), pid);

      is_init = true;
    }

    public: void Apply()
    {
      if (is_init == false) init_joint_controller_();

      controller->SetPositionTarget(joint->GetScopedName(), value);
      controller->Update();

      math::Angle angle = joint->GetAngle(0);
    }

    public: std::string ToString() const
    {
      std::stringstream ss;

      ss << "JointInfo{";
      ss << "name=" << name << ", ";
      ss << "pid=(" << pid.GetPGain() << ", " << pid.GetIGain() << ", " << pid.GetDGain() << "), ";
      ss << "value=" << value;
      ss << "}";

      return ss.str();
    }

    std::string name;
    common::PID pid;
    double value;
    physics::ModelPtr model;
    physics::JointPtr joint;
    physics::JointControllerPtr controller;
    bool is_init;
  };

  class PIDPositionPlayerPlugin : public ModelPlugin
  {
    public: PIDPositionPlayerPlugin()
      : ModelPlugin(), tl_finished_(false)
    {
    }

    public: virtual ~PIDPositionPlayerPlugin() 
    {
    }

    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
    {
      this->model_ = _parent;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&PIDPositionPlayerPlugin::OnUpdate, this, _1));

      // prepare capture_file
      if (_sdf->HasElement("capture_file")) {
        std::string capture_file = _sdf->Get<std::string>("capture_file");
        DEBUG_PRINT("PIDPositionPlayerPlugin::Load() : capture_file=%s\n", capture_file.c_str());

        ReadCaptureFile(capture_file);
      }
    }

    public: void ReadCaptureFile(const std::string &filename)
    {
      int idx;
      std::string l;

      std::ifstream ifs(filename);
      if (ifs.fail() || ifs.bad()) {
        printf("PIDPositionPlayerPlugin::Load() : cannot open capture_file...file=%s\n", filename.c_str());
        return;
      }

      // header line
      std::getline(ifs, l);
      
      // PID line
      std::getline(ifs, l);
      std::vector<std::string> csv = split_comma(l);
      
      idx = 0;
      while(true) {
        if (csv.size() <= idx || csv[idx] == "") break;
        std::string name = csv[idx];
        double p             = std::stof(csv[idx + 1]);
        double i             = std::stof(csv[idx + 2]);
        double d             = std::stof(csv[idx + 3]);
        double initial_value = std::stof(csv[idx + 4]);

        JointInfo info(name, p, i, d, initial_value, model_);
        map_joint_[name] = info;
        //DEBUG_PRINT("PIDPositionPlayerPlugin::Load() : %s\n", info.ToString().c_str());

        idx += 5;
      }

      // Position line
      while(true) {
        if (ifs.eof()) break;

        std::getline(ifs, l);
        if (l == "") break;

        std::vector<std::string> csv = split_comma(l);
        if (csv.size() < 2) break;

        double t = std::stof(csv[1]);
        TimeCode tc(t);

        idx = 2;
        while(true) {
          if (csv.size() <= idx || csv[idx] == "") break;

          std::string name = csv[idx];
          double value     = std::stof(csv[idx + 1]);
          tc.map[name] = value;
          idx += 2;
        }
        //DEBUG_PRINT("PIDPositionPlayerPlugin::Load() : %s\n", tc.ToString().c_str());
        tl_.push_back(tc);
      }
    }

    public: void OnUpdate(const common::UpdateInfo & _info)
    {
      ProcessTimeLine();
      ProcessJoint();
    }

    public: void ProcessTimeLine()
    {
      double sim_t = GetSimTime();

      while(true) {
        if (tl_.size() == 0) {
          if (tl_finished_ == false) {
            tl_finished_ = true;
            DEBUG_PRINT("PIDPositionPlayerPlugin::ProcessTimeLine() : timeline finished...sim_time=%f\n", sim_t);
          }
          return;
        }

        TimeCode tc = tl_[0];
        if (tc.t > sim_t) break;

        // apply timeline position
        tl_.pop_front();

        std::map<std::string, double>::const_iterator it;
        for (it = tc.map.begin(); it != tc.map.end(); ++it) {
          std::string name = it->first;
          double value     = it->second;
          if (map_joint_.find(name) != map_joint_.end()) {
              map_joint_[name].value = value;
          }
        }
      }
    }

    public: void ProcessJoint()
    {
      std::map<std::string, JointInfo>::iterator ji;
      for (ji = map_joint_.begin(); ji != map_joint_.end(); ++ji) {
        ji->second.Apply();
      }
    }

    public: double GetSimTime()
    {
      common::Time t = this->model_->GetWorld()->GetSimTime();
      double d = (double)t.sec + (double)(t.nsec / (double)1e9);
      return d;
    }

    private: std::vector<std::string> split_comma(const std::string &str){
      std::istringstream iss(str);
      std::string tmp;
      std::vector<std::string> rv;
      while (getline(iss, tmp, ',')) {
        rv.push_back(tmp);
       }
       return rv;
    }

    protected: 
      event::ConnectionPtr updateConnection;

      physics::ModelPtr model_;

      TimeLine  tl_;
      bool tl_finished_;

      std::map<std::string, JointInfo> map_joint_;
  };

  GZ_REGISTER_MODEL_PLUGIN(PIDPositionPlayerPlugin)
}
