#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/JointController.hh>

#include <boost/thread.hpp>
#include <map>
#include <sstream>
#include <iostream>

#include <oscpack/osc/OscReceivedElements.h>
#include <oscpack/osc/OscPacketListener.h>
#include <oscpack/ip/UdpSocket.h>

#define DEBUG_PRINT(...) printf(__VA_ARGS__)

namespace gazebo
{
  class OSCThread : public osc::OscPacketListener {
    public: OSCThread() : port_(0), osc_recv_socket_(nullptr)
    {
    }

    public: virtual ~OSCThread()
    {
      Fini();
    }

    public: virtual bool Init(const unsigned short port)
    {
      port_ = port;

      try {
        osc_recv_socket_ = new UdpListeningReceiveSocket(IpEndpointName(IpEndpointName::ANY_ADDRESS, port), this);
      }
      catch (osc::Exception& e) {
        printf("OSCThread::Init() : osc::Exception!!! port %d already in use...e.what()=%s\n", port, e.what());
        return false;
      }
      catch (...) {
        printf("OSCThread::Init() : Exception!!! port %d already in use...\n", port);
        return false;
      }

      thread_.reset(new boost::thread([this] { Run(); }));
      DEBUG_PRINT("OSCThread::Init() : OSC thread start. port=%d\n", port);
    }

    public: virtual void Run() {
      osc_recv_socket_->Run();
    }

    public: virtual void Fini() {
      if (IsRunning()) {
          osc_recv_socket_->AsynchronousBreak();
          if (thread_->joinable()) {
            thread_->join();
          }
          thread_.reset();
          delete osc_recv_socket_;
          osc_recv_socket_ = nullptr;
      }
    }

    protected: bool IsRunning() const
    {
      if (thread_ && thread_->joinable()) return true;
      return false;
    }

    protected: virtual void ProcessMessage(const osc::ReceivedMessage& m, const IpEndpointName& remoteEndpoint)
    {
      try{
        std::string key = m.AddressPattern();

        if (key.substr(0, 1) == "/") {
          key = key.substr(1);
        }

        // check arguments
        osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
        if (arg == m.ArgumentsEnd()) {
           throw osc::ExcessArgumentException();
        }
        double value = (double)((arg++)->AsFloat());
        Set(key, value);
      }
      catch( osc::Exception& e ){
        printf("OSCThread::ProcessMessage() : error...%s, %s\n", m.AddressPattern(), e.what());
      }
    }

    protected: void Set(const std::string &key, double value)
    {
      boost::mutex::scoped_lock lock(mutex_);
      map_[key] = value;
    }

    public: std::map<std::string, double>  GetMap() {
      boost::mutex::scoped_lock lock(mutex_);
      return map_;
    }

    private:
      unsigned short port_;
      std::map<std::string, double> map_;

      boost::shared_ptr<boost::thread> thread_;
      boost::mutex mutex_;
      UdpListeningReceiveSocket *osc_recv_socket_;
  };

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

      std::string ToString() const
      {
        std::stringstream ss;

        ss << "JointInfo{";
        ss << "name=" << name << ", ";
        ss << "pid=(" << pid.GetPGain() << ", " << pid.GetIGain() << ", " << pid.GetDGain() << "), ";
        ss << "value=" << value;
        ss << "}";

        return ss.str();
      }

      std::string ToCSV() const 
      {
        std::stringstream ss;

        ss << name << ",";
        ss << pid.GetPGain() << ",";
        ss << pid.GetIGain() << ",";
        ss << pid.GetDGain() << ",";
        ss << value;

        return ss.str();
      }

    public:
      std::string name;
      common::PID pid;
      double value;
      physics::ModelPtr model;
      physics::JointPtr joint;
      physics::JointControllerPtr controller;
      bool is_init;

  };

  class OSCPIDControllerPlugin : public ModelPlugin
  {
    public: OSCPIDControllerPlugin()
      : ModelPlugin()
    {
    }

    public: virtual ~OSCPIDControllerPlugin() 
    {
      osc_thread_.Fini();
    }

    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
    {
      this->model_ = _parent;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&OSCPIDControllerPlugin::OnUpdate, this, _1));

      // prepare capture_file
      if (_sdf->HasElement("capture_file")) {
        capture_file_ = _sdf->Get<std::string>("capture_file");
        DEBUG_PRINT("OSCPIDControllerPlugin::Load() : capture_file=%s\n", capture_file_.c_str());
        ofs_.open(capture_file_);

        if (ofs_.bad() || ofs_.fail()) {
          DEBUG_PRINT("OSCPIDControllerPlugin::Load() : cannot open file...capture_file=%s\n", capture_file_.c_str());
          ofs_.close();
        }

        ofs_ << "PID_position_file_format,version,1," << std::endl;
      }

      // read <joint> tag
      sdf::ElementPtr e = _sdf->GetFirstElement();
      while(e != nullptr) {
        if (e->GetName() == "joint") {
          std::string joint_name;
          double p, i, d, initial_value;
          e->GetAttribute("name")->Get<std::string>(joint_name);
          e->GetAttribute("p")->Get<double>(p);
          e->GetAttribute("i")->Get<double>(i);
          e->GetAttribute("d")->Get<double>(d);
          e->GetAttribute("initial_value")->Get<double>(initial_value);
          JointInfo info(joint_name, p, i, d, initial_value, _parent);
          map_joint_[joint_name] = info;
          DEBUG_PRINT("OSCPIDControllerPlugin::Load() : joint_info=%s\n", info.ToString().c_str());

          if (ofs_.is_open()) {
            ofs_ << info.ToCSV() << ",";
          }
        }
        e = e->GetNextElement();
      }
      if (ofs_.is_open()) {
        ofs_ << std::endl;
      }

      // init osc_thread
      unsigned short port = 7000;
      if (_sdf->HasElement("osc_port")) {
        _sdf->Get<unsigned short>("osc_port");
      }
      osc_thread_.Init(port);
    }

    public: void OnUpdate(const common::UpdateInfo & _info)
    {
      ProcessOSC();
      ProcessJoint();
      CaptureFile();
    }

    public: void ProcessOSC()
    {
      std::map<std::string, double> map = osc_thread_.GetMap();

      std::map<std::string, double>::const_iterator it;
      for (it = map.begin(); it != map.end(); ++it) {
        std::string joint_name = it->first;

        std::map<std::string, JointInfo>::iterator ji = map_joint_.find(joint_name);
        if (ji != map_joint_.end()) {
          // apply the value set from OSC
          if (ji->second.value != it->second) {
            ji->second.value = it->second;
            //DEBUG_PRINT("OSCPIDControllerPlugin::ProcessOSC() : set to map_joint_...name=%s, value=%f\n", joint_name.c_str(), it->second);
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

    public: void CaptureFile()
    {
      if (!ofs_.is_open()) return;

      std::stringstream ss;

      std::map<std::string, JointInfo>::iterator ji;
      for (ji = map_joint_.begin(); ji != map_joint_.end(); ++ji) {
        ss << ji->second.name;
        ss << ",";
        ss << ji->second.value;
        ss << ",";
      }

      if (ss.str() == last_ofs_line_) return;

      double t = GetSimTime();
      ofs_ << "t," << t << "," << ss.str() << std::endl;
      last_ofs_line_ = ss.str();
      ofs_.flush();
    }

    public: double GetSimTime()
    {
      common::Time t = this->model_->GetWorld()->GetSimTime();
      double d = (double)t.sec + (double)(t.nsec / (double)1e9);
      return d;
    }
      

    protected: 
      event::ConnectionPtr updateConnection;
      OSCThread osc_thread_;

      std::string capture_file_;
      std::map<std::string, JointInfo> map_joint_;

      physics::ModelPtr model_;

      std::ofstream ofs_;
      std::string last_ofs_line_;
  };

  GZ_REGISTER_MODEL_PLUGIN(OSCPIDControllerPlugin)
}
