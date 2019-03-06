#include <elfin_ethercat_driver_v2/elfin_ethercat_driver_v2.h>

#define TEST_DR_ELFINROBOT

namespace dr
{

typedef struct{
  std::string name;
  double reduction_ratio;
  double count_rad_factor;
  double count_rad_per_s_factor;
  double count_Nm_factor;
  int32_t count_zero;

  double axis_position_factor;
  double axis_torque_factor;

  double position;
  double velocity;
  double effort;

  double position_cmd;
  double velocity_cmd;
  double vel_ff_cmd;
  double effort_cmd;
}AxisInfo;

typedef struct{
  elfin_ethercat_driver_v2::ElfinEtherCATClient* client_ptr;
  AxisInfo axis1;
  AxisInfo axis2;
}ModuleInfo;

class Elfin_Robot
{
private:
  /* data */
  ros::NodeHandle nh_;
  elfin_ethercat_driver_v2::ElfinEtherCATDriver *p_ethercat_drivers_;
  std::vector<ModuleInfo> module_infos_;

public:
  Elfin_Robot(elfin_ethercat_driver_v2::EtherCatManager *manager,
      const ros::NodeHandle &nh=ros::NodeHandle("~"));
  ~Elfin_Robot();

  void state_update();
  void write_update();

  void test_six_axis_move(double cmd = 0.00005);

  bool enable_robot();
  bool disable_robot();

  bool setGroupPosMode();
};

Elfin_Robot::Elfin_Robot(elfin_ethercat_driver_v2::EtherCatManager *manager,
    const ros::NodeHandle &nh):
    nh_(nh)
{
  // get the ethercat driver pointer
  p_ethercat_drivers_ = new elfin_ethercat_driver_v2::ElfinEtherCATDriver(manager, "elfin");
  // get axis module info
  module_infos_.clear();

  for (size_t j = 0; j < p_ethercat_drivers_->getEtherCATClientNumber(); j++)
  {
    ModuleInfo module_info_tmp;

    module_info_tmp.client_ptr = p_ethercat_drivers_->getEtherCATClientPtr(j);
    module_info_tmp.axis1.name = p_ethercat_drivers_->getJointName(2*j);

    module_info_tmp.axis1.reduction_ratio = p_ethercat_drivers_->getReductionRatio(2*j);
    module_info_tmp.axis1.axis_position_factor = p_ethercat_drivers_->getAxisPositionFactor(2*j);
    module_info_tmp.axis1.count_zero = p_ethercat_drivers_->getCountZero(2*j);
    module_info_tmp.axis1.axis_torque_factor = p_ethercat_drivers_->getAxisTorqueFactor(2*j);

    module_info_tmp.axis2.name = p_ethercat_drivers_->getJointName(2*j+1);
    module_info_tmp.axis2.reduction_ratio = p_ethercat_drivers_->getReductionRatio(2*j+1);
    module_info_tmp.axis2.axis_position_factor = p_ethercat_drivers_->getAxisPositionFactor(2*j+1);
    module_info_tmp.axis2.count_zero = p_ethercat_drivers_->getCountZero(2*j+1);
    module_info_tmp.axis2.axis_torque_factor = p_ethercat_drivers_->getAxisTorqueFactor(2*j+1);

    module_infos_.push_back(module_info_tmp);
  }

  for(size_t i=0; i<module_infos_.size(); i++)
  {
    module_infos_[i].axis1.count_rad_factor=module_infos_[i].axis1.reduction_ratio*module_infos_[i].axis1.axis_position_factor/(2*M_PI);
    module_infos_[i].axis1.count_rad_per_s_factor=module_infos_[i].axis1.count_rad_factor/750.3;
    module_infos_[i].axis1.count_Nm_factor=module_infos_[i].axis1.axis_torque_factor/module_infos_[i].axis1.reduction_ratio;

    module_infos_[i].axis2.count_rad_factor=module_infos_[i].axis2.reduction_ratio*module_infos_[i].axis2.axis_position_factor/(2*M_PI);
    module_infos_[i].axis2.count_rad_per_s_factor=module_infos_[i].axis2.count_rad_factor/750.3;
    module_infos_[i].axis2.count_Nm_factor=module_infos_[i].axis2.axis_torque_factor/module_infos_[i].axis2.reduction_ratio;
  }

}

Elfin_Robot::~Elfin_Robot()
{
  if (p_ethercat_drivers_ != NULL)
    delete p_ethercat_drivers_;
}

void Elfin_Robot::state_update()
{
  for(size_t i=0; i<module_infos_.size(); i++)
  {
    int32_t pos_count1=module_infos_[i].client_ptr->getAxis1PosCnt();
    int16_t vel_count1=module_infos_[i].client_ptr->getAxis1VelCnt();
    int16_t trq_count1=module_infos_[i].client_ptr->getAxis1TrqCnt();
    int32_t pos_count_diff_1=pos_count1-module_infos_[i].axis1.count_zero;

    double position_tmp1=-1*pos_count_diff_1/module_infos_[i].axis1.count_rad_factor;
    module_infos_[i].axis1.position=position_tmp1;
    module_infos_[i].axis1.velocity=-1*vel_count1/module_infos_[i].axis1.count_rad_per_s_factor;
    module_infos_[i].axis1.effort=-1*trq_count1/module_infos_[i].axis1.count_Nm_factor;

    int32_t pos_count2=module_infos_[i].client_ptr->getAxis2PosCnt();
    int16_t vel_count2=module_infos_[i].client_ptr->getAxis2VelCnt();
    int16_t trq_count2=module_infos_[i].client_ptr->getAxis2TrqCnt();
    int32_t pos_count_diff_2=pos_count2-module_infos_[i].axis2.count_zero;

    double position_tmp2=-1*pos_count_diff_2/module_infos_[i].axis2.count_rad_factor;
    module_infos_[i].axis2.position=position_tmp2;
    module_infos_[i].axis2.velocity=-1*vel_count2/module_infos_[i].axis2.count_rad_per_s_factor;
    module_infos_[i].axis2.effort=-1*trq_count2/module_infos_[i].axis2.count_Nm_factor;
  }

  std::cout << "Axis " << module_infos_[2].axis2.name << " "
            <<"Postion " << module_infos_[2].axis2.position << " "
            <<"velocity " << module_infos_[2].axis2.velocity << " "
            <<"effort " << module_infos_[2].axis2.effort << " "
            << std::endl;
  std::cout << "--" <<  module_infos_[0].axis1.position
            << "--" <<  module_infos_[0].axis2.position
            << "--" <<  module_infos_[1].axis1.position
            << "--" <<  module_infos_[1].axis2.position
            << "--" <<  module_infos_[2].axis1.position
            << "--" <<  module_infos_[2].axis2.position
            << std::endl;
}

  bool Elfin_Robot::enable_robot()
  {
    std_srvs::SetBool::Request req;
    req.data = true;

    std_srvs::SetBool::Response resp;

    p_ethercat_drivers_->enableRobot_cb(req, resp);
  }

  bool Elfin_Robot::disable_robot()
  {
    std_srvs::SetBool::Request req;
    req.data = true;

    std_srvs::SetBool::Response resp;

    p_ethercat_drivers_->disableRobot_cb(req, resp);
  }

  bool Elfin_Robot::setGroupPosMode()
  {
    std::vector<int> module_no;
    module_no.clear();

    for (int i = 0; i < module_infos_.size(); i++)
    {
      module_no.push_back(i);
    }

    usleep(5000);

    for(int j=0; j<module_no.size(); j++)
    {
      module_infos_[module_no[j]].client_ptr->setAxis1VelFFCnt(0x0);
      module_infos_[module_no[j]].client_ptr->setAxis2VelFFCnt(0x0);

      module_infos_[module_no[j]].client_ptr->setAxis1TrqCnt(0x0);
      module_infos_[module_no[j]].client_ptr->setAxis2TrqCnt(0x0);
    }

    for(int j=0; j<module_no.size(); j++)
    {
        module_infos_[module_no[j]].client_ptr->setPosMode();
    }

    usleep(10000);

    for(int j=0; j<module_no.size(); j++)
    {
        if(!module_infos_[module_no[j]].client_ptr->inPosMode())
        {
          ROS_ERROR("module[%i]: set position mode failed", module_no[j]);

          return false;
        }
    }
    return true;
  }

  void Elfin_Robot::write_update()
  {
    for (size_t i = 0; i < module_infos_.size(); i++)
    {
      if(!module_infos_[i].client_ptr->inPosBasedMode())
      {
        module_infos_[i].axis1.position_cmd=module_infos_[i].axis1.position;
        module_infos_[i].axis2.position_cmd=module_infos_[i].axis2.position;

        std::cout << "Caution! Not in PosBasedMode!\n";
      }

      // update position cmd
      double position_cmd_count1=-1 * module_infos_[i].axis1.position_cmd * module_infos_[i].axis1.count_rad_factor + module_infos_[i].axis1.count_zero;

      double position_cmd_count2=-1 * module_infos_[i].axis2.position_cmd * module_infos_[i].axis2.count_rad_factor + module_infos_[i].axis2.count_zero;

      module_infos_[i].client_ptr->setAxis1PosCnt(int32_t(position_cmd_count1));
      module_infos_[i].client_ptr->setAxis2PosCnt(int32_t(position_cmd_count2));

      if (i == 0)
      {
        std::cout << "\n cmd counter: "
                  << int32_t(position_cmd_count1) - module_infos_[i].axis1.count_zero
                  << "  --  "
                  << int32_t(position_cmd_count2) - module_infos_[i].axis2.count_zero
                  << "  --  ";
      }
      else if (i == 1)
      {
        std::cout << int32_t(position_cmd_count1) - module_infos_[i].axis1.count_zero
                  << "  --  "
                  << int32_t(position_cmd_count2) - module_infos_[i].axis2.count_zero
                  << "  --  ";
      }
      else if (i == 2)
      {
        std::cout << int32_t(position_cmd_count1) - module_infos_[i].axis1.count_zero
                  << "  --  "
                  << int32_t(position_cmd_count2) - module_infos_[i].axis2.count_zero
                  << "\n";
        std::cout << "position cmd:  "
                  << module_infos_[i].axis1.position_cmd
                  << module_infos_[i].axis1.position_cmd
                  << "\n";
      }

      // for test position cmd purpose so remove send velocity and effort command

      double vel_ff_cmd_count1=-1 * module_infos_[i].axis1.vel_ff_cmd * module_infos_[i].axis1.count_rad_per_s_factor / 16.0;
      double vel_ff_cmd_count2=-1 * module_infos_[i].axis2.vel_ff_cmd * module_infos_[i].axis2.count_rad_per_s_factor / 16.0;

      module_infos_[i].client_ptr->setAxis1VelFFCnt(int16_t(vel_ff_cmd_count1));
      module_infos_[i].client_ptr->setAxis2VelFFCnt(int16_t(vel_ff_cmd_count2));

      double torque_cmd_count1=-1 * module_infos_[i].axis1.effort_cmd * module_infos_[i].axis1.count_Nm_factor;
      double torque_cmd_count2=-1 * module_infos_[i].axis2.effort_cmd * module_infos_[i].axis2.count_Nm_factor;

      module_infos_[i].client_ptr->setAxis1TrqCnt(int16_t(torque_cmd_count1));
      module_infos_[i].client_ptr->setAxis2TrqCnt(int16_t(torque_cmd_count2));
    }
  }

  void Elfin_Robot::test_six_axis_move(double cmd)
  {
    module_infos_[2].axis2.position_cmd = cmd;
  }

}   // end of namespace dr



int main(int argc, char** argv)
{
  ros::init(argc,argv,"elfin_ethercat_driver_v2_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh("elfin_ethercat_driver_v2_node");

  // init the ethercat manager
  std::string ethernet_name;
  ethernet_name = nh.param<std::string>("elfin_ethernet_name", "eth0");
  elfin_ethercat_driver_v2::EtherCatManager em(ethernet_name);

  dr::Elfin_Robot robot(&em, nh);

  std::cout << "--------------------------------------\n";
  std::cout << "Publish Robot status\n";
  std::cout << "--------------------------------------\n";
  size_t counter = 0;
  for (counter = 0; counter < 2; counter++)
  {
    robot.state_update();
    usleep(100000);
  }

  std::cout << "--------------------------------------\n";
  std::cout << "Enable Robot\n";
  std::cout << "--------------------------------------\n";
  robot.enable_robot();
  for (counter = 0 ; counter < 2; counter++)
  {
    robot.state_update();
    usleep(100000);
  }

  std::cout << "--------------------------------------\n";
  std::cout << "Set Robot Arm to postion Mode\n";
  std::cout << "--------------------------------------\n";
  if (robot.setGroupPosMode() != true)
  {
    std::cout << "Set robot arm to postion mode is ERROR!\n";
  }
  for (counter = 0 ; counter < 2; counter++)
  {
    robot.state_update();
    usleep(1000000);
  }

  std::cout << "--------------------------------------\n";
  std::cout << "Move Robot Six Axis\n";
  std::cout << "--------------------------------------\n";
  for (counter = 0 ; counter < 10; counter++)
  {
    double cmd = 0.00003;

    robot.state_update();
    robot.test_six_axis_move(cmd);
    robot.write_update();
    usleep(500);
  }

  for (counter = 0 ; counter < 2; counter++)
  {
    robot.state_update();
    usleep(1000000);
    ros::spinOnce();
  }

  std::cout << "--------------------------------------\n";
  std::cout << "Disable robot Arm\n";
  std::cout << "--------------------------------------\n";
  robot.disable_robot();
  for (counter = 0 ; counter < 5; counter++)
  {
    robot.state_update();
    usleep(1000000);
  }

  return 0;
}
