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

  void test_six_axis_move();

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
      module_no.push_back(i)
    }

    for(int j=0; j<module_no.size(); j++)
    {
        boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[j]]);
        pre_switch_flags_[module_no[j]]=true;
        pre_switch_flags_lock.unlock();
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

            for(int k=0; k<module_no.size(); k++)
            {
                boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[k]]);
                pre_switch_flags_[module_no[k]]=false;
                pre_switch_flags_lock.unlock();
            }

            return false;
        }
    }
    return true;
  }

  void Elfin_Robot::write_update()
  {
    for (size_t i = 0; i < module_infos_.size(); i++)
    {
      // update position cmd
      double position_cmd_count1=-1 * module_infos_[i].axis1.position_cmd * module_infos_[i].axis1.count_rad_factor + module_infos_[i].axis1.count_zero;
      double position_cmd_count2=-1 * module_infos_[i].axis2.position_cmd * module_infos_[i].axis2.count_rad_factor + module_infos_[i].axis2.count_zero;

      module_infos_[i].client_ptr->setAxis1PosCnt(int32_t(position_cmd_count1));
      module_infos_[i].client_ptr->setAxis2PosCnt(int32_t(position_cmd_count2));

      // double vel_ff_cmd_count1=-1 * module_infos_[i].axis1.vel_ff_cmd * module_infos_[i].axis1.count_rad_per_s_factor / 16.0;
      // double vel_ff_cmd_count2=-1 * module_infos_[i].axis2.vel_ff_cmd * module_infos_[i].axis2.count_rad_per_s_factor / 16.0;

      // module_infos_[i].client_ptr->setAxis1VelFFCnt(int16_t(vel_ff_cmd_count1));
      // module_infos_[i].client_ptr->setAxis2VelFFCnt(int16_t(vel_ff_cmd_count2));

      // double torque_cmd_count1=-1 * module_infos_[i].axis1.effort_cmd * module_infos_[i].axis1.count_Nm_factor;
      // double torque_cmd_count2=-1 * module_infos_[i].axis2.effort_cmd * module_infos_[i].axis2.count_Nm_factor;

      // module_infos_[i].client_ptr->setAxis1TrqCnt(int16_t(torque_cmd_count1));
      // module_infos_[i].client_ptr->setAxis2TrqCnt(int16_t(torque_cmd_count2))
    }
  }

  void Elfin_Robot::test_six_axis_move()
  {
    // set to zero axis
    module_infos_[2].axis2.position_cmd = 0;

    // set to increase 0.1 rad about 5.7 degree
    module_infos_[2].axis2.position_cmd = module_infos_[2].axis2.position + 0.1;
  }

}   // end of namespace dr

#ifdef TEST_DR_ELFINROBOT

int main(int argc, char** argv)
{
  ros::init(argc,argv,"elfin_ethercat_driver_v2_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh("elfin_ethercat_driver_v2_node");

  // init the ethercat manager
  std::string ethernet_name;
  ethernet_name = nh.param<std::string>("elfin_ethernet_name", "eth0");
  elfin_ethercat_driver_v2::EtherCatManager em(ethernet_name);

  dr::Elfin_Robot robot(&em, nh);


  size_t counter = 0;

  for (counter = 0; counter < 5; counter++)
  {
    robot.state_update();
    usleep(1000000);
  }

  robot.enable_robot();

  for (counter = 0 ; counter < 10; counter++)
  {
    robot.state_update();
    usleep(1000000);
  }

  if (robot.setGroupPosMode() != true)
  {
    std::cout << "Set robot arm to postion mode is ERROR!\n";
  }
  for (counter = 0 ; counter < 10; counter++)
  {
    robot.state_update();
    usleep(1000000);
  }



  robot.disable_robot();

  for (counter = 0 ; counter < 10; counter++)
  {
    robot.state_update();
    usleep(1000000);
  }

  // while(1)
  // {
  //   robot.state_update();
  //   usleep(1000000);
  // }

  return 0;
}

#else

int main(int argc, char** argv)
{
    ros::init(argc,argv,"elfin_ethercat_driver_v2", ros::init_options::AnonymousName);

    ros::NodeHandle nh("elfin_ethercat_driver_v2");

    std::string ethernet_name;
    ethernet_name=nh.param<std::string>("elfin_ethernet_name", "eth0");

    elfin_ethercat_driver_v2::EtherCatManager em("eno1");
    elfin_ethercat_driver_v2::ElfinEtherCATDriver ed(&em, "elfin");

    ros::spin();
}

#endif
