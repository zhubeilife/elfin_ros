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
};

Elfin_Robot::Elfin_Robot(elfin_ethercat_driver_v2::EtherCatManager *manager,
    const ros::NodeHandle &nh):
    nh_(nh)
{
  // get the ethercat drvier pointer
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

    while(1)
    {
      robot.state_update();
      usleep(1000000);
    }

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

