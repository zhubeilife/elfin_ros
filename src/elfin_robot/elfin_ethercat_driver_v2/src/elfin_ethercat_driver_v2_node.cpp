#include <elfin_ethercat_driver_v2/elfin_ethercat_driver_v2.h>

namespace dr
{

class Elfin_Robot
{
private:
  /* data */
  ros::NodeHandle nh_;
  elfin_ethercat_driver_v2::ElfinEtherCATDriver *p_ethercat_drivers_;

public:
  Elfin_Robot(elfin_ethercat_driver_v2::EtherCatManager *manager,
      const ros::NodeHandle &nh=ros::NodeHandle("~"));
  ~Elfin_Robot();
};

Elfin_Robot::Elfin_Robot(elfin_ethercat_driver_v2::EtherCatManager *manager,
    const ros::NodeHandle &nh):
    nh_(nh)
{
  p_ethercat_drivers_ = new elfin_ethercat_driver_v2::ElfinEtherCATDriver(manager, "elfin");
}

Elfin_Robot::~Elfin_Robot()
{
  if (p_ethercat_drivers_ != NULL)
    delete p_ethercat_drivers_;
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

    ros::spin();
}