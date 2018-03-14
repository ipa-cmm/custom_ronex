

#ifndef _CUSTOM_SPI_CONTROLLER_H
#define _CUSTOM_SPI_CONTROLLER_H

#include <ros/node_handle.h>

#include <controller_interface/controller.h>
#include <ros_ethercat_model/robot_state_interface.hpp>
#include <custom_ronex_interface/custom_ronex_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <sr_ronex_utilities/sr_ronex_utilities.hpp>
#include <queue>
#include <utility>
#include <string>
#include <vector>
#include <boost/circular_buffer.hpp>

#include <string>
#include <sstream>

namespace ronex {

class customController : public controller_interface::Controller<ros_ethercat_model::RobotStateInterface>{
public:
std::string s;
std::stringstream ss;
double analogIN[6];
ronex::sensorData sens[4];
int count;
ronex::CustomRonex *joint_;

// realtime_tools::RealtimePublisher<customMsgs::analogMsgs> *realtime_pub;

/******************
***** Methods ****
******************/
bool init(ros_ethercat_model::RobotStateInterface* hw, ros::NodeHandle &n)
{
        count = 0;
        std::string joint_name;
        //joint_ = hw;

        std::string ronex_id;
        if (!n.getParam("ronex_id", ronex_id))
        {
                ROS_ERROR("No RoNeX ID given (namespace: %s)", n.getNamespace().c_str());
                return false;
        }

        std::string path;
        int parameter_id = get_ronex_param_id(ronex_id);
        {
                if ( parameter_id == -1 )
                {
                        ROS_ERROR_STREAM("Could not find the RoNeX id in the parameter server: " << ronex_id <<
                                         " not loading the controller.");
                        return false;
                }
                else
                {
                        std::stringstream ss;
                        ss << "/ronex/devices/" << parameter_id << "/path";
                        if ( !ros::param::get(ss.str(), path) )
                        {
                                ROS_ERROR_STREAM("Couldn't read the parameter " << ss.str() <<
                                                 " from the parameter server. Not loading the controller.");
                                return false;
                        }
                }
        }

        std::string robot_state_name;
        n.param<std::string>("robot_state_name", robot_state_name, "unique_robot_hw");
        ROS_INFO_STREAM("RobotStateName: " << robot_state_name);

        try
        {
                joint_ = static_cast<ronex::CustomRonex*>(hw->getHandle(robot_state_name).getState()->getCustomHW(path));
        }
        catch(const hardware_interface::HardwareInterfaceException& e)
        {
                ROS_ERROR_STREAM("Could not find robot state: " << robot_state_name << " Not loading the controller. " << e.what());
                return false;
        }
        clock_gettime(CLOCK_MONOTONIC, &this->no);

        //realtime_pub = new realtime_tools::RealtimePublisher<customMsgs::analogMsgs>(n,"AnalogDataMessage",25);

        s = " ";
        count = 0;
        return true;
}

void update(const ros::Time& time, const ros::Duration& period)
{

        //get Sensorvalues
        joint_->setDigitalOut(true,0);
        joint_->setDigitalOut(true,1);
        joint_->setDigitalOut(true,2);
        joint_->setDigitalOut(true,3);
        joint_->setDigitalOut(false,4);
        joint_->setDigitalOut(false,5);

        //joint_->setCommand(255,2);

        if(count%(5*1000)==0) {
                /*
                   for(int i = 0; i<6; ++i)
                          ss << joint_->sens.analog[i] << " ";
                 */
                sens[0] = joint_->getSensorData(0);
                sens[1] = joint_->getSensorData(1);
                sens[2] = joint_->getSensorData(2);
                sens[3] = joint_->getSensorData(3);

                uint16_t dio = 0;
                if (joint_->dgO[0]) dio |= PIN_OUTPUT_STATE_DIO_0;
                if (joint_->dgO[1]) dio |= PIN_OUTPUT_STATE_DIO_1;
                if (joint_->dgO[2]) dio |= PIN_OUTPUT_STATE_DIO_2;
                if (joint_->dgO[3]) dio |= PIN_OUTPUT_STATE_DIO_3;
                if (joint_->dgO[4]) dio |= PIN_OUTPUT_STATE_DIO_4;
                if (joint_->dgO[5]) dio |= PIN_OUTPUT_STATE_DIO_5;

                ss << "dio: " << std::setw(10) << dio;
                s = ss.str();
                ROS_INFO_STREAM(s);
                s.clear();
                ss.str("");


                if(joint_->dgO[0]){
                  ss << "DigitalOut[0]: " << std::setw(10) << joint_->dgO[0];
                  s = ss.str();
                  ROS_INFO_STREAM(s);
                  s.clear();
                  ss.str("");
                }
                if(joint_->dgO[1]){
                  ss << "DigitalOut[1]: " << std::setw(10) << joint_->dgO[1];
                  s = ss.str();
                  ROS_INFO_STREAM(s);
                  s.clear();
                  ss.str("");
                }


                ss << "timestp: " << std::setw(10) << sens[0].timestamp << " | " << std::setw(10) << sens[1].timestamp << " | " << std::setw(10) << sens[2].timestamp << " | " << std::setw(10) << sens[3].timestamp;
                s = ss.str();
                ROS_INFO_STREAM(s);
                s.clear();
                ss.str("");

                ss << "sensor0: " << std::setw(10) << sens[0].sensor0 << " | " << std::setw(10) << sens[1].sensor0 << " | " << std::setw(10) << sens[2].sensor0 << " | " << std::setw(10) << sens[3].sensor0;
                s = ss.str();
                ROS_INFO_STREAM(s);
                s.clear();
                ss.str("");

                ss << "sensor1: " << std::setw(10) << sens[0].sensor1 << " | " << std::setw(10) << sens[1].sensor1 << " | " << std::setw(10) << sens[2].sensor1 << " | " << std::setw(10) << sens[3].sensor1;
                s = ss.str();
                ROS_INFO_STREAM(s);
                s.clear();
                ss.str("");

                ss << "sensor2: " << std::setw(10) << sens[0].sensor2 << " | " << std::setw(10) << sens[1].sensor2 << " | " << std::setw(10) << sens[2].sensor2 << " | " << std::setw(10) << sens[3].sensor2 << "\n";
                s = ss.str();
                ROS_INFO_STREAM(s);
                s.clear();
                ss.str("");
        }
        clock_gettime(CLOCK_MONOTONIC, &this->no);
        count++;

}

void starting(const ros::Time& time) {
}
void stopping(const ros::Time& time) {
}

private:
struct timespec no;

};
}

#endif /* _SPI_BASE_CONTROLLER_H_ */
