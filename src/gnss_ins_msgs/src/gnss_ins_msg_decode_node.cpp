/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-01-31 14:20:34
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <can_msgs/msg/frame.hpp>
#include <mutex>
#include "gnss_ins_msgs/can_message/gnss_ins_can_msg_decode_motorola.hpp"


#define G 9.7879

using namespace std::chrono_literals;

class GnssSubscriber : public rclcpp::Node{

    public:
        GnssSubscriber() : Node("gnss_ins_sub") , accel_data_arrived_(false) , angrate_data_arrived_(false){
            gnss_ins_sub_ = this->create_subscription<can_msgs::msg::Frame>("from_can_bus1" , 1000 ,
                                                                            std::bind(&GnssSubscriber::callback , this , std::placeholders::_1));
        
            gnss_ins_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data" , 1000);
        }


    private:
        void callback(const can_msgs::msg::Frame::SharedPtr msg);
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr gnss_ins_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gnss_ins_pub_;
        bool accel_data_arrived_;
        bool angrate_data_arrived_;
        sensor_msgs::msg::Imu current_imu_data_;
        uint64_t time_gap_ = 0;


};


void gnss_ins_can_msgs_decode_test(const can_msgs::msg::Frame::SharedPtr msg){
    const uint32_t frame_id = msg->id;

    if(frame_id == ANG_RATE_RAW_IMU_FRAME_ID){
        gnss_ins::Ang_Rate_Raw_IMU angRateRawIMU;
        angRateRawIMU.set_data(msg->data);

        std::cout << "----------------- Ang_Rate_Raw_IMU message decode ------------------------" << std::endl;
        
        std::cout << "angRateRawIMU.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)angRateRawIMU.get_data_array().at(i) << " , ";
            else
                std::cout << (int)angRateRawIMU.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "angRateRawIMU.angraterawx = " << angRateRawIMU.angraterawx.raw_data() << " " << angRateRawIMU.angraterawx.unit() << std::endl;
        std::cout << "angRateRawIMU.angraterawy = " << angRateRawIMU.angraterawy.raw_data() << " " << angRateRawIMU.angraterawy.unit() << std::endl;
        std::cout << "angRateRawIMU.angraterawz = " << angRateRawIMU.angraterawz.raw_data() << " " << angRateRawIMU.angraterawz.unit() << std::endl;
    }

    if(frame_id == ACCEL_IMU_RAW_FRAME_ID){
        gnss_ins::Accel_IMU_Raw accelIMURaw;
        accelIMURaw.set_data(msg->data);
        
        std::cout << "----------------- Accel_IMU_Raw message decode ------------------------" << std::endl;
        
        std::cout << "accelIMURaw.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)accelIMURaw.get_data_array().at(i) << " , ";
            else
                std::cout << (int)accelIMURaw.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "accelIMURaw.accelrawx = " << accelIMURaw.accelrawx.raw_data() << " " << accelIMURaw.accelrawx.unit() << std::endl;
        std::cout << "accelIMURaw.accelrawy = " << accelIMURaw.accelrawy.raw_data() << " " << accelIMURaw.accelrawy.unit() << std::endl;
        std::cout << "accelIMURaw.accelrawz = " << accelIMURaw.accelrawz.raw_data() << " " << accelIMURaw.accelrawz.unit() << std::endl;

    }

    if(frame_id == LATITUDE_FRAME_ID){
        gnss_ins::Latitude latitude;
        latitude.set_data(msg->data);
        
        std::cout << "----------------- Latitude message decode ------------------------" << std::endl;
        
        std::cout << "latitude.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)latitude.get_data_array().at(i) << " , ";
            else
                std::cout << (int)latitude.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "latitude.poslat2 = " << latitude.poslat2.raw_data() << " " << latitude.poslat2.unit() << std::endl;
    
    }

    if(frame_id == LONGITUDE_FRAME_ID){
        gnss_ins::Longitude longtitude;
        longtitude.set_data(msg->data);
        
        std::cout << "----------------- Longitude message decode ------------------------" << std::endl;
        
        std::cout << "longtitude.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)longtitude.get_data_array().at(i) << " , ";
            else
                std::cout << (int)longtitude.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "longtitude.poslon2 = " << longtitude.poslon2.raw_data() << " " << longtitude.poslon2.unit() << std::endl;
    }

    if(frame_id == ANGRATEVEHICLE_FRAME_ID){
        gnss_ins::AngRateVehicle angRateVehicle;
        angRateVehicle.set_data(msg->data);
        
        std::cout << "----------------- AngRateVehicle message decode ------------------------" << std::endl;
        
        std::cout << "angRateVehicle.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)angRateVehicle.get_data_array().at(i) << " , ";
            else
                std::cout << (int)angRateVehicle.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "angRateVehicle.angratex = " << angRateVehicle.angratex.raw_data() << " " << angRateVehicle.angratex.unit() << std::endl;
        std::cout << "angRateVehicle.angratey = " << angRateVehicle.angratey.raw_data() << " " << angRateVehicle.angratey.unit() << std::endl;
        std::cout << "angRateVehicle.angratez = " << angRateVehicle.angratez.raw_data() << " " << angRateVehicle.angratez.unit() << std::endl;
    
    }

    if(frame_id == HEADINGPITCHROLLSIGMA_FRAME_ID){
        gnss_ins::HeadingPitchRollSigma headingPitchRollSigma;
        headingPitchRollSigma.set_data(msg->data);
        
        std::cout << "----------------- HeadingPitchRollSigma message decode ------------------------" << std::endl;
        
        std::cout << "headingPitchRollSigma.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)headingPitchRollSigma.get_data_array().at(i) << " , ";
            else
                std::cout << (int)headingPitchRollSigma.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "headingPitchRollSigma.angleheadingsigma = " << headingPitchRollSigma.angleheadingsigma.raw_data() << " " << headingPitchRollSigma.angleheadingsigma.unit() << std::endl;
        std::cout << "headingPitchRollSigma.anglepitchsigma = " << headingPitchRollSigma.anglepitchsigma.raw_data() << " " << headingPitchRollSigma.anglepitchsigma.unit() << std::endl;
        std::cout << "headingPitchRollSigma.anglerollsigma = " << headingPitchRollSigma.anglerollsigma.raw_data() << " " << headingPitchRollSigma.anglerollsigma.unit() << std::endl;
    }

    if(frame_id == HEADINGPITCHROLL_FRAME_ID){
        gnss_ins::HeadingPitchRoll headingPitchRoll;
        headingPitchRoll.set_data(msg->data);
        
        std::cout << "----------------- HeadingPitchRoll message decode ------------------------" << std::endl;
        
        std::cout << "headingPitchRoll.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)headingPitchRoll.get_data_array().at(i) << " , ";
            else
                std::cout << (int)headingPitchRoll.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "headingPitchRoll.angleheading = " << headingPitchRoll.angleheading.raw_data() << " " << headingPitchRoll.angleheading.unit() << std::endl;
        std::cout << "headingPitchRoll.anglepitch = " << headingPitchRoll.anglepitch.raw_data() << " " << headingPitchRoll.anglepitch.unit() << std::endl;
        std::cout << "headingPitchRoll.angleroll = " << headingPitchRoll.angleroll.raw_data() << " " << headingPitchRoll.angleroll.unit() << std::endl;

    }

    if(frame_id == ACCEL_VEHICLE_FRAME_ID){
        gnss_ins::Accel_Vehicle accelVehicle;
        accelVehicle.set_data(msg->data);
        
        std::cout << "----------------- Accel_Vehicle message decode ------------------------" << std::endl;
        
        std::cout << "accelVehicle.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)accelVehicle.get_data_array().at(i) << " , ";
            else
                std::cout << (int)accelVehicle.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "accelVehicle.accelx = " << accelVehicle.accelx.raw_data() << " " << accelVehicle.accelx.unit() << std::endl;
        std::cout << "accelVehicle.accely = " << accelVehicle.accely.raw_data() << " " << accelVehicle.accely.unit() << std::endl;
        std::cout << "accelVehicle.accelz = " << accelVehicle.accelz.raw_data() << " " << accelVehicle.accelz.unit() << std::endl;
        
    }

    if(frame_id == VELOCITYLEVELSIGMA_FRAME_ID){
        gnss_ins::VelocityLevelSigma velocityLevelSigma;
        velocityLevelSigma.set_data(msg->data);
        
        std::cout << "----------------- VelocityLevelSigma message decode ------------------------" << std::endl;
        
        std::cout << "velocityLevelSigma.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)velocityLevelSigma.get_data_array().at(i) << " , ";
            else
                std::cout << (int)velocityLevelSigma.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "velocityLevelSigma.velesigma = " << velocityLevelSigma.velesigma.raw_data() << " " << velocityLevelSigma.velesigma.unit() << std::endl;
        std::cout << "velocityLevelSigma.velnsigma = " << velocityLevelSigma.velnsigma.raw_data() << " " << velocityLevelSigma.velnsigma.unit() << std::endl;
        std::cout << "velocityLevelSigma.velusigma = " << velocityLevelSigma.velusigma.raw_data() << " " << velocityLevelSigma.velusigma.unit() << std::endl;
        std::cout << "velocityLevelSigma.velsigma = " << velocityLevelSigma.velsigma.raw_data() << " " << velocityLevelSigma.velsigma.unit() << std::endl;
        
    }

    if(frame_id == TIME_FRAME_ID){
        gnss_ins::Time time;
        time.set_data(msg->data);
        
        std::cout << "----------------- Time message decode ------------------------" << std::endl;
        
        std::cout << "time.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)time.get_data_array().at(i) << " , ";
            else
                std::cout << (int)time.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "time.gpsweek = " << time.gpsweek.raw_data() << " " << time.gpsweek.unit() << std::endl;
        std::cout << "time.gpstime = " << time.gpstime.raw_data() << " " << time.gpstime.unit() << std::endl;
    }

    if(frame_id == VELOCITYLEVEL_FRAME_ID){
        gnss_ins::VelocityLevel velocityLevel;
        velocityLevel.set_data(msg->data);
        
        std::cout << "----------------- VelocityLevel message decode ------------------------" << std::endl;
        
        std::cout << "velocityLevel.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)velocityLevel.get_data_array().at(i) << " , ";
            else
                std::cout << (int)velocityLevel.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "velocityLevel.vele = " << velocityLevel.vele.raw_data() << " " << velocityLevel.vele.unit() << std::endl;
        std::cout << "velocityLevel.veln = " << velocityLevel.veln.raw_data() << " " << velocityLevel.veln.unit() << std::endl;
        std::cout << "velocityLevel.velu = " << velocityLevel.velu.raw_data() << " " << velocityLevel.velu.unit() << std::endl;
        std::cout << "velocityLevel.vel = " << velocityLevel.vel.raw_data() << " " << velocityLevel.vel.unit() << std::endl;
    }

    if(frame_id == POSSIGMA_FRAME_ID){
        gnss_ins::PosSigma posSigma;
        posSigma.set_data(msg->data);
        
        std::cout << "----------------- PosSigma message decode ------------------------" << std::endl;
        
        std::cout << "posSigma.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)posSigma.get_data_array().at(i) << " , ";
            else
                std::cout << (int)posSigma.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "posSigma.posesigma = " << posSigma.posesigma.raw_data() << " " << posSigma.posesigma.unit() << std::endl;
        std::cout << "posSigma.posnsigma = " << posSigma.posnsigma.raw_data() << " " << posSigma.posnsigma.unit() << std::endl;
        std::cout << "posSigma.posusigma = " << posSigma.posusigma.raw_data() << " " << posSigma.posusigma.unit() << std::endl;

    }


    if(frame_id == ALTITUDE_FRAME_ID){
        gnss_ins::Altitude altitude;
        altitude.set_data(msg->data);
        
        std::cout << "----------------- Altitude message decode ------------------------" << std::endl;
        
        std::cout << "altitude.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)altitude.get_data_array().at(i) << " , ";
            else
                std::cout << (int)altitude.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "altitude.posalt = " << altitude.posalt.raw_data() << " " << altitude.posalt.unit() << std::endl;
 
    }

    if(frame_id == LATITUDELONGITUDE_FRAME_ID){
        gnss_ins::LatitudeLongitude latitudeLongitude;
        latitudeLongitude.set_data(msg->data);
        
        std::cout << "----------------- LatitudeLongitude message decode ------------------------" << std::endl;
        
        std::cout << "latitudeLongitude.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)latitudeLongitude.get_data_array().at(i) << " , ";
            else
                std::cout << (int)latitudeLongitude.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "latitudeLongitude.posalt = " << latitudeLongitude.poslat.raw_data() << " " << latitudeLongitude.poslat.unit() << std::endl;
        std::cout << "latitudeLongitude.poslon = " << latitudeLongitude.poslon.raw_data() << " " << latitudeLongitude.poslon.unit() << std::endl;

    }

    if(frame_id == INSSTATUS_FRAME_ID){
        gnss_ins::InsStatus insStatus;
        insStatus.set_data(msg->data);
        
        std::cout << "----------------- InsStatus message decode ------------------------" << std::endl;
        
        std::cout << "insStatus.data_array = [" ;
        for(int i = 0 ; i < 8 ; i++){

            if(i != 7)
                std::cout << (int)insStatus.get_data_array().at(i) << " , ";
            else
                std::cout << (int)insStatus.get_data_array().at(i) << "]" << std::endl;

        }

        std::cout << "insStatus.system_state = " << insStatus.system_state.raw_data() << " " << insStatus.system_state.unit() << std::endl;
        std::cout << "insStatus.gpsage = " << insStatus.gpsage.raw_data() << " " << insStatus.gpsage.unit() << std::endl;
        std::cout << "insStatus.gpsnumsats2 = " << insStatus.gpsnumsats2.raw_data() << " " << insStatus.gpsnumsats2.unit() << std::endl;
        std::cout << "insStatus.gpsnumsats2used = " << insStatus.gpsnumsats2used.raw_data() << " " << insStatus.gpsnumsats2used.unit() << std::endl;
        std::cout << "insStatus.gpsnumsats = " << insStatus.gpsnumsats.raw_data() << " " << insStatus.gpsnumsats.unit() << std::endl;
        std::cout << "insStatus.gpsnumsatsused = " << insStatus.gpsnumsatsused.raw_data() << " " << insStatus.gpsnumsatsused.unit() << std::endl;
        std::cout << "insStatus.satellite_status = " << insStatus.satellite_status.raw_data() << " " << insStatus.satellite_status.unit() << std::endl;

    }


}


double reg2rad(const double deg){ return deg / 180.0 * M_PI; }
uint64_t getTimeHost(void)
{
  std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
  std::chrono::system_clock::duration t_s = t.time_since_epoch();

  std::chrono::duration<uint64_t, std::ratio<1l, 1000000l>> t_us = 
    std::chrono::duration_cast<std::chrono::duration<uint64_t, std::ratio<1l, 1000000l>>>(t_s);
  return t_us.count();
}

void GnssSubscriber::callback(const can_msgs::msg::Frame::SharedPtr msg){
    // gnss_ins_can_msgs_decode_test(msg);
    
    const uint32_t frame_id = msg->id;
    if(frame_id == ANG_RATE_RAW_IMU_FRAME_ID){
        gnss_ins::Ang_Rate_Raw_IMU angRateRaw;
        angRateRaw.set_data(msg->data);

        time_gap_ = static_cast<uint64_t>(msg->header.stamp.sec * 1e9  + msg->header.stamp.nanosec);
        current_imu_data_.angular_velocity.x = reg2rad(angRateRaw.angraterawx.raw_data());
        current_imu_data_.angular_velocity.y = reg2rad(angRateRaw.angraterawy.raw_data());
        current_imu_data_.angular_velocity.z = reg2rad(angRateRaw.angraterawz.raw_data());

        angrate_data_arrived_ = true;
    }

    if(frame_id == ACCEL_IMU_RAW_FRAME_ID){
        gnss_ins::Accel_IMU_Raw accelIMURaw;
        accelIMURaw.set_data(msg->data);

        current_imu_data_.header.frame_id = "base_link";
        current_imu_data_.header.stamp = rclcpp::Time(getTimeHost() * 1.0e3);
        time_gap_ = static_cast<uint64_t>(msg->header.stamp.sec * 1e9  + msg->header.stamp.nanosec) - time_gap_;

        current_imu_data_.linear_acceleration.x = accelIMURaw.accelrawx.raw_data() * G;
        current_imu_data_.linear_acceleration.y = accelIMURaw.accelrawy.raw_data() * G;
        current_imu_data_.linear_acceleration.z = accelIMURaw.accelrawz.raw_data() * G;
        accel_data_arrived_ = true;
    }

    if(accel_data_arrived_ && angrate_data_arrived_){
        gnss_ins_pub_->publish(current_imu_data_);
        angrate_data_arrived_ = false;
        accel_data_arrived_ = false;
        // RCLCPP_INFO(this->get_logger() , "time_gap = %d (ns)" , time_gap_);
    }
}


int main(int argc , char* argv[]){
    rclcpp::init(argc , argv);
    rclcpp::spin(std::make_shared<GnssSubscriber>());
    rclcpp::shutdown();
    return 0;
}