#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithm>
#include <cmath>

using std::placeholders::_1;

class AutonomousRobot : public rclcpp::Node
{
public:
  AutonomousRobot() : Node("autonomous_robot_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive/cmd_vel", 10);

    rclcpp::QoS qos_settings = rclcpp::SensorDataQoS();
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/diff_drive/scan", 
      qos_settings, 
      std::bind(&AutonomousRobot::scan_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Robot Démarré : Affichage de la distance en temps réel.");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    float min_front = 100.0;
    float min_left = 100.0;
    float min_right = 100.0;
    float min_back  = 100.0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        
        // On filtre les erreurs
        if (std::isinf(range)) continue; 
        if (range < msg->range_min || range > msg->range_max) continue;

        float angle = msg->angle_min + i * msg->angle_increment;

        // DEFINITION DES 4 ZONES (en Radians)
        
        // Zone AVANT : -45° à +45° (-0.78 à +0.78 rad)
        if (angle > -0.78 && angle < 0.78) {
             if (range < min_front) min_front = range;
        }
        // Zone GAUCHE : +45° à +135° (+0.78 à +2.35 rad)
        else if (angle > 0.78 && angle < 2.35) {
             if (range < min_left) min_left = range;
        }
        // Zone DROITE : -135° à -45° (-2.35 à -0.78 rad)
        else if (angle < -0.78 && angle > -2.35) {
             if (range < min_right) min_right = range;
        }
        // Zone ARRIERE : Tout le reste (les angles > 135° ou < -135°)
        else {
             if (range < min_back) min_back = range;
        }
    }

    // --- AFFICHAGE TEMPS REEL (POUR DEBUG) ---
    // Cela va s'afficher en continu pour que vous voyiez le robot approcher
    if (min_front < 2.0 || min_left < 1.0 || min_right < 1.0 || min_back < 1.0) {
        RCLCPP_INFO(this->get_logger(), 
            "SCAN -> Devant:%.1f  G:%.1f  D:%.1f  Arriere:%.1f", 
            min_front, min_left, min_right, min_back);
    }

    // --- DECISION ---
    auto vel = geometry_msgs::msg::Twist();
    float dist_stop = 2.0; 
    float dist_maintain = 0.8;

    // SECURITE CRITIQUE
    if (min_front < 0.5) {
        RCLCPP_ERROR(this->get_logger(), "TROP PROCHE (%.2f) ! ARRET URGENCE.", min_front);
        vel.linear.x = -0.2; // Recule un peu
    }
    // CAS 1 : MUR DEVANT
    else if (min_front < dist_stop) {
        
        // NOUVEAU : Vérification Arrière
        // Si bloqué devant ET bloqué derrière (< 0.5m), on ne recule pas, on tourne sur place !
        if (min_back < 1.5) {
             RCLCPP_ERROR(this->get_logger(), "PRIS EN SANDWICH ! Rotation sur place.");
             vel.linear.x = 0.0;
        }
        else {
             // Sinon, on a de la place derrière, on peut reculer un tout petit peu en tournant
             // (vitesse positive faible pour un virage en arc, ou négative pour reculer)
             vel.linear.x = 0.1; 
        }

        // Choix de direction
        if (min_left > min_right) vel.angular.z = 0.6;
        else vel.angular.z = -0.6;
    }
    // CAS 2 : MUR TROP PROCHE A DROITE -> CORRECTION
    else if (min_right < dist_maintain) {
        // On continue d'avancer, mais on braque à gauche pour s'éloigner
        RCLCPP_INFO(this->get_logger(), "Mur Droite (%.2f) -> Correction Gauche", min_right);
        vel.linear.x = 0.4;  
        vel.angular.z = 0.4; // Tourne gauche
    }
    // CAS 3 : MUR TROP PROCHE A GAUCHE -> CORRECTION
    else if (min_left < dist_maintain) {
        // On continue d'avancer, mais on braque à droite pour s'éloigner
        RCLCPP_INFO(this->get_logger(), "Mur Gauche (%.2f) -> Correction Droite", min_left);
        vel.linear.x = 0.4;   
        vel.angular.z = -0.4; // Tourne droite
    }
    // VOIE LIBRE
    else {
        vel.linear.x = 0.5; // Avance
        vel.angular.z = 0.0;
    }

    publisher_->publish(vel);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonomousRobot>());
  rclcpp::shutdown();
  return 0;
}