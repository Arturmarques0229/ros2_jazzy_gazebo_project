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
    // 1. Publisher : Commande de vitesse
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive/cmd_vel", 10);

    // 2. Subscriber : Lecture du Lidar (QoS Best Effort)
    rclcpp::QoS qos_settings = rclcpp::SensorDataQoS();
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/diff_drive/scan", 
      qos_settings, 
      std::bind(&AutonomousRobot::scan_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Robot Autonome Démarré : Mode 'Anti-Piège' Actif.");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Initialisation des distances minimales par zone
    float min_front = 100.0;
    float min_left  = 100.0;
    float min_right = 100.0;
    float min_back  = 100.0;

    // --- 1. ANALYSE DU LIDAR (360°) ---
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        
        // Filtrage des données invalides
        if (std::isinf(range) || std::isnan(range)) continue; 
        if (range < msg->range_min || range > msg->range_max) continue;

        float angle = msg->angle_min + i * msg->angle_increment;

        // Répartition dans les 4 zones (sans trous)
        if (angle > -0.15 && angle < 0.15) {           // DEVANT (~60°)
             if (range < min_front) min_front = range;
        }
        else if (angle > 0.15 && angle < 2.5) {       // GAUCHE
             if (range < min_left) min_left = range;
        }
        else if (angle < -0.15 && angle > -2.5) {     // DROITE
             if (range < min_right) min_right = range;
        }
        else {                                       // ARRIERE
             if (range < min_back) min_back = range;
        }
    }

    RCLCPP_INFO(this->get_logger(), "A: %.2f| G: %.2f | D: %.2f| Ar: %.2f",min_front, min_left, min_right,min_back);

    // --- 2. LOGIQUE DE NAVIGATION ---
    auto vel = geometry_msgs::msg::Twist();
    
    // Seuils de distance
    float dist_emergency = 0.8; // Arrêt critique
    float dist_obstacle  = 1.4; // Détection d'obstacle devant
    float dist_wall      = 1.0; // Distance latérale idéale

    // CAS 0 : URGENCE ABSOLUE (Trop près devant)
    if (min_front < dist_emergency) {
        RCLCPP_ERROR(this->get_logger(), "URGENCE : Trop proche ! Recul.");
        vel.linear.x = -0.2; // Recule doucement
        vel.angular.z = 0.0;
    }
    // CAS 0-BIS : URGENCE ARRIERE (Si on recule dans un mur)
    else if (min_back < dist_emergency) {
        RCLCPP_WARN(this->get_logger(), "Attention arrière !");
        vel.linear.x = 0.1; // On avance un peu pour dégager l'arrière
        vel.angular.z = 0.0;
    }

    // CAS 1 : OBSTACLE DEVANT (Gestion des Coins)
    else if (min_front < dist_obstacle) {
        
        // --- DETECTION DE PIEGE (COIN) ---
        // Si mur devant ET murs proches sur les côtés, c'est un coin.
        bool trap_left  = (min_left < 1.25);
        bool trap_right = (min_right < 1.25);

        if (trap_left && trap_right) {
             // C'est un COIN !
             // REGLE D'OR : On n'avance SURTOUT PAS. On tourne sur place.
             RCLCPP_INFO(this->get_logger(), "COIN DETECTE : Rotation sur place !");
             
             vel.linear.x = 0.0;   // STOP AVANCE (Empêche l'oscillation vers le mur)
             
             vel.angular.z = 0.8;
             
        }
        else {
             // Obstacle simple (ex: un mur seul), on peut avancer doucement en tournant
             // pour faire un virage fluide
             RCLCPP_INFO(this->get_logger(), "Virage fluide.");
             vel.linear.x = 0.15;
             
             if (min_left > min_right) vel.angular.z = 0.6;
             else vel.angular.z = -0.6;
        }
    }
    
    // CAS 2 : SUIVI DE MUR A DROITE (Si voie libre devant)
    else if (min_right < dist_wall) {
        RCLCPP_INFO(this->get_logger(), "Mur Droite -> Correction");
        vel.linear.x = 0.4;  
        vel.angular.z = 0.3; // Tourne un peu à gauche
    }	
    // CAS 3 : SUIVI DE MUR A GAUCHE (Si voie libre devant)
    else if (min_left < dist_wall) {
        RCLCPP_INFO(this->get_logger(), "Mur Gauche -> Correction");
        vel.linear.x = 0.4;   
        vel.angular.z = -0.3; // Tourne un peu à droite
    }
    
    // CAS 4 : VOIE LIBRE
    else {
        vel.linear.x = 0.5; // Vitesse de croisière
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