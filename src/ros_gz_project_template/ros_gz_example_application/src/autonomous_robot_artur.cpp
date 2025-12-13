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
    // 1. TROUVER LA MEILLEURE DIRECTION (L'ANTICIPATION)
    // On cherche le rayon le plus long (là où il y a le plus d'espace)
    
    float max_distance = -1.0;
    float best_angle = 0.0;
    float min_distance_overall = 100.0; // Pour la sécurité

    // On parcourt tout le lidar
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        float angle = msg->angle_min + i * msg->angle_increment;

        // Nettoyage des données
        if (std::isinf(range) || std::isnan(range)) range = msg->range_max;
        
        // Sécurité : on retient l'obstacle le plus proche absolu
        if (range < min_distance_overall) min_distance_overall = range;

        // Stratégie d'Anticipation :
        // On cherche le point le plus loin, MAIS on privilégie ce qui est devant nous.
        // On applique une "pénalité" aux angles arrière pour éviter que le robot 
        // ne veuille faire demi-tour dès qu'il voit le fond de la pièce derrière lui.
        
        // Facteur de poids : 1.0 devant, diminue sur les côtés
        float weight = 1.0 - (std::abs(angle) / M_PI); 
        
        // Score = Distance * Poids directionnel
        float score = range * weight;

        if (score > max_distance) {
            max_distance = score;
            best_angle = angle;
        }
    }

    auto vel = geometry_msgs::msg::Twist();

    // 2. DECISION (PRIORITE SECURITE vs ANTICIPATION)

    // A. SECURITE CRITIQUE (Réflexe de survie)
    // Si quoi que ce soit est à moins de 0.5m, on oublie l'anticipation, on freine !
    if (min_distance_overall < 0.5) {
        RCLCPP_ERROR(this->get_logger(), "Urgence ! Obstacle à %.2f m", min_distance_overall);
        vel.linear.x = -0.1; // Recule doucement
        vel.angular.z = 0.0;
    }
    // B. NAVIGATION FLUIDE (Anticipation)
    else {
        // Le robot a choisi son cap ("best_angle"). 
        // Il tourne vers ce cap de manière proportionnelle.
        
        // Plus l'angle est grand, plus on tourne fort (P-Controller)
        // Le facteur 2.0 est le "gain" (la nervosité du robot)
        vel.angular.z = best_angle * 1.5; 

        // Gestion intelligente de la vitesse :
        // Si on doit tourner beaucoup, on ralentit. Si c'est tout droit, on fonce.
        // Vitesse max 0.5, diminue si on tourne
        vel.linear.x = 0.5 * (1.0 - std::abs(best_angle) / M_PI);
        
        // On garde une vitesse mini pour ne pas s'arrêter
        if (vel.linear.x < 0.1) vel.linear.x = 0.1;

        RCLCPP_INFO(this->get_logger(), "Cible: %.2f rad | Vitesse: %.2f", best_angle, vel.linear.x);
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