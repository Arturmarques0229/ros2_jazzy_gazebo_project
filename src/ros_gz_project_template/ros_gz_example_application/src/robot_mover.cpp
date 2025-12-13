#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class RobotMover : public rclcpp::Node
{
public:
  RobotMover() : Node("robot_mover_node")
  {
    // Création du publisher sur le topic 'cmd_vel'
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_drive/cmd_vel", 10);
    
    // Timer pour appeler la fonction de mouvement toutes les 500ms (2Hz)
    timer_ = this->create_wall_timer(
      500ms, std::bind(&RobotMover::move_straight, this));
      
    RCLCPP_INFO(this->get_logger(), "Le robot commence à avancer !");
  }

private:
  void move_straight()
  {
    auto message = geometry_msgs::msg::Twist();

    // --- CONFIGURATION DU MOUVEMENT ---
    
    // Pour avancer droit : on met de la vitesse sur l'axe X (avant/arrière)
    message.linear.x = 0.5;  // 0.5 m/s vers l'avant
    message.linear.y = 0.0;  // 0.0 pour un robot à entraînement différentiel
    message.linear.z = 0.0;

    // Pour ne pas tourner : on s'assure que la rotation Z est à 0
    message.angular.x = 0.0;
    message.angular.y = 0.0;
    message.angular.z = 0.0; 

    // Publication du message
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotMover>());
  rclcpp::shutdown();
  return 0;
}