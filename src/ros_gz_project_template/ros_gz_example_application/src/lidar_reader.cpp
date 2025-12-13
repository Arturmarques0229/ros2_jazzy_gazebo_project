#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithm> // Pour std::min_element

class LidarReader : public rclcpp::Node
{
public:
  LidarReader() : Node("lidar_reader_node")
  {
    // Utilisation de SensorDataQoS pour être compatible "Best Effort" (très important pour le LiDAR)
    rclcpp::QoS qos_settings = rclcpp::SensorDataQoS();

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/diff_drive/scan", // Le nom de votre topic
      qos_settings,
      std::bind(&LidarReader::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Le radar est actif et écoute...");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Trouver la distance la plus courte parmi tous les rayons du laser
    // On ignore les valeurs infinies (qui signifient "rien en vue")
    float min_distance = 100.0; // Une valeur max arbitraire au départ

    for (float range : msg->ranges) {
        if (range < min_distance && range > msg->range_min) {
            min_distance = range;
        }
    }

    // Afficher l'alerte
    if (min_distance < 1.5) {
        // En ROUGE (Warn) si très proche
        RCLCPP_WARN(this->get_logger(), "DANGER ! Mur détecté à %.2f mètres !", min_distance);
    } else {
        // En VERT/BLANC (Info) si tout va bien
        RCLCPP_INFO(this->get_logger(), "Voie libre (Obstacle le plus proche : %.2f m)", min_distance);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarReader>());
  rclcpp::shutdown();
  return 0;
}