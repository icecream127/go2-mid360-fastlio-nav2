#include "go2_real_localization/gicp_localizer_node.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<go2_real_localization::GicpLocalizerNode>()); }
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("gicp_localizer"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
