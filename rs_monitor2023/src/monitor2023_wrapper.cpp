/*** Qtのmainに当たる部分を書いてcomponent化する ***/
/* ROS2 */
#include <rclcpp/rclcpp.hpp>

/* C++ */
#include <memory> //スマートポインタ用
#include <thread>

/* Qt */
#include <QApplication>

/* monitorの動作 */
#include "rs_monitor2023/monitor2023.h"

namespace monitor2023
{

class ROS2WrapperMonitor2023 : public rclcpp::Node
{
public:
  ROS2WrapperMonitor2023(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~ROS2WrapperMonitor2023();

private:
  void run();
  std::unique_ptr<std::thread> thread_;
  // std::thread thread_;
};

ROS2WrapperMonitor2023::ROS2WrapperMonitor2023(rclcpp::NodeOptions options) : Node("rs_monitor2023_wrapper", options)
{
  // thread_ = std::thread(&ROS2WrapperMonitor2023::run);
  thread_ = std::make_unique<std::thread>(&ROS2WrapperMonitor2023::run, this);
  // thread_ = std::make_unique<std::thread>(&ROS2WrapperMonitor2023::run, this);
}

ROS2WrapperMonitor2023::~ROS2WrapperMonitor2023()
{
  // thread_.detach();
  thread_.release();
}

void ROS2WrapperMonitor2023::run()
{
  RCLCPP_INFO(this->get_logger(), "%s has started. thread id = %0x", this->get_name(), std::this_thread::get_id());

  /*** QApplicationクラスのオブジェクト生成のためにargcとargvを作る ***/
  int argc = 1;
  char** argv = new char*[1];
  argv[0] = "rs_monitor2023";

  /*** Qtのmain ***/
  QApplication app(argc, argv);
  Monitor2023 window(this->shared_from_this());
  window.show();
  app.exec();

  /*** メモリ解放 ***/
  delete argv[0];
  delete argv;

  RCLCPP_INFO(this->get_logger(), "%s has stopped.", this->get_name());
}

}
/*** ROS2WrapperMonitor2023クラスをコンポーネントとして登録 ***/
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(monitor2023::ROS2WrapperMonitor2023)