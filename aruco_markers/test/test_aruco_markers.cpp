#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <memory>
#include "aruco_markers/aruco_markers.hpp"
#include "aruco_markers/utils.hpp"

using namespace aruco_markers;

// Test fixture for ArucoMarkersNode
class ArucoMarkersNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<ArucoMarkersNode>();
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node->get_node_base_interface());
  }

  void TearDown() override
  {
    executor->remove_node(node->get_node_base_interface());
    executor.reset();
    node.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<ArucoMarkersNode> node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
};

// Test dictNameToEnum function
TEST_F(ArucoMarkersNodeTest, testDictNameToEnum)
{
  EXPECT_EQ(cv::aruco::DICT_ARUCO_ORIGINAL, utils::dictNameToEnum("DICT_ARUCO_ORIGINAL"));
  EXPECT_EQ(cv::aruco::DICT_4X4_50, utils::dictNameToEnum("DICT_4X4_50"));
  EXPECT_EQ(cv::aruco::DICT_5X5_100, utils::dictNameToEnum("DICT_5X5_100"));
  EXPECT_THROW(utils::dictNameToEnum("INVALID_DICT"), std::invalid_argument);
}

// Test isVec3dZero function
TEST_F(ArucoMarkersNodeTest, testIsVec3dZero)
{
  cv::Vec3d zero_vec(0.0, 0.0, 0.0);
  cv::Vec3d non_zero_vec(1.0, 0.0, 0.0);
  cv::Vec3d neg_vec(-1.0, 0.0, 0.0);

  EXPECT_TRUE(aruco_markers::utils::isVec3dZero(zero_vec));
  EXPECT_FALSE(aruco_markers::utils::isVec3dZero(non_zero_vec));
  EXPECT_FALSE(aruco_markers::utils::isVec3dZero(neg_vec));
}

// Test isVec3dZero function
TEST_F(ArucoMarkersNodeTest, testWaitForCameraInfo)
{
  // Log the node's camera_info_topic_ (assumes getter or public access)
  RCLCPP_INFO(node->get_logger(), "Node camera_info_topic: %s", node->camera_info_topic_.c_str());

  // Create publisher with node's topic
  auto publisher = node->create_publisher<sensor_msgs::msg::CameraInfo>(
    node->camera_info_topic_, 10); // Use node's topic

  // Populate CameraInfo message
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header.stamp = rclcpp::Clock().now();
  camera_info.width = 640;
  camera_info.height = 480;
  double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0;
  camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  double k1 = 0.1, k2 = 0.2, t1 = 0.01, t2 = 0.01, k3 = 0.0;
  camera_info.d = {k1, k2, t1, t2, k3};

  RCLCPP_INFO(node->get_logger(), "Starting thread to publish CameraInfo with delay");
  std::thread publish_thread([publisher, camera_info]() {
    // Introduce a delay before publishing
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      RCLCPP_INFO(rclcpp::get_logger("ArucoMarkersNodeTest"), "Publishing CameraInfo message");
      publisher->publish(camera_info);
    });

  // Immediately initialize in the main thread
  ASSERT_NO_THROW(node->initialize());

  // Spin to process
  executor->spin_some(std::chrono::milliseconds(100));

  // Ensure the publishing thread completes
  if (publish_thread.joinable()) {
    publish_thread.join();
  }

  EXPECT_TRUE(node->hasReceivedCameraInfo());
}

// Test process_camera_info function
TEST_F(ArucoMarkersNodeTest, testProcessCameraInfo)
{
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.width = 640;
  camera_info.height = 480;
  double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0;
  camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  double k1 = 0.1, k2 = 0.2, t1 = 0.01, t2 = 0.01, k3 = 0.0;
  camera_info.d = {k1, k2, t1, t2, k3};

  node->process_camera_info(camera_info);

  cv::Mat expected_camera_matrix =
    (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_DOUBLE_EQ(
          expected_camera_matrix.at<double>(i, j), node->getCameraMatrix().at<double>(
                                                       i,
                                                       j));
    }
  }

  cv::Mat expected_distortion = (cv::Mat_<double>(1, 5) << k1, k2, t1, t2, k3);
  for (int i = 0; i < 5; ++i) {
    EXPECT_DOUBLE_EQ(
        expected_distortion.at<double>(0, i),
        node->getCameraDistortion().at<double>(0, i));
  }

  EXPECT_TRUE(node->hasReceivedCameraInfo());
}

int main(int argc, char **argv)
{
  // rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  // rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}
