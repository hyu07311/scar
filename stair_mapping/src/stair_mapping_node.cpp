#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"

class StairMappingNode : public rclcpp::Node {
public:
    StairMappingNode() : Node("stair_mapping_node") {
        // 1. 파라미터 선언 (Launch 파일 없이도 여기서 기본값 제어 가능)
        this->declare_parameter("voxel_leaf_size", 0.02); // 2cm
        this->declare_parameter("roi_max_distance", 3.0); // 3m까지 매핑
        this->declare_parameter("input_topic", "/camera/depth/color/points");

        // 2. 파라미터 값 읽기
        voxel_size_ = this->get_parameter("voxel_leaf_size").as_double();
        max_dist_ = this->get_parameter("roi_max_distance").as_double();
        std::string input_topic = this->get_parameter("input_topic").as_string();

        // 3. Pub/Sub 설정
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10,
            std::bind(&StairMappingNode::process_callback, this, std::placeholders::_1));
        
        // RTAB-Map이 구독할 전용 토픽
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);

        RCLCPP_INFO(this->get_logger(), "Stair Mapping C++ Node Initialized. Voxel Size: %.3f", voxel_size_);
    }

private:
    void process_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // ROS 메시지 -> PCL 변환
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*msg, *cloud);

        // [단계 1] Voxel Grid Filter (지도 정밀도 결정)
        pcl::PCLPointCloud2::Ptr cloud_voxeled(new pcl::PCLPointCloud2());
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter.filter(*cloud_voxeled);

        // [단계 2] PassThrough Filter (불필요한 원거리 데이터 제거)
        // RTAB-Map의 연산 부하를 줄이기 위해 필요한 영역만 자릅니다.
        pcl::PCLPointCloud2::Ptr cloud_final(new pcl::PCLPointCloud2());
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloud_voxeled);
        pass.setFilterFieldName("z"); // 카메라 기준 정면
        pass.setFilterLimits(0.1, max_dist_);
        pass.filter(*cloud_final);

        // PCL -> ROS 메시지 변환 및 발행
        sensor_msgs::msg::PointCloud2 output;
        pcl_conversions::fromPCL(*cloud_final, output);
        output.header = msg->header;
        publisher_->publish(output);
    }

    double voxel_size_;
    double max_dist_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StairMappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
