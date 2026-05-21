#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

class MapCleanerNode : public rclcpp::Node {
public:
    MapCleanerNode() : Node("map_cleaner_node") {
        // 파라미터 설정
        this->declare_parameter("mean_k", 50);             // 분석할 이웃 점의 개수
        this->declare_parameter("stddev_thresh", 1.0);     // 표준편차 배수 (낮을수록 엄격함)

        // 구독: 이전 단계에서 나온 필터링된 맵 데이터
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/filtered_points", 10,
            std::bind(&MapCleanerNode::callback, this, std::placeholders::_1));

        // 발행: 노이즈가 제거된 최종 데이터
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cleaned", 10);

        RCLCPP_INFO(this->get_logger(), "Statistical Outlier Removal Node Started.");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*msg, *cloud);

        if (cloud->width * cloud->height == 0) return;

        // Statistical Outlier Removal 실행
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(this->get_parameter("mean_k").as_int());
        sor.setStddevMulThresh(this->get_parameter("stddev_thresh").as_double());
        sor.filter(*cloud_filtered);

        sensor_msgs::msg::PointCloud2 output;
        pcl_conversions::fromPCL(*cloud_filtered, output);
        output.header = msg->header;
        pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapCleanerNode>());
    rclcpp::shutdown();
    return 0;
}
