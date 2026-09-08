#include "utils.h"
#include <algorithm>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::pointCloud2ToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZI> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);

    const int stride = std::max(1, filter_num);
    cloud->reserve(input_cloud.size() / stride + 1);
    for (std::size_t i = 0; i < input_cloud.size(); i += static_cast<std::size_t>(stride))
    {
        const auto &source = input_cloud.points[i];
        if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z))
            continue;

        const float squared_range = source.x * source.x + source.y * source.y + source.z * source.z;
        if (squared_range < min_range * min_range || squared_range > max_range * max_range)
            continue;

        pcl::PointXYZINormal point;
        point.x = source.x;
        point.y = source.y;
        point.z = source.z;
        point.intensity = source.intensity;
        point.curvature = 0.0f;
        cloud->push_back(point);
    }
    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
