#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <map>

// TUM CSV 데이터를 저장할 구조체
struct TumPose
{
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

// TUM CSV를 읽어서 map에 저장
std::map<double, TumPose> loadTumCsv(const std::string &filename)
{
    std::map<double, TumPose> tum_data;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open TUM CSV file!");
        return tum_data;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        double t, px, py, pz, qx, qy, qz, qw;
        ss >> t >> px >> py >> pz >> qx >> qy >> qz >> qw;

        TumPose pose;
        pose.timestamp = t;
        pose.position = Eigen::Vector3d(px, py, pz);
        pose.orientation = Eigen::Quaterniond(qw, qx, qy, qz);
        tum_data[t] = pose;
    }
    file.close();
    return tum_data;
}

// 가장 가까운 timestamp를 찾는 함수 (offset & 최대 시간 차이 적용)
bool findClosestPose(double timestamp, double offset, double max_time_diff, const std::map<double, TumPose> &tum_data, TumPose &closest_pose)
{
    double query_time = timestamp + offset;
    auto it = tum_data.lower_bound(query_time);

    if (it == tum_data.end())
        return false;
    if (it == tum_data.begin())
    {
        if (std::abs(it->first - query_time) > max_time_diff)
            return false;
        closest_pose = it->second;
        return true;
    }

    auto prev_it = std::prev(it);
    if (std::abs(prev_it->first - query_time) > max_time_diff && std::abs(it->first - query_time) > max_time_diff)
        return false;

    closest_pose = (std::abs(it->first - query_time) < std::abs(prev_it->first - query_time)) ? it->second : prev_it->second;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag_pcl_transformer");
    ros::NodeHandle nh;
    ros::Publisher pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/transformed_pcl", 1);

    // 명령줄 인자 받기: bag_file, csv_file, topic_name, time_offset
    if (argc < 5)
    {
        ROS_ERROR("Usage: rosrun package bag_pcl_transformer bag_file csv_file topic_name time_offset");
        return -1;
    }

    std::string bag_file = argv[1];
    std::string tum_csv = argv[2];
    std::string pcl_topic = argv[3];
    double time_offset = std::stod(argv[4]);
    double max_time_diff = 0.05; // 최대 허용 시간 차이 (예: 50ms)

    // TUM CSV 데이터 로드
    auto tum_data = loadTumCsv(tum_csv);
    if (tum_data.empty())
    {
        ROS_ERROR("No TUM data loaded!");
        return -1;
    }

    // ROSBag 읽기
    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(pcl_topic));

    for (const auto &m : view)
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (!cloud_msg)
            continue;

        double msg_time = cloud_msg->header.stamp.toSec();
        TumPose closest_pose;

        // offset 및 최대 허용 시간 차이를 반영하여 timestamp 찾기
        if (!findClosestPose(msg_time, time_offset, max_time_diff, tum_data, closest_pose))
        {
            ROS_WARN("Skipping pointcloud at %.6f: No matching TUM pose found", msg_time);
            continue; // 최대 시간 차이를 넘으면 publish하지 않음
        }

        // 변환 행렬 생성 (TUM 좌표 -> world 좌표)
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.translation() = closest_pose.position;
        transform.linear() = closest_pose.orientation.toRotationMatrix();

        // PCL 변환
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud, transformed_cloud;
        pcl::fromROSMsg(*cloud_msg, pcl_cloud);
        pcl::transformPointCloud(pcl_cloud, transformed_cloud, transform);

        // 변환된 PointCloud Publish
        sensor_msgs::PointCloud2 transformed_msg;
        pcl::toROSMsg(transformed_cloud, transformed_msg);
        transformed_msg.header.frame_id = "world";
        transformed_msg.header.stamp = cloud_msg->header.stamp;
        pub_pcl.publish(transformed_msg);

        // 1ms sleep
        ros::Duration(0.001).sleep();
    }

    bag.close();
    return 0;
}
