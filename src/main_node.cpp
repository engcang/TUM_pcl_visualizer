#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <map>
#include <yaml-cpp/yaml.h> // YAML 라이브러리
#include <livox_ros_driver/CustomMsg.h>

// TUM CSV 데이터를 저장할 구조체
struct TumPose
{
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

// YAML 파일에서 extrinsic_R (3x3 행렬)과 extrinsic_T (3x1 벡터) 읽기
void loadExtrinsics(const std::string &config_file, Eigen::Matrix3d &extrinsic_R, Eigen::Vector3d &extrinsic_T, std::string &topic_name)
{
    YAML::Node config = YAML::LoadFile(config_file);

    if (!config["extrinsic_R"] || !config["extrinsic_R"].IsSequence() || config["extrinsic_R"].size() != 9)
    {
        throw std::runtime_error("Invalid extrinsic_R format in YAML file! Expected 9 elements.");
    }

    for (int i = 0; i < 9; ++i)
    {
        extrinsic_R(i / 3, i % 3) = config["extrinsic_R"][i].as<double>();
    }

    if (!config["extrinsic_T"] || !config["extrinsic_T"].IsSequence() || config["extrinsic_T"].size() != 3)
    {
        throw std::runtime_error("Invalid extrinsic_T format in YAML file! Expected 3 elements.");
    }

    for (int i = 0; i < 3; ++i)
    {
        extrinsic_T(i) = config["extrinsic_T"][i].as<double>();
    }


    topic_name = config["topic_name"].as<std::string>();

    // 디버깅용 출력
    std::cout << "extrinsic_R:" << std::endl;
    std::cout << extrinsic_R << std::endl;
    std::cout << "extrinsic_T: " << extrinsic_T.transpose() << std::endl;
}

// 변환 행렬 생성 함수 (TUM 포즈 → Eigen pose 변환 행렬)
Eigen::Matrix4d getTransformMatrix(const TumPose &pose, const Eigen::Matrix4d &extrinsic_transform)
{
    Eigen::Matrix4d pose_eig_out = Eigen::Matrix4d::Identity();

    // 1. 쿼터니언을 TF로 변환
    tf::Quaternion quat(pose.orientation.x(), pose.orientation.y(), pose.orientation.z(), pose.orientation.w());

    // 2. TF 행렬을 Eigen 행렬로 변환
    tf::Matrix3x3 tf_mat(quat);
    Eigen::Matrix3d tmp_rot_mat;
    tf::matrixTFToEigen(tf_mat, tmp_rot_mat);

    // 3. 회전 행렬 적용 (기존 회전에 extrinsic_R 추가 적용)
    pose_eig_out.block<3, 3>(0, 0) = tmp_rot_mat;

    // 4. 위치 변환 적용 (기존 위치에 extrinsic_R, extrinsic_T 적용)
    pose_eig_out.block<3, 1>(0, 3) = pose.position;

    // 5. Extrinsics 변환 행렬 적용

    return pose_eig_out * extrinsic_transform;
}

// PCL 다운샘플링 함수
pcl::PointCloud<pcl::PointXYZI> downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZI> &cloud, int filter_num)
{
    pcl::PointCloud<pcl::PointXYZI> filtered_cloud;
    filtered_cloud.reserve(cloud.size() / filter_num);

    for (size_t i = 0; i < cloud.size(); i += filter_num)
    {
        filtered_cloud.push_back(cloud[i]);
    }

    return filtered_cloud;
}

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

// Livox CustomMsg를 PCL 포인트 클라우드로 변환
pcl::PointCloud<pcl::PointXYZI> convertLivoxToPCL(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    size_t num_points = msg->point_num;
    pcl_cloud.reserve(num_points);

    for (size_t i = 1; i < num_points; i++)
    {
        if ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)
        {
            pcl::PointXYZI point;
            point.x = msg->points[i].x;
            point.y = msg->points[i].y;
            point.z = msg->points[i].z;
            point.intensity = msg->points[i].reflectivity;
            pcl_cloud.push_back(point);
        }
    }
    return pcl_cloud;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag_pcl_transformer");
    ros::NodeHandle nh;
    ros::Publisher pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/tum_pcl_visualize", 1);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/tum_path_visualize", 1);
    nav_msgs::Path tum_path;
    tum_path.header.frame_id = "map";

    // 명령줄 인자 받기
    if (argc != 8)
    {
        ROS_ERROR("Usage: rosrun package bag_pcl_transformer config_file bag_file csv_file publish_hz point_filter_num max_time_diff time_offset");
        return -1;
    }

    std::string config_file = argv[1];
    std::string bag_file = argv[2];
    std::string tum_csv = argv[3];
    double publish_hz = std::stod(argv[4]);
    int point_filter_num = std::stoi(argv[5]);
    double max_time_diff = std::stod(argv[6]);
    double time_offset = std::stod(argv[7]);

    // TUM CSV 데이터 로드
    auto tum_data = loadTumCsv(tum_csv);
    if (tum_data.empty())
    {
        ROS_ERROR("No TUM data loaded!");
        return -1;
    }

    if (point_filter_num < 1)
    {
        ROS_ERROR("Invalid point_filter_num! It must be >= 1");
        return -1;
    }

    // Config 파일에서 extrinsic_R, extrinsic_T, Topic name 로드
    Eigen::Matrix3d extrinsic_R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d extrinsic_T = Eigen::Vector3d::Zero();
    std::string pcl_topic;
    loadExtrinsics(config_file, extrinsic_R, extrinsic_T, pcl_topic);
    Eigen::Matrix4d extrinsic_transform = Eigen::Matrix4d::Identity();
    extrinsic_transform.block<3, 3>(0, 0) = extrinsic_R;
    extrinsic_transform.block<3, 1>(0, 3) = extrinsic_T;

    // ROSBag 읽기
    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(pcl_topic));

    // 메시지 타입 확인
    std::string topic_type;
    for (const auto &m : view)
    {
        topic_type = m.getDataType();
        break;
    }
    ROS_INFO("Detected topic type: %s", topic_type.c_str());

    for (const auto &m : view)
    {
        sensor_msgs::PointCloud2 transformed_msg;
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

        // 메시지 타입에 따라 처리
        if (topic_type == "sensor_msgs/PointCloud2")
        {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (!cloud_msg)
                continue;

            pcl::fromROSMsg(*cloud_msg, pcl_cloud);
            transformed_msg.header.stamp = cloud_msg->header.stamp;
        }
        else if (topic_type == "livox_ros_driver/CustomMsg")
        {
            livox_ros_driver::CustomMsg::ConstPtr livox_msg = m.instantiate<livox_ros_driver::CustomMsg>();
            if (!livox_msg)
                continue;

            pcl_cloud = convertLivoxToPCL(livox_msg);
            transformed_msg.header.stamp = ros::Time(livox_msg->timebase * 1e-9); // timestamp 변환
        }
        else
        {
            ROS_ERROR("Unsupported topic type: %s", topic_type.c_str());
            break;
        }

        double msg_time = transformed_msg.header.stamp.toSec();
        TumPose closest_pose;

        // offset 및 최대 허용 시간 차이를 반영하여 timestamp 찾기
        if (!findClosestPose(msg_time, time_offset, max_time_diff, tum_data, closest_pose))
        {
            ROS_WARN("Skipping pointcloud at %.6f: No matching TUM pose found", msg_time);
            continue;
        }

        // TF 변환을 사용하여 변환 행렬 생성
        Eigen::Matrix4d transform_matrix = getTransformMatrix(closest_pose, extrinsic_transform);

        // PCL 변환
        pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
        pcl::transformPointCloud(pcl_cloud, transformed_cloud, transform_matrix);
        pcl::PointCloud<pcl::PointXYZI> filtered_cloud = downsamplePointCloud(transformed_cloud, point_filter_num);

        // 변환된 PointCloud Publish
        pcl::toROSMsg(filtered_cloud, transformed_msg);
        transformed_msg.header.frame_id = "map";
        pub_pcl.publish(transformed_msg);

        // TUM Path Publish
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = transformed_msg.header.stamp;
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = closest_pose.position.x();
        pose_stamped.pose.position.y = closest_pose.position.y();
        pose_stamped.pose.position.z = closest_pose.position.z();
        pose_stamped.pose.orientation.x = closest_pose.orientation.x();
        pose_stamped.pose.orientation.y = closest_pose.orientation.y();
        pose_stamped.pose.orientation.z = closest_pose.orientation.z();
        pose_stamped.pose.orientation.w = closest_pose.orientation.w();

        tum_path.poses.push_back(pose_stamped);
        tum_path.header.stamp = pose_stamped.header.stamp;
        path_pub.publish(tum_path);

        // sleep
        ros::Duration(1.0 / publish_hz).sleep();
    }

    bag.close();
    return 0;
}
