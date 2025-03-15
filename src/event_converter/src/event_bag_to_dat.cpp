#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <dvs_msgs/EventArray.h>
#include <fstream>

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "event_bag_to_dat");
    ros::NodeHandle nh;

    // 检查输入参数
    if (argc < 3) {
        ROS_ERROR("Usage: rosrun your_package event_bag_to_dat <input_bag_file> <output_dat_file>");
        return -1;
    }

    // 获取输入和输出文件路径
    std::string input_bag_file = argv[1];
    std::string output_dat_file = argv[2];

    // 打开 ROS bag 文件
    rosbag::Bag bag;
    try {
        bag.open(input_bag_file, rosbag::bagmode::Read);
    } catch (rosbag::BagException& e) {
        ROS_ERROR("Failed to open bag file: %s", e.what());
        return -1;
    }

    // 设置要读取的 topic
    std::vector<std::string> topics = {"/dvs/events"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // 打开输出文件
    std::ofstream dat_file(output_dat_file, std::ios::out | std::ios::binary);
    if (!dat_file.is_open()) {
        ROS_ERROR("Failed to open output .dat file");
        return -1;
    }

    // 遍历 bag 文件中的消息
    for (const rosbag::MessageInstance& msg : view) {
        // 检查消息类型是否为 dvs_msgs::EventArray
        dvs_msgs::EventArray::ConstPtr event_array = msg.instantiate<dvs_msgs::EventArray>();
        if (event_array != nullptr) {
            // 遍历所有事件
            for (const auto& event : event_array->events) {
                // 将事件数据写入 .dat 文件
                uint64_t timestamp = event.ts.toNSec();  // 将时间戳转换为纳秒
                uint16_t x = event.x;                   // x 坐标
                uint16_t y = event.y;                   // y 坐标
                uint8_t polarity = event.polarity ? 1 : 0; // 极性（True -> 1, False -> 0）

                // 写入二进制文件
                dat_file.write(reinterpret_cast<const char*>(&timestamp), sizeof(timestamp));
                dat_file.write(reinterpret_cast<const char*>(&x), sizeof(x));
                dat_file.write(reinterpret_cast<const char*>(&y), sizeof(y));
                dat_file.write(reinterpret_cast<const char*>(&polarity), sizeof(polarity));
            }
        }
    }

    // 关闭文件
    dat_file.close();
    bag.close();

    ROS_INFO("Conversion completed successfully. Output saved to %s", output_dat_file.c_str());
    return 0;
}