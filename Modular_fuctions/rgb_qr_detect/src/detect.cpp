#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <std_msgs/Int32.h>   // 发布整数所需头文件
#include <std_msgs/String.h>  // 发布字符串所需头文件
#include <map>
#include <vector>
#include <string>
#include <regex>

// 定义颜色范围的辅助结构体
struct ColorMask {
    cv::Scalar lower;
    cv::Scalar upper;
};

// 全局颜色范围配置
std::map<std::string, std::vector<ColorMask>> COLOR_RANGES = {
    {"Red",    {{{0, 70, 50}, {10, 255, 255}}, {{170, 70, 50}, {180, 255, 255}}}},
    {"Green",  {{{35, 50, 50}, {77, 255, 255}}}},
    {"Blue",   {{{100, 80, 50}, {130, 255, 255}}}}
};

// 颜色名称到代码的映射
std::map<std::string, int> COLOR_MAP = {{"Red", 0}, {"Green", 1}, {"Blue", 2}};

// 1. 二维码检测函数
std::string detect_qrcode(cv::Mat& frame) {
    cv::QRCodeDetector qr_detector;
    std::vector<cv::Point> points;
    
    std::string data = qr_detector.detectAndDecode(frame, points);
    
    if (!data.empty() && points.size() == 4) {
        // 绘制二维码边框
        for (int i = 0; i < 4; i++) {
            cv::line(frame, points[i], points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }
        // 绘制二维码内容文本
        cv::putText(frame, data, points[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }
    return data;
}

// 2. 圆形检测函数
int detect_circle(cv::Mat& frame, const std::map<std::string, std::vector<ColorMask>>& ranges, 
                  const cv::Mat& morph_kernel, int min_area, double min_circularity, 
                  cv::Point& out_center) {
    
    cv::Mat blurred, hsv;
    cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    std::string detected_name = "None";
    out_center = cv::Point(-1, -1);
    int img_h = frame.rows;
    int img_w = frame.cols;

    int box_size = static_cast<int>(std::min(img_w, img_h) * 0.25);
    int x1 = img_w / 2 - box_size / 2;
    int y1 = img_h / 2 - box_size / 2;
    int x2 = x1 + box_size;
    int y2 = y1 + box_size;

    cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 255), 2);

    for (const auto& pair : ranges) {
        const std::string& color_name = pair.first;
        const std::vector<ColorMask>& masks = pair.second;

        cv::Mat full_mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
        for (const auto& m : masks) {
            cv::Mat temp_mask;
            cv::inRange(hsv, m.lower, m.upper, temp_mask);
            cv::bitwise_or(full_mask, temp_mask, full_mask);
        }

        // 形态学操作
        cv::Mat morph_mask;
        cv::morphologyEx(full_mask, morph_mask, cv::MORPH_OPEN, morph_kernel);
        cv::morphologyEx(morph_mask, morph_mask, cv::MORPH_CLOSE, morph_kernel);

        // 寻找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(morph_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> best_cnt;
        double max_area = min_area;

        for (const auto& cnt : contours) {
            double area = cv::contourArea(cnt);
            if (area < max_area) continue;

            double perimeter = cv::arcLength(cnt, true);
            if (perimeter == 0) continue;

            double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
            if (circularity < min_circularity) continue;

            best_cnt = cnt;
            max_area = area;
        }

        if (!best_cnt.empty()) {
            cv::Point2f center_f;
            float radius_f;
            cv::minEnclosingCircle(best_cnt, center_f, radius_f);

            if (radius_f < 20) continue;

            int cx = static_cast<int>(center_f.x);
            int cy = static_cast<int>(center_f.y);
            int r = static_cast<int>(radius_f);

            if (cx - r > 2 && cx + r < img_w - 2 && cy - r > 2 && cy + r < img_h - 2) {
                if (cx >= x1 && cx <= x2 && cy >= y1 && cy <= y2) {
                    out_center = cv::Point(cx, cy);
                    detected_name = color_name;

                    cv::circle(frame, out_center, r, cv::Scalar(0, 255, 0), 2);
                    cv::circle(frame, out_center, 3, cv::Scalar(0, 0, 255), -1);
                    break; // 找到符合要求的颜色，跳出循环
                }
            }
        }
    }

    if (COLOR_MAP.find(detected_name) != COLOR_MAP.end()) {
        return COLOR_MAP[detected_name];
    }
    return -1;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_detector");
    ros::NodeHandle nh;

    // --- 创建话题发布者 ---
    // 队列大小设为 10，足以应对 30Hz 的发布频率
    ros::Publisher color_pub = nh.advertise<std_msgs::Int32>("/detected_color", 10);
    ros::Publisher target_x_pub = nh.advertise<std_msgs::Int32>("/target_x", 10);
    ros::Publisher target_y_pub = nh.advertise<std_msgs::Int32>("/target_y", 10);
    ros::Publisher qr_data_pub = nh.advertise<std_msgs::String>("/qr_data", 10);

    cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));

    // 打开摄像头
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FPS, 30);

    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera!");
        return -1;
    }

    ros::Rate rate(30); // 30Hz 循环
    int color_code = -1;

    while (ros::ok()) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            ROS_WARN("Empty frame grabbed.");
            break;
        }

        // --- 性能优化核心：提前缩放图像 ---
        int process_width = 640;
        if (frame.cols > process_width) {
            double scale = static_cast<double>(process_width) / frame.cols;
            cv::resize(frame, frame, cv::Size(process_width, static_cast<int>(frame.rows * scale)));
        }

        // 1. 二维码识别
        std::string qrcode_data = detect_qrcode(frame);
        if (!qrcode_data.empty()) {
            // 发布二维码原始字符串
            std_msgs::String qr_msg;
            qr_msg.data = qrcode_data;
            qr_data_pub.publish(qr_msg);
            
            // 解析 (1,2) 格式
            std::regex pattern(R"(\((\d+),(\d+)\))");
            std::smatch match;
            if (std::regex_match(qrcode_data, match, pattern)) {
                int target_x = std::stoi(match[1].str());
                int target_y = std::stoi(match[2].str());
                
                // 发布 X 和 Y 坐标
                std_msgs::Int32 x_msg, y_msg;
                x_msg.data = target_x;
                y_msg.data = target_y;
                target_x_pub.publish(x_msg);
                target_y_pub.publish(y_msg);

                ROS_INFO("二维码内容: %s -> 已发送坐标 X=%d, Y=%d", qrcode_data.c_str(), target_x, target_y);
            }
        }

        // 2. 圆形识别
        cv::Point center;
        color_code = detect_circle(frame, COLOR_RANGES, morph_kernel, 1200, 0.65, center);
        
        // 发布颜色代码
        std_msgs::Int32 color_msg;
        color_msg.data = color_code;
        color_pub.publish(color_msg);

        // 显示结果
        cv::imshow("RGB & QR Detection", frame);

        // 退出机制
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 'q' || key == 27) { // 'q' 或 ESC 键退出
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}