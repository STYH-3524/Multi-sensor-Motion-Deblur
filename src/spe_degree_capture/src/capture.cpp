#include <memory>
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>
#include <filesystem> // 必须包含这个头文件
namespace fs = std::filesystem;

class SpeDegreeCaptureNode : public rclcpp::Node {
public:
    SpeDegreeCaptureNode() : Node("spe_degree_capture_node") {
        if (!InitCamera()) {
            RCLCPP_ERROR(this->get_logger(), "海康相机初始化或软触发设置失败！");
            return;
        }

        // 初始化视频生成器 (建议 FPS 设为 10)
        // 建议改用 XVID 编码，存为 .avi，这种格式即便程序崩溃也容易被修复播放
        video_writer_.open("/home/styh/rotation_panorama.avi", 
                        cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 
                        10, 
                        cv::Size(1280, 720));

        sub_angle_ = this->create_subscription<std_msgs::msg::Float32>(
            "/sim_yaw", 10, std::bind(&SpeDegreeCaptureNode::AngleCallback, this, std::placeholders::_1));

        last_trigger_yaw_ = -1.0;
        trigger_interval_ = 120.0; // 论文设定：120度采样实现360度覆盖
        RCLCPP_INFO(this->get_logger(), "节点启动成功！等待角度信号触发...");
    }

    ~SpeDegreeCaptureNode() {
        if (handle_) {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
        }
        video_writer_.release();
    }

private:
    bool InitCamera() {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        
        if (stDeviceList.nDeviceNum == 0) return false;
        MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[0]);
        MV_CC_OpenDevice(handle_);
        

        // 设置增益值（建议先从 10.0 开始尝试，最大值通常在 15.0 - 20.0 左右）
        // 注意：增益过大会产生噪点（雪花点）
        
        // 软触发配置
        MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
        MV_CC_SetEnumValue(handle_, "TriggerSource", 7); 
        
        // 设置为手动曝光并调大增益 (解决画面黑的问题)
        MV_CC_SetEnumValue(handle_, "ExposureAuto", 0); 
        MV_CC_SetFloatValue(handle_, "ExposureTime", 5000.0); // 室内建议先调到 5ms 测试
        MV_CC_SetEnumValue(handle_, "GainAuto", 0);
        MV_CC_SetFloatValue(handle_, "Gain", 50.0); // 调大增益

        return (MV_CC_StartGrabbing(handle_) == MV_OK);
    }

    void AngleCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        float current_yaw = msg->data;
        if (last_trigger_yaw_ < 0 || std::abs(current_yaw - last_trigger_yaw_) >= trigger_interval_) {
            RCLCPP_INFO(this->get_logger(), "触发拍摄角度: %.1f", current_yaw);
            
            // 1. 发送软触发指令
            if (MV_CC_SetCommandValue(handle_, "TriggerSoftware") == MV_OK) {
                // 2. 立即抓取并处理图像
                CaptureAndSave(current_yaw);
                last_trigger_yaw_ = current_yaw;
            }
        }
    }

    void CaptureAndSave(float yaw) {
        MV_FRAME_OUT stOutFrame = {0};
        if (MV_CC_GetImageBuffer(handle_, &stOutFrame, 1000) == MV_OK) {
            // 创建临时 BGR 缓冲区
            unsigned int nDataSize = stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3;
            unsigned char* pDataForBGR = (unsigned char*)malloc(nDataSize);

            MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
            stConvertParam.nWidth = stOutFrame.stFrameInfo.nWidth;
            stConvertParam.nHeight = stOutFrame.stFrameInfo.nHeight;
            stConvertParam.pSrcData = stOutFrame.pBufAddr;
            stConvertParam.nSrcDataLen = stOutFrame.stFrameInfo.nFrameLen;
            stConvertParam.enSrcPixelType = stOutFrame.stFrameInfo.enPixelType;
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            stConvertParam.pDstBuffer = pDataForBGR;
            stConvertParam.nDstBufferSize = nDataSize;

            if (MV_CC_ConvertPixelType(handle_, &stConvertParam) == MV_OK) {
                cv::Mat bgr_frame(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, CV_8UC3, pDataForBGR);
                
                // 【关键】强制缩放到 1280x720 以匹配 VideoWriter
                cv::Mat resized_frame;
                cv::resize(bgr_frame, resized_frame, cv::Size(1280, 720));

                // 保存图片到特定文件夹
                std::string filename = "/home/styh/quad_captures/yaw_" + std::to_string((int)yaw) + ".jpg";
                cv::imwrite(filename, resized_frame);
                
                // 写入视频流
                if (video_writer_.isOpened()) {
                    video_writer_.write(resized_frame);
                }
                RCLCPP_INFO(this->get_logger(), "⭐⭐ 已写入视频帧，角度: %.1f", yaw);
            }
            free(pDataForBGR);
            MV_CC_FreeImageBuffer(handle_, &stOutFrame);
        }
    }

    void* handle_ = nullptr;
    float last_trigger_yaw_;
    float trigger_interval_;
    cv::VideoWriter video_writer_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_angle_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeDegreeCaptureNode>());
    rclcpp::shutdown();
    return 0;
}