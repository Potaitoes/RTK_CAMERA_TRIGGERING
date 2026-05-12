#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <cmath>
#include <algorithm>

// ===== CONFIG =====
const std::string VIDEO_DEV = "/dev/video0";
const double TARGET_BRIGHTNESS = 120.0;
const int MIN_EXPOSURE = 1;
const int MAX_EXPOSURE = 1000;
const double KP = 0.5;  // Proportional gain for control loop
const int CONTROL_INTERVAL_MS = 500;

// ===== CONTROL FUNCTIONS =====
void set_exposure(int exposure_value) {
    // Set auto_exposure to manual mode (1)
    std::string cmd1 = "v4l2-ctl -d " + VIDEO_DEV + " --set-ctrl=auto_exposure=1";
    int ret1 = system(cmd1.c_str());
    
    if (ret1 != 0) {
        std::cerr << "Warning: Failed to set auto_exposure mode" << std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Set exposure time
    std::string cmd2 = "v4l2-ctl -d " + VIDEO_DEV + " --set-ctrl=exposure_time_absolute=" + std::to_string(exposure_value);
    int ret2 = system(cmd2.c_str());
    
    if (ret2 == 0) {
        std::cout << "✅ Exposure set to " << exposure_value << std::endl;
    } else {
        std::cerr << "❌ Failed to set exposure" << std::endl;
    }
}

double measure_brightness(cv::VideoCapture& cap) {
    std::vector<double> brightnesses;
    
    for (int i = 0; i < 3; ++i) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Warning: Empty frame" << std::endl;
            continue;
        }
        
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::Scalar mean_val = cv::mean(gray);
        brightnesses.push_back(mean_val[0]);
    }
    
    if (brightnesses.size() == 3) {
        return *std::min_element(brightnesses.begin(), brightnesses.end());
    } else {
        return -1.0;  // Error indicator
    }
}

int main() {
    cv::VideoCapture cap(VIDEO_DEV);

    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open video device " << VIDEO_DEV << std::endl;
        return 1;
    }

    std::cout << "Starting autoexposure control loop..." << std::endl;
    std::cout << "Target brightness: " << TARGET_BRIGHTNESS << std::endl;
    
    int current_exposure = 167;  // Default exposure value
    set_exposure(current_exposure);

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(CONTROL_INTERVAL_MS));
        
        // Measure brightness
        double measured_brightness = measure_brightness(cap);
        
        if (measured_brightness < 0) {
            std::cout << "Warning: Could not measure brightness" << std::endl;
            continue;
        }
        
        // Calculate error
        double error = TARGET_BRIGHTNESS - measured_brightness;
        
        // Simple proportional control: adjust exposure based on error
        int exposure_adjustment = static_cast<int>(error * KP);
        int new_exposure = current_exposure + exposure_adjustment;
        
        // Clamp exposure within valid range
        new_exposure = std::max(MIN_EXPOSURE, std::min(MAX_EXPOSURE, new_exposure));
        
        std::cout << "Brightness: " << measured_brightness 
                  << " | Error: " << error 
                  << " | Exposure: " << new_exposure << std::endl;
        
        // Only update exposure if change is significant
        if (std::abs(new_exposure - current_exposure) > 1) {
            set_exposure(new_exposure);
            current_exposure = new_exposure;
        }
    }

    cap.release();
    return 0;
}