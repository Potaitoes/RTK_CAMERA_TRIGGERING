#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <signal.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <libserialport.h>

#include "ubx_parser.h"

// ===== CONFIG =====
constexpr int JPEG_QUALITY = 85;
constexpr const char* VIDEO_DEV = "/dev/video0";
constexpr const char* RTK_PORT = "/dev/ttyACM0";
constexpr int RTK_BAUD = 115200;
constexpr const char* OUTPUT_DIR = "./recordings";
constexpr int CAMERA_BUFFER_SIZE = 1;
constexpr int POST_TRIGGER_FLUSH_FRAMES = 8;
constexpr int POST_TRIGGER_FLUSH_TIMEOUT_MS = 3000;
constexpr double GPS_UNIX_EPOCH = 315964800.0;  // 1980-01-06 00:00:00 UTC
constexpr double GPS_WEEK_SECONDS = 604800.0;
constexpr double NS_TO_S = 1e-9;

// ===== GLOBALS =====
std::atomic<bool> stop_flag(false);
std::atomic<bool> stream_ready(false);
std::atomic<bool> frame_logging_enabled(false);
std::atomic<bool> post_trigger_flush_done(false);
std::atomic<uint64_t> frame_count(0);
std::atomic<int> discard_frames_remaining(0);

std::queue<std::pair<std::string, cv::Mat>> save_queue;
std::mutex save_queue_mtx;
std::condition_variable save_queue_cv;

int tm2_count = 0;

int64_t monotonic_time_us() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts); 
    return static_cast<int64_t>(ts.tv_sec) * 1000000LL
         + static_cast<int64_t>(ts.tv_nsec) / 1000LL;
}

double tm2_rising_edge_unix_seconds(const TIM_TM2& tm2) {
    return GPS_UNIX_EPOCH
         + static_cast<double>(tm2.wnR) * GPS_WEEK_SECONDS
         + static_cast<double>(tm2.towMsR) / 1000.0
         + static_cast<double>(tm2.towSubMsR) * NS_TO_S;
}

// ===== SIGNAL HANDLER =====
void signal_handler(int sig) {
    std::cout << "\n[Main] Stopping..." << std::endl;
    stop_flag = true;
}

// ===== CAMERA THREAD =====
void camera_thread(const std::string& video_dev, const std::string& frames_dir, const std::string& frames_csv_path) {
    cv::VideoCapture cap(video_dev, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        int dev_index = 0;
        try {
            size_t pos = video_dev.find_last_not_of("0123456789");
            dev_index = std::stoi(video_dev.substr(pos + 1));
        } catch (...) {}
        cap.open(dev_index, cv::CAP_V4L2);
    }
    if (!cap.isOpened()) {
        std::cerr << "[Camera] Could not open " << video_dev << std::endl;
        stop_flag = true;
        return;
    }

    std::cout << "[Camera] Opened " << video_dev << std::endl;

    // Set 1920x1080 MJPG
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    if (!cap.set(cv::CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)) {
        std::cout << "[Camera] WARNING: CAP_PROP_BUFFERSIZE not supported by backend/driver" << std::endl;
    }
    std::cout << "[Camera] Resolution: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;

    // Open frames CSV
    std::ofstream frames_csv(frames_csv_path);
    frames_csv << "frame_index,filename,recv_time_us,brightness\n";
    frames_csv.flush();

    cv::Mat frame;
    uint32_t read_fail_count = 0;

    while (!stop_flag) {
        bool read_ok = false;
        try {
            read_ok = cap.read(frame);
        } catch (const cv::Exception& e) {
            std::cerr << "[Camera] WARNING: cap.read() exception: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        if (!read_ok) {
            read_fail_count++;
            if (read_fail_count % 100 == 0) {
                std::cout << "[Camera] WARNING: cap.read() failing repeatedly" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        read_fail_count = 0;

        if (frame.empty()) continue;  // skip corrupt/empty frames
        stream_ready = true;

        if (!frame_logging_enabled.load()) {
            int remaining = discard_frames_remaining.load();
            if (remaining > 0) {
                int prev_remaining = discard_frames_remaining.fetch_sub(1);
                if (prev_remaining <= 1) {
                    discard_frames_remaining.store(0);
                    post_trigger_flush_done = true;
                }
            }
            continue;
        }

        int64_t recv_us = monotonic_time_us();

        uint64_t idx = frame_count++;

        // Compute brightness
        cv::Scalar mean_val = cv::mean(frame);
        double brightness = (mean_val[0] + mean_val[1] + mean_val[2]) / 3.0;

        // Build filename
        std::ostringstream oss;
        oss << "frame_" << std::setfill('0') << std::setw(6) << idx << ".jpg";
        std::string basename = oss.str();
        std::string filepath = frames_dir + "/" + basename;

        // Queue for saving
        {
            std::lock_guard<std::mutex> lock(save_queue_mtx);
            save_queue.push({filepath, frame.clone()});
            save_queue_cv.notify_one();
        }

        // Log to frames CSV
        frames_csv << idx << "," << basename << "," << recv_us << ","
                   << std::fixed << std::setprecision(1) << brightness << "\n";
        if (idx % 10 == 0) frames_csv.flush();

        if (idx % 50 == 0) {
            std::cout << "[Camera] frame " << idx << "  bright=" << std::fixed
                      << std::setprecision(1) << brightness << std::endl;
        }
    }

    frames_csv.flush();
    frames_csv.close();
    cap.release();
    std::cout << "[Camera] Stopped. Total frames: " << frame_count << std::endl;
}

// ===== FRAME SAVER THREAD =====
void saver_thread() {
    while (!stop_flag || !save_queue.empty()) {
        std::unique_lock<std::mutex> lock(save_queue_mtx);
        if (save_queue.empty()) {
            save_queue_cv.wait_for(lock, std::chrono::milliseconds(100));
            if (save_queue.empty()) continue;
        }

        auto [filename, frame] = save_queue.front();
        save_queue.pop();
        lock.unlock();

        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY};
        try {
            if (!frame.empty()) {
                cv::imwrite(filename, frame, params);
            }
        } catch (const cv::Exception& e) {
            std::cerr << "[Saver] WARNING: Failed to encode " << filename << ": " << e.what() << std::endl;
        }
    }
    std::cout << "[Saver] Stopped." << std::endl;
}

// ===== RTK LOGGER THREAD =====
void rtk_logger_thread(const std::string& rtk_port, const std::string& csv_path) {
    struct sp_port* port = nullptr;
    struct sp_port_config* config = nullptr;

    // Open serial port
    if (sp_get_port_by_name(rtk_port.c_str(), &port) != SP_OK) {
        std::cerr << "[RTK] Could not find " << rtk_port << std::endl;
        stop_flag = true;
        return;
    }

    if (sp_open(port, SP_MODE_READ) != SP_OK) {
        std::cerr << "[RTK] Could not open " << rtk_port << std::endl;
        stop_flag = true;
        return;
    }

    if (sp_new_config(&config) != SP_OK) {
        sp_close(port);
        stop_flag = true;
        return;
    }

    sp_set_config_baudrate(config, RTK_BAUD);
    sp_set_config_bits(config, 8);
    sp_set_config_parity(config, SP_PARITY_NONE);
    sp_set_config_stopbits(config, 1);
    sp_set_config_flowcontrol(config, SP_FLOWCONTROL_NONE);

    if (sp_set_config(port, config) != SP_OK) {
        sp_close(port);
        sp_free_config(config);
        stop_flag = true;
        return;
    }

    sp_free_config(config);

    std::cout << "[RTK] Opened " << rtk_port << " @ " << RTK_BAUD << std::endl;

    // Open CSV for TM2 events
    std::ofstream csv_file(csv_path);
    csv_file << "tm2_index,arrival_us,ch,flags,newRisingEdge,newFallingEdge,count,wnR,wnF,towMsR,towSubMsR,towMsF,towSubMsF,accEst\n";
    csv_file.flush();

    std::cout << "[RTK] Logging TIM-TM2 → " << csv_path << std::endl;

    UBXParser parser;
    uint8_t byte;
    TIM_TM2 tm2;
    bool logged_offset_estimate = false;

    while (!stop_flag) {
        int bytes_read = sp_blocking_read(port, &byte, 1, 100);
        if (bytes_read <= 0) continue;

        if (parser.feed_byte(byte)) {
            if (parser.parse_message(parser.payload, tm2)) {
                if (!tm2.newRisingEdge) continue;

                int64_t arrival_us = monotonic_time_us();
                double rising_edge_utc_s = tm2_rising_edge_unix_seconds(tm2);

                if (!logged_offset_estimate) {
                    double gps_minus_mono_offset_s =
                        rising_edge_utc_s - static_cast<double>(arrival_us) / 1e6;
                    std::cout << "[RTK] Initial GPS-minus-monotonic offset estimate: "
                              << std::fixed << std::setprecision(6)
                              << gps_minus_mono_offset_s << " s" << std::endl;
                    logged_offset_estimate = true;
                }

                csv_file << tm2_count << ","
                         << arrival_us << ","
                         << static_cast<int>(tm2.ch) << ","
                         << static_cast<int>(tm2.flags) << ","
                         << (tm2.newRisingEdge ? 1 : 0) << ","
                         << (tm2.newFallingEdge ? 1 : 0) << ","
                         << tm2.count << ","
                         << tm2.wnR << ","
                         << tm2.wnF << ","
                         << tm2.towMsR << ","
                         << tm2.towSubMsR << ","
                         << tm2.towMsF << ","
                         << tm2.towSubMsF << ","
                         << tm2.accEst << "\n";
                csv_file.flush();

                tm2_count++;
                if (tm2_count % 10 == 0) {
                    std::cout << "[RTK] TM2 #" << tm2_count
                              << "  towR=" << tm2.towMsR << "." << tm2.towSubMsR
                              << "  count=" << tm2.count << std::endl;
                }
            }
        }
    }

    sp_close(port);
    csv_file.close();
    std::cout << "[RTK] Stopped. Total TM2 events: " << tm2_count << std::endl;
}

// ===== MAIN =====
int main(int argc, char* argv[]) {
    std::string video_dev = VIDEO_DEV;
    std::string rtk_port = RTK_PORT;
    std::string output_dir = OUTPUT_DIR;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--video" && i + 1 < argc) video_dev = argv[++i];
        else if (arg == "--rtk" && i + 1 < argc) rtk_port = argv[++i];
        else if (arg == "--out" && i + 1 < argc) output_dir = argv[++i];
    }

    // Create session directory
    std::string frames_dir = output_dir + "/images/camera1";
    std::string frames_csv_path = output_dir + "/frames.csv";
    std::string tm2_csv_path = output_dir + "/tm2.csv";

    int ret0 = system(("mkdir -p " + frames_dir).c_str());
    (void)ret0;  // Suppress unused result warning

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Force default mode first so camera can produce initial stream frames
    std::cout << "[Camera] Setting default mode for stream warm-up..." << std::endl;
    int ret_pre = system("python3 trigger_mode.py 0");
    (void)ret_pre;

    // Start camera thread first, then enable trigger mode after stream is active
    std::thread cam_t(camera_thread, video_dev, frames_dir, frames_csv_path);
    std::cout << "[Camera] Waiting for stream to become active..." << std::endl;
    auto wait_start = std::chrono::steady_clock::now();
    while (!stop_flag && !stream_ready) {
        if (std::chrono::steady_clock::now() - wait_start > std::chrono::seconds(5)) {
            std::cerr << "[Camera] WARNING: Stream not active yet after 5 seconds; continuing startup." << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::thread saver_t(saver_thread);

    std::cout << "[Camera] Stream active. Setting trigger mode..." << std::endl;
    int ret1 = system("python3 trigger_mode.py 1");
    (void)ret1;  // Suppress unused result warning

    discard_frames_remaining = POST_TRIGGER_FLUSH_FRAMES;
    post_trigger_flush_done = false;
    std::cout << "[Camera] Flushing " << POST_TRIGGER_FLUSH_FRAMES
              << " queued frames after trigger-mode switch..." << std::endl;
    auto flush_start = std::chrono::steady_clock::now();
    while (!stop_flag && !post_trigger_flush_done.load()) {
        if (std::chrono::steady_clock::now() - flush_start >
            std::chrono::milliseconds(POST_TRIGGER_FLUSH_TIMEOUT_MS)) {
            std::cerr << "[Camera] WARNING: Timed out waiting for post-trigger flush; continuing."
                      << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    frame_logging_enabled = true;
    std::cout << "[Camera] Frame logging enabled after trigger-mode flush." << std::endl;

    std::thread rtk_t(rtk_logger_thread, rtk_port, tm2_csv_path);

    std::cout << "[Session] Running. Press Ctrl+C to stop." << std::endl;

    // Wait for stop
    while (!stop_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Join threads
    cam_t.join();
    saver_t.join();
    rtk_t.join();

    // Reset camera
    std::cout << "[Camera] Resetting to default mode..." << std::endl;
    int ret2 = system("python3 trigger_mode.py 0");
    (void)ret2;  // Suppress unused result warning

    std::cout << "[Session] Done." << std::endl;
    std::cout << "  Frames   : " << frames_dir << "/" << std::endl;
    std::cout << "  Frames CSV: " << frames_csv_path << std::endl;
    std::cout << "  TM2 CSV  : " << tm2_csv_path << std::endl;
    std::cout << "  Frames   : " << frame_count << " saved" << std::endl;
    std::cout << "  TM2      : " << tm2_count << " events" << std::endl;

    return 0;
}
