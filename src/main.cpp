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

// ===== GLOBALS =====
std::atomic<bool> stop_flag(false);
std::atomic<bool> stream_ready(false);

// Latest frame + condition variable for signaling new arrivals
cv::Mat latest_frame;
std::chrono::system_clock::time_point latest_frame_time;
uint64_t frame_seq = 0;  // increments on every new frame
std::mutex frame_mtx;
std::condition_variable frame_cv;

std::queue<std::pair<std::string, cv::Mat>> save_queue;
std::mutex save_queue_mtx;
std::condition_variable save_queue_cv;

int tm2_count = 0;

// ===== SIGNAL HANDLER =====
void signal_handler(int sig) {
    std::cout << "\n[Main] Stopping..." << std::endl;
    stop_flag = true;
    frame_cv.notify_all();
}

// ===== CAMERA THREAD =====
void camera_thread(const std::string& video_dev) {
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
        frame_cv.notify_all();
        return;
    }

    std::cout << "[Camera] Opened " << video_dev << std::endl;

    // Set 1920x1080 MJPG
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    std::cout << "[Camera] Resolution: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;

    cv::Mat frame;
    uint32_t read_fail_count = 0;

    while (!stop_flag) {
        if (!cap.read(frame)) {
            read_fail_count++;
            if (read_fail_count % 100 == 0) {
                std::cout << "[Camera] WARNING: cap.read() failing repeatedly" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        read_fail_count = 0;
        stream_ready = true;

        auto recv_time = std::chrono::system_clock::now();

        {
            std::lock_guard<std::mutex> lock(frame_mtx);
            latest_frame = frame.clone();
            latest_frame_time = recv_time;
            frame_seq++;
        }
        frame_cv.notify_one();
    }

    cap.release();
    std::cout << "[Camera] Stopped." << std::endl;
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
        cv::imwrite(filename, frame, params);
    }
    std::cout << "[Saver] Stopped." << std::endl;
}

// ===== RTK LOGGER THREAD =====
void rtk_logger_thread(const std::string& rtk_port, const std::string& frames_dir, const std::string& csv_path) {
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

    // Open CSV
    std::ofstream csv_file(csv_path);
    csv_file << "filename,unix_time,nuc_frame_recv_us,frame_seq,ch,flags,newRisingEdge,newFallingEdge,count,wnR,wnF,towMsR,towSubMsR,towMsF,towSubMsF,accEst\n";
    csv_file.flush();

    std::cout << "[RTK] Logging TIM-TM2 → " << csv_path << std::endl;

    UBXParser parser;
    uint8_t byte;
    TIM_TM2 tm2;

    while (!stop_flag) {
        int bytes_read = sp_blocking_read(port, &byte, 1, 100);
        if (bytes_read <= 0) continue;

        if (parser.feed_byte(byte)) {
            if (parser.parse_message(parser.payload, tm2)) {
                if (!tm2.newRisingEdge) continue;

                // Skip TM2 events that arrive before camera is producing frames
                if (!stream_ready) continue;

                // Snapshot frame_seq at TM2 arrival
                uint64_t seq_at_tm2;
                {
                    std::lock_guard<std::mutex> lock(frame_mtx);
                    seq_at_tm2 = frame_seq;
                }

                // Wait for a NEW frame (one that arrived after this TM2)
                cv::Mat frame;
                std::chrono::system_clock::time_point frame_recv_time;
                uint64_t paired_seq;
                {
                    std::unique_lock<std::mutex> lock(frame_mtx);
                    bool got = frame_cv.wait_for(lock, std::chrono::milliseconds(200), [&] {
                        return stop_flag.load() || frame_seq > seq_at_tm2;
                    });

                    if (stop_flag) break;

                    if (!got || latest_frame.empty()) {
                        std::cerr << "[RTK] WARNING: TM2 rising edge but no new frame within 200ms" << std::endl;
                        continue;
                    }

                    frame = latest_frame.clone();
                    frame_recv_time = latest_frame_time;
                    paired_seq = frame_seq;
                }

                // Queue frame for saving
                std::string filename = frames_dir + "/frame_" +
                    std::string(6 - std::to_string(tm2_count).length(), '0') +
                    std::to_string(tm2_count) + ".jpg";

                {
                    std::lock_guard<std::mutex> lock(save_queue_mtx);
                    save_queue.push({filename, frame});
                    save_queue_cv.notify_one();
                }

                // Log to CSV
                auto now = std::chrono::system_clock::now();
                auto time_t_now = std::chrono::system_clock::to_time_t(now);
                auto frame_recv_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    frame_recv_time.time_since_epoch()).count();

                csv_file << "frame_" << std::setfill('0') << std::setw(6) << tm2_count << ".jpg,"
                         << time_t_now << ","
                         << frame_recv_us << ","
                         << paired_seq << ","
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
                std::cout << "[RTK] Saved frame_" << std::setfill('0') << std::setw(6) << (tm2_count - 1)
                          << ".jpg  count=" << tm2.count << "  seq=" << paired_seq << std::endl;
            }
        }
    }

    sp_close(port);
    csv_file.close();
    std::cout << "[RTK] Stopped." << std::endl;
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
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string session_name = oss.str();

    std::string frames_dir = output_dir + "/" + session_name;
    std::string csv_path = output_dir + "/" + session_name + ".csv";

    int ret0 = system(("mkdir -p " + frames_dir).c_str());
    (void)ret0;  // Suppress unused result warning

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Force default mode first so camera can produce initial stream frames
    std::cout << "[Camera] Setting default mode for stream warm-up..." << std::endl;
    int ret_pre = system("python3 trigger_mode.py 0");
    (void)ret_pre;

    // Start camera thread first, then enable trigger mode after stream is active
    std::thread cam_t(camera_thread, video_dev);
    std::cout << "[Camera] Waiting for stream to become active..." << std::endl;
    auto wait_start = std::chrono::steady_clock::now();
    while (!stop_flag && !stream_ready) {
        if (std::chrono::steady_clock::now() - wait_start > std::chrono::seconds(5)) {
            std::cerr << "[Camera] WARNING: Stream not active yet after 5 seconds; continuing startup." << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "[Camera] Stream active. Setting trigger mode..." << std::endl;
    int ret1 = system("python3 trigger_mode.py 1");
    (void)ret1;  // Suppress unused result warning

    std::thread saver_t(saver_thread);
    std::thread rtk_t(rtk_logger_thread, rtk_port, frames_dir, csv_path);

    std::cout << "[Session] Running. Press Ctrl+C to stop." << std::endl;

    // Wait for stop
    while (!stop_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    frame_cv.notify_all();

    // Join threads
    cam_t.join();
    saver_t.join();
    rtk_t.join();

    // Reset camera
    std::cout << "[Camera] Resetting to default mode..." << std::endl;
    int ret2 = system("python3 trigger_mode.py 0");
    (void)ret2;  // Suppress unused result warning

    std::cout << "[Session] Done." << std::endl;
    std::cout << "  Frames : " << frames_dir << "/" << std::endl;
    std::cout << "  CSV    : " << csv_path << std::endl;
    std::cout << "  Total  : " << tm2_count << " frames saved" << std::endl;

    return 0;
}
