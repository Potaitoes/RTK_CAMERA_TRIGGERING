#include <iostream>
#include <thread>
#include <queue>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <algorithm>
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
constexpr auto FRAME_MATCH_TIMEOUT = std::chrono::milliseconds(120);
constexpr auto MAX_FRAME_DELAY_AFTER_TM2 = std::chrono::milliseconds(50);
constexpr size_t MAX_FRAME_BUFFER_SIZE = 16;

struct BufferedFrame {
    cv::Mat image;
    std::chrono::system_clock::time_point recv_time;
    uint64_t seq = 0;
};

// ===== GLOBALS =====
std::atomic<bool> stop_flag(false);
std::atomic<bool> stream_ready(false);

// Buffered frames + condition variable for signaling new arrivals
std::deque<BufferedFrame> frame_buffer;
uint64_t frame_seq = 0;  // increments on every new frame
uint64_t last_paired_seq = 0;
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
            frame_seq++;
            frame_buffer.push_back({frame.clone(), recv_time, frame_seq});
            if (frame_buffer.size() > MAX_FRAME_BUFFER_SIZE) {
                frame_buffer.pop_front();
            }
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

std::string make_frame_basename(int index) {
    std::ostringstream oss;
    oss << "frame_" << std::setfill('0') << std::setw(6) << index << ".jpg";
    return oss.str();
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
    csv_file << "status,filename,unix_time,tm2_arrival_us,nuc_frame_recv_us,frame_delay_us,frame_seq,ch,flags,newRisingEdge,newFallingEdge,count,wnR,wnF,towMsR,towSubMsR,towMsF,towSubMsF,accEst\n";
    csv_file.flush();

    std::cout << "[RTK] Logging TIM-TM2 → " << csv_path << std::endl;

    UBXParser parser;
    uint8_t byte;
    TIM_TM2 tm2;

    auto log_tm2_row = [&](const std::string& status,
                           const std::string& filename,
                           const TIM_TM2& event,
                           int64_t tm2_arrival_us,
                           int64_t frame_recv_us,
                           int64_t frame_delay_us,
                           uint64_t paired_frame_seq) {
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);

        csv_file << status << ","
                 << filename << ","
                 << time_t_now << ","
                 << tm2_arrival_us << ",";

        if (frame_recv_us >= 0) {
            csv_file << frame_recv_us;
        }
        csv_file << ",";

        if (frame_delay_us >= 0) {
            csv_file << frame_delay_us;
        }
        csv_file << ",";

        if (paired_frame_seq > 0) {
            csv_file << paired_frame_seq;
        }

        csv_file << ","
                 << static_cast<int>(event.ch) << ","
                 << static_cast<int>(event.flags) << ","
                 << (event.newRisingEdge ? 1 : 0) << ","
                 << (event.newFallingEdge ? 1 : 0) << ","
                 << event.count << ","
                 << event.wnR << ","
                 << event.wnF << ","
                 << event.towMsR << ","
                 << event.towSubMsR << ","
                 << event.towMsF << ","
                 << event.towSubMsF << ","
                 << event.accEst << "\n";
        csv_file.flush();
    };

    while (!stop_flag) {
        int bytes_read = sp_blocking_read(port, &byte, 1, 100);
        if (bytes_read <= 0) continue;

        if (parser.feed_byte(byte)) {
            if (parser.parse_message(parser.payload, tm2)) {
                if (!tm2.newRisingEdge) continue;

                // Skip TM2 events that arrive before camera is producing frames
                if (!stream_ready) continue;

                auto tm2_arrival_time = std::chrono::system_clock::now();
                auto tm2_arrival_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    tm2_arrival_time.time_since_epoch()).count();

                // Snapshot frame_seq at TM2 arrival
                uint64_t seq_at_tm2;
                {
                    std::lock_guard<std::mutex> lock(frame_mtx);
                    seq_at_tm2 = frame_seq;
                }

                uint64_t min_required_seq = std::max(last_paired_seq, seq_at_tm2);

                // Wait for the first eligible frame that arrives after this TM2.
                cv::Mat frame;
                std::chrono::system_clock::time_point frame_recv_time;
                uint64_t paired_seq;
                bool matched = false;
                bool late_frame_rejected = false;
                {
                    std::unique_lock<std::mutex> lock(frame_mtx);
                    auto deadline = std::chrono::steady_clock::now() + FRAME_MATCH_TIMEOUT;

                    while (!stop_flag) {
                        auto candidate = std::find_if(frame_buffer.begin(), frame_buffer.end(), [&](const BufferedFrame& buffered_frame) {
                            return buffered_frame.seq > min_required_seq;
                        });

                        if (candidate != frame_buffer.end()) {
                            if (candidate->recv_time <= tm2_arrival_time) {
                                min_required_seq = candidate->seq;
                                continue;
                            }

                            auto frame_delay = candidate->recv_time - tm2_arrival_time;
                            if (frame_delay > MAX_FRAME_DELAY_AFTER_TM2) {
                                late_frame_rejected = true;
                                break;
                            }

                            frame = candidate->image.clone();
                            frame_recv_time = candidate->recv_time;
                            paired_seq = candidate->seq;
                            last_paired_seq = paired_seq;
                            matched = true;
                            break;
                        }

                        if (frame_cv.wait_until(lock, deadline) == std::cv_status::timeout) {
                            break;
                        }
                    }
                }

                if (stop_flag) break;

                if (!matched) {
                    const std::string status = late_frame_rejected ? "missed_late_frame" : "missed_timeout";
                    std::cerr << "[RTK] WARNING: TM2 rising edge missed; reason=" << status << std::endl;
                    log_tm2_row(status, "", tm2, tm2_arrival_us, -1, -1, 0);
                    continue;
                }

                auto frame_recv_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    frame_recv_time.time_since_epoch()).count();
                auto frame_delay_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    frame_recv_time - tm2_arrival_time).count();
                std::string frame_basename = make_frame_basename(tm2_count);

                // Queue frame for saving
                std::string filename = frames_dir + "/" + frame_basename;

                {
                    std::lock_guard<std::mutex> lock(save_queue_mtx);
                    save_queue.push({filename, frame});
                    save_queue_cv.notify_one();
                }

                log_tm2_row("saved",
                            frame_basename,
                            tm2,
                            tm2_arrival_us,
                            frame_recv_us,
                            frame_delay_us,
                            paired_seq);

                tm2_count++;
                std::cout << "[RTK] Saved frame_" << std::setfill('0') << std::setw(6) << (tm2_count - 1)
                          << ".jpg  count=" << tm2.count << "  seq=" << paired_seq
                          << "  delay_us=" << frame_delay_us << std::endl;
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
