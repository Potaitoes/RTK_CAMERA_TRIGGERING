#include <iostream>
#include <thread>
#include <queue>
#include <deque>
#include <algorithm>
#include <cmath>
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
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>


#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <libserialport.h>

#include "ubx_parser.h"

// ===== CONFIG =====
constexpr int JPEG_QUALITY = 85;
constexpr const char* VIDEO_DEV = "/dev/video0";
constexpr const char* RTK_PORT = "/dev/ttyACM0";
constexpr int RTK_BAUD = 115200;
constexpr const char* OUTPUT_DIR = "./recordings";

// ===== AUTO-EXPOSURE CONFIG =====
constexpr int TARGET_PERCENTILE_VALUE = 120;
constexpr int HISTOGRAM_PERCENTILE = 75;
constexpr int MIN_EXPOSURE = 8;     // Adjust based on camera capabilities
constexpr int MAX_EXPOSURE = 300;  // Adjust based on camera capabilities
constexpr double EXPOSURE_GAIN_K = 0.008;  // Proportional gain for exposure adjustment
constexpr double MAX_EXPOSURE_MULTIPLIER_PER_UPDATE = 1.20;
constexpr double PERCENTILE_FILTER_ALPHA = 0.15;
constexpr double DEADZONE = 2.0;
constexpr int MAX_EXPOSURE_STEP = 5;
constexpr int PERCENTILE_HISTORY_LENGTH = 4;
constexpr int PERCENTILE_DARK_FRAMES = 2;

constexpr bool LUMINANCE_USE_GREEN = false;
constexpr int HISTOGRAM_BINS = 64;
constexpr int DOWNSAMPLE_TARGET_WIDTH = 180;
constexpr int DOWNSAMPLE_TARGET_HEIGHT = 135;
constexpr double HISTOGRAM_BORDER_CROP_RATIO = 0.10;
constexpr double HISTOGRAM_TOP_CROP_RATIO = 0.15;
constexpr double HISTOGRAM_CENTER_WEIGHT_RATIO = 0.50;
constexpr int HISTOGRAM_CENTER_WEIGHT = 2;
constexpr int HISTOGRAM_OUTER_WEIGHT = 1;

// ===== GLOBALS =====
std::atomic<bool> stop_flag(false);
std::atomic<bool> stream_ready(false);
std::atomic<uint64_t> frame_count(0);

std::queue<std::pair<std::string, cv::Mat>> save_queue;
std::mutex save_queue_mtx;
std::condition_variable save_queue_cv;

int tm2_count = 0;

// ===== SIGNAL HANDLER =====
void signal_handler(int sig) {
    std::cout << "\n[Main] Stopping..." << std::endl;
    stop_flag = true;
}

// ===== EXPOSURE CONTROL =====
bool set_manual_exposure(int fd, int exposure_value) {
    // Set auto_exposure to manual mode (1)
    v4l2_control ctrl{};
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl.value = V4L2_EXPOSURE_MANUAL;
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0) {
        std::cerr << "[Exposure] Warning: Failed to set auto_exposure mode" << std::endl;
    }

    // Set exposure time (absolute)
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = exposure_value;
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0) {
        std::cerr << "[Exposure] Failed to set exposure to " << exposure_value << std::endl;
        return false;
    }
    return true;
}

static inline uint8_t compute_luminance(const cv::Vec3b& pixel) {
    if (LUMINANCE_USE_GREEN) {
        return pixel[1];
    }
    return static_cast<uint8_t>((77 * pixel[2] + 150 * pixel[1] + 29 * pixel[0] + 128) >> 8);
}

double compute_histogram_percentile(const cv::Mat& frame, int percentile) {
    if (frame.empty() || frame.channels() < 3) {
        return -1.0;
    }

    int width = frame.cols;
    int height = frame.rows;
    int step_x = std::max(1, width / DOWNSAMPLE_TARGET_WIDTH);
    int step_y = std::max(1, height / DOWNSAMPLE_TARGET_HEIGHT);

    int left = static_cast<int>(width * HISTOGRAM_BORDER_CROP_RATIO);
    int right = std::max(left + 1, width - left);
    int top = static_cast<int>(height * HISTOGRAM_TOP_CROP_RATIO);
    int bottom = std::max(top + 1, height - static_cast<int>(height * HISTOGRAM_BORDER_CROP_RATIO));

    int center_x0 = std::max(left, static_cast<int>(width * (0.5 - HISTOGRAM_CENTER_WEIGHT_RATIO / 2.0)));
    int center_x1 = std::min(right, static_cast<int>(width * (0.5 + HISTOGRAM_CENTER_WEIGHT_RATIO / 2.0)));
    int center_y0 = std::max(top, static_cast<int>(height * (0.5 - HISTOGRAM_CENTER_WEIGHT_RATIO / 2.0)));
    int center_y1 = std::min(bottom, static_cast<int>(height * (0.5 + HISTOGRAM_CENTER_WEIGHT_RATIO / 2.0)));

    uint32_t hist[HISTOGRAM_BINS] = {};
    uint32_t total_weight = 0;

    for (int y = top; y < bottom; y += step_y) {
        const cv::Vec3b* row = frame.ptr<cv::Vec3b>(y);
        for (int x = left; x < right; x += step_x) {
            uint8_t luminance = compute_luminance(row[x]);
            int bin = luminance >> 2;
            uint32_t weight = (x >= center_x0 && x < center_x1 && y >= center_y0 && y < center_y1)
                                  ? HISTOGRAM_CENTER_WEIGHT
                                  : HISTOGRAM_OUTER_WEIGHT;
            hist[bin] += weight;
            total_weight += weight;
        }
    }

    if (total_weight == 0) {
        return -1.0;
    }

    uint32_t threshold = (static_cast<uint64_t>(total_weight) * percentile + 99) / 100;
    uint32_t cumulative = 0;
    for (int bin = 0; bin < HISTOGRAM_BINS; ++bin) {
        cumulative += hist[bin];
        if (cumulative >= threshold) {
            return static_cast<double>(bin * 4 + 2);
        }
    }

    return 255.0;
}

// ===== CAMERA THREAD =====
void camera_thread(const std::string& video_dev,
                   const std::string& frames_dir,
                   const std::string& frames_csv_path)
{
    int fd = open(video_dev.c_str(), O_RDWR);
    if (fd < 0) {
        perror("[Camera] open");
        stop_flag = true;
        return;
    }

    std::cout << "[Camera] Opened " << video_dev << std::endl;

    // ===== SET FORMAT =====
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 3840;
    fmt.fmt.pix.height = 2160;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("[Camera] VIDIOC_S_FMT");
    }

    std::cout << "[Camera] Resolution: "
              << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height << std::endl;

    // ===== REQUEST BUFFERS =====
    v4l2_requestbuffers req{};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("[Camera] VIDIOC_REQBUFS");
        stop_flag = true;
        close(fd);
        return;
    }

    std::vector<void*> buffers(req.count);
    std::vector<size_t> lengths(req.count);

    for (uint32_t i = 0; i < req.count; i++) {
        v4l2_buffer buf{};
        buf.type = req.type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("[Camera] VIDIOC_QUERYBUF");
            stop_flag = true;
            close(fd);
            return;
        }

        buffers[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd, buf.m.offset);
        lengths[i] = buf.length;

        if (buffers[i] == MAP_FAILED) {
            perror("[Camera] mmap");
            stop_flag = true;
            close(fd);
            return;
        }

        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("[Camera] VIDIOC_QBUF");
        }
    }

    // ===== START STREAM =====
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("[Camera] VIDIOC_STREAMON");
        stop_flag = true;
        close(fd);
        return;
    }

    // ===== CSV =====
    std::ofstream frames_csv(frames_csv_path);
    frames_csv << "frame_index,filename,recv_time_us,p75_luminance\n";
    frames_csv.flush();

    // ===== EXPOSURE CONTROL =====
    int current_exposure = 167;  // Default exposure value
    auto last_exposure_update = std::chrono::steady_clock::now();
    const auto exposure_update_interval = std::chrono::milliseconds(200);
    set_manual_exposure(fd, current_exposure);
    double filtered_percentile = TARGET_PERCENTILE_VALUE;
    std::deque<double> percentile_history;

    cv::Mat frame;
    uint32_t read_fail_count = 0;

    // ===== MAIN LOOP =====
    while (!stop_flag) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) continue;

            read_fail_count++;
            if (read_fail_count % 100 == 0) {
                std::cerr << "[Camera] WARNING: DQBUF failing repeatedly\n";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        read_fail_count = 0;

        // ===== DROP EMPTY / INVALID BUFFERS =====
        if (buf.bytesused == 0 || buf.bytesused < 100) {
            ioctl(fd, VIDIOC_QBUF, &buf);
            continue;
        }

        // ===== KERNEL TIMESTAMP =====
        uint64_t ts_us =
            (uint64_t)buf.timestamp.tv_sec * 1000000ULL +
            (uint64_t)buf.timestamp.tv_usec;

        // ===== MJPEG → OpenCV =====
        std::vector<uchar> jpeg_data(
            (uchar*)buffers[buf.index],
            (uchar*)buffers[buf.index] + buf.bytesused
        );

        if (jpeg_data.empty()) {
            ioctl(fd, VIDIOC_QBUF, &buf);
            continue;
        }

        try {
            frame = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
        } catch (const cv::Exception& e) {
            std::cerr << "[Camera] WARNING: imdecode failed: " << e.what() << std::endl;
            ioctl(fd, VIDIOC_QBUF, &buf);
            continue;
        }

        if (frame.empty()) {
            ioctl(fd, VIDIOC_QBUF, &buf);
            continue;
        }

        stream_ready = true;

        uint64_t idx = frame_count++;

        double measured_percentile = compute_histogram_percentile(frame, HISTOGRAM_PERCENTILE);
        if (measured_percentile < 0.0) {
            if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
                perror("[Camera] VIDIOC_QBUF");
            }
            continue;
        }

        percentile_history.push_back(measured_percentile);
        if (percentile_history.size() > PERCENTILE_HISTORY_LENGTH) {
            percentile_history.pop_front();
        }

        double control_percentile = measured_percentile;
        if (percentile_history.size() >= PERCENTILE_DARK_FRAMES) {
            std::vector<double> sorted_history(percentile_history.begin(), percentile_history.end());
            std::sort(sorted_history.begin(), sorted_history.end());
            control_percentile = 0.0;
            for (int i = 0; i < PERCENTILE_DARK_FRAMES; ++i) {
                control_percentile += sorted_history[i];
            }
            control_percentile /= static_cast<double>(PERCENTILE_DARK_FRAMES);
        }

        // ===== FILENAME =====
        std::ostringstream oss;
        oss << "frame_" << std::setfill('0') << std::setw(6) << idx << ".jpg";
        std::string basename = oss.str();
        std::string filepath = frames_dir + "/" + basename;

        // ===== QUEUE SAVE =====
        {
            std::lock_guard<std::mutex> lock(save_queue_mtx);
            save_queue.push({filepath, frame.clone()});
            save_queue_cv.notify_one();
        }

        // ===== LOG CSV =====
        frames_csv << idx << "," << basename << "," << ts_us << ","
                   << std::fixed << std::setprecision(1) << measured_percentile << "\n";

        if (idx % 10 == 0) frames_csv.flush();

        if (idx % 50 == 0) {
            std::cout << "[Camera] frame " << idx
                      << " p" << HISTOGRAM_PERCENTILE << "=" << measured_percentile
                      << " ts=" << ts_us << std::endl;
        }

        // ===== EXPOSURE CONTROL =====
        auto now = std::chrono::steady_clock::now();
        if (now - last_exposure_update >= exposure_update_interval) {
            filtered_percentile = (1.0 - PERCENTILE_FILTER_ALPHA) * filtered_percentile
                                + PERCENTILE_FILTER_ALPHA * control_percentile;
            double error = TARGET_PERCENTILE_VALUE - filtered_percentile;
            double multiplier = 1.0;

            if (std::abs(error) >= DEADZONE) {
                multiplier = std::exp(EXPOSURE_GAIN_K * error);
                multiplier = std::clamp(multiplier,
                                        1.0 / MAX_EXPOSURE_MULTIPLIER_PER_UPDATE,
                                        MAX_EXPOSURE_MULTIPLIER_PER_UPDATE);
            }

            int target_exposure = static_cast<int>(std::round(current_exposure * multiplier));
            int delta = target_exposure - current_exposure;
            if (std::abs(delta) > MAX_EXPOSURE_STEP) {
                target_exposure = current_exposure + (delta > 0 ? MAX_EXPOSURE_STEP : -MAX_EXPOSURE_STEP);
            }
            target_exposure = std::max(MIN_EXPOSURE, std::min(MAX_EXPOSURE, target_exposure));

            if (target_exposure != current_exposure) {
                if (set_manual_exposure(fd, target_exposure)) {
                    current_exposure = target_exposure;
                    std::cout << "[Exposure] Adjusted to " << current_exposure
                              << " (meas=" << measured_percentile
                              << ", ctrl=" << control_percentile
                              << ", target=" << TARGET_PERCENTILE_VALUE
                              << ", error=" << error
                              << ", mult=" << multiplier << ")"
                              << std::endl;
                }
            }
            last_exposure_update = now;
        }

        // ===== REQUEUE =====
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("[Camera] VIDIOC_QBUF");
        }
    }

    // ===== CLEANUP =====
    ioctl(fd, VIDIOC_STREAMOFF, &type);

    for (size_t i = 0; i < buffers.size(); i++) {
        munmap(buffers[i], lengths[i]);
    }

    close(fd);

    frames_csv.flush();
    frames_csv.close();

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

    while (!stop_flag) {
        int bytes_read = sp_blocking_read(port, &byte, 1, 100);
        if (bytes_read <= 0) continue;

        if (parser.feed_byte(byte)) {
            if (parser.parse_message(parser.payload, tm2)) {
                if (!tm2.newRisingEdge) continue;

                // Use kernel monotonic time to match v4l2 buffer timestamps
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                uint64_t arrival_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000;

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

    std::cout << "[Camera] Stream active. Setting trigger mode..." << std::endl;
    int ret1 = system("python3 trigger_mode.py 1");
    (void)ret1;  // Suppress unused result warning

    std::thread saver_t(saver_thread);
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

