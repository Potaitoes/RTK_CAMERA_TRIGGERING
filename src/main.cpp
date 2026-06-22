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
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/time.h>
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
constexpr const char* VIDEO_DEV = "/dev/video1";
constexpr const char* RTK_PORT = "/dev/ttyACM0";
constexpr int RTK_BAUD = 115200;
constexpr const char* OUTPUT_DIR = "./recordings";

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
    fmt.fmt.pix.width = 1280;
    fmt.fmt.pix.height = 720;
    // Prefer packed 4:2:2 (UYVY) which this camera supports; fall back to
    // MJPEG or other formats if the driver chooses differently.
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("[Camera] VIDIOC_S_FMT");
    }

    // Query the active format from the driver so we know what to expect
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
        perror("[Camera] VIDIOC_G_FMT");
    }

    std::cout << "[Camera] Resolution: "
              << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height << std::endl;

    // ===== REQUEST BUFFERS =====
    v4l2_requestbuffers req{};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    bool use_read_fallback = false;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("[Camera] VIDIOC_REQBUFS");

        // Try falling back to MJPEG if the driver rejected the preferred format
        std::cerr << "[Camera] INFO: attempting MJPEG fallback after REQBUFS failure" << std::endl;
        v4l2_format try_fmt = fmt;
        try_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        if (ioctl(fd, VIDIOC_S_FMT, &try_fmt) == 0) {
            // update fmt from driver
            if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
                perror("[Camera] VIDIOC_G_FMT (after MJPEG fallback)");
            }
            if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
                perror("[Camera] VIDIOC_REQBUFS (MJPEG fallback)");
                // As a last resort, enable read() fallback which works for many drivers
                std::cerr << "[Camera] INFO: enabling read() fallback (driver rejected REQBUFS)" << std::endl;
                use_read_fallback = true;
            }
        } else {
            perror("[Camera] VIDIOC_S_FMT (MJPEG fallback)");
            std::cerr << "[Camera] INFO: enabling read() fallback (format switch to MJPEG failed)" << std::endl;
            use_read_fallback = true;
        }
    }

    std::vector<void*> buffers;
    std::vector<size_t> lengths;

    if (!use_read_fallback) {
        buffers.resize(req.count);
        lengths.resize(req.count);

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
    }

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
        if (!use_read_fallback) {
            stop_flag = true;
            close(fd);
            return;
        } else {
            std::cerr << "[Camera] INFO: STREAMON failed but continuing using read() fallback" << std::endl;
        }
    }

    // Prepare read() buffer when in fallback mode
    std::vector<uint8_t> read_buf;
    if (use_read_fallback) {
        size_t sizeimg = fmt.fmt.pix.sizeimage;
        if (sizeimg == 0) {
            // ensure a conservative buffer size for common formats
            if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG) {
                sizeimg = fmt.fmt.pix.width * fmt.fmt.pix.height; // compressed estimate
            } else {
                // assume packed 2 bytes per pixel as conservative default
                sizeimg = fmt.fmt.pix.width * fmt.fmt.pix.height * 2;
            }
        }
        read_buf.resize(sizeimg);
    }

    // ===== CSV =====
    std::ofstream frames_csv(frames_csv_path);
    frames_csv << "frame_index,filename,recv_time_us,brightness\n";
    frames_csv.flush();

    cv::Mat frame;
    uint32_t read_fail_count = 0;

    // ===== MAIN LOOP =====
    while (!stop_flag) {
        uint8_t* data_ptr = nullptr;
        size_t bytesused = 0;
        uint64_t ts_us = 0;

        if (use_read_fallback) {
            // Blocking read() of a frame-sized chunk
            ssize_t r = ::read(fd, read_buf.data(), read_buf.size());
            if (r < 0) {
                if (errno == EAGAIN || errno == EINTR) continue;
                perror("[Camera] read");
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            if (r == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            data_ptr = read_buf.data();
            bytesused = (size_t)r;

            timeval tv{};
            if (gettimeofday(&tv, nullptr) == 0) {
                ts_us = (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
            } else {
                ts_us = 0;
            }
        } else {
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
            ts_us = (uint64_t)buf.timestamp.tv_sec * 1000000ULL + (uint64_t)buf.timestamp.tv_usec;

            data_ptr = reinterpret_cast<uint8_t*>(buffers[buf.index]);
            bytesused = buf.bytesused;

            // We'll requeue `buf` later after processing
            // store index for requeue
            // use a small local var to hold the index
            int dq_index = buf.index;

            // ===== Frame → OpenCV =====
            if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG) {
                std::vector<uchar> jpeg_data((uchar*)data_ptr, (uchar*)data_ptr + bytesused);
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

                // requeue
                if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
                    perror("[Camera] VIDIOC_QBUF");
                }
            } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY || fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
                int w = fmt.fmt.pix.width;
                int h = fmt.fmt.pix.height;
                size_t expected = (size_t)w * (size_t)h * 2;
                if (bytesused < 1 || bytesused < expected) {
                    ioctl(fd, VIDIOC_QBUF, &buf);
                    continue;
                }

                try {
                    cv::Mat yuv2(h, w, CV_8UC2, data_ptr);
                    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY)
                        cv::cvtColor(yuv2, frame, cv::COLOR_YUV2BGR_UYVY);
                    else
                        cv::cvtColor(yuv2, frame, cv::COLOR_YUV2BGR_YUY2);
                } catch (const cv::Exception& e) {
                    std::cerr << "[Camera] WARNING: packed YUV->BGR convert failed: " << e.what() << std::endl;
                    ioctl(fd, VIDIOC_QBUF, &buf);
                    continue;
                }

                if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
                    perror("[Camera] VIDIOC_QBUF");
                }
            } else {
                int w = fmt.fmt.pix.width;
                int h = fmt.fmt.pix.height;
                size_t expected = (size_t)w * (size_t)h * 3 / 2;
                if (bytesused < 1 || bytesused < expected) {
                    ioctl(fd, VIDIOC_QBUF, &buf);
                    continue;
                }

                try {
                    cv::Mat yuv(h + h/2, w, CV_8UC1, data_ptr);
                    cv::cvtColor(yuv, frame, cv::COLOR_YUV2BGR_I420);
                } catch (const cv::Exception& e) {
                    std::cerr << "[Camera] WARNING: YUV->BGR convert failed: " << e.what() << std::endl;
                    ioctl(fd, VIDIOC_QBUF, &buf);
                    continue;
                }

                if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
                    perror("[Camera] VIDIOC_QBUF");
                }
            }
        }

        // If we used the read() fallback, process frames here (shared code below will also handle)
        if (use_read_fallback) {
            if (bytesused == 0) continue;

            if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG) {
                std::vector<uchar> jpeg_data(read_buf.begin(), read_buf.begin() + bytesused);
                try {
                    frame = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
                } catch (const cv::Exception& e) {
                    std::cerr << "[Camera] WARNING: imdecode failed (read fallback): " << e.what() << std::endl;
                    continue;
                }
            } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY || fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
                int w = fmt.fmt.pix.width;
                int h = fmt.fmt.pix.height;
                size_t expected = (size_t)w * (size_t)h * 2;
                if (bytesused < 1 || bytesused < expected) continue;

                try {
                    cv::Mat yuv2(h, w, CV_8UC2, read_buf.data());
                    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY)
                        cv::cvtColor(yuv2, frame, cv::COLOR_YUV2BGR_UYVY);
                    else
                        cv::cvtColor(yuv2, frame, cv::COLOR_YUV2BGR_YUY2);
                } catch (const cv::Exception& e) {
                    std::cerr << "[Camera] WARNING: packed YUV->BGR convert failed (read fallback): " << e.what() << std::endl;
                    continue;
                }
            } else {
                int w = fmt.fmt.pix.width;
                int h = fmt.fmt.pix.height;
                size_t expected = (size_t)w * (size_t)h * 3 / 2;
                if (bytesused < 1 || bytesused < expected) continue;

                try {
                    cv::Mat yuv(h + h/2, w, CV_8UC1, read_buf.data());
                    cv::cvtColor(yuv, frame, cv::COLOR_YUV2BGR_I420);
                } catch (const cv::Exception& e) {
                    std::cerr << "[Camera] WARNING: YUV->BGR convert failed (read fallback): " << e.what() << std::endl;
                    continue;
                }
            }
        }

        if (frame.empty()) {
            continue;
        }

        stream_ready = true;

        uint64_t idx = frame_count++;

        // ===== BRIGHTNESS =====
        cv::Scalar mean_val = cv::mean(frame);
        double brightness = (mean_val[0] + mean_val[1] + mean_val[2]) / 3.0;

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
                   << std::fixed << std::setprecision(1) << brightness << "\n";

        if (idx % 10 == 0) frames_csv.flush();

        if (idx % 50 == 0) {
            std::cout << "[Camera] frame " << idx
                      << " bright=" << brightness
                      << " ts=" << ts_us << std::endl;
        }

        // (Requeue is handled inside the MMAP branch where applicable)
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

