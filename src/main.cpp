/**
 * V4L2 双线程采集 + 应用层队列缓冲
 * 编译: g++ -std=c++17 -pthread main.cpp -o v4l2_dual_thread
 * 运行: ./v4l2_dual_thread /dev/video0
 */

#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <cstring>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

// ==================== 配置参数 ====================
constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;
constexpr int BUFFER_COUNT = 4;          // V4L2 内核缓冲区数量
constexpr int QUEUE_MAX_SIZE = 8;        // 应用层队列最大长度（背压控制）
constexpr int CAPTURE_FRAMES = 300;      // 默认采集 300 帧
constexpr uint32_t PIXEL_FORMAT = V4L2_PIX_FMT_YUYV;

// ==================== 帧结构（支持移动语义） ====================
struct Frame {
    std::vector<uint8_t> data;           // 应用层独立内存
    size_t size = 0;
    uint64_t timestamp_us = 0;
    uint32_t sequence = 0;

    Frame() = default;
    
    // 移动构造
    Frame(Frame&& other) noexcept
        : data(std::move(other.data)),
          size(other.size),
          timestamp_us(other.timestamp_us),
          sequence(other.sequence) {
        other.size = 0;
        other.timestamp_us = 0;
        other.sequence = 0;
    }
    
    // 移动赋值
    Frame& operator=(Frame&& other) noexcept {
        if (this != &other) {
            data = std::move(other.data);
            size = other.size;
            timestamp_us = other.timestamp_us;
            sequence = other.sequence;
            other.size = 0;
            other.timestamp_us = 0;
            other.sequence = 0;
        }
        return *this;
    }
    
    // 禁止拷贝（避免意外深拷贝）
    Frame(const Frame&) = delete;
    Frame& operator=(const Frame&) = delete;
};

// ==================== 线程安全队列（生产者-消费者） ====================
class FrameQueue {
public:
    explicit FrameQueue(size_t max_size) : max_size_(max_size) {}

    // 生产者：采集线程 push
    void push(Frame&& frame) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            
            // 背压策略：队列满时丢弃最旧帧
            if (queue_.size() >= max_size_) {
                queue_.pop();
                dropped_++;
            }
            
            queue_.push(std::move(frame));
        }
        cond_.notify_one();  // 唤醒可能等待的消费者
    }

    // 消费者：处理线程 pop
    bool pop(Frame& frame) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        // 等待条件：队列非空 或 停止信号
        cond_.wait(lock, [this] { return !queue_.empty() || stop_; });
        
        if (queue_.empty()) return false;  // stop_ 且队列空
        
        frame = std::move(queue_.front());  // 所有权转移给处理线程
        queue_.pop();
        return true;
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cond_.notify_all();  // 唤醒所有等待线程
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    size_t dropped() const { return dropped_; }

private:
    std::queue<Frame> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cond_;
    size_t max_size_;
    std::atomic<bool> stop_{false};
    std::atomic<size_t> dropped_{0};
};

// ==================== V4L2 采集器 ====================
class V4L2Capture {
public:
    struct Buffer {
        void* start = nullptr;
        size_t length = 0;
    };

    explicit V4L2Capture(const char* device) : device_(device) {}
    ~V4L2Capture() { cleanup(); }

    bool init() {
        // 1. 打开设备
        fd_ = open(device_, O_RDWR | O_NONBLOCK, 0);
        if (fd_ < 0) {
            perror("open");
            return false;
        }

        // 2. 查询设备能力
        struct v4l2_capability cap{};
        if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
            perror("VIDIOC_QUERYCAP");
            return false;
        }
        std::cout << "[V4L2] Driver: " << cap.driver 
                  << ", Card: " << cap.card << std::endl;

        // 3. 设置视频格式
        struct v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = WIDTH;
        fmt.fmt.pix.height = HEIGHT;
        fmt.fmt.pix.pixelformat = PIXEL_FORMAT;
        fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
            perror("VIDIOC_S_FMT");
            return false;
        }
        std::cout << "[V4L2] Format: " << WIDTH << "x" << HEIGHT 
                  << ", YUYV, " << CAPTURE_FRAMES << " frames" << std::endl;

        // 4. 申请内核缓冲区
        struct v4l2_requestbuffers req{};
        req.count = BUFFER_COUNT;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
            perror("VIDIOC_REQBUFS");
            return false;
        }
        buffers_.resize(req.count);

        // 5. mmap 映射
        for (size_t i = 0; i < buffers_.size(); ++i) {
            struct v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
                perror("VIDIOC_QUERYBUF");
                return false;
            }
            buffers_[i].length = buf.length;
            buffers_[i].start = mmap(nullptr, buf.length,
                                     PROT_READ | PROT_WRITE, MAP_SHARED,
                                     fd_, buf.m.offset);
            if (buffers_[i].start == MAP_FAILED) {
                perror("mmap");
                return false;
            }
        }
        std::cout << "[V4L2] Mmap " << buffers_.size() << " buffers" << std::endl;

        // 6. 入队所有缓冲区
        for (size_t i = 0; i < buffers_.size(); ++i) {
            struct v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
                perror("VIDIOC_QBUF");
                return false;
            }
        }

        // 7. 启动视频流
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
            perror("VIDIOC_STREAMON");
            return false;
        }
        std::cout << "[V4L2] Stream ON" << std::endl;
        return true;
    }

    // 采集一帧：DQBUF -> 拷贝到应用层 -> QBUF 归还
    bool captureFrame(Frame& out_frame) {
        struct v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // 阻塞等待帧就绪（select 做事件等待）
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        struct timeval tv{2, 0}; // 2秒超时
        int r = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (r <= 0) {
            std::cerr << "[V4L2] select timeout or error" << std::endl;
            return false;
        }

        // 取出内核缓冲区
        if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            perror("VIDIOC_DQBUF");
            return false;
        }

        // 拷贝到应用层（关键：立即归还内核缓冲区）
        out_frame.data.resize(buf.bytesused);
        memcpy(out_frame.data.data(), buffers_[buf.index].start, buf.bytesused);
        out_frame.size = buf.bytesused;
        out_frame.timestamp_us = buf.timestamp.tv_sec * 1000000ULL + buf.timestamp.tv_usec;
        out_frame.sequence = buf.sequence;

        // 立即归还缓冲区到驱动
        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            return false;
        }
        return true;
    }

    void cleanup() {
        if (fd_ < 0) return;
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);
        for (auto& b : buffers_) {
            if (b.start && b.start != MAP_FAILED) munmap(b.start, b.length);
        }
        close(fd_);
        fd_ = -1;
    }

private:
    const char* device_;
    int fd_ = -1;
    std::vector<Buffer> buffers_;
};

// ==================== 全局控制 ====================
std::atomic<bool> g_running{true};

// ==================== 采集线程（生产者） ====================
void captureThread(V4L2Capture& capture, FrameQueue& queue) {
    std::cout << "[Thread-Capture] Started" << std::endl;
    auto t_start = std::chrono::steady_clock::now();

    for (int i = 0; i < CAPTURE_FRAMES && g_running; ++i) {
        Frame frame;
        if (!capture.captureFrame(frame)) {
            std::cerr << "[Thread-Capture] Capture failed at frame " << i << std::endl;
            break;
        }

        // 推入应用层队列（所有权转移给队列）
        queue.push(std::move(frame));

        // 每 30 帧打印状态
        if ((i + 1) % 30 == 0) {
            auto elapsed = std::chrono::steady_clock::now() - t_start;
            double fps = (i + 1) / std::chrono::duration<double>(elapsed).count();
            std::cout << "[Thread-Capture] Pushed: " << (i + 1) 
                      << ", Queue size: " << queue.size()
                      << ", FPS: " << fps << std::endl;
        }
    }

    // 采集结束，通知消费者退出
    queue.stop();
    std::cout << "[Thread-Capture] Finished, total pushed: " << CAPTURE_FRAMES << std::endl;
}

// ==================== 处理线程（消费者） ====================
void processThread(FrameQueue& queue) {
    std::cout << "[Thread-Process] Started" << std::endl;
    
    FILE* yuv_file = fopen("/home/lxxh/Videos/output_dual_thread.yuv", "wb");
    if (!yuv_file) {
        perror("fopen");
        return;
    }

    int processed = 0;
    auto t_start = std::chrono::steady_clock::now();

    Frame frame;
    while (queue.pop(frame)) {
        // 模拟处理耗时（编码/图像处理等）
        // 这里仅做文件写入 + 可选的模拟延迟
        fwrite(frame.data.data(), 1, frame.size, yuv_file);
        
        // 可选：模拟处理抖动（取消注释以测试队列缓冲）
        // std::this_thread::sleep_for(std::chrono::milliseconds(20 + rand() % 20));
        
        processed++;

        // 每 30 帧打印状态
        if (processed % 30 == 0) {
            auto elapsed = std::chrono::steady_clock::now() - t_start;
            double fps = processed / std::chrono::duration<double>(elapsed).count();
            std::cout << "[Thread-Process] Processed: " << processed 
                      << ", Queue dropped: " << queue.dropped()
                      << ", Avg FPS: " << fps << std::endl;
        }
    }

    fclose(yuv_file);
    auto total_elapsed = std::chrono::steady_clock::now() - t_start;
    double avg_fps = processed / std::chrono::duration<double>(total_elapsed).count();
    
    std::cout << "[Thread-Process] Finished. Total: " << processed 
              << ", Dropped: " << queue.dropped()
              << ", Final Avg FPS: " << avg_fps << std::endl;
}

// ==================== 主函数 ====================
int main(int argc, char** argv) {
    const char* device = (argc > 1) ? argv[1] : "/dev/video0";
    
    V4L2Capture capture(device);
    if (!capture.init()) {
        std::cerr << "V4L2 init failed" << std::endl;
        return -1;
    }

    FrameQueue queue(QUEUE_MAX_SIZE);

    // 启动双线程
    std::thread t_capture(captureThread, std::ref(capture), std::ref(queue));
    std::thread t_process(processThread, std::ref(queue));

    t_capture.join();
    t_process.join();

    std::cout << "[Main] All threads joined. Check output_dual_thread.yuv" << std::endl;
    
    // 查看结果
    std::cout << "Play with: ffplay -f rawvideo -pixel_format yuyv422 -video_size " 
              << WIDTH << "x" << HEIGHT << " output_dual_thread.yuv" << std::endl;
    
    return 0;
}