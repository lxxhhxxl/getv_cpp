/**
 * V4L2 双线程采集 + x264 编码
 * 编译: g++ -std=c++17 -pthread main.cpp -o v4l2_encode \
 *       $(pkg-config --cflags --libs libavcodec libavutil libswscale)
 * 运行: ./v4l2_encode /dev/video0
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

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

// ==================== 配置参数 ====================
constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;
constexpr int FPS = 30;
constexpr int BUFFER_COUNT = 4;          // V4L2 内核缓冲区数量
constexpr int QUEUE_MAX_SIZE = 8;        // 应用层队列最大长度
constexpr int CAPTURE_FRAMES = 300;      // 默认采集 300 帧
constexpr uint32_t PIXEL_FORMAT = V4L2_PIX_FMT_YUYV;

// ==================== 帧结构（移动语义，所有权转移） ====================
struct RawFrame {
    std::vector<uint8_t> data;           // 应用层独立内存（从内核拷贝）
    size_t size = 0;
    uint64_t timestamp_us = 0;         // 采集时刻，用于 pts 计算
    uint32_t sequence = 0;               // 帧序号

    RawFrame() = default;
    RawFrame(RawFrame&&) = default;
    RawFrame& operator=(RawFrame&&) = default;
    RawFrame(const RawFrame&) = delete;
    RawFrame& operator=(const RawFrame&) = delete;
};

// ==================== 线程安全队列（生产者-消费者） ====================
class FrameQueue {
public:
    explicit FrameQueue(size_t max_size) : max_size_(max_size) {}

    // 生产者：采集线程调用，把帧所有权转移给队列
    void push(RawFrame&& frame) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            // 背压：队列满时丢弃最旧帧
            if (queue_.size() >= max_size_) {
                queue_.pop();
                dropped_++;
            }
            queue_.push(std::move(frame));  // 移动构造入队
        }
        cond_.notify_one();  // 唤醒可能等待的消费者
    }

    // 消费者：编码线程调用，从队列取出帧所有权
    bool pop(RawFrame& frame) {
        std::unique_lock<std::mutex> lock(mutex_);
        // 等待条件：队列非空 或 停止信号
        cond_.wait(lock, [this] { return !queue_.empty() || stop_; });
        if (queue_.empty()) return false;  // stop_ 且队列空，退出
        frame = std::move(queue_.front());  // 移动赋值，所有权转移给编码线程
        queue_.pop();
        return true;
    }

    // 通知所有等待线程退出
    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cond_.notify_all();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    size_t dropped() const { return dropped_; }

private:
    std::queue<RawFrame> queue_;           // 标准队列，存储 Frame 对象
    mutable std::mutex mutex_;             // 互斥锁，保护 queue_ 访问
    std::condition_variable cond_;         // 条件变量，阻塞等待/唤醒
    size_t max_size_;                      // 最大长度，背压控制
    std::atomic<bool> stop_{false};        // 停止标志
    std::atomic<size_t> dropped_{0};       // 丢弃计数
};

// ==================== V4L2 采集器 ====================
class V4L2Capture {
public:
    struct Buffer {
        void* start = nullptr;             // mmap 映射的用户态地址
        size_t length = 0;                 // 缓冲区总大小
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

        // 查询实际参数（含 bytesperline，可能有 padding）
        struct v4l2_format actual_fmt{};
        actual_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_G_FMT, &actual_fmt);
        bytesperline_ = actual_fmt.fmt.pix.bytesperline;

        std::cout << "[V4L2] Format: " << WIDTH << "x" << HEIGHT 
                  << ", YUYV, bytesperline=" << bytesperline_
                  << ", frames=" << CAPTURE_FRAMES << std::endl;

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

        // 5. mmap 映射内核缓冲区到用户空间
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

        // 6. 入队所有缓冲区（告诉驱动：这些缓冲区空闲，可以给 DMA 写）
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

    // 采集一帧：DQBUF（取出）→ 拷贝到应用层 → QBUF（归还）
    bool captureFrame(RawFrame& out_frame) {
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

        // 从驱动 done_queue 取出缓冲区
        if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            perror("VIDIOC_DQBUF");
            return false;
        }

        // 拷贝像素数据到应用层（关键：立即归还内核缓冲区）
        out_frame.data.resize(buf.bytesused);
        memcpy(out_frame.data.data(), buffers_[buf.index].start, buf.bytesused);
        out_frame.size = buf.bytesused;
        out_frame.timestamp_us = buf.timestamp.tv_sec * 1000000ULL + buf.timestamp.tv_usec;
        out_frame.sequence = buf.sequence;

        // 归还缓冲区到驱动 incoming_queue（DMA 可以继续写入）
        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            return false;
        }
        return true;
    }

    int getBytesPerLine() const { return bytesperline_; }

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
    int bytesperline_ = WIDTH * 2;       // 实际从驱动查询
    std::vector<Buffer> buffers_;         // mmap 映射的缓冲区
};

// ==================== 全局控制 ====================
std::atomic<bool> g_running{true};

// ==================== 采集线程（生产者） ====================
void captureThread(V4L2Capture& capture, FrameQueue& queue) {
    std::cout << "[Thread-Capture] Started" << std::endl;
    auto t_start = std::chrono::steady_clock::now();

    for (int i = 0; i < CAPTURE_FRAMES && g_running; ++i) {
        RawFrame frame;
        if (!capture.captureFrame(frame)) {
            std::cerr << "[Thread-Capture] Capture failed at frame " << i << std::endl;
            break;
        }

        // 把帧所有权转移给队列（移动语义，零拷贝）
        queue.push(std::move(frame));

        if ((i + 1) % 30 == 0) {
            auto elapsed = std::chrono::steady_clock::now() - t_start;
            double fps = (i + 1) / std::chrono::duration<double>(elapsed).count();
            std::cout << "[Thread-Capture] Pushed: " << (i + 1) 
                      << ", Queue size: " << queue.size()
                      << ", FPS: " << fps << std::endl;
        }
    }

    queue.stop();  // 通知编码线程退出
    std::cout << "[Thread-Capture] Finished" << std::endl;
}

// ==================== 编码线程（消费者） ====================
void encodeThread(FrameQueue& queue, int src_bytesperline) {
    std::cout << "[Thread-Encode] Started" << std::endl;

    // ========== 1. 初始化 x264 编码器 ==========
    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec) {
        std::cerr << "[Thread-Encode] x264 codec not found" << std::endl;
        queue.stop();
        return;
    }

    AVCodecContext* ctx = avcodec_alloc_context3(codec);
    if (!ctx) {
        queue.stop();
        return;
    }

    ctx->width = WIDTH;
    ctx->height = HEIGHT;
    ctx->time_base = (AVRational){1, FPS};       // 时基：1/30秒
    ctx->framerate = (AVRational){FPS, 1};
    ctx->pix_fmt = AV_PIX_FMT_YUV420P;           // 编码器要求 YUV420P
    ctx->bit_rate = 2000000;                     // 2Mbps
    ctx->gop_size = 30;                          // 30帧一个I帧
    ctx->max_b_frames = 0;                       // 禁用B帧，降低延迟

    av_opt_set(ctx->priv_data, "preset", "ultrafast", 0);
    av_opt_set(ctx->priv_data, "tune", "zerolatency", 0);

    if (avcodec_open2(ctx, codec, nullptr) < 0) {
        std::cerr << "[Thread-Encode] Failed to open codec" << std::endl;
        avcodec_free_context(&ctx);
        queue.stop();
        return;
    }

    // ========== 2. 分配 AVFrame（编码器输入，YUV420P 三平面） ==========
    AVFrame* frame = av_frame_alloc();
    frame->width = WIDTH;
    frame->height = HEIGHT;
    frame->format = AV_PIX_FMT_YUV420P;
    av_frame_get_buffer(frame, 64);  // 64字节对齐

    // ========== 3. 初始化 sws_scale（YUYV422 → YUV420P 转换） ==========
    SwsContext* sws = sws_getContext(
        WIDTH, HEIGHT, AV_PIX_FMT_YUYV422,   // 源：V4L2 采集格式
        WIDTH, HEIGHT, AV_PIX_FMT_YUV420P,   // 目标：编码器要求
        SWS_FAST_BILINEAR, nullptr, nullptr, nullptr
    );

    // ========== 4. 打开输出文件（裸 H.264 流） ==========
    FILE* h264_file = fopen("output.h264", "wb");
    if (!h264_file) {
        perror("fopen");
        avcodec_free_context(&ctx);
        queue.stop();
        return;
    }

    // ========== 5. 编码循环 ==========
    int encoded = 0;
    int64_t pts = 0;
    auto t_start = std::chrono::steady_clock::now();

    RawFrame raw;
    while (queue.pop(raw)) {  // 从队列取出帧所有权（阻塞等待）
        
        // 5.1 格式转换：YUYV422（交错单平面）→ YUV420P（三平面分离）
        const uint8_t* src_data[1] = { raw.data.data() };
        int src_linesize[1] = { src_bytesperline };

        sws_scale(sws, src_data, src_linesize, 0, HEIGHT,
                  frame->data, frame->linesize);

        // 5.2 设置时间戳（微秒 → 编码器时基）
        frame->pts = pts++;

        // 5.3 发送帧到编码器
        int ret = avcodec_send_frame(ctx, frame);
        if (ret < 0) {
            std::cerr << "[Thread-Encode] send_frame failed" << std::endl;
            break;
        }

        // 5.4 接收编码后的 packet（循环取，编码器可能缓冲）
        AVPacket* pkt = av_packet_alloc();
        while (true) {
            ret = avcodec_receive_packet(ctx, pkt);
            if (ret == AVERROR(EAGAIN)) break;  // 需要更多帧
            if (ret == AVERROR_EOF) break;
            if (ret < 0) break;

            // 写入 H.264 文件（Annex B 格式，带 NAL 起始码）
            fwrite(pkt->data, 1, pkt->size, h264_file);

            av_packet_unref(pkt);
        }
        av_packet_free(&pkt);

        encoded++;

        // 每 30 帧打印状态
        if (encoded % 30 == 0) {
            auto elapsed = std::chrono::steady_clock::now() - t_start;
            double fps = encoded / std::chrono::duration<double>(elapsed).count();
            std::cout << "[Thread-Encode] Encoded: " << encoded 
                      << ", Queue dropped: " << queue.dropped()
                      << ", FPS: " << fps << std::endl;
        }
    }

    // ========== 6. 刷新编码器（取出缓冲的 packet） ==========
    avcodec_send_frame(ctx, nullptr);  // 发送 EOF
    AVPacket* pkt = av_packet_alloc();
    while (true) {
        int ret = avcodec_receive_packet(ctx, pkt);
        if (ret == AVERROR_EOF) break;
        if (ret < 0) break;
        fwrite(pkt->data, 1, pkt->size, h264_file);
        av_packet_unref(pkt);
    }
    av_packet_free(&pkt);

    // ========== 7. 清理 ==========
    fclose(h264_file);
    sws_freeContext(sws);
    av_frame_free(&frame);
    avcodec_free_context(&ctx);

    auto total_elapsed = std::chrono::steady_clock::now() - t_start;
    double avg_fps = encoded / std::chrono::duration<double>(total_elapsed).count();
    std::cout << "[Thread-Encode] Finished. Total: " << encoded 
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

    // 启动双线程：采集 + 编码
    std::thread t_capture(captureThread, std::ref(capture), std::ref(queue));
    std::thread t_encode(encodeThread, std::ref(queue), capture.getBytesPerLine());

    t_capture.join();
    t_encode.join();

    std::cout << "[Main] All threads joined. Check output.h264" << std::endl;
    std::cout << "Play with: ffplay -f h264 output.h264" << std::endl;
    
    return 0;
}