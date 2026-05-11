/**
 * V4L2 直播推流程序
 * 功能：采集 → 编码 → RTMP推流（无限循环，类似直播）
 * 编译: g++ -std=c++17 -pthread main.cpp -o live_stream \
 *       $(pkg-config --cflags --libs libavcodec libavutil libswscale libavformat)
 * 运行: ./live_stream rtmp://localhost/live/stream
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
#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <libavutil/time.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
#include <libavformat/avformat.h>
}

// ==================== 配置参数 ====================
constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;
constexpr int FPS = 30;
constexpr int BUFFER_COUNT = 4;
constexpr int RAW_QUEUE_SIZE = 8;        // 原始帧队列
constexpr int PKT_QUEUE_SIZE = 20;       // 编码后 packet 队列
constexpr uint32_t PIXEL_FORMAT = V4L2_PIX_FMT_YUYV;

// ==================== 全局控制（信号处理用） ====================
std::atomic<bool> g_running{true};

void signal_handler(int sig) {
    std::cout << "\n[Signal] Caught " << sig << ", stopping..." << std::endl;
    g_running = false;
}

// ==================== 帧结构 ====================
struct RawFrame {
    std::vector<uint8_t> data;
    uint64_t timestamp_us = 0;
    uint32_t sequence = 0;

    RawFrame() = default;
    RawFrame(RawFrame&&) = default;
    RawFrame& operator=(RawFrame&&) = default;
    RawFrame(const RawFrame&) = delete;
    RawFrame& operator=(const RawFrame&) = delete;
};

struct EncodedPacket {
    std::vector<uint8_t> data;
    int64_t pts = 0;
    int64_t dts = 0;
    bool is_keyframe = false;

    EncodedPacket() = default;
    EncodedPacket(EncodedPacket&&) = default;
    EncodedPacket& operator=(EncodedPacket&&) = default;
    EncodedPacket(const EncodedPacket&) = delete;
    EncodedPacket& operator=(const EncodedPacket&) = delete;
};

// ==================== 线程安全队列 ====================
template<typename T>
class ThreadQueue {
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cond_;
    size_t max_size_;
    std::atomic<bool> stop_{false};
    std::atomic<size_t> dropped_{0};

public:
    explicit ThreadQueue(size_t max) : max_size_(max) {}

    void push(T&& item) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (queue_.size() >= max_size_) {
                queue_.pop();
                dropped_++;
            }
            queue_.push(std::move(item));
        }
        cond_.notify_one();
    }

    bool pop(T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this] { return !queue_.empty() || stop_; });
        if (queue_.empty()) return false;
        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cond_.notify_all();
    }

    size_t dropped() const { return dropped_; }
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
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
        fd_ = open(device_, O_RDWR | O_NONBLOCK, 0);
        if (fd_ < 0) {
            perror("open");
            return false;
        }

        struct v4l2_capability cap{};
        if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
            perror("VIDIOC_QUERYCAP");
            return false;
        }
        std::cout << "[V4L2] Driver: " << cap.driver << ", Card: " << cap.card << std::endl;

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

        struct v4l2_format actual_fmt{};
        actual_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_G_FMT, &actual_fmt);
        bytesperline_ = actual_fmt.fmt.pix.bytesperline;

        struct v4l2_requestbuffers req{};
        req.count = BUFFER_COUNT;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
            perror("VIDIOC_REQBUFS");
            return false;
        }
        buffers_.resize(req.count);

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

        for (size_t i = 0; i < buffers_.size(); ++i) {
            struct v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            ioctl(fd_, VIDIOC_QBUF, &buf);
        }

        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMON, &type);
        std::cout << "[V4L2] Stream ON, bytesperline=" << bytesperline_ << std::endl;
        return true;
    }

    bool captureFrame(RawFrame& out) {
        struct v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        struct timeval tv{2, 0};
        if (select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) return false;

        if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) return false;

        out.data.resize(buf.bytesused);
        memcpy(out.data.data(), buffers_[buf.index].start, buf.bytesused);
        out.timestamp_us = buf.timestamp.tv_sec * 1000000ULL + buf.timestamp.tv_usec;
        out.sequence = buf.sequence;

        ioctl(fd_, VIDIOC_QBUF, &buf);
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
    int bytesperline_ = WIDTH * 2;
    std::vector<Buffer> buffers_;
};

// ==================== 采集线程（无限循环） ====================
void captureThread(V4L2Capture& capture, ThreadQueue<RawFrame>& raw_queue) {
    std::cout << "[Thread-Capture] Started" << std::endl;
    auto t_start = std::chrono::steady_clock::now();
    int count = 0;

    while (g_running) {
        RawFrame frame;
        if (!capture.captureFrame(frame)) {
            std::cerr << "[Thread-Capture] Capture failed" << std::endl;
            break;
        }

        raw_queue.push(std::move(frame));
        count++;

        if (count % 30 == 0) {
            auto elapsed = std::chrono::steady_clock::now() - t_start;
            double fps = count / std::chrono::duration<double>(elapsed).count();
            std::cout << "[Thread-Capture] Pushed: " << count 
                      << ", Queue: " << raw_queue.size()
                      << ", FPS: " << fps << std::endl;
        }
    }

    raw_queue.stop();
    std::cout << "[Thread-Capture] Finished, total: " << count << std::endl;
}

// ==================== 编码线程（无限循环） ====================
void encodeThread(ThreadQueue<RawFrame>& raw_queue,
                  ThreadQueue<EncodedPacket>& pkt_queue,
                  int src_bytesperline) {
    std::cout << "[Thread-Encode] Started" << std::endl;

    // 初始化 x264 编码器
    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec) {
        std::cerr << "[Thread-Encode] x264 not found" << std::endl;
        raw_queue.stop();
        return;
    }

    AVCodecContext* ctx = avcodec_alloc_context3(codec);
    ctx->width = WIDTH;
    ctx->height = HEIGHT;
    ctx->time_base = AVRational{1, FPS};
    ctx->framerate = AVRational{FPS, 1};
    ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    ctx->bit_rate = 2000000;
    ctx->gop_size = FPS;               // 1秒一个I帧
    ctx->max_b_frames = 0;

    av_opt_set(ctx->priv_data, "preset", "ultrafast", 0);
    av_opt_set(ctx->priv_data, "tune", "zerolatency", 0);

    if (avcodec_open2(ctx, codec, nullptr) < 0) {
        std::cerr << "[Thread-Encode] Failed to open codec" << std::endl;
        avcodec_free_context(&ctx);
        raw_queue.stop();
        return;
    }

    // 分配 AVFrame 和 sws
    AVFrame* frame = av_frame_alloc();
    frame->width = WIDTH;
    frame->height = HEIGHT;
    frame->format = AV_PIX_FMT_YUV420P;
    av_frame_get_buffer(frame, 64);

    SwsContext* sws = sws_getContext(
        WIDTH, HEIGHT, AV_PIX_FMT_YUYV422,
        WIDTH, HEIGHT, AV_PIX_FMT_YUV420P,
        SWS_FAST_BILINEAR, nullptr, nullptr, nullptr
    );

    int64_t pts = 0;
    int encoded = 0;
    auto t_start = std::chrono::steady_clock::now();
    RawFrame raw;

    while (g_running && raw_queue.pop(raw)) {
        // YUYV422 → YUV420P
        const uint8_t* src_data[1] = { raw.data.data() };
        int src_linesize[1] = { src_bytesperline };
        sws_scale(sws, src_data, src_linesize, 0, HEIGHT,
                  frame->data, frame->linesize);

        frame->pts = pts++;

        // 编码
        avcodec_send_frame(ctx, frame);
        AVPacket* pkt = av_packet_alloc();

        while (true) {
            int ret = avcodec_receive_packet(ctx, pkt);
            if (ret == AVERROR(EAGAIN)) break;
            if (ret < 0) break;

            EncodedPacket ep;
            ep.data.resize(pkt->size);
            memcpy(ep.data.data(), pkt->data, pkt->size);
            ep.pts = pkt->pts;
            ep.dts = pkt->dts;
            ep.is_keyframe = (pkt->flags & AV_PKT_FLAG_KEY) != 0;

            pkt_queue.push(std::move(ep));
            av_packet_unref(pkt);
        }
        av_packet_free(&pkt);
        encoded++;

        if (encoded % 30 == 0) {
            auto elapsed = std::chrono::steady_clock::now() - t_start;
            double fps = encoded / std::chrono::duration<double>(elapsed).count();
            std::cout << "[Thread-Encode] Encoded: " << encoded 
                      << ", Queue dropped: " << raw_queue.dropped()
                      << ", FPS: " << fps << std::endl;
        }
    }

    // 刷新编码器
    avcodec_send_frame(ctx, nullptr);
    AVPacket* pkt = av_packet_alloc();
    while (avcodec_receive_packet(ctx, pkt) >= 0) {
        EncodedPacket ep;
        ep.data.resize(pkt->size);
        memcpy(ep.data.data(), pkt->data, pkt->size);
        ep.pts = pkt->pts;
        ep.dts = pkt->dts;
        ep.is_keyframe = (pkt->flags & AV_PKT_FLAG_KEY) != 0;
        pkt_queue.push(std::move(ep));
        av_packet_unref(pkt);
    }
    av_packet_free(&pkt);

    sws_freeContext(sws);
    av_frame_free(&frame);
    avcodec_free_context(&ctx);
    pkt_queue.stop();

    std::cout << "[Thread-Encode] Finished, total: " << encoded << std::endl;
}

// ==================== 推流线程（无限循环） ====================
void streamThread(ThreadQueue<EncodedPacket>& pkt_queue, const char* rtmp_url) {
    std::cout << "[Thread-Stream] Started, URL: " << rtmp_url << std::endl;

    // 初始化 FFmpeg 输出上下文
    AVFormatContext* fmt_ctx = nullptr;
    avformat_alloc_output_context2(&fmt_ctx, nullptr, "flv", rtmp_url);
    if (!fmt_ctx) {
        std::cerr << "[Thread-Stream] Failed to alloc output context" << std::endl;
        pkt_queue.stop();
        return;
    }

    // 创建视频流
    AVStream* stream = avformat_new_stream(fmt_ctx, nullptr);
    stream->id = 0;
    stream->time_base = (AVRational){1, FPS};

    AVCodecParameters* codecpar = stream->codecpar;
    codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    codecpar->codec_id = AV_CODEC_ID_H264;
    codecpar->width = WIDTH;
    codecpar->height = HEIGHT;
    codecpar->format = AV_PIX_FMT_YUV420P;

    // 打开 RTMP 连接
    if (avio_open(&fmt_ctx->pb, rtmp_url, AVIO_FLAG_WRITE) < 0) {
        std::cerr << "[Thread-Stream] Failed to open RTMP" << std::endl;
        avformat_free_context(fmt_ctx);
        pkt_queue.stop();
        return;
    }

    // 写 FLV 头
    avformat_write_header(fmt_ctx, nullptr);
    std::cout << "[Thread-Stream] RTMP connected, streaming..." << std::endl;

    // 推流循环
    EncodedPacket ep;
    //int64_t start_time = av_gettime_relative();
    auto start_time = std::chrono::steady_clock::now();
    int sent = 0;

    while (g_running && pkt_queue.pop(ep)) {
        AVPacket pkt;
        av_init_packet(&pkt);
        pkt.data = ep.data.data();
        pkt.size = ep.data.size();
        pkt.pts = ep.pts;
        pkt.dts = ep.dts;
        pkt.stream_index = 0;

        if (ep.is_keyframe) {
            pkt.flags |= AV_PKT_FLAG_KEY;
        }

        // 时间戳转换
        av_packet_rescale_ts(&pkt, (AVRational){1, FPS}, stream->time_base);

        // 写入（FFmpeg 内部：NAL → AVCC → FLV Tag → RTMP Chunk）
        av_interleaved_write_frame(fmt_ctx, &pkt);

        sent++;

        if (sent % 30 == 0) {
            std::cout << "[Thread-Stream] Sent: " << sent << std::endl;
        }
    }

    // 结束
    av_write_trailer(fmt_ctx);
    avio_close(fmt_ctx->pb);
    avformat_free_context(fmt_ctx);

    std::cout << "[Thread-Stream] Finished, total: " << sent << std::endl;
}

// ==================== 主函数 ====================
int main(int argc, char** argv) {
    // 信号处理：Ctrl+C 优雅退出
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    const char* device = (argc > 2) ? argv[2] : "/dev/video0";
    const char* rtmp_url = (argc > 1) ? argv[1] : "rtmp://8.138.232.209/live/test001";

    // 初始化 FFmpeg 网络库
    avformat_network_init();

    V4L2Capture capture(device);
    if (!capture.init()) {
        std::cerr << "V4L2 init failed" << std::endl;
        return -1;
    }

    ThreadQueue<RawFrame> raw_queue(RAW_QUEUE_SIZE);
    ThreadQueue<EncodedPacket> pkt_queue(PKT_QUEUE_SIZE);

    // 启动三级线程
    std::thread t_capture(captureThread, std::ref(capture), std::ref(raw_queue));
    std::thread t_encode(encodeThread, std::ref(raw_queue), std::ref(pkt_queue),
                         capture.getBytesPerLine());
    std::thread t_stream(streamThread, std::ref(pkt_queue), rtmp_url);

    t_capture.join();
    t_encode.join();
    t_stream.join();

    avformat_network_deinit();
    std::cout << "[Main] Pipeline stopped gracefully" << std::endl;

    return 0;
}