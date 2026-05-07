// ============================================
// v4l2_capture_encoder.cpp
// 完整项目：V4L2 采集 + 多线程 + H.264 编码
// ============================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <signal.h>

// FFmpeg 编码相关
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <queue>
#include <atomic>

// ============================================
// 配置参数
// ============================================
#define VIDEO_DEVICE    "/dev/video0"
#define OUTPUT_FILE     "output.h264"
#define WIDTH           1280
#define HEIGHT          720
#define FPS             30
#define BUFFER_COUNT    4           // V4L2 buffer 数量
#define RING_BUFFER_SIZE 8          // 环形缓冲区大小

// ============================================
// V4L2 Buffer 结构 - 直接映射内核内存
// ============================================
struct V4L2Buffer {
    void*   start;      // mmap 映射的起始地址
    size_t  length;     // 映射长度
    int     index;      // buffer 索引
};

// ============================================
// 视频帧结构 - 用于线程间传递
// ============================================
struct VideoFrame {
    uint8_t*    yuv_data;       // YUV420P 数据
    size_t      size;           // 数据大小
    uint64_t    timestamp;      // 采集时间戳 (us)
    int         width;
    int         height;
    bool        valid;          // 是否有效
};

// ============================================
// 环形缓冲区 - 线程安全
// ============================================
class RingBuffer {
private:
    VideoFrame  buffers[RING_BUFFER_SIZE];
    int         head;           // 写入位置
    int         tail;           // 读取位置
    int         count;          // 当前帧数
    pthread_mutex_t mutex;
    pthread_cond_t  cond_not_full;
    pthread_cond_t  cond_not_empty;

public:
    RingBuffer() : head(0), tail(0), count(0) {
        pthread_mutex_init(&mutex, NULL);
        pthread_cond_init(&cond_not_full, NULL);
        pthread_cond_init(&cond_not_empty, NULL);
        
        for (int i = 0; i < RING_BUFFER_SIZE; i++) {
            buffers[i].yuv_data = NULL;
            buffers[i].valid = false;
        }
    }

    ~RingBuffer() {
        pthread_mutex_destroy(&mutex);
        pthread_cond_destroy(&cond_not_full);
        pthread_cond_destroy(&cond_not_empty);
    }

    // 初始化缓冲区内存
    bool init(int frame_size) {
        for (int i = 0; i < RING_BUFFER_SIZE; i++) {
            buffers[i].yuv_data = (uint8_t*)aligned_alloc(32, frame_size);
            if (!buffers[i].yuv_data) return false;
            buffers[i].size = frame_size;
            buffers[i].valid = false;
        }
        return true;
    }

    // 生产者：写入帧（阻塞式）
    void push(const uint8_t* data, size_t size, uint64_t ts, int w, int h) {
        pthread_mutex_lock(&mutex);
        
        // 等待缓冲区不满
        while (count >= RING_BUFFER_SIZE) {
            pthread_cond_wait(&cond_not_full, &mutex);
        }
        
        // 写入数据
        VideoFrame* frame = &buffers[head];
        memcpy(frame->yuv_data, data, size);
        frame->timestamp = ts;
        frame->width = w;
        frame->height = h;
        frame->valid = true;
        
        head = (head + 1) % RING_BUFFER_SIZE;
        count++;
        
        pthread_cond_signal(&cond_not_empty);
        pthread_mutex_unlock(&mutex);
    }

    // 消费者：读取帧（阻塞式）
    VideoFrame* pop() {
        pthread_mutex_lock(&mutex);
        
        // 等待缓冲区不空
        while (count == 0) {
            pthread_cond_wait(&cond_not_empty, &mutex);
        }
        
        VideoFrame* frame = &buffers[tail];
        tail = (tail + 1) % RING_BUFFER_SIZE;
        count--;
        
        pthread_cond_signal(&cond_not_full);
        pthread_mutex_unlock(&mutex);
        
        return frame;
    }

    // 非阻塞检查
    bool is_empty() {
        pthread_mutex_lock(&mutex);
        bool empty = (count == 0);
        pthread_mutex_unlock(&mutex);
        return empty;
    }

    void cleanup() {
        for (int i = 0; i < RING_BUFFER_SIZE; i++) {
            if (buffers[i].yuv_data) {
                free(buffers[i].yuv_data);
                buffers[i].yuv_data = NULL;
            }
        }
    }
};

// ============================================
// 全局状态
// ============================================
std::atomic<bool> g_running(true);
RingBuffer g_ring_buffer;

// ============================================
// V4L2 采集类 - 直接 ioctl 操作
// ============================================
class V4L2Capture {
private:
    int                 fd;             // 设备文件描述符
    V4L2Buffer*         buffers;        // MMAP buffer 数组
    struct v4l2_buffer  buf;            // 当前操作的 buffer
    int                 buffer_count;
    int                 width, height;

    // 查询设备能力
    bool query_capability() {
        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
            perror("VIDIOC_QUERYCAP");
            return false;
        }
        
        printf("[V4L2] 设备: %s\n", cap.card);
        printf("[V4L2] 驱动: %s\n", cap.driver);
        printf("[V4L2] 总线: %s\n", cap.bus_info);
        
        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            fprintf(stderr, "设备不支持视频采集\n");
            return false;
        }
        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            fprintf(stderr, "设备不支持流式 I/O (MMAP)\n");
            return false;
        }
        return true;
    }

    // 设置视频格式
    bool set_format(int w, int h) {
        struct v4l2_format fmt = {0};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = w;
        fmt.fmt.pix.height = h;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;  // 请求 YUYV
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            perror("VIDIOC_S_FMT");
            return false;
        }
        
        // 驱动可能返回不同的参数，检查实际值
        width = fmt.fmt.pix.width;
        height = fmt.fmt.pix.height;
        printf("[V4L2] 实际分辨率: %dx%d\n", width, height);
        printf("[V4L2] 实际格式: %c%c%c%c\n",
            fmt.fmt.pix.pixelformat & 0xFF,
            (fmt.fmt.pix.pixelformat >> 8) & 0xFF,
            (fmt.fmt.pix.pixelformat >> 16) & 0xFF,
            (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
        
        return true;
    }

    // 请求 MMAP Buffer
    bool request_buffers(int count) {
        struct v4l2_requestbuffers req = {0};
        req.count = count;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        
        if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            perror("VIDIOC_REQBUFS");
            return false;
        }
        
        buffer_count = req.count;
        printf("[V4L2] 分配 %d 个 MMAP buffer\n", buffer_count);
        return true;
    }

    // 映射 Buffer 到用户空间
    bool map_buffers() {
        buffers = new V4L2Buffer[buffer_count];
        
        for (int i = 0; i < buffer_count; i++) {
            struct v4l2_buffer buf = {0};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            
            if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
                perror("VIDIOC_QUERYBUF");
                return false;
            }
            
            // 核心：mmap 映射内核分配的内存到用户空间
            buffers[i].start = mmap(NULL, buf.length,
                                   PROT_READ | PROT_WRITE,
                                   MAP_SHARED, fd, buf.m.offset);
            if (buffers[i].start == MAP_FAILED) {
                perror("mmap");
                return false;
            }
            
            buffers[i].length = buf.length;
            buffers[i].index = i;
            printf("[V4L2] Buffer %d: addr=%p, length=%zu\n", 
                   i, buffers[i].start, buffers[i].length);
        }
        return true;
    }

    // 将 Buffer 入队（Queue）- 交给驱动填充
    bool queue_buffer(int index) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = index;
        
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            return false;
        }
        return true;
    }

    // 从驱动取出 Buffer（Dequeue）- 获取图像数据
    bool dequeue_buffer(struct v4l2_buffer* out_buf) {
        if (ioctl(fd, VIDIOC_DQBUF, out_buf) < 0) {
            if (errno == EAGAIN) return false;  // 无数据
            perror("VIDIOC_DQBUF");
            return false;
        }
        return true;
    }

public:
    V4L2Capture() : fd(-1), buffers(NULL), buffer_count(0) {}
    
    ~V4L2Capture() {
        stop();
    }

    bool init(const char* device, int w, int h) {
        // 1. 打开设备
        fd = open(device, O_RDWR | O_NONBLOCK, 0);
        if (fd < 0) {
            perror("打开视频设备失败");
            return false;
        }
        printf("[V4L2] 打开设备: %s\n", device);

        // 2. 查询能力
        if (!query_capability()) return false;

        // 3. 设置格式
        if (!set_format(w, h)) return false;

        // 4. 请求 MMAP Buffer
        if (!request_buffers(BUFFER_COUNT)) return false;

        // 5. 映射到用户空间
        if (!map_buffers()) return false;

        // 6. 所有 Buffer 入队（Queue All）
        for (int i = 0; i < buffer_count; i++) {
            if (!queue_buffer(i)) return false;
        }
        printf("[V4L2] 所有 buffer 已入队\n");

        return true;
    }

    bool start_streaming() {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            perror("VIDIOC_STREAMON");
            return false;
        }
        printf("[V4L2] 流已开启\n");
        return true;
    }

    bool stop_streaming() {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
            perror("VIDIOC_STREAMOFF");
            return false;
        }
        printf("[V4L2] 流已停止\n");
        return true;
    }

    // 采集一帧 - 核心循环：DQBUF -> 处理 -> QBUF
    bool capture_frame(uint8_t* yuv_out, size_t yuv_size, uint64_t* timestamp) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        // 1. Dequeue - 从驱动取出已填充的 buffer
        if (!dequeue_buffer(&buf)) {
            return false;  // 无数据
        }
        
        int index = buf.index;
        *timestamp = buf.timestamp.tv_sec * 1000000ULL + buf.timestamp.tv_usec;
        
        // 2. 转换格式：YUYV -> YUV420P（x264 需要）
        // YUYV 格式：每个像素 2 字节，Y0 U Y1 V
        // YUV420P：先存所有 Y，再存 U，再存 V
        convert_yuyv_to_yuv420p(
            (uint8_t*)buffers[index].start,
            yuv_out,
            width, height
        );
        
        // 3. 重新 Queue - 把 buffer 还给驱动继续采集
        if (!queue_buffer(index)) {
            fprintf(stderr, "重新入队失败\n");
            return false;
        }
        
        return true;
    }

    // YUYV 转 YUV420P - 理解内存布局的关键
    static void convert_yuyv_to_yuv420p(const uint8_t* yuyv, 
                                        uint8_t* yuv420p,
                                        int width, int height) {
        int frame_size = width * height;
        uint8_t* y_plane = yuv420p;
        uint8_t* u_plane = yuv420p + frame_size;
        uint8_t* v_plane = yuv420p + frame_size + frame_size / 4;
        
        int y_index = 0, u_index = 0, v_index = 0;
        
        for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col += 2) {
                int pos = (row * width + col) * 2;
                
                // YUYV 打包格式: [Y0, U, Y1, V]
                uint8_t y0 = yuyv[pos];
                uint8_t u  = yuyv[pos + 1];
                uint8_t y1 = yuyv[pos + 2];
                uint8_t v  = yuyv[pos + 3];
                
                // Y 平面 - 每个像素一个 Y
                y_plane[y_index++] = y0;
                y_plane[y_index++] = y1;
                
                // U/V 平面 - 每 2x2 像素共享一个 U 和 V
                // 只取偶数行和偶数列
                if (row % 2 == 0) {
                    u_plane[u_index++] = u;
                    v_plane[v_index++] = v;
                }
            }
        }
    }

    void stop() {
        if (fd >= 0) {
            stop_streaming();
            
            // 解除 mmap
            if (buffers) {
                for (int i = 0; i < buffer_count; i++) {
                    if (buffers[i].start != MAP_FAILED) {
                        munmap(buffers[i].start, buffers[i].length);
                    }
                }
                delete[] buffers;
                buffers = NULL;
            }
            
            close(fd);
            fd = -1;
        }
    }

    int get_width() const { return width; }
    int get_height() const { return height; }
};

// ============================================
// H.264 编码器 - 使用 FFmpeg libx264
// ============================================
class H264Encoder {
private:
    AVCodecContext*     codec_ctx;
    AVFrame*            frame;
    AVPacket*           packet;
    struct SwsContext*  sws_ctx;
    int                 width, height;
    int                 fps;
    FILE*               output_fp;

public:
    H264Encoder() : codec_ctx(NULL), frame(NULL), packet(NULL), 
                    sws_ctx(NULL), output_fp(NULL) {}

    ~H264Encoder() {
        cleanup();
    }

    bool init(int w, int h, int frame_rate, const char* filename) {
        width = w;
        height = h;
        fps = frame_rate;

        // 1. 查找编码器
        const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
        if (!codec) {
            fprintf(stderr, "找不到 H264 编码器\n");
            return false;
        }
        printf("[ENC] 编码器: %s\n", codec->name);

        // 2. 分配编码上下文
        codec_ctx = avcodec_alloc_context3(codec);
        if (!codec_ctx) return false;

        // 3. 设置编码参数
        codec_ctx->width = width;
        codec_ctx->height = height;
        codec_ctx->time_base = (AVRational){1, fps};
        codec_ctx->framerate = (AVRational){fps, 1};
        codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;  // x264 需要 YUV420P
        codec_ctx->bit_rate = 4000000;              // 4 Mbps
        codec_ctx->gop_size = 30;                   // GOP 大小
        codec_ctx->max_b_frames = 2;

        // 4. 设置 x264 特定选项
        av_opt_set(codec_ctx->priv_data, "preset", "fast", 0);
        av_opt_set(codec_ctx->priv_data, "tune", "zerolatency", 0);
        av_opt_set(codec_ctx->priv_data, "profile", "high", 0);

        // 5. 打开编码器
        if (avcodec_open2(codec_ctx, codec, NULL) < 0) {
            fprintf(stderr, "打开编码器失败\n");
            return false;
        }

        // 6. 分配 AVFrame - 理解内存布局
        frame = av_frame_alloc();
        if (!frame) return false;
        
        frame->format = codec_ctx->pix_fmt;
        frame->width = codec_ctx->width;
        frame->height = codec_ctx->height;

        // 关键：分配实际图像数据
        // av_image_alloc 会按 linesize 对齐要求分配内存
        if (av_frame_get_buffer(frame, 32) < 0) {  // 32 字节对齐
            fprintf(stderr, "分配 frame buffer 失败\n");
            return false;
        }

        // 7. 理解 AVFrame 的内存布局
        printf("[ENC] AVFrame 布局:\n");
        printf("      Y linesize: %d, size: %d\n", frame->linesize[0], 
               frame->linesize[0] * height);
        printf("      U linesize: %d, size: %d\n", frame->linesize[1], 
               frame->linesize[1] * height / 2);
        printf("      V linesize: %d, size: %d\n", frame->linesize[2], 
               frame->linesize[2] * height / 2);
        printf("      总大小: %d bytes\n", 
               frame->linesize[0] * height + 
               frame->linesize[1] * height / 2 + 
               frame->linesize[2] * height / 2);

        // 8. 分配 packet
        packet = av_packet_alloc();
        if (!packet) return false;

        // 9. 打开输出文件
        output_fp = fopen(filename, "wb");
        if (!output_fp) {
            perror("打开输出文件失败");
            return false;
        }

        printf("[ENC] 编码器初始化完成: %dx%d @ %dfps -> %s\n", 
               width, height, fps, filename);
        return true;
    }

    // 编码一帧 YUV420P 数据
    bool encode_frame(const uint8_t* yuv_data, int64_t pts) {
        // 1. 确保 frame 可写
        if (av_frame_make_writable(frame) < 0) return false;

        // 2. 将外部 YUV 数据拷贝到 AVFrame
        // 注意 linesize 可能不等于 width，需要逐行拷贝
        int y_size = width * height;
        
        // Y 平面
        for (int i = 0; i < height; i++) {
            memcpy(frame->data[0] + i * frame->linesize[0],
                   yuv_data + i * width,
                   width);
        }
        
        // U 平面
        for (int i = 0; i < height / 2; i++) {
            memcpy(frame->data[1] + i * frame->linesize[1],
                   yuv_data + y_size + i * (width / 2),
                   width / 2);
        }
        
        // V 平面
        for (int i = 0; i < height / 2; i++) {
            memcpy(frame->data[2] + i * frame->linesize[2],
                   yuv_data + y_size + y_size / 4 + i * (width / 2),
                   width / 2);
        }

        frame->pts = pts;

        // 3. 发送到编码器
        int ret = avcodec_send_frame(codec_ctx, frame);
        if (ret < 0) {
            fprintf(stderr, "发送 frame 失败: %d\n", ret);
            return false;
        }

        // 4. 接收编码后的 packet
        while (ret >= 0) {
            ret = avcodec_receive_packet(codec_ctx, packet);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                break;  // 需要更多数据
            } else if (ret < 0) {
                fprintf(stderr, "编码错误\n");
                return false;
            }

            // 写入文件
            fwrite(packet->data, 1, packet->size, output_fp);
            av_packet_unref(packet);
        }

        return true;
    }

    // 刷新编码器
    void flush() {
        avcodec_send_frame(codec_ctx, NULL);  // 发送 NULL 表示结束
        
        int ret;
        while ((ret = avcodec_receive_packet(codec_ctx, packet)) >= 0) {
            fwrite(packet->data, 1, packet->size, output_fp);
            av_packet_unref(packet);
        }
    }

    void cleanup() {
        if (output_fp) {
            flush();
            fclose(output_fp);
            output_fp = NULL;
        }
        if (packet) av_packet_free(&packet);
        if (frame) av_frame_free(&frame);
        if (codec_ctx) avcodec_free_context(&codec_ctx);
        if (sws_ctx) sws_freeContext(sws_ctx);
    }
};

// ============================================
// 线程函数
// ============================================

// 线程 1: 视频采集线程
void* capture_thread(void* arg) {
    V4L2Capture* capture = (V4L2Capture*)arg;
    
    int yuv_size = capture->get_width() * capture->get_height() * 3 / 2;
    uint8_t* yuv_buffer = (uint8_t*)malloc(yuv_size);
    
    printf("[THREAD-CAP] 采集线程启动\n");
    
    struct timeval tv_start;
    gettimeofday(&tv_start, NULL);
    int frame_count = 0;
    
    while (g_running) {
        uint64_t timestamp;
        
        // 采集一帧：DQBUF -> 转换 -> QBUF
        if (capture->capture_frame(yuv_buffer, yuv_size, &timestamp)) {
            // 推入环形缓冲区
            g_ring_buffer.push(yuv_buffer, yuv_size, timestamp,
                              capture->get_width(), capture->get_height());
            
            frame_count++;
            if (frame_count % 30 == 0) {
                struct timeval tv_now;
                gettimeofday(&tv_now, NULL);
                double elapsed = (tv_now.tv_sec - tv_start.tv_sec) + 
                                (tv_now.tv_usec - tv_start.tv_usec) / 1e6;
                printf("[THREAD-CAP] 已采集 %d 帧, %.1f fps\n", 
                       frame_count, frame_count / elapsed);
            }
        } else {
            usleep(1000);  // 1ms 等待
        }
    }
    
    free(yuv_buffer);
    printf("[THREAD-CAP] 采集线程退出，共采集 %d 帧\n", frame_count);
    return NULL;
}

// 线程 2: 编码线程
void* encode_thread(void* arg) {
    H264Encoder* encoder = (H264Encoder*)arg;
    
    printf("[THREAD-ENC] 编码线程启动\n");
    
    int64_t pts = 0;
    int frame_count = 0;
    struct timeval tv_start;
    gettimeofday(&tv_start, NULL);
    
    while (g_running || !g_ring_buffer.is_empty()) {
        // 从环形缓冲区取出帧
        VideoFrame* frame = g_ring_buffer.pop();
        
        if (frame && frame->valid) {
            // 编码
            encoder->encode_frame(frame->yuv_data, pts++);
            
            frame_count++;
            if (frame_count % 30 == 0) {
                struct timeval tv_now;
                gettimeofday(&tv_now, NULL);
                double elapsed = (tv_now.tv_sec - tv_start.tv_sec) + 
                                (tv_now.tv_usec - tv_start.tv_usec) / 1e6;
                printf("[THREAD-ENC] 已编码 %d 帧, %.1f fps\n", 
                       frame_count, frame_count / elapsed);
            }
        }
    }
    
    printf("[THREAD-ENC] 编码线程退出，共编码 %d 帧\n", frame_count);
    return NULL;
}

// 线程 3: 文件写入线程（这里编码后直接写入，可扩展为异步 IO）
// 实际上编码线程已经包含文件写入，这里演示如何扩展
void* writer_thread(void* arg) {
    printf("[THREAD-WR] 写入线程启动（当前编码器直接写入文件）\n");
    
    // 可以在这里实现：
    // 1. 从编码线程接收 packet 数据
    // 2. 使用 writev 批量写入
    // 3. 实现文件分段（每 5 分钟一个文件）
    // 4. 网络发送
    
    while (g_running) {
        usleep(100000);  // 100ms
    }
    
    printf("[THREAD-WR] 写入线程退出\n");
    return NULL;
}

// ============================================
// 信号处理
// ============================================
void signal_handler(int sig) {
    printf("\n收到信号 %d，准备退出...\n", sig);
    g_running = false;
}

// ============================================
// 主函数
// ============================================
int main(int argc, char** argv) {
    printf("========================================\n");
    printf("  V4L2 多线程视频采集编码器\n");
    printf("========================================\n\n");
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 初始化 FFmpeg
    avcodec_register_all();

    // 初始化环形缓冲区
    int yuv_size = WIDTH * HEIGHT * 3 / 2;  // YUV420P
    if (!g_ring_buffer.init(yuv_size)) {
        fprintf(stderr, "初始化环形缓冲区失败\n");
        return 1;
    }

    // 初始化 V4L2 采集
    V4L2Capture capture;
    if (!capture.init(VIDEO_DEVICE, WIDTH, HEIGHT)) {
        return 1;
    }

    // 初始化编码器
    H264Encoder encoder;
    if (!encoder.init(capture.get_width(), capture.get_height(), 
                      FPS, OUTPUT_FILE)) {
        return 1;
    }

    // 启动采集流
    if (!capture.start_streaming()) {
        return 1;
    }

    // 创建线程
    pthread_t tid_cap, tid_enc, tid_wr;
    
    printf("\n启动线程...\n");
    pthread_create(&tid_cap, NULL, capture_thread, &capture);
    pthread_create(&tid_enc, NULL, encode_thread, &encoder);
    pthread_create(&tid_wr, NULL, writer_thread, NULL);

    // 主线程等待
    printf("\n运行中... 按 Ctrl+C 停止\n\n");
    
    pthread_join(tid_cap, NULL);
    pthread_join(tid_enc, NULL);
    pthread_join(tid_wr, NULL);

    // 清理
    g_ring_buffer.cleanup();
    
    printf("\n程序正常退出\n");
    return 0;
}