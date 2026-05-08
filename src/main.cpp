#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <iostream>
#include <vector>
#include <cstdio>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <termios.h>


// ========== 录制状态 ==========
enum RecordState {
    IDLE,       // 初始状态，等待按键
    RECORDING,  // 正在录制
    PAUSED      // 暂停
};


std::atomic<RecordState> g_state(IDLE);
std::atomic<bool> g_running(true);      // 程序总开关
std::atomic<bool> g_key_pressed(false); // 有按键事件

// 录制参数
std::atomic<int> g_frame_count(0);
std::atomic<double> g_record_time(0);   // 已录制时间（秒）
const double MAX_RECORD_TIME = 60.0;    // 最大 60 秒


struct Buffer {
    void* start;
    size_t length;
};

// 辅助函数：检查 ioctl 返回值
static int xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}


void input_thread() {
    // 设置终端为非规范模式（无需回车，立即响应）
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~(ICANON | ECHO);  // 关闭规范模式和回显
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    std::cout << "\n=== 控制说明 ===" << std::endl;
    std::cout << "g - 开始录制" << std::endl;
    std::cout << "p - 暂停/继续" << std::endl;
    std::cout << "s - 停止并保存" << std::endl;
    std::cout << "q - 退出程序" << std::endl;
    std::cout << "==============\n" << std::endl;

    while (g_running) {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            switch (c) {
            case 'g':
            case 'G':
                if (g_state == IDLE) {
                    g_state = RECORDING;
                    g_key_pressed = true;
                    std::cout << "[命令] 开始录制!" << std::endl;
                }
                break;
                
            case 'p':
            case 'P':
                if (g_state == RECORDING) {
                    g_state = PAUSED;
                    std::cout << "[命令] 暂停录制" << std::endl;
                } else if (g_state == PAUSED) {
                    g_state = RECORDING;
                    g_key_pressed = true;  // 唤醒采集线程
                    std::cout << "[命令] 继续录制" << std::endl;
                }
                break;
                
            case 's':
            case 'S':
                if (g_state == RECORDING || g_state == PAUSED) {
                    g_state = IDLE;
                    g_key_pressed = true;  // 唤醒采集线程
                    std::cout << "[命令] 停止录制，保存文件" << std::endl;
                }
                break;
                
            case 'q':
            case 'Q':
                g_running = false;
                g_key_pressed = true;
                std::cout << "[命令] 退出程序" << std::endl;
                break;
            }
        }
        usleep(10000);  // 10ms 轮询，降低 CPU
    }

    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}


// ========== 初始化 V4L2 ==========
int init_camera(int& fd, struct v4l2_format& fmt, std::vector<Buffer>& buffers) {
    // ========== 1. 打开设备 ==========
    fd = open("/dev/video0", O_RDWR /*| O_NONBLOCK*/, 0);
    if (fd < 0) {
        perror("open /dev/video0 failed");
        return -1;
    }

    // ========== 2. 查询设备能力 ==========
    struct v4l2_capability cap;
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        perror("VIDIOC_QUERYCAP failed");
        close(fd);
        return -1;
    }

    std::cout << "Driver: " << cap.driver << std::endl;
    std::cout << "Card: " << cap.card << std::endl;
    std::cout << "Bus: " << cap.bus_info << std::endl;

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        std::cerr << "Not a video capture device" << std::endl;
        close(fd);
        return -1;
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        std::cerr << "Not support streaming I/O" << std::endl;
        close(fd);
        return -1;
    }


    // ========== 3. 设置视频格式 ==========
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 640;
    fmt.fmt.pix.height = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {      // ← 检查返回值
        perror("VIDIOC_S_FMT failed");
        close(fd);
        return -1;
    }

    if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {      // ← 检查返回值
        perror("VIDIOC_G_FMT failed");
        close(fd);
        return -1;
    }

    std::cout << "Actual: " << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height 
              << " format=" << std::hex << fmt.fmt.pix.pixelformat << std::dec 
              << std::endl;


    // ========== 获取当前帧率 ==========

    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if (xioctl(fd, VIDIOC_G_PARM, &parm) == 0) {
        struct v4l2_fract *tf = &parm.parm.capture.timeperframe;
        printf("当前帧率: %u/%u = %.2f fps\n",
               tf->numerator, tf->denominator,
               (double)tf->denominator / tf->numerator);
        
        printf("能力标志: 0x%x\n", parm.parm.capture.capability);
        if (parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)
            printf("  → 支持帧率设置\n");
    }

    // ========== 枚举支持的帧率 ==========
    struct v4l2_frmivalenum fival;
    memset(&fival, 0, sizeof(fival));
    fival.pixel_format = fmt.fmt.pix.pixelformat;
    fival.width = fmt.fmt.pix.width;
    fival.height = fmt.fmt.pix.height;

    printf("\n支持的帧率列表:\n");
    while (xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fival) == 0) {
        if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            printf("  [%d] %u/%u = %.2f fps\n", fival.index,
                   fival.discrete.numerator, fival.discrete.denominator,
                   (double)fival.discrete.denominator / fival.discrete.numerator);
        }
        fival.index++;
    }
    
    // ========== 设置帧率（例如 30fps） ==========
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = 30;

    printf("\n请求帧率: 30 fps\n");
    if (xioctl(fd, VIDIOC_S_PARM, &parm) < 0) {
        perror("S_PARM failed");
    } else {
        struct v4l2_fract *tf = &parm.parm.capture.timeperframe;
        printf("实际帧率: %u/%u = %.2f fps\n",
               tf->numerator, tf->denominator,
               (double)tf->denominator / tf->numerator);
    }

    // ========== 4. 申请内核缓冲区 ==========
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {   // ← 检查返回值
        perror("VIDIOC_REQBUFS failed");
        close(fd);
        return -1;
    }

    if (req.count < 2) {
        std::cerr << "Insufficient buffer memory, got " << req.count << std::endl;
        close(fd);
        return -1;
    }
    std::cout << "Got " << req.count << " buffers" << std::endl;

    // ========== 5. 映射缓冲区 ==========
    buffers.resize(req.count);

    for (unsigned int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {  // ← 检查返回值
            perror("VIDIOC_QUERYBUF failed");
            // 清理
            for (unsigned int j = 0; j < i; j++) {
                munmap(buffers[j].start, buffers[j].length);
            }
            close(fd);
            return -1;
        }

        buffers[i].start = mmap(
            nullptr,
            buf.length,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            fd,
            buf.m.offset
        );
        buffers[i].length = buf.length;

        if (buffers[i].start == MAP_FAILED) {
            perror("mmap failed");
            for (unsigned int j = 0; j < i; j++) {
                munmap(buffers[j].start, buffers[j].length);
            }
            close(fd);
            return -1;
        }
        std::cout << "Buffer " << i << " mapped at " << buffers[i].start 
                  << " size=" << buffers[i].length << std::endl;
    }


    // ========== 6. 入队 ==========
    for (unsigned int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {      // ← 检查返回值
            perror("VIDIOC_QBUF failed");
            // 清理...
            return -1;
        }
    }



    return 0;
}


// ========== 主函数 ==========
int main() {
    int fd;
    struct v4l2_format fmt;
    std::vector<Buffer> buffers;

    // 初始化摄像头
    if (init_camera(fd, fmt, buffers) < 0) return -1;

    // 启动输入线程
    std::thread input(input_thread);

    // 启动流（但暂不采集）
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd, VIDIOC_STREAMON, &type);
    std::cout << "摄像头就绪，按 g 开始录制..." << std::endl;

    // 主循环：状态机
    while (g_running) {
        // ====== IDLE 状态：等待命令 ======
        if (g_state == IDLE) {
            // 清空标志，等待下次按键
            g_key_pressed = false;
            
            // 等待 g 键或 q 键
            while (g_state == IDLE && g_running) {
                usleep(10000);  // 10ms
            }
            
            if (!g_running) break;  // 退出程序
            
            // 用户按了 g，开始录制
            if (g_state == RECORDING) {
                // 打开新文件
                char filename[256];
                struct timeval tv;
                gettimeofday(&tv, nullptr);
                snprintf(filename, sizeof(filename), 
                         "/home/lxxh/Videos/rec_%ld_%06ld.yuv",
                         tv.tv_sec, tv.tv_usec);
                
                FILE* fp = fopen(filename, "wb");
                if (!fp) {
                    perror("fopen");
                    g_state = IDLE;
                    continue;
                }
                
                std::cout << "开始录制到: " << filename << std::endl;
                
                // 录制循环
                g_frame_count = 0;
                g_record_time = 0;
                struct timeval record_start;
                gettimeofday(&record_start, nullptr);
                
                while (g_state == RECORDING || g_state == PAUSED) {
                    // 检查是否超时（60秒）
                    struct timeval now;
                    gettimeofday(&now, nullptr);
                    double elapsed = (now.tv_sec - record_start.tv_sec) +
                                     (now.tv_usec - record_start.tv_usec) / 1000000.0;
                    g_record_time = elapsed;
                    
                    if (elapsed >= MAX_RECORD_TIME) {
                        std::cout << "达到最大录制时间 60 秒，自动保存" << std::endl;
                        g_state = IDLE;
                        g_key_pressed = true;  // 跳出内层循环
                        break;
                    }
                    
                    // PAUSED 状态：等待继续或停止
                    if (g_state == PAUSED) {
                        while (g_state == PAUSED && g_running) {
                            usleep(10000);
                        }
                        // 更新 record_start，扣除暂停时间
                        struct timeval resume_time;
                        gettimeofday(&resume_time, nullptr);
                        // 简单处理：暂停时间算入总时长
                        continue;
                    }
                    
                    // RECORDING 状态：采集一帧
                    struct v4l2_buffer buf;
                    memset(&buf, 0, sizeof(buf));
                    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                    buf.memory = V4L2_MEMORY_MMAP;
                    
                    // 非阻塞检查，或阻塞等待
                    if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
                        if (errno == EAGAIN) continue;  // 无帧，继续
                        perror("DQBUF");
                        break;
                    }
                    
                    // 写入文件
                    fwrite(buffers[buf.index].start, 1, buf.bytesused, fp);
                    g_frame_count++;
                    
                    // 每 30 帧打印进度
                    if (g_frame_count % 30 == 0) {
                        printf("录制中: %d 帧, %.1f 秒\n", 
                               (int)g_frame_count, g_record_time.load());
                    }
                    
                    // 归还缓冲区
                    xioctl(fd, VIDIOC_QBUF, &buf);
                    
                    // 检查按键事件（停止）
                    if (g_key_pressed && g_state == IDLE) {
                        g_key_pressed = false;
                        break;
                    }
                }
                
                // 关闭文件
                fclose(fp);
                printf("录制结束: %d 帧, %.2f 秒, 文件已保存\n",
                       (int)g_frame_count, g_record_time.load());
            }
        }
    }

    // 清理
    xioctl(fd, VIDIOC_STREAMOFF, &type);
    for (auto& b : buffers) munmap(b.start, b.length);
    close(fd);

    // 等待输入线程结束
    g_running = false;
    input.join();

    std::cout << "程序退出" << std::endl;
    return 0;
}