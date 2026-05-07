#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <iostream>
#include <vector>
#include <cstdio>
#include <cstring>

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

int main() {
    // ========== 1. 打开设备 ==========
    int fd = open("/dev/video0", O_RDWR /*| O_NONBLOCK*/, 0);
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
    struct v4l2_format fmt;
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
    std::vector<Buffer> buffers(req.count);

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

    // ========== 7. 启动视频流 ==========
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {    // ← 检查返回值
        perror("VIDIOC_STREAMON failed");
        // 清理...
        return -1;
    }
    std::cout << "Stream ON" << std::endl;

    // ========== 8. 打开输出文件 ==========
    FILE* yuv_file = fopen("/home/lxxh/Videos/output.yuv", "wb");
    if (!yuv_file) {
        perror("fopen failed");
        ioctl(fd, VIDIOC_STREAMOFF, &type);
        // 清理...
        return -1;
    }

    // ========== 9. 采集循环 ==========
    for (int frame = 0; frame < 100; frame++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {    // ← 检查返回值
            perror("VIDIOC_DQBUF failed");
            break;
        }

        void* frame_data = buffers[buf.index].start;
        size_t frame_size = buf.bytesused;

        std::cout << "Frame " << buf.sequence 
                  << " buf[" << buf.index << "]"
                  << " size=" << frame_size 
                  << " ts=" << buf.timestamp.tv_sec << "." << buf.timestamp.tv_usec
                  << std::endl;

        fwrite(frame_data, 1, frame_size, yuv_file);

        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {     // ← 检查返回值
            perror("VIDIOC_QBUF failed");
            break;
        }
    }

    fclose(yuv_file);

    // ========== 10. 停止流 ==========
    xioctl(fd, VIDIOC_STREAMOFF, &type);

    // ========== 11. 释放资源 ==========
    for (auto& buf : buffers) {
        munmap(buf.start, buf.length);
    }
    close(fd);

    return 0;
}