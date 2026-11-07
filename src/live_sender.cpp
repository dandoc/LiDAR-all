#include "xtsdk.h"
#include "target_detector.h"
#include "box_tracker.h"
#include <deque>
#include <unordered_map>
#include <memory>
#include <utility>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;
static volatile std::sig_atomic_t running = 1;
static void onSignal(int) { running = 0; }

static void put32(std::vector<uint8_t>& out, uint32_t value)
{
    for (int shift = 0; shift < 32; shift += 8) out.push_back((value >> shift) & 255);
}
static void put16(std::vector<uint8_t>& out, uint16_t value)
{
    out.push_back(value & 255); out.push_back((value >> 8) & 255);
}
static void putFloat(std::vector<uint8_t>& out, float value)
{
    static_assert(sizeof(float) == 4, "Protocol needs 32-bit float");
    uint32_t bits; std::memcpy(&bits, &value, 4); put32(out, bits);
}

static int connectViewer(const sockaddr_in& address, const std::atomic<bool>& stop)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0) { close(fd); return -1; }
    if (connect(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0) {
        if (errno != EINPROGRESS) { close(fd); return -1; }
        const auto deadline = Clock::now() + std::chrono::seconds(2);
        while (!stop && Clock::now() < deadline) {
            pollfd p{fd, POLLOUT, 0};
            int result = poll(&p, 1, 100);
            if (result < 0 && errno == EINTR) continue;
            if (result < 0) break;
            if (!result) continue;
            int error = 0; socklen_t length = sizeof(error);
            if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &length) == 0 && !error)
                return fd;
            break;
        }
        close(fd); return -1;
    }
    return fd;
}

static bool sendPacket(int fd, const std::vector<uint8_t>& packet,
                       const std::atomic<bool>& stop)
{
    size_t offset = 0;
    const auto deadline = Clock::now() + std::chrono::seconds(1);
    while (offset < packet.size() && !stop && Clock::now() < deadline) {
        ssize_t count = send(fd, packet.data() + offset, packet.size() - offset, MSG_NOSIGNAL);
        if (count > 0) { offset += static_cast<size_t>(count); continue; }
        if (count < 0 && errno == EINTR) continue;
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd p{fd, POLLOUT, 0};
            int result = poll(&p, 1, 100);
            if (result < 0 && errno != EINTR) return false;
            continue;
        }
        return false;
    }
    return offset == packet.size();
}

static void printDepthDiagnostics(const XinTan::Frame& frame)
{
    static auto last = Clock::now() - std::chrono::seconds(1);
    const auto now = Clock::now();
    if (now - last < std::chrono::seconds(1)) return;
    last = now;
    size_t kept=0, near=0, zero=0, invalid=0, badXYZ=0;
    const size_t total=static_cast<size_t>(frame.width)*frame.height;
    for (size_t i=0; i<total; ++i) {
        const auto depth=frame.distData[i];
        const auto& point=frame.points[i];
        if (!depth) ++zero;
        else if (depth >= Watch::INVALID_DEPTH_MM) ++invalid;
        else if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                 !std::isfinite(point.z)) ++badXYZ;
        else if (depth < Watch::MIN_DEPTH_MM) ++near;
        else ++kept;
    }
    const size_t center=static_cast<size_t>(frame.height/2)*frame.width + frame.width/2;
    std::cout << "DEPTH frame=" << frame.frame_id
              << " kept=" << kept << " near=" << near
              << " zero=" << zero << " invalid=" << invalid
              << " badXYZ=" << badXYZ
              << " center_mm=" << frame.distData.at(center) << std::endl;
}

static std::vector<uint8_t> encode(
    const XinTan::Frame& frame, const Watch::Result& result,
    const std::vector<Watch::Target>& targets)
{
    std::vector<uint8_t> payload;
    uint32_t count=0;
    const size_t total=static_cast<size_t>(frame.width)*frame.height;
    payload.reserve(total*20 + targets.size()*48);
    for (size_t i=0; i<total; ++i) {
        const auto& p=frame.points[i];
        const auto depth=frame.distData[i];
        if (depth < Watch::MIN_DEPTH_MM || depth >= Watch::INVALID_DEPTH_MM || !std::isfinite(p.x) ||
            !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        putFloat(payload,p.x); putFloat(payload,p.y); putFloat(payload,p.z);
        putFloat(payload,static_cast<float>(depth));
        put32(payload,result.labels.at(i));
        ++count;
    }
    for (const auto& t : targets) {
        put32(payload,t.id);
        putFloat(payload,t.x); putFloat(payload,t.y); putFloat(payload,t.z);
        put32(payload,t.count); put32(payload,t.age); put32(payload,t.is_new ? 1 : 0);
        putFloat(payload,t.width); putFloat(payload,t.height); putFloat(payload,t.extent_z);
        putFloat(payload,t.nearest); put32(payload,t.flags);
    }
    // XTO1 cluster status, then XTM5 records with box/confirmed/predicted flags. Status counts observed components only.
    std::vector<uint8_t> packet{'X','T','O','1'};
    const auto& a=result.automatic;
    put32(packet,a.components);put32(packet,a.accepted);put32(packet,a.rejected);
    put32(packet,a.edge_checks);put32(packet,a.labelled);put32(packet,a.valid);
    putFloat(packet,a.processing_ms);
    packet.insert(packet.end(),{'X','T','M','5'});
    put32(packet,static_cast<uint32_t>(payload.size()));
    put32(packet,static_cast<uint32_t>(frame.frame_id));
    put32(packet,static_cast<uint32_t>(frame.frame_id>>32));
    put16(packet,frame.width); put16(packet,frame.height);
    put32(packet,count); put32(packet,static_cast<uint32_t>(targets.size()));
    put32(packet,result.background_frames); put32(packet,result.ready ? 1 : 0);
    packet.insert(packet.end(),payload.begin(),payload.end());
    return packet;
}

int main(int argc, char** argv)
{
    if (argc != 4) {
        std::cerr << "Usage: zybo_live SENSOR_IP BOARD_IP VIEWER_IP\n";
        return 1;
    }
    sockaddr_in address{};
    address.sin_family = AF_INET; address.sin_port = htons(9000);
    if (inet_pton(AF_INET, argv[3], &address.sin_addr) != 1) {
        std::cerr << "Invalid viewer IP\n"; return 1;
    }
    std::signal(SIGINT, onSignal); std::signal(SIGTERM, onSignal);
    std::atomic<bool> stop{false}, ready{false};
    std::mutex mutex;
    std::condition_variable changed;
    std::shared_ptr<XinTan::Frame> latest;
    Clock::time_point latestTime;
    std::mutex packetMutex;
    std::condition_variable packetChanged;
    std::shared_ptr<std::vector<uint8_t>> latestPacket;
    std::atomic<bool> resetBackground{false}, processingFailed{false};
    XinTan::XtSdk sdk;
    sdk.setConnectIpaddress(argv[1]);
    sdk.setSdkCloudCoordType(XinTan::ClOUDCOORD_CAMERA);
    sdk.setUdpDestIp(argv[2], 7687);
    sdk.setCallback(
        [](const std::shared_ptr<XinTan::CBEventData>& event) {
            if (event->eventstr == "sdkState")
                std::cout << "SDK event cmd=" << static_cast<unsigned>(event->cmdid) << std::endl;
        },
        [&](const std::shared_ptr<XinTan::Frame>& frame) {
            if (!ready) return;
            const size_t total = static_cast<size_t>(frame->width) * frame->height;
            if (!total || frame->points.size() < total || frame->distData.size() < total) return;
            { std::lock_guard<std::mutex> lock(mutex); latest = frame;
                latestTime = Clock::now(); }
            changed.notify_one();
        });
    sdk.startup();
    const auto deadline = Clock::now() + std::chrono::seconds(30);
    while (running && !sdk.isconnect() && Clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!running || !sdk.isconnect()) {
        std::cerr << "Sensor connection cancelled or timed out\n";
        sdk.setCallback(); sdk.shutdown(); return 2;
    }
    sdk.stop();
    constexpr uint8_t LIDAR_FPS = 30;
    if (!sdk.setMaxFps(LIDAR_FPS)) {
        std::cerr << "LiDAR FPS setting failed" << std::endl;
        sdk.setCallback(); sdk.shutdown(); return 3;
    }
    if (!sdk.setUdpDestIp(argv[2], 7687) || !sdk.start(XinTan::IMG_POINTCLOUD)) {
        std::cerr << "UDP destination or measurement start failed\n";
        sdk.setCallback(); sdk.shutdown(); return 3;
    }
    ready = true;
    std::thread processor([&]() {
        try {
            Watch::Detector detector;
            Watch::BoxTracker tracker;
            auto lastClusterLog=Clock::time_point{};
            while (!stop) {
                std::shared_ptr<XinTan::Frame> frame;
                Clock::time_point frameTime;

                {
                    std::unique_lock<std::mutex> lock(mutex);
                    changed.wait(lock, [&]() { return stop.load() || latest != nullptr; });
                    if (stop) break;
                    frame = std::move(latest);
                    frameTime = latestTime;

                }

                if (resetBackground.exchange(false)) {
                    detector.reset();tracker.reset();

                    std::cout << "Box tracks reset" << std::endl;
                }

                const size_t total = static_cast<size_t>(frame->width) * frame->height;
                std::vector<Watch::Pixel> pixels(total);
                for (size_t i = 0; i < total; ++i) {
                    const auto& p = frame->points[i];
                    pixels[i] = {p.x, p.y, p.z, frame->distData[i]};
                }

                printDepthDiagnostics(*frame);
                const auto processingStart=Clock::now();
                auto result = detector.process(frame->width, frame->height, pixels);
                tracker.update(result,std::chrono::duration<double>(frameTime.time_since_epoch()).count());
                result.automatic.processing_ms=std::chrono::duration<float,std::milli>(Clock::now()-processingStart).count();
                if(Clock::now()-lastClusterLog>std::chrono::seconds(1)) {
                    const auto& a=result.automatic;
                    std::cout << "CLUSTER valid=" << a.valid << " candidates=" << a.components
                              << " accepted=" << a.accepted << " rejected=" << a.rejected
                              << " labelled=" << a.labelled << " plane_fits=" << a.plane_fits
                              << " local_walls=" << a.local_walls << " split_checks=" << a.split_checks << " split_objects=" << a.split_objects << " track_records=" << result.targets.size() << " ms=" << a.processing_ms << std::endl;
                    lastClusterLog=Clock::now();
                }

                auto packet = std::make_shared<std::vector<uint8_t>>(encode(*frame, result, result.targets));
                {
                    std::lock_guard<std::mutex> lock(packetMutex);
                    latestPacket = std::move(packet);
                }
                packetChanged.notify_one();
            }
        } catch (const std::exception& error) {
            std::cerr << "Detector error: " << error.what() << std::endl;
            processingFailed = true;
        }
    });

    std::thread sender([&]() {
        int fd = -1;
        bool reportedWaiting = false;
        auto lastLog = Clock::now() - std::chrono::seconds(1);
        while (!stop) {
            if (fd < 0) {
                fd = connectViewer(address, stop);
                if (fd < 0) {
                    if (!reportedWaiting) std::cout << "Waiting for viewer " << argv[3] << ":9000" << std::endl;
                    reportedWaiting = true;
                    std::unique_lock<std::mutex> lock(packetMutex);
                    packetChanged.wait_for(lock, std::chrono::milliseconds(500), [&]() { return stop.load(); });
                    continue;
                }
                reportedWaiting = false;
                std::cout << "Viewer connected: " << argv[3] << ":9000" << std::endl;
            }
            char commands[32];
            const auto n = recv(fd, commands, sizeof(commands), MSG_DONTWAIT);
            if (n == 0 || (n < 0 && errno != EAGAIN &&
                          errno != EWOULDBLOCK && errno != EINTR)) {
                close(fd); fd = -1; continue;
            }
            for (ssize_t i = 0; i < n; ++i)
                if (commands[i] == 'B') resetBackground = true;

            std::shared_ptr<std::vector<uint8_t>> packet;
            {
                std::unique_lock<std::mutex> lock(packetMutex);
                packetChanged.wait_for(lock, std::chrono::milliseconds(200),
                    [&]() { return stop.load() || latestPacket != nullptr; });
                if (stop) break;
                if (!latestPacket) continue;
                packet = std::move(latestPacket);
            }
            if (!sendPacket(fd, *packet, stop)) {
                close(fd); fd = -1;
                if (!stop) std::cout << "Viewer send interrupted; reconnecting" << std::endl;
                continue;
            }
            const auto now = Clock::now();
            if (now - lastLog >= std::chrono::seconds(1)) {
                lastLog = now;
                std::cout << "Sent point cloud + targets: " << packet->size() << " bytes" << std::endl;
            }
        }
        if (fd >= 0) close(fd);
    });
    std::cout << "BOX TRACK v1; sensor-relative tracking; no ground or fixed background; minimum depth 1500 mm; Ctrl+C to stop" << std::endl;
    while (running && !processingFailed) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ready = false; stop = true;
    changed.notify_all(); packetChanged.notify_all();
    processor.join(); sender.join();
    sdk.stop(); sdk.setCallback(); sdk.shutdown();
    return processingFailed ? 4 : 0;
}
