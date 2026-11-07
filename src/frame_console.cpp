#include "xtsdk.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

static volatile std::sig_atomic_t running = 1;
static void onSignal(int) { running = 0; }

int main(int argc, char** argv)
{
    if (argc != 4) {
        std::cerr << "Usage: zybo_frame SENSOR_IP BOARD_IP OUTPUT.csv\n";
        return 1;
    }
    std::ofstream output(argv[3]);
    if (!output) { std::cerr << "Cannot open output file\n"; return 1; }
    output << "frame,row,col,depth_mm,x_m,y_m,z_m\n" << std::setprecision(8);
    std::atomic<unsigned> saved{0};
    std::atomic<bool> ready{false}, failed{false};
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
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
            if (!ready || saved >= 30 || failed) return;
            const size_t total = static_cast<size_t>(frame->width) * frame->height;
            if (!total || frame->points.size() < total || frame->distData.size() < total) return;
            size_t valid = 0;
            for (size_t i = 0; i < total; ++i) {
                const auto& p = frame->points[i];
                if (frame->distData[i] > 0 && frame->distData[i] < 964000 &&
                    std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) ++valid;
            }
            output << "#frame," << frame->frame_id << ',' << frame->width << ','
                   << frame->height << ',' << total << ',' << valid << '\n';
            for (size_t i = 0; i < total; ++i) {
                const auto& p = frame->points[i];
                const auto depth = frame->distData[i];
                if (!depth || depth >= 964000 || !std::isfinite(p.x) ||
                    !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                output << frame->frame_id << ',' << i / frame->width << ',' << i % frame->width
                       << ',' << depth << ',' << p.x << ',' << p.y << ',' << p.z << '\n';
            }
            output.flush();
            if (!output) { failed = true; return; }
            const unsigned count = saved.fetch_add(1) + 1;
            std::cout << "Saved " << count << "/30 frame=" << frame->frame_id
                      << " valid=" << valid << '/' << total << std::endl;
        });
    sdk.startup();
    const auto connectionDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (running && !sdk.isconnect() && std::chrono::steady_clock::now() < connectionDeadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!running || !sdk.isconnect()) {
        std::cerr << "Connection cancelled or timed out\n";
        sdk.setCallback(); sdk.shutdown(); return 2;
    }
    sdk.stop();
    if (!sdk.setUdpDestIp(argv[2], 7687) || !sdk.start(XinTan::IMG_POINTCLOUD)) {
        std::cerr << "UDP destination or measurement start failed\n";
        sdk.setCallback(); sdk.shutdown(); return 3;
    }
    ready = true;
    const auto captureDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    while (running && saved < 30 && !failed && std::chrono::steady_clock::now() < captureDeadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ready = false;
    sdk.stop(); sdk.setCallback(); sdk.shutdown();
    output.close();
    std::cout << "Saved frames=" << saved << " file=" << argv[3] << std::endl;
    if (failed) { std::cerr << "File write failed\n"; return 4; }
    return saved == 30 ? 0 : 5;
}
