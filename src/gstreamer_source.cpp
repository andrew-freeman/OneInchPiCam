#include "gstreamer_source.h"
#include "frame_source.h"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>

namespace rawudp {

namespace {

uint64_t monotonic_us()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(
        steady_clock::now().time_since_epoch()).count();
}

bool gst_init_once()
{
    static std::once_flag once;
    static bool ok = false;

    std::call_once(once, [] {
        GError* err = nullptr;
        ok = gst_init_check(nullptr, nullptr, &err);
        if (!ok) {
            std::cerr << "gst_init failed: "
                      << (err ? err->message : "unknown error") << "\n";
            if (err)
                g_error_free(err);
        }
    });

    return ok;
}

} // namespace

// ============================================================================
// GStreamerFrameSource
// ============================================================================

class GStreamerFrameSource final : public FrameSource {
public:
    explicit GStreamerFrameSource(const GStreamerConfig& cfg)
        : cfg_(cfg) {}

    ~GStreamerFrameSource() override
    {
        shutdown();
    }

    bool get_frame(uint8_t*& data,
                   size_t& size_bytes,
                   uint64_t& timestamp_us,
                   FrameMeta& meta) override;

private:
    bool build_pipeline();
    void shutdown();

private:
    GStreamerConfig cfg_;

    GstElement* pipeline_ = nullptr;
    GstElement* appsink_  = nullptr;

    std::vector<uint8_t> frame_buffer_;
    FrameMeta meta_;
    bool meta_initialized_ = false;
};

// ============================================================================
// Pipeline setup
// ============================================================================

bool GStreamerFrameSource::build_pipeline()
{
    if (!gst_init_once())
        return false;

    /*
     * IMPORTANT:
     * This pipeline is intentionally hard-coded to the known-good configuration.
     * Do not generalize until everything works end-to-end.
     */
    const char* pipeline_desc =
        "libcamerasrc "
        "camera-name=/base/axi/pcie@1000120000/rp1/i2c@88000/imx283@1a "
        "! video/x-bayer,format=rggb16,width=2784,height=1828,framerate=3/1 "
        "! queue max-size-buffers=2 leaky=downstream "
        "! appsink name=sink sync=false drop=true";

    GError* err = nullptr;
    pipeline_ = gst_parse_launch(pipeline_desc, &err);
    if (!pipeline_) {
        std::cerr << "Failed to create pipeline: "
                  << (err ? err->message : "unknown error") << "\n";
        if (err)
            g_error_free(err);
        return false;
    }

    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (!appsink_) {
        std::cerr << "Failed to get appsink element\n";
        return false;
    }

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);

    return true;
}

// ============================================================================
// FrameSource API
// ============================================================================

bool GStreamerFrameSource::get_frame(uint8_t*& data,
                                     size_t& size_bytes,
                                     uint64_t& timestamp_us,
                                     FrameMeta& meta)
{
    if (!pipeline_) {
        if (!build_pipeline())
            return false;
    }

    GstSample* sample =
        gst_app_sink_pull_sample(GST_APP_SINK(appsink_));
    if (!sample)
        return false;

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps     = gst_sample_get_caps(sample);

    if (!buffer || !caps) {
        gst_sample_unref(sample);
        return false;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return false;
    }

    // Copy buffer into owned storage
    frame_buffer_.assign(map.data, map.data + map.size);
    gst_buffer_unmap(buffer, &map);

    // Initialize metadata once (caps are fixed)
    if (!meta_initialized_) {
        GstStructure* s = gst_caps_get_structure(caps, 0);

        int width = 0, height = 0;
        gst_structure_get_int(s, "width", &width);
        gst_structure_get_int(s, "height", &height);

        meta_.width = width;
        meta_.height = height;
        meta_.pixel_format = PixelFormat::RGGB;
        meta_.container_bits = 16;
        meta_.effective_bits = cfg_.effective_bits;

        std::cerr << "GStreamer RAW stream:\n"
                  << "  " << width << "x" << height << "\n"
                  << "  format: RGGB16\n"
                  << "  frame bytes: " << frame_buffer_.size() << "\n";

        meta_initialized_ = true;
    }

    GstClockTime pts = GST_BUFFER_PTS(buffer);
    timestamp_us = GST_CLOCK_TIME_IS_VALID(pts)
                       ? static_cast<uint64_t>(pts / 1000)
                       : monotonic_us();

    data = frame_buffer_.data();
    size_bytes = frame_buffer_.size();
    meta = meta_;

    gst_sample_unref(sample);
    return true;
}

// ============================================================================
// Shutdown
// ============================================================================

void GStreamerFrameSource::shutdown()
{
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }

    pipeline_ = nullptr;
    appsink_ = nullptr;
}

// ============================================================================
// Factory
// ============================================================================

std::unique_ptr<FrameSource> make_gstreamer_source(const GStreamerConfig& cfg)
{
    return std::make_unique<GStreamerFrameSource>(cfg);
}

} // namespace rawudp
