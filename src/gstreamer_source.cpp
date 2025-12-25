#include "gstreamer_source.h"

#include <chrono>
#include <iostream>

#ifdef RAWUDP_HAVE_GSTREAMER

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace rawudp {

namespace {

uint64_t monotonic_timestamp_us() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

PixelFormat parse_bayer_format(const std::string& fmt) {
    if (fmt.find("RGGB") != std::string::npos) return PixelFormat::RGGB;
    if (fmt.find("BGGR") != std::string::npos) return PixelFormat::BGGR;
    if (fmt.find("GRBG") != std::string::npos) return PixelFormat::GRBG;
    if (fmt.find("GBRG") != std::string::npos) return PixelFormat::GBRG;
    return PixelFormat::MONO;
}

uint16_t detect_container_bits(const std::string& fmt, uint16_t fallback) {
    for (char c : fmt) {
        if (c >= '0' && c <= '9') {
            uint16_t bits = static_cast<uint16_t>(std::stoi(fmt.substr(fmt.find(c))));
            if (bits > 0 && bits <= 16) return bits;
            break;
        }
    }
    return fallback;
}

bool gst_init_once() {
    static std::once_flag once;
    static bool ok = false;
    std::call_once(once, []() {
        GError* err = nullptr;
        ok = gst_init_check(nullptr, nullptr, &err);
        if (!ok) {
            if (err) {
                std::cerr << "gst_init_check failed: " << err->message << "\n";
                g_error_free(err);
            } else {
                std::cerr << "gst_init_check failed\n";
            }
        }
    });
    return ok;
}

} // namespace

class GStreamerFrameSource : public FrameSource {
public:
    explicit GStreamerFrameSource(const GStreamerConfig& cfg) : cfg_(cfg) {}
    ~GStreamerFrameSource() override;

    bool start();
    bool get_frame(uint8_t*& data, size_t& size_bytes, uint64_t& timestamp_us,
                   FrameMeta& meta) override;

private:
    bool build_pipeline();
    bool pull_sample(uint64_t timeout_us);
    void shutdown();

    GStreamerConfig cfg_;
    GstElement* pipeline_ = nullptr;
    GstElement* appsink_ = nullptr;
    std::vector<uint8_t> frame_bytes_;
    FrameMeta meta_;
    uint64_t last_timestamp_us_ = 0;
};

GStreamerFrameSource::~GStreamerFrameSource() {
    shutdown();
}

void GStreamerFrameSource::shutdown() {
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }
    pipeline_ = nullptr;
    appsink_ = nullptr;
}

bool GStreamerFrameSource::build_pipeline() {
    if (!gst_init_once()) {
        return false;
    }

    pipeline_ = gst_pipeline_new("rawudp-pipeline");
    GstElement* source = gst_element_factory_make("libcamerasrc", "source");
    GstElement* capsfilter = gst_element_factory_make("capsfilter", "caps");
    appsink_ = gst_element_factory_make("appsink", "sink");
    if (!pipeline_ || !source || !capsfilter || !appsink_) {
        std::cerr << "Failed to create GStreamer elements\n";
        shutdown();
        return false;
    }

    GstCaps* caps = gst_caps_new_simple("video/x-bayer", "format", G_TYPE_STRING,
                                        cfg_.format.c_str(), "width", G_TYPE_INT, cfg_.width,
                                        "height", G_TYPE_INT, cfg_.height, nullptr);
    if (!caps) {
        std::cerr << "Failed to create caps\n";
        shutdown();
        return false;
    }
    g_object_set(G_OBJECT(capsfilter), "caps", caps, nullptr);
    gst_caps_unref(caps);

    g_object_set(G_OBJECT(appsink_), "drop", TRUE, "sync", FALSE, "max-buffers", 2,
                 "emit-signals", FALSE, nullptr);

    gst_bin_add_many(GST_BIN(pipeline_), source, capsfilter, appsink_, nullptr);
    if (!gst_element_link_many(source, capsfilter, appsink_, nullptr)) {
        std::cerr << "Failed to link GStreamer pipeline\n";
        shutdown();
        return false;
    }

    meta_.container_bits = cfg_.container_bits;
    meta_.effective_bits = cfg_.effective_bits;

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "Failed to set pipeline to PLAYING\n";
        shutdown();
        return false;
    }

    return true;
}

bool GStreamerFrameSource::pull_sample(uint64_t timeout_us) {
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_),
                                                     timeout_us * GST_USECOND);
    if (!sample) {
        return false;
    }

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    if (!buffer || !caps) {
        gst_sample_unref(sample);
        return false;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return false;
    }

    frame_bytes_.assign(map.data, map.data + map.size);
    gst_buffer_unmap(buffer, &map);

    GstStructure* s = gst_caps_get_structure(caps, 0);
    int width = 0;
    int height = 0;
    const gchar* fmt = gst_structure_get_string(s, "format");
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);

    meta_.width = width;
    meta_.height = height;
    meta_.pixel_format = parse_bayer_format(fmt ? fmt : "");
    meta_.container_bits = detect_container_bits(fmt ? fmt : "", meta_.container_bits);

    GstClockTime pts = GST_BUFFER_PTS(buffer);
    if (GST_CLOCK_TIME_IS_VALID(pts)) {
        last_timestamp_us_ = static_cast<uint64_t>(pts / 1000);
    } else {
        last_timestamp_us_ = monotonic_timestamp_us();
    }

    gst_sample_unref(sample);
    return true;
}

bool GStreamerFrameSource::get_frame(uint8_t*& data, size_t& size_bytes, uint64_t& timestamp_us,
                                     FrameMeta& meta) {
    if (!pipeline_) {
        if (!build_pipeline()) {
            return false;
        }
    }

    // Pull with a short timeout; drop if nothing arrives within that window.
    while (true) {
        if (!pull_sample(100000)) { // 100 ms
            continue;
        }
        data = frame_bytes_.data();
        size_bytes = frame_bytes_.size();
        timestamp_us = last_timestamp_us_;
        meta = meta_;
        return true;
    }
}

std::unique_ptr<FrameSource> make_gstreamer_source(const GStreamerConfig& cfg) {
    std::unique_ptr<GStreamerFrameSource> src = std::make_unique<GStreamerFrameSource>(cfg);
    if (!src->start()) {
        return nullptr;
    }
    return src;
}

bool GStreamerFrameSource::start() {
    return build_pipeline();
}

} // namespace rawudp

#else // RAWUDP_HAVE_GSTREAMER

namespace rawudp {

std::unique_ptr<FrameSource> make_gstreamer_source(const GStreamerConfig&) {
    std::cerr << "GStreamer support not built; rebuild with RAWUDP_HAVE_GSTREAMER\n";
    return nullptr;
}

} // namespace rawudp

#endif
