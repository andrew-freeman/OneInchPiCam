#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    gst_init(&argc, &argv);

    const char* pipeline_desc =
        "libcamerasrc camera-name=/base/axi/pcie@1000120000/rp1/i2c@88000/imx283@1a "
        "! video/x-bayer,format=rggb16,width=2784,height=1828,framerate=3/1 "
        "! queue max-size-buffers=2 leaky=downstream "
        "! appsink name=sink sync=false drop=true";

    GError* err = nullptr;
    GstElement* pipeline = gst_parse_launch(pipeline_desc, &err);
    if (!pipeline) {
        std::cerr << "Failed to create pipeline: " << err->message << "\n";
        return 1;
    }

    GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    GstAppSink* appsink = GST_APP_SINK(sink);

    gst_app_sink_set_emit_signals(appsink, false);
    gst_app_sink_set_max_buffers(appsink, 2);
    gst_app_sink_set_drop(appsink, true);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    while (true) {
        GstSample* sample =
            gst_app_sink_try_pull_sample(appsink, GST_SECOND);
        if (!sample)
            continue;

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            // map.data → pointer to RGGB16 frame
            // map.size → frame size in bytes

            std::cout << "Got frame: " << map.size << " bytes\n";

            // HERE:
            // send map.data + map.size to rawudp_sender

            gst_buffer_unmap(buffer, &map);
        }

        gst_sample_unref(sample);
    }

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    return 0;
}

