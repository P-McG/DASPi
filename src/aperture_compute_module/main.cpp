
/*
 *### Steps
1. **Prepare the Frames**: Ensure the frames are in the correct format (e.g., raw I420).
2. **Set Up GStreamer**: Use GStreamer to encode the frames and send them over a network (e.g., UDP).
3. **Control the Frame Rate**: Use a timer (like `std::chrono` or similar) to ensure frames are sent at 30 FPS. 
 
### Explanation
1. **Pipeline**:
   - The `appsrc` element allows you to push raw frames into the pipeline.
   - The `x264enc` element encodes the frames into H.264.
   - The `udpsink` element sends the frames over UDP.

2. **Frame Timing**:
   - Use `std::chrono` to control the frame rate.
   - Each frame is sent after `1000 / 30` milliseconds (i.e., 33.33 ms).

3. **Dummy Frame**:
   - Replace the dummy frame data with actual frame data (e.g., captured from a camera).

4. **Caps**:
   - Ensure the `caps` match your frame format, resolution, and framerate.

5. **Error Handling**:
   - Basic error handling is included, but you can expand it as needed.

 */

#include <gst/gst.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <vector>

// Function to send a single frame
void send_frame(GstElement* appsrc, const std::vector<uint8_t>& frame_data) {
	GstBuffer* buffer = gst_buffer_new_allocate(nullptr, frame_data.size(), nullptr);
	gst_buffer_fill(buffer, 0, frame_data.data(), frame_data.size());

	GST_BUFFER_PTS(buffer) = gst_clock_get_time(gst_system_clock_obtain());
	GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 30); // Duration for 30 FPS

	GstFlowReturn ret;
	g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
	gst_buffer_unref(buffer);

	if (ret != GST_FLOW_OK) {
    	std::cerr << "Error sending buffer to appsrc\n";
	}
}

int main(int argc, char* argv[]) {
	gst_init(&argc, &argv);

	// Create GStreamer pipeline
	GstElement* pipeline = gst_parse_launch(
//    	"appsrc name=appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast ! rtph264pay ! udpsink host=127.0.0.1 port=5000",
    	"appsrc name=appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast ! rtph264pay ! udpsink host=127.0.0.1 port=5000",
    	nullptr);

	if (!pipeline) {
    	std::cerr << "Failed to create pipeline\n";
    	return -1;
	}

	GstElement* appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "appsrc");
	g_object_set(G_OBJECT(appsrc), "caps",
             	gst_caps_new_simple("video/x-raw",
                                 	"format", G_TYPE_STRING, "I420",
                                 	"width", G_TYPE_INT, 1280,
                                 	"height", G_TYPE_INT, 720,
                                 	"framerate", GST_TYPE_FRACTION, 30, 1,
                                 	nullptr),
             	nullptr);

	gst_element_set_state(pipeline, GST_STATE_PLAYING);

	// Prepare a dummy frame (replace this with your actual frame data)
	std::vector<uint8_t> frame_data(1280 * 720 * 3 / 2, 0); // Assuming I420 format

	auto start_time = std::chrono::steady_clock::now();

	// Send frames at 30 FPS
	for (int i = 0; i < 300; ++i) { // Send 300 frames (~10 seconds)
    	send_frame(appsrc, frame_data);

    	// Wait for the next frame time
    	auto frame_time = start_time + std::chrono::milliseconds(1000 / 30 * (i + 1));
    	std::this_thread::sleep_until(frame_time);
	}

	// Cleanup
	gst_element_send_event(pipeline, gst_event_new_eos());
	gst_element_set_state(pipeline, GST_STATE_NULL);
	gst_object_unref(appsrc);
	gst_object_unref(pipeline);
	gst_deinit();

	return 0;
}
