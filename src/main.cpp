// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#define YAML_NAME "hsv_settings.yml"
#define SHOW_HSV true

using namespace cv;

int HL = 0, 
	 HH = 255, 
	 SL = 0, 
	 SH = 255, 
	 VL = 0, 
	 VH = 255;

void display_histogram (int height, int width, Mat& histogram) {
	cv::Point max_hist;
	cv::minMaxLoc(histogram, 0, 0, 0, &max_hist);
	float max = histogram.at<float>(max_hist) / 1.5;
	Mat hist_display (Size(width, height), CV_8UC1);
	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {
			if (y > ((histogram.at<float>(0, x) * height) / max)) {
				hist_display.at<uchar>(height - 1 - y, x) = 0;
			} else {
				hist_display.at<uchar>(height - 1 - y, x) = 255;
			}
		}
	}
	//blur(hist_display, hist_display, Size(5, 5));
	imshow("Hist", hist_display);
}

float get_depth_scale(rs2::device dev) {
	// Go over the device's sensors
	for (rs2::sensor &sensor : dev.query_sensors()) {
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}

int main() {
	{
		FileStorage hsv_settings (YAML_NAME, FileStorage::READ);
		hsv_settings["HL"] >> HL;
		hsv_settings["HH"] >> HH;
		hsv_settings["SL"] >> SL;
		hsv_settings["SH"] >> SH;
		hsv_settings["VL"] >> VL;
		hsv_settings["VH"] >> VH;
	}

#if SHOW_HSV
	cv::namedWindow("HSV");
	cv::createTrackbar("HL", "HSV", &HL, 255);
	cv::createTrackbar("HH", "HSV", &HH, 255);
	cv::createTrackbar("SL", "HSV", &SL, 255);
	cv::createTrackbar("SH", "HSV", &SH, 255);
	cv::createTrackbar("VL", "HSV", &VL, 255);
	cv::createTrackbar("VH", "HSV", &VH, 255);
#endif

	Mat hsv, //HSV converted color image
		 hsv_threshold; //HSV thresholded binary image

	// Declare depth colorizer for pretty visualization of depth data
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	rs2::align align(RS2_STREAM_COLOR);
	rs2::pipeline pipe;
	pipe.start(cfg);
	float depth_scale = get_depth_scale(pipe.get_active_profile().get_device());

	namedWindow("Depth", WINDOW_AUTOSIZE);

	while (waitKey(1) != 'q' && cvGetWindowHandle("Depth")) {
		rs2::frameset data = align.process(pipe.wait_for_frames());
		rs2::frame depth = data.get_depth_frame();
		rs2::frame color = data.get_color_frame();

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat depth_cv(Size(depth.as<rs2::video_frame>().get_width(),
					depth.as<rs2::video_frame>().get_height()),
				CV_16UC1, (void *)depth.get_data(), Mat::AUTO_STEP);

		Mat color_cv(Size(color.as<rs2::video_frame>().get_width(),
					color.as<rs2::video_frame>().get_height()),
				CV_8UC3, (void *)color.get_data(), Mat::AUTO_STEP);

		Mat depth_slope_cv(Size(color.as<rs2::video_frame>().get_width(),
					color.as<rs2::video_frame>().get_height()),
				CV_32FC1);

		cvtColor (color_cv, hsv, COLOR_BGR2HSV);
		inRange (hsv,
				Scalar (HL, SL, VL),
				Scalar (HH, SH, VH),
				hsv_threshold);

		Mat horiz_blur_depth;
		blur(depth_cv, horiz_blur_depth, Size(5, 1));
		Sobel(horiz_blur_depth, depth_slope_cv, CV_32F, 1, 0);

		Mat histogram;
		Mat nonzero_hsv;

		depth_cv.convertTo(nonzero_hsv, CV_8U);
		bitwise_and(nonzero_hsv, hsv_threshold, nonzero_hsv);

		Mat back_projection;
		float range[] = { -0.1, 0.1 } ;
		const float* hist_range = { range };
		int hist_dim = 500;
		float scale = 3000.0;

		depth_slope_cv /= scale;
		calcHist(&depth_slope_cv, 1, 0, nonzero_hsv, histogram, 1, &hist_dim, &hist_range);

		cv::Point max_hist;
		cv::minMaxLoc(histogram, 0, 0, 0, &max_hist);
		float factor = 1.0 / histogram.at<float>(max_hist);
		calcBackProject(&depth_slope_cv, 1, 0, histogram, back_projection, &hist_range, factor);

		Mat backproject_out;
		back_projection.copyTo(backproject_out, nonzero_hsv);

		Mat is_cube_face;
		threshold(backproject_out, is_cube_face, 0.95, 255, THRESH_BINARY);

		// Update the window with new data
		imshow("Depth", is_cube_face);
		imshow("Color", color_cv);

#if SHOW_HSV
		imshow("HSV", hsv_threshold);
#endif
	}

	FileStorage hsv_settings (YAML_NAME, FileStorage::WRITE);
	hsv_settings << "HL" << HL;
	hsv_settings << "HH" << HH;
	hsv_settings << "SL" << SL;
	hsv_settings << "SH" << SH;
	hsv_settings << "VL" << VL;
	hsv_settings << "VH" << VH;
	hsv_settings.release();

	return EXIT_SUCCESS;
}
