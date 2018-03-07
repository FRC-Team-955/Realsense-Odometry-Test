#include <iostream>
#include <vector>
#include <memory>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>

using namespace cv;

const float min_depth = 0.2f;        // in meters
const float max_depth = 10.0f;        // in meters
const float max_depth_diff = 0.10f;  // in meters
const float max_points_part = 0.09f;

float get_depth_scale(rs2::device dev) {
	// Go over the device's sensors
	for (rs2::sensor &sensor : dev.query_sensors()) {
		std::cout << sensor.get_stream_profiles().back().as<rs2::video_stream_profile>().get_intrinsics().fx << std::endl;
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}

int main () {
	std::cout << "A E S T H E T I C" << std::endl;	

	rs2::config cfg;
	//cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
	//cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

	rs2::align align(RS2_STREAM_COLOR);

	rs2::pipeline pipe;

	pipe.start(cfg);

	float depth_scale = get_depth_scale(pipe.get_active_profile().get_device());


	rs2::frameset data = align.process(pipe.wait_for_frames());
	auto intrinsics = data.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

	/*
	auto intrinsics = pipe
		.get_active_profile()
		.get_device()
		.query_sensors()
		.back()
		.get_stream_profiles()
		.back()
		.as<rs2::video_stream_profile>()
		.get_intrinsics();
		*/
#define PRINTINFO(NAME) std::cout << #NAME << " " << intrinsics.NAME << std::endl;
	PRINTINFO(width);
	PRINTINFO(height);
	PRINTINFO(ppx);
	PRINTINFO(ppy);
	PRINTINFO(fx);
	PRINTINFO(fy);
#undef PRINTINFO
	float vals[] = {
		intrinsics.fx,  0.0,            intrinsics.ppx,
		0.0,            intrinsics.fy,  intrinsics.ppy,
		0.0,            0.0,            1.0};
	const Mat cameraMatrix = Mat(3, 3, CV_32FC1, vals);

	std::vector<int> iterCounts(4);
	iterCounts[0] = 7;
	iterCounts[1] = 7;
	iterCounts[2] = 7;
	iterCounts[3] = 10;

	std::vector<float> minGradMagnitudes(4);
	minGradMagnitudes[0] = 12;
	minGradMagnitudes[1] = 5;
	minGradMagnitudes[2] = 3;
	minGradMagnitudes[3] = 1;

	std::shared_ptr<rgbd::RgbdOdometry> odom = std::make_shared<rgbd::RgbdOdometry>(
			cameraMatrix, min_depth, max_depth, max_depth_diff, iterCounts,
			minGradMagnitudes, max_points_part,
			rgbd::Odometry::RIGID_BODY_MOTION);

	Mat traj = Mat::zeros(800, 800, CV_8UC3);

	Mat rotationMatrix, translationMatrix;

	Mat gray_last, scaled_depth_last, bool_depth_last;
	bool first = true;
	bool second = true;
	unsigned int hmm = 0;
	while (waitKey(1) != 'q') {
		// Align and create realsense images
		rs2::frameset data = align.process(pipe.wait_for_frames());
		rs2::frame depth = data.get_depth_frame();
		rs2::frame color = data.get_color_frame();

		// Create matrices
		Mat depth_cv(Size(depth.as<rs2::video_frame>().get_width(),
					depth.as<rs2::video_frame>().get_height()),
				CV_16UC1, (void *)depth.get_data(), Mat::AUTO_STEP);

		Mat color_cv(Size(color.as<rs2::video_frame>().get_width(),
					color.as<rs2::video_frame>().get_height()),
				CV_8UC3, (void *)color.get_data(), Mat::AUTO_STEP);

		Mat gray, scaled_depth, bool_depth;
		cvtColor(color_cv, gray, COLOR_BGR2GRAY);
		depth_cv.convertTo(scaled_depth, CV_32FC1, depth_scale);
		depth_cv.convertTo(bool_depth, CV_8UC1);

		Mat rigidTransform;
		Mat rotationMat;
		Mat translateMat;
		if (!first) {
			bool isSuccess = odom->compute(
					gray_last,
					scaled_depth_last,
					//Mat(),
					bool_depth_last,
					gray,
					scaled_depth,
					bool_depth,
					//Mat(),
					rigidTransform);

			if (isSuccess) {
				Mat rotationMat = rigidTransform(cv::Rect(0, 0, 3, 3)).clone();
				Mat translateMat = rigidTransform(cv::Rect(3, 0, 1, 3)).clone();
				if (second) {
					translationMatrix = translateMat.clone();
					rotationMatrix = rotationMat.clone();
					second = false;
				}
				translationMatrix = translationMatrix + (rotationMatrix * translateMat);
				rotationMatrix = rotationMat * rotationMatrix;
				int x =
					int(60.0 * translationMatrix.at<double>(0)) +
					800 / 2;
				int y =
					int(60.0 * translationMatrix.at<double>(2)) +
					800 / 2;

				circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);
				rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0),
						CV_FILLED);
				/*
					std::cout << 
					translationMatrix.at<float>(0) << " : " <<
					translationMatrix.at<float>(1) << " : " <<
					translationMatrix.at<float>(2) << " : " <<
					std::endl;
					*/
			} else {
				std::cerr << hmm++ << std::endl;
			}

		}
		imshow("RGBD Trajectory", traj);

		//imshow("Color", gray);
		//imshow("Depth", scaled_depth);
		first = false;
		gray_last = gray;
		scaled_depth_last = scaled_depth;
		bool_depth_last = bool_depth;
	}
}
