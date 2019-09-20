// This file is part of the Orbbec Astra SDK [https://orbbec3d.com]
// Copyright (c) 2015-2017 Orbbec 3D
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Be excellent to each other.
#include "stdafx.h"
#include <SFML/Graphics.hpp>
#include <astra/astra.hpp>
#include <iostream>
#include <cstring>
#include <math.h>
#include <fstream>

//Kinova::Header
#include <Windows.h>
#include "kinova/CommunicationLayerWindows.h"
#include "kinova/CommandLayer.h"
#include <conio.h>
#include "kinova/KinovaTypes.h"
//Kinova::Header [end]

// OpenCV 4.1.0
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <royale.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sample_utils/PlatformResources.hpp>

using namespace std;

//Kinova API
//A handle to the API.
HINSTANCE commandLayer_handle;

//Function pointers to the functions we need
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MySendBasicTrajectory)(TrajectoryPoint command);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);
int(*MyMoveHome)();
int(*MyInitFingers)();
int(*MyGetAngularCommand)(AngularPosition &);
int(*MyGetCartesianCommand)(CartesianPosition &);

int(*MyStartForceControl)();
int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
//Kinova API End

//Global Vars
int NumofBodies = 0;
int FirstDetect = 0;
astra::Vector3f Right_Hand_Pos = astra::Vector3f();
astra::Vector3f Left_Hand_Pos = astra::Vector3f();
int Right_Hand_Grip = -1;
int Left_Hand_Grip = -1;
int imagetype = -1;


int Grasping_Start = 0;
cv::Vec3f d_XYZt_Kinova = { 0,0,0 };

enum G_Mode{
	Direct,
	SA,
	Joystick
};

G_Mode G_Mode_S = Joystick;
string filename = "(KYJ)_Direct_Trial_01.txt";

#define PI 3.141592

class sfLine : public sf::Drawable
{
public:
	sfLine(const sf::Vector2f& point1, const sf::Vector2f& point2, sf::Color color, float thickness)
		: color_(color)
	{
		const sf::Vector2f direction = point2 - point1;
		const sf::Vector2f unitDirection = direction / std::sqrt(direction.x*direction.x + direction.y*direction.y);
		const sf::Vector2f normal(-unitDirection.y, unitDirection.x);

		const sf::Vector2f offset = (thickness / 2.f) * normal;

		vertices_[0].position = point1 + offset;
		vertices_[1].position = point2 + offset;
		vertices_[2].position = point2 - offset;
		vertices_[3].position = point1 - offset;

		for (int i = 0; i<4; ++i)
			vertices_[i].color = color;
	}

	void draw(sf::RenderTarget &target, sf::RenderStates states) const
	{
		target.draw(vertices_, 4, sf::Quads, states);
	}

private:
	sf::Vertex vertices_[4];
	sf::Color color_;
};

class BodyVisualizer : public astra::FrameListener
{
public:
	static sf::Color get_body_color(std::uint8_t bodyId)
	{
		if (bodyId == 0)
		{
			// Handle no body separately - transparent
			return sf::Color(0x00, 0x00, 0x00, 0x00);
		}
		// Case 0 below could mean bodyId == 25 or
		// above due to the "% 24".
		switch (bodyId % 6) {
		case 0:
			return sf::Color(0x00, 0x88, 0x00, 0xFF);
		case 1:
			return sf::Color(0x00, 0x00, 0xFF, 0xFF);
		case 2:
			return sf::Color(0x88, 0x00, 0x00, 0xFF);
		case 3:
			return sf::Color(0x00, 0xFF, 0x00, 0xFF);
		case 4:
			return sf::Color(0x00, 0x00, 0x88, 0xFF);
		case 5:
			return sf::Color(0xFF, 0x00, 0x00, 0xFF);
		default:
			return sf::Color(0xAA, 0xAA, 0xAA, 0xFF);
		}
	}

	void init_depth_texture(int width, int height)
	{
		if (displayBuffer_ == nullptr || width != depthWidth_ || height != depthHeight_)
		{
			depthWidth_ = width;
			depthHeight_ = height;
			int byteLength = depthWidth_ * depthHeight_ * 4;

			displayBuffer_ = BufferPtr(new uint8_t[byteLength]);
			std::memset(displayBuffer_.get(), 0, byteLength);

			texture_.create(depthWidth_, depthHeight_);
			sprite_.setTexture(texture_, true);
			sprite_.setPosition(0, 0);
		}
	}

	void init_overlay_texture(int width, int height)
	{
		if (overlayBuffer_ == nullptr || width != overlayWidth_ || height != overlayHeight_)
		{
			overlayWidth_ = width;
			overlayHeight_ = height;
			int byteLength = overlayWidth_ * overlayHeight_ * 4;

			overlayBuffer_ = BufferPtr(new uint8_t[byteLength]);
			std::fill(&overlayBuffer_[0], &overlayBuffer_[0] + byteLength, 0);

			overlayTexture_.create(overlayWidth_, overlayHeight_);
			overlaySprite_.setTexture(overlayTexture_, true);
			overlaySprite_.setPosition(0, 0);
		}
	}

	void check_fps()
	{
		double fpsFactor = 0.02;

		std::clock_t newTimepoint = std::clock();
		long double frameDuration = (newTimepoint - lastTimepoint_) / static_cast<long double>(CLOCKS_PER_SEC);

		frameDuration_ = frameDuration * fpsFactor + frameDuration_ * (1 - fpsFactor);
		lastTimepoint_ = newTimepoint;
		double fps = 1.0 / frameDuration_;

		//printf("FPS: %3.1f (%3.4Lf ms)\n", fps, frameDuration_ * 1000);
	}

	void processDepth(astra::Frame& frame)
	{
		const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();

		if (!depthFrame.is_valid()) { return; }

		int width = depthFrame.width();
		int height = depthFrame.height();

		init_depth_texture(width, height);

		const int16_t* depthPtr = depthFrame.data();
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				int index = (x + y * width);
				int index4 = index * 4;

				int16_t depth = depthPtr[index];
				uint8_t value = depth % 255;
				// Normalize depth
				// uint8_t value = round((depth / 3400) * 255);			// Too Dark..				

				displayBuffer_[index4] = value;
				displayBuffer_[index4 + 1] = value;
				displayBuffer_[index4 + 2] = value;
				displayBuffer_[index4 + 3] = 255;
			}
		}

		texture_.update(displayBuffer_.get());
	}

	void processBodies(astra::Frame& frame)
	{
		astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();

		jointPositions_.clear();
		circles_.clear();
		circleShadows_.clear();
		boneLines_.clear();
		boneShadows_.clear();

		if (!bodyFrame.is_valid() || bodyFrame.info().width() == 0 || bodyFrame.info().height() == 0)
		{
			clear_overlay();
			NumofBodies = 0;
			return;
		}

		const float jointScale = bodyFrame.info().width() / 120.f;

		const auto& bodies = bodyFrame.bodies();

		// Detect Human Body -> Immediately Stop
		NumofBodies = bodies.size();
		if (NumofBodies > 0 && FirstDetect == 0) {
			FirstDetect = 1;
			return;
		}

		for (auto& body : bodies)
		{
			/*printf("Processing frame #%d body %d left hand: %u\n",
			bodyFrame.frame_index(), body.id(), unsigned(body.hand_poses().left_hand()))*/;
		for (auto& joint : body.joints())
		{
			jointPositions_.push_back(joint.depth_position());
		}

		update_body(body, jointScale);
		}

		const auto& floor = bodyFrame.floor_info(); //floor
		if (floor.floor_detected())
		{
			const auto& p = floor.floor_plane();

		}

		const auto& bodyMask = bodyFrame.body_mask();
		const auto& floorMask = floor.floor_mask();

		update_overlay(bodyMask, floorMask);
	}

	void update_body(astra::Body body,
		const float jointScale)
	{
		const auto& joints = body.joints();

		if (joints.empty())
		{
			return;
		}

		for (const auto& joint : joints)
		{
			astra::JointType type = joint.type();
			const auto& pos = joint.depth_position();

			if (joint.status() == astra::JointStatus::NotTracked)
			{
				continue;
			}

			auto radius = jointRadius_ * jointScale; // pixels
			sf::Color circleShadowColor(0, 0, 0, 255);

			auto color = sf::Color(0x00, 0xFF, 0x00, 0xFF);

			if (type == astra::JointType::LeftHand)
			{
				if (astra::HandPose::Grip == body.hand_poses().left_hand()) {
					Left_Hand_Grip = 1;
					radius *= 1.5f;
					circleShadowColor = sf::Color(255, 255, 255, 255);
					color = sf::Color(0x00, 0xAA, 0xFF, 0xFF);
				}
				else {
					Left_Hand_Grip = 0;
				}
			}

			if (type == astra::JointType::RightHand)
			{
				if (astra::HandPose::Grip == body.hand_poses().right_hand()) {
					Right_Hand_Grip = 1;
					radius *= 1.5f;
					circleShadowColor = sf::Color(255, 255, 255, 255);
					color = sf::Color(0x00, 0xAA, 0xFF, 0xFF);
				}
				else {
					Right_Hand_Grip = 0;
				}
			}

			const auto shadowRadius = radius + shadowRadius_ * jointScale;
			const auto radiusDelta = shadowRadius - radius;

			sf::CircleShape circle(radius);

			circle.setFillColor(sf::Color(color.r, color.g, color.b, 255));
			circle.setPosition(pos.x - radius, pos.y - radius);
			circles_.push_back(circle);

			sf::CircleShape shadow(shadowRadius);
			shadow.setFillColor(circleShadowColor);
			shadow.setPosition(circle.getPosition() - sf::Vector2f(radiusDelta, radiusDelta));
			circleShadows_.push_back(shadow);
		}

		update_bone(joints, jointScale, astra::JointType::Head, astra::JointType::ShoulderSpine);

		update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::LeftShoulder);
		update_bone(joints, jointScale, astra::JointType::LeftShoulder, astra::JointType::LeftElbow);
		update_bone(joints, jointScale, astra::JointType::LeftElbow, astra::JointType::LeftHand);

		update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::RightShoulder);
		update_bone(joints, jointScale, astra::JointType::RightShoulder, astra::JointType::RightElbow);
		update_bone(joints, jointScale, astra::JointType::RightElbow, astra::JointType::RightHand);

		update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::MidSpine);
		update_bone(joints, jointScale, astra::JointType::MidSpine, astra::JointType::BaseSpine);

		update_bone(joints, jointScale, astra::JointType::BaseSpine, astra::JointType::LeftHip);
		update_bone(joints, jointScale, astra::JointType::LeftHip, astra::JointType::LeftKnee);
		update_bone(joints, jointScale, astra::JointType::LeftKnee, astra::JointType::LeftFoot);

		update_bone(joints, jointScale, astra::JointType::BaseSpine, astra::JointType::RightHip);
		update_bone(joints, jointScale, astra::JointType::RightHip, astra::JointType::RightKnee);
		update_bone(joints, jointScale, astra::JointType::RightKnee, astra::JointType::RightFoot);
	}

	void update_bone(const astra::JointList& joints,
		const float jointScale, astra::JointType j1,
		astra::JointType j2)
	{
		const auto& joint1 = joints[int(j1)];
		const auto& joint2 = joints[int(j2)];
		const auto& jp1w = joint1.world_position();
		const auto& jp2w = joint2.world_position();

		switch (j1) {
		case astra::JointType::LeftElbow:
			switch (j2) {
			case astra::JointType::LeftHand:
				Left_Hand_Pos = jp2w;
			}
		case astra::JointType::RightElbow:
			switch (j2) {
			case astra::JointType::RightHand:
				Right_Hand_Pos = jp2w;
			}
		}

		if (joint1.status() == astra::JointStatus::NotTracked ||
			joint2.status() == astra::JointStatus::NotTracked)
		{
			//don't render bones between untracked joints
			return;
		}

		const auto& jp1 = joint1.depth_position();
		const auto& jp2 = joint2.depth_position();

		auto p1 = sf::Vector2f(jp1.x, jp1.y);
		auto p2 = sf::Vector2f(jp2.x, jp2.y);

		sf::Color color(255, 255, 255, 255);
		float thickness = lineThickness_ * jointScale;
		if (joint1.status() == astra::JointStatus::LowConfidence ||
			joint2.status() == astra::JointStatus::LowConfidence)
		{
			color = sf::Color(128, 128, 128, 255);
			thickness *= 0.5f;
		}

		boneLines_.push_back(sfLine(p1,
			p2,
			color,
			thickness));
		const float shadowLineThickness = thickness + shadowRadius_ * jointScale * 2.f;
		boneShadows_.push_back(sfLine(p1,
			p2,
			sf::Color(0, 0, 0, 255),
			shadowLineThickness));
	}

	void update_overlay(const astra::BodyMask& bodyMask,
		const astra::FloorMask& floorMask)
	{
		const auto* bodyData = bodyMask.data();
		const auto* floorData = floorMask.data();
		const int width = bodyMask.width();
		const int height = bodyMask.height();

		init_overlay_texture(width, height);

		const int length = width * height;

		for (int i = 0; i < length; i++)
		{
			const auto bodyId = bodyData[i];
			const auto isFloor = floorData[i];

			sf::Color color(0x0, 0x0, 0x0, 0x0);

			if (bodyId != 0)
			{
				color = get_body_color(bodyId);
			}
			else if (isFloor != 0)
			{
				color = sf::Color(0x0, 0x0, 0xFF, 0x88);
			}

			const int rgbaOffset = i * 4;
			overlayBuffer_[rgbaOffset] = color.r;
			overlayBuffer_[rgbaOffset + 1] = color.g;
			overlayBuffer_[rgbaOffset + 2] = color.b;
			overlayBuffer_[rgbaOffset + 3] = color.a;
		}

		overlayTexture_.update(overlayBuffer_.get());
	}

	void clear_overlay()
	{
		int byteLength = overlayWidth_ * overlayHeight_ * 4;
		std::fill(&overlayBuffer_[0], &overlayBuffer_[0] + byteLength, 0);

		overlayTexture_.update(overlayBuffer_.get());
	}

	virtual void on_frame_ready(astra::StreamReader& reader,
		astra::Frame& frame) override
	{
		processDepth(frame);
		processBodies(frame);

		check_fps();
	}

	void draw_bodies(sf::RenderWindow& window)
	{
		const float scaleX = window.getView().getSize().x / overlayWidth_;
		const float scaleY = window.getView().getSize().y / overlayHeight_;

		sf::RenderStates states;
		sf::Transform transform;
		transform.scale(scaleX, scaleY);
		states.transform *= transform;

		for (const auto& bone : boneShadows_)
			window.draw(bone, states);

		for (const auto& c : circleShadows_)
			window.draw(c, states);

		for (const auto& bone : boneLines_)
			window.draw(bone, states);

		for (auto& c : circles_)
			window.draw(c, states);

	}

	void draw_to(sf::RenderWindow& window)
	{
		if (displayBuffer_ != nullptr)
		{
			const float scaleX = window.getView().getSize().x / depthWidth_;
			const float scaleY = window.getView().getSize().y / depthHeight_;
			sprite_.setScale(scaleX, scaleY);

			window.draw(sprite_); // depth
		}

		if (overlayBuffer_ != nullptr)
		{
			const float scaleX = window.getView().getSize().x / overlayWidth_;
			const float scaleY = window.getView().getSize().y / overlayHeight_;
			overlaySprite_.setScale(scaleX, scaleY);
			window.draw(overlaySprite_); //bodymask and floormask
		}

		draw_bodies(window);
	}

private:
	long double frameDuration_{ 0 };
	std::clock_t lastTimepoint_{ 0 };
	sf::Texture texture_;
	sf::Sprite sprite_;

	using BufferPtr = std::unique_ptr < uint8_t[] >;
	BufferPtr displayBuffer_{ nullptr };

	std::vector<astra::Vector2f> jointPositions_;

	int depthWidth_{ 0 };
	int depthHeight_{ 0 };
	int overlayWidth_{ 0 };
	int overlayHeight_{ 0 };

	std::vector<sfLine> boneLines_;
	std::vector<sfLine> boneShadows_;
	std::vector<sf::CircleShape> circles_;
	std::vector<sf::CircleShape> circleShadows_;

	float lineThickness_{ 0.5f }; // pixels
	float jointRadius_{ 1.0f };   // pixels
	float shadowRadius_{ 0.5f };  // pixels

	BufferPtr overlayBuffer_{ nullptr };
	sf::Texture overlayTexture_;
	sf::Sprite overlaySprite_;

};

astra::DepthStream configure_depth(astra::StreamReader& reader)
{
	auto depthStream = reader.stream<astra::DepthStream>();

	//We don't have to set the mode to start the stream, but if you want to here is how:
	astra::ImageStreamMode depthMode;

	depthMode.set_width(640);
	depthMode.set_height(480);
	depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
	depthMode.set_fps(30);

	depthStream.set_mode(depthMode);

	return depthStream;
}


// Pico Flexx
class MyListener : public royale::IDepthDataListener
{

public:

	MyListener() : undistortImage(false)
	{
	}

	void onNewData(const royale::DepthData *data)
	{
		// this callback function will be called for every new depth frame

		std::lock_guard<std::mutex> lock(flagMutex);

		float *zRowPtr, *grayRowPtr = NULL;
		// zImage
		zImage.create(cv::Size(data->width, data->height), CV_32FC1);
		zImage = cv::Scalar::all(0); // set the image to zero

		int k = 0;
		for (int y = 0; y < zImage.rows; y++)
		{
			zRowPtr = zImage.ptr<float>(y);

			for (int x = 0; x < zImage.cols; x++, k++)
			{
				auto curPoint = data->points.at(k);
				if (curPoint.depthConfidence > 0)
				{
					// if the point is valid, map the pixel from 3D world
					// coordinates to a 2D plane (this will distort the image)					
					zRowPtr[x] = adjustZValue(curPoint.z);
				}
				else {
					zRowPtr[x] = 255; //distortion part => black
				}
			}
		}
		zImage8.create(cv::Size(data->width, data->height), CV_8UC1);
		zImage.convertTo(zImage8, CV_8UC1);		// normalize(zImage, zImage8, 0, 255, NORM_MINMAX, CV_8UC1)

		if (undistortImage)
		{
			// call the undistortion function on the z image
			cv::Mat temp = zImage8.clone();
			undistort(temp, zImage8, cameraMatrix, distortionCoefficients);
		}

		scaledZImage.create(cv::Size(data->width * 4, data->height * 4), CV_8UC1);		// scale and display the depth image
		cv::resize(zImage8, scaledZImage, scaledZImage.size());

		//cv::imshow("Depth", scaledZImage);

		// grayImage
		grayImage.create(cv::Size(data->width, data->height), CV_32FC1);
		grayImage = cv::Scalar::all(0); // set the image to zero

		k = 0;
		for (int y = 0; y < grayImage.rows; y++)
		{
			grayRowPtr = grayImage.ptr<float>(y);

			for (int x = 0; x < grayImage.cols; x++, k++)
			{
				auto curPoint = data->points.at(k);
				if (curPoint.depthConfidence > 0)
				{
					// if the point is valid, map the pixel from 3D world
					// coordinates to a 2D plane (this will distort the image)
					grayRowPtr[x] = adjustGrayValue(curPoint.grayValue);
				}
				else {
					grayRowPtr[x] = 255; //distortion part => black
				}
			}
		}

		grayImage8.create(cv::Size(data->width, data->height), CV_8UC1);
		grayImage.convertTo(grayImage8, CV_8UC1);		// normalize(grayImage, grayImage8, 0, 255, NORM_MINMAX, CV_8UC1)

		if (undistortImage)
		{
			// call the undistortion function on the gray image
			cv::Mat temp = grayImage8.clone();
			undistort(temp, grayImage8, cameraMatrix, distortionCoefficients);
		}

		// scale and display the gray image
		scaledGrayImage.create(cv::Size(data->width * 4, data->height * 4), CV_8UC1);
		cv::resize(grayImage8, scaledGrayImage, scaledGrayImage.size());

		//cv::imshow("Gray", scaledGrayImage);

		cv::Vec4f pOutLine;

		imagetype = 0;

		result_ZImage.create(cv::Size(data->width * 4, data->height * 4), CV_8UC1);
		overlay_Bounding_Box(scaledZImage, data, result_ZImage, &pOutLine);
		Kinova_calib(&pOutLine, &d_XYZt_Kinova);
		cv::imshow("Test_Z", result_ZImage);

		//imagetype = 1;		
		//result_GImage.create(Size(data->width * 4, data->height * 4), CV_8UC1);
		//overlay_Bounding_Box(scaledGrayImage, data, result_GImage, &pOutLine);// read from gray image
		//Kinova_calib(&pOutLine, &d_XYZt_Kinova);
		//imshow("Test_G", result_GImage);
	}

	void Kinova_calib(cv::Vec4f* OutLine, cv::Vec3f *d_XYZt_Kinova) {
		// Kinova Gripper Initial : (Z=0, theta=0) ===>> pico : (X =470/960, Y=174/720), real_world : (0.055 m, 0.02 m)
		float dx = ((*OutLine)[2] * 0.0322 - 15)*0.01;
		float dy = ((*OutLine)[3] * -0.032 + 4)*0.01;
		float dth = atan((float)(*OutLine)[1] / (float)(*OutLine)[0]) - PI / 2.0f + 0.2f;

		*d_XYZt_Kinova = { dx, dy, dth };

		/*	cout << " Out : " << dx << ", " << dy << ", " << dth << endl;
		cout << " OutLine : " << *OutLine << endl;
		cout << " d_Kinova : " << *d_XYZt_Kinova << endl;*/
	}


	void overlay_Bounding_Box(cv::Mat tImage, const royale::DepthData *data, cv::Mat result_tImage, cv::Vec4f *pOutLine) {

		normalize(tImage, tImage8, 0, 255, cv::NORM_MINMAX, CV_8UC1);

		if (imagetype == 0) { // Z image
			cv::threshold(~tImage8, tImage8, 150, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);// +cv::THRESH_BINARY); cv::THRESH_OTSU
																							//Adaptive Thresholding을 한다.
																							//adaptiveThreshold(tImage8, tImage8, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 5, 10);
		}
		else if (imagetype == 1) { // Gray image
			cv::threshold(tImage8, tImage8, 127, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);// +cv::THRESH_BINARY);
		}

		cv::GaussianBlur(tImage8, tImage8, cv::Size(5, 5), 1, 1, 1);
		cv::Mat tImageMat = tImage8.clone();


		double area, max_area = 0;
		int max_id = 0;
		cv::Rect bounding_rect;
		vector<cv::Vec4i> hierarchy;
		vector<vector<cv::Point> > contours;
		findContours(tImageMat, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < contours.size(); i++) {
			area = contourArea(contours[i], false);
			if (area > max_area) {
				max_area = area;
				max_id = i;
				bounding_rect = boundingRect(contours[i]);
			}
		}
		rectangle(tImage8, bounding_rect, cv::Scalar(100, 100, 100), 1, 0);

		cv::Vec4f OutLine;
		fitLine(cv::Mat(contours[max_id]), OutLine, cv::DIST_L2, 0, 0.01, 0.01);
		/****************/
		//cout << "OutLine Data" << OutLine << endl;
		cv::line(tImage8, cv::Point(OutLine[2], OutLine[3]), cv::Point(100 * OutLine[0] + OutLine[2], 100 * OutLine[1] + OutLine[3]), cv::Scalar(50, 50, 50), 2);
		//line(tImage, Point(0,0), Point(200,200), Scalar(50, 50, 50), 2);
		cv::circle(tImage8, cv::Point(OutLine[2], OutLine[3]), 4, cv::Scalar(0, 255, 0), 2);
		cv::circle(tImage8, cv::Point(20 * OutLine[0] + OutLine[2], 20 * OutLine[1] + OutLine[3]), 4, cv::Scalar(0, 255, 0), 2);

		//Sleep(1);		
		/*scaledtImage.create(Size(data->width * 4, data->height * 4), CV_8UC1);
		resize(tImage8, scaledtImage, scaledtImage.size());
		imshow("Test", scaledtImage);*/
		*pOutLine = OutLine;

		resize(tImage8, result_tImage, result_tImage.size());
	}

	void setLensParameters(const royale::LensParameters &lensParameters)
	{
		// Construct the camera matrix
		// (fx   0    cx)
		// (0    fy   cy)
		// (0    0    1 )
		cameraMatrix = (cv::Mat1d(3, 3) << lensParameters.focalLength.first, 0, lensParameters.principalPoint.first,
			0, lensParameters.focalLength.second, lensParameters.principalPoint.second,
			0, 0, 1);

		// Construct the distortion coefficients
		// k1 k2 p1 p2 k3
		distortionCoefficients = (cv::Mat1d(1, 5) << lensParameters.distortionRadial[0],
			lensParameters.distortionRadial[1],
			lensParameters.distortionTangential.first,
			lensParameters.distortionTangential.second,
			lensParameters.distortionRadial[2]);
	}

	void toggleUndistort()
	{
		std::lock_guard<std::mutex> lock(flagMutex);
		undistortImage = !undistortImage;
	}

private:

	// adjust z value to fit fixed scaling, here max dist is 2.5m
	// the max dist here is used as an example and can be modified
	float adjustZValue(float zValue)
	{
		float clampedDist = std::min(0.44f, zValue);
		float newZValue = clampedDist / 0.44f * 255.0f;
		return newZValue;
	}

	// adjust gray value to fit fixed scaling, here max value is 180
	// the max value here is used as an example and can be modified
	float adjustGrayValue(uint16_t grayValue)
	{
		float clampedVal = std::min(2350.0f, grayValue * 1.0f); // 180.0f (Original clamp Value)
		float newGrayValue = clampedVal / 2350.f * 255.0f;
		return newGrayValue;
	}

	// define images for depth and gray
	// and for their 8Bit and scaled versions
	cv::Mat zImage, zImage8, scaledZImage, result_ZImage;
	cv::Mat grayImage, grayImage8, scaledGrayImage, result_GImage;

	// 2018Oct27 YJ
	cv::Mat tImage, tImage8, scaledtImage; //Test image

									   // lens matrices used for the undistortion of
									   // the image
	cv::Mat cameraMatrix;
	cv::Mat distortionCoefficients;

	std::mutex flagMutex;
	bool undistortImage;
};

int main(int argc, char** argv)
{
	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

	int programResult = 0;

	//Initialise the function pointer from the API
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyGetDevices = (int(*)(KinovaDevice[MAX_KINOVA_DEVICE], int&)) GetProcAddress(commandLayer_handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
	MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
	MyGetAngularCommand = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularCommand");
	MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");
	MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
	MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");
	MyStartForceControl = (int(*)()) GetProcAddress(commandLayer_handle, "StartForceControl");
	MyRunGravityZEstimationSequence = (int(*)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) GetProcAddress(commandLayer_handle, "RunGravityZEstimationSequence");

	//Verify that all functions has been loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
		(MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetAngularCommand == NULL) ||
		(MyMoveHome == NULL) || (MyInitFingers == NULL) || (MyGetCartesianCommand == NULL))

	{
		std::cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
		programResult = 0;
		_getch();
		return 0;
	}

	std::cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

	int result = (*MyInitAPI)();

	AngularPosition currentCommand;

	std::cout << "Initialization's result :" << result << endl;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);
	std::cout << "Device Count : " << devicesCount << endl;

	char noarm = 'n';
	char single = 'n';
	if (devicesCount == 0) {
		std::cout << "No Arm Detected. Continue without arm control? (y/n)";
		cin >> noarm;
		if (noarm == 'n' || noarm == 'N') {
			return 0;
		}

	}
	else {
		std::cout << "Initializing Right Arm (S/N : " << list[0].SerialNumber << ")" << endl;
		MySetActiveDevice(list[0]);
		std::cout << "Send the robot to HOME position" << endl;
		MyMoveHome();

		std::cout << "Initializing the fingers" << endl;
		MyInitFingers();

		TrajectoryPoint Initial_Position;
		Initial_Position.InitStruct();
		Initial_Position.Position.Type = CARTESIAN_POSITION;
		CartesianPosition currentPosition;
		MyGetCartesianCommand(currentPosition);

		Initial_Position.Position.CartesianPosition.X = -0.126;
		Initial_Position.Position.CartesianPosition.Y = -0.396;
		Initial_Position.Position.CartesianPosition.Z = 0.0046;
		Initial_Position.Position.CartesianPosition.ThetaX = -3.14;
		Initial_Position.Position.CartesianPosition.ThetaY = 0.0068;
		Initial_Position.Position.CartesianPosition.ThetaZ = 0.2835;
		MySendBasicTrajectory(Initial_Position);
	}


	int Tmap[1] = { 0 };
	int countloop = 0;
	int Maxloop = 1000;

	// Start Astra Camera Initialization 
	std::cout << "Start Camera Initialization" << endl;
	astra::initialize();

	const char* licenseString = "<INSERT LICENSE KEY HERE>";
	orbbec_body_tracking_set_license(licenseString);

	sf::RenderWindow window(sf::VideoMode(600, 450), "Simple Body Viewer");

#ifdef _WIN32
	auto fullscreenStyle = sf::Style::None;
#else
	auto fullscreenStyle = sf::Style::Fullscreen;
#endif

	const sf::VideoMode fullScreenMode = sf::VideoMode::getFullscreenModes()[0];
	const sf::VideoMode windowedMode(1280, 960);
	bool isFullScreen = false;

	astra::StreamSet sensor;
	astra::StreamReader reader = sensor.create_reader();

	BodyVisualizer listener;

	auto depthStream = configure_depth(reader);
	depthStream.start();

	reader.stream<astra::BodyStream>().start();
	reader.add_listener(listener);

	// Pico Flexx Camera Initialization
	sample_utils::PlatformResources resources;
	MyListener Picolistener;
	std::unique_ptr<royale::ICameraDevice> cameraDevice;
	{
		royale::CameraManager manager;

		royale::Vector<royale::String> camlist(manager.getConnectedCameraList());
		cout << "Detected " << camlist.size() << " camera(s)." << endl;

		if (!camlist.empty())
		{
			cameraDevice = manager.createCamera(camlist[0]);
		}
		else
		{
			cerr << "No suitable camera device detected." << endl;
			return 1;
		}

		camlist.clear();
	}

	if (cameraDevice == nullptr)
	{
		// no cameraDevice available
		cerr << "Cannot create the camera device" << endl;
		return 1;
	}

	// IMPORTANT: call the initialize method before working with the camera device
	auto status = cameraDevice->initialize();
	if (status != royale::CameraStatus::SUCCESS)
	{
		cerr << "Cannot initialize the camera device, error string : " << getErrorString(status) << endl;
		return 1;
	}

	royale::String usecaseName = "MODE_5_35FPS_600"; // MODE_9_5FPS_2000,		MODE_9_10FPS_1000,		MODE_9_15FPS_700,		MODE_9_25FPS_450,
													 // MODE_5_35FPS_600,		MODE_5_45FPS_500,		MODE_MIXED_30_5

	status = cameraDevice->setUseCase(usecaseName);
	if (status == royale::CameraStatus::SUCCESS) {
		cout << " The use cases is successfully adapted to " << usecaseName << endl;
	}
	else {
		cout << "Fail to set use case." << endl;
	}

	// retrieve the lens parameters from Royale
	royale::LensParameters lensParameters;
	status = cameraDevice->getLensParameters(lensParameters);
	if (status != royale::CameraStatus::SUCCESS)
	{
		cerr << "Can't read out the lens parameters" << endl;
		return 1;
	}

	Picolistener.setLensParameters(lensParameters);
	Picolistener.toggleUndistort(); //한번 toggle하고 시작

									// register a data listener
	if (cameraDevice->registerDataListener(&Picolistener) != royale::CameraStatus::SUCCESS)
	{
		cerr << "Error registering data listener" << endl;
		return 1;
	}

	// create windows
	//cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
	//cv::namedWindow("Gray", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Test_Z", cv::WINDOW_AUTOSIZE);
	//namedWindow("Test_G", WINDOW_AUTOSIZE);

	// start capture mode
	if (cameraDevice->startCapture() != royale::CameraStatus::SUCCESS)
	{
		cerr << "Error starting the capturing" << endl;
		return 1;
	}


	int T_gap = 1000;
	int c_gap = -5000;
	
	int Bodyflag = 0;

	std::clock_t start;
	double duration;


	int t = 0;
	float Robot_Pos[6000*6] = { 0 };
	float L_Hand_Pos[6000*3] = { 0 };
	float R_Hand_Pos[6000*3] = { 0 };
	while (window.isOpen())
	{
		astra_update();
		
		if (c_gap > T_gap && Bodyflag == 0) {
			MySetActiveDevice(list[0]); // Right arm 
			c_gap = 0;
			std::cout << "No Body detected" << endl;
			TrajectoryPoint Initial_Position;
			Initial_Position.InitStruct();
			Initial_Position.Position.Type = CARTESIAN_POSITION;

			Initial_Position.Position.CartesianPosition.X = -0.126;
			Initial_Position.Position.CartesianPosition.Y = -0.396;
			Initial_Position.Position.CartesianPosition.Z = 0.0046;
			Initial_Position.Position.CartesianPosition.ThetaX = -3.14;
			Initial_Position.Position.CartesianPosition.ThetaY = 0.0068;
			Initial_Position.Position.CartesianPosition.ThetaZ = 0.2835;
			MySendBasicTrajectory(Initial_Position);
		}

		if (NumofBodies > 0) {
			// Data Logging
			/*CartesianPosition currentPosition;
			MyGetCartesianCommand(currentPosition);
			Robot_Pos[t * 6] = currentPosition.Coordinates.X;
			Robot_Pos[(t * 6) + 1] = currentPosition.Coordinates.Y;
			Robot_Pos[(t * 6) + 2] = currentPosition.Coordinates.Z;
			Robot_Pos[(t * 6) + 3] = currentPosition.Coordinates.ThetaX;
			Robot_Pos[(t * 6) + 4] = currentPosition.Coordinates.ThetaY;
			Robot_Pos[(t * 6) + 5] = currentPosition.Coordinates.ThetaZ;*/

			float L_X = ((Left_Hand_Pos.x * 0.001) - 0.22);
			float L_Y = (-sin(PI / 4.0f)*(Left_Hand_Pos.z * 0.001) - sin(PI / 4.0f)*(Left_Hand_Pos.y * 0.001) - 0.05);
			float L_Z = (-sin(PI / 4.0f)*(Left_Hand_Pos.z * 0.001) + sin(PI / 4.0f)*(Left_Hand_Pos.y * 0.001) + 1.20);
			//L_Hand_Pos[(t * 3)] = L_X;
			//L_Hand_Pos[(t * 3) + 1] = L_Y;
			//L_Hand_Pos[(t * 3) + 2] = L_Z;

			float R_X = ((Right_Hand_Pos.x * 0.001) - 0.22);
			float R_Y = (-sin(PI / 4.0f)*(Right_Hand_Pos.z * 0.001) - sin(PI / 4.0f)*(Right_Hand_Pos.y * 0.001) - 0.05);
			float R_Z = (-sin(PI / 4.0f)*(Right_Hand_Pos.z * 0.001) + sin(PI / 4.0f)*(Right_Hand_Pos.y * 0.001) + 1.20);
			//R_Hand_Pos[(t * 3)] = R_X;
			//R_Hand_Pos[(t * 3) + 1] = R_Y;
			//R_Hand_Pos[(t * 3) + 2] = R_Z;

			//t++;

			if (Bodyflag == 0) {
				Bodyflag = 1;
				start = std::clock();
				std::cout << "Bodies: " << NumofBodies << endl;
				c_gap = 0;
			}
			else { // Bodyflag ==1
				if (Grasping_Start == 1) {
					switch (G_Mode_S) {
					case Direct: {
						// Lifting 동작
						TrajectoryPoint pointToSend;
						pointToSend.InitStruct();
						pointToSend.Position.Type = CARTESIAN_POSITION;
						MyInitFingers();
						CartesianPosition currentPosition;
						MyGetCartesianCommand(currentPosition);
						pointToSend.Position.CartesianPosition.X = currentPosition.Coordinates.X;
						pointToSend.Position.CartesianPosition.Y = currentPosition.Coordinates.Y;
						pointToSend.Position.CartesianPosition.Z = 0.10;
						pointToSend.Position.CartesianPosition.ThetaX = currentPosition.Coordinates.ThetaX;
						pointToSend.Position.CartesianPosition.ThetaY = currentPosition.Coordinates.ThetaY;
						pointToSend.Position.CartesianPosition.ThetaZ = currentPosition.Coordinates.ThetaZ;

						pointToSend.Position.Fingers.Finger1 = 5000;
						pointToSend.Position.Fingers.Finger2 = 5000;
						pointToSend.Position.Fingers.Finger3 = 5000;
						MySendBasicTrajectory(pointToSend);
						Sleep(3000);
						Grasping_Start = 0;
						cout << "Grasping Done" << endl;
						window.close();
						break;
					}
					case SA: {
						// Grasping approach 동작
						float dx = d_XYZt_Kinova[0];
						float dy = d_XYZt_Kinova[1];
						float dist_err = sqrt(dx*dx + dy*dy);
						float dth = d_XYZt_Kinova[2];
						if (dist_err > 0.01) {
							printf("**** Distance Error is %6.3f. Moving to the Object. \n", dist_err);
							cout << "!!!!!!!!!!!!!!Move!!!!!!!!!!!!!!" << endl;
							TrajectoryPoint pointToSend;
							CartesianPosition currentCommand;
							MyGetCartesianCommand(currentCommand);
							pointToSend.InitStruct();
							pointToSend.Position.Type = CARTESIAN_POSITION;
							// (1.1f - countloop*0.1f) is to prevent repetitive swaying
							pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X + (1.1f - countloop*0.1f)*dx;
							pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y + (1.1f - countloop*0.1f)*dy;
							pointToSend.Position.CartesianPosition.Z = 0.0048;
							pointToSend.Position.CartesianPosition.ThetaX = currentCommand.Coordinates.ThetaX;
							pointToSend.Position.CartesianPosition.ThetaY = currentCommand.Coordinates.ThetaY;
							pointToSend.Position.CartesianPosition.ThetaZ = currentCommand.Coordinates.ThetaZ;
							MySendBasicTrajectory(pointToSend);
							Sleep(1000);
						}
						else {
							printf("**** Distance Error is %6.3f. STOP Moving. \n", dist_err);
							printf("**** Rotate Hand to Orientation. Start Moving. \n", dist_err);
							TrajectoryPoint pointToSend;
							CartesianPosition currentCommand;
							MyGetCartesianCommand(currentCommand);
							dth = currentCommand.Coordinates.ThetaZ - dth;
							pointToSend.InitStruct();
							pointToSend.Position.Type = CARTESIAN_POSITION;
							pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X;
							pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y;
							pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z;
							pointToSend.Position.CartesianPosition.ThetaX = currentCommand.Coordinates.ThetaX;
							pointToSend.Position.CartesianPosition.ThetaY = currentCommand.Coordinates.ThetaY;
							pointToSend.Position.CartesianPosition.ThetaZ = currentCommand.Coordinates.ThetaZ - dth;
							MySendBasicTrajectory(pointToSend);
							Sleep(3000);

							std::cout << "Move to Grasping position" << endl;
							
							pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X;
							pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y;
							pointToSend.Position.CartesianPosition.Z = -0.33;
							pointToSend.Position.CartesianPosition.ThetaX = currentCommand.Coordinates.ThetaX;
							pointToSend.Position.CartesianPosition.ThetaY = currentCommand.Coordinates.ThetaY;
							pointToSend.Position.CartesianPosition.ThetaZ = currentCommand.Coordinates.ThetaZ - dth;
							MySendBasicTrajectory(pointToSend);
							Sleep(2000);
							std::cout << "Start Grasping" << endl;

							pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X;
							pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y;
							pointToSend.Position.CartesianPosition.Z = -0.33;
							pointToSend.Position.CartesianPosition.ThetaX = currentCommand.Coordinates.ThetaX;
							pointToSend.Position.CartesianPosition.ThetaY = currentCommand.Coordinates.ThetaY;
							pointToSend.Position.CartesianPosition.ThetaZ = currentCommand.Coordinates.ThetaZ - dth;

							pointToSend.Position.Fingers.Finger1 = 5000;
							pointToSend.Position.Fingers.Finger2 = 5000;
							pointToSend.Position.Fingers.Finger3 = 5000;

							MySendBasicTrajectory(pointToSend);
							Sleep(2000);

							std::cout << "Start Lifting" << endl;

							pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X;
							pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y;
							pointToSend.Position.CartesianPosition.Z = 0.0;
							pointToSend.Position.CartesianPosition.ThetaX = currentCommand.Coordinates.ThetaX;
							pointToSend.Position.CartesianPosition.ThetaY = currentCommand.Coordinates.ThetaY;
							pointToSend.Position.CartesianPosition.ThetaZ = currentCommand.Coordinates.ThetaZ - dth;;

							pointToSend.Position.Fingers.Finger1 = 5000;
							pointToSend.Position.Fingers.Finger2 = 5000;
							pointToSend.Position.Fingers.Finger3 = 5000;

							MySendBasicTrajectory(pointToSend);
							Sleep(2000);

							Grasping_Start = 0;
							Grasping_Start = 0;
							cout << "Grasping Done" << endl;
							countloop = 0;
							window.close();
							break;
						}
					}					
					}
				}
				
				if (c_gap > T_gap) {
					switch (G_Mode_S) {
					case Joystick: {
						cout << "Joystick Mode" << endl;
					}
					case Direct:
					case SA:
					{
						TrajectoryPoint Hand_Track;
						Hand_Track.InitStruct();
						Hand_Track.Position.Type = CARTESIAN_POSITION;

						//Current Cartesian Position retrieve
						CartesianPosition currentPosition;
						MyGetCartesianCommand(currentPosition);

						float X = R_X;
						float Y = R_Y;
						float Z = R_Z;
						//int Hand_Grip = Left_Hand_Grip;
						int Hand_Grip = 0;
						//cout << "L_Z : " << L_Z << endl;
						if (L_Z > 0.4) {
							Hand_Grip = 1;
						}

						std::cout << "x: " << X << "y: " << Y << "z: " << Z << std::endl;
						//if next coordinate is too far to move than go back to initial position
						if ((abs(currentPosition.Coordinates.X - X) > 0.3)
							|| (abs(currentPosition.Coordinates.Y - Y) > 0.8)
							|| (abs(currentPosition.Coordinates.Z - Z) > 0.3)
							) {
							std::cout << "You moved too fast!" << endl;
						}
						else {
							Hand_Track.Position.CartesianPosition.X = X + 0.0;
							Hand_Track.Position.CartesianPosition.Y = Y + 0.6;
							Hand_Track.Position.CartesianPosition.Z = Z + 0.0;
							Hand_Track.Position.CartesianPosition.ThetaX = -3.14;
							Hand_Track.Position.CartesianPosition.ThetaY = 0.0068;
							Hand_Track.Position.CartesianPosition.ThetaZ = 0.2835;


							// Z가 0 이하이고, Hand::Grip 일 때, Grasping 시작
							if (currentPosition.Coordinates.Z < 0 && Hand_Grip == 1) {
								Grasping_Start = 1;
								switch (G_Mode_S) {
								case Direct: {
									MyInitFingers();
									Hand_Track.Position.Fingers.Finger1 = 5000;
									Hand_Track.Position.Fingers.Finger2 = 5000;
									Hand_Track.Position.Fingers.Finger3 = 5000;
									MySendBasicTrajectory(Hand_Track);
									Sleep(3000);
								}
								case SA: {
									Hand_Track.Position.CartesianPosition.Z = 0.0048;
								}
								}

							}
							MySendBasicTrajectory(Hand_Track);
						}					
					}					
					}
					c_gap = 0;
				}
			}
		}
		else {
			Bodyflag = 0;
			FirstDetect = 0;
		}
		c_gap++;

		sf::Event event;
		while (window.pollEvent(event))
		{
			switch (event.type)
			{
			case sf::Event::Closed:
				window.close();
				break;
			case sf::Event::KeyPressed:

			{
				if (event.key.code == sf::Keyboard::C && event.key.control)
				{
					window.close();
				}
				switch (event.key.code)
				{
				case sf::Keyboard::Escape:
					window.close();
					break;
				case sf::Keyboard::F:
					if (isFullScreen)
					{
						window.create(windowedMode, "Simple Body Viewer", sf::Style::Default);
					}
					else
					{
						window.create(fullScreenMode, "Simple Body Viewer", fullscreenStyle);
					}
					isFullScreen = !isFullScreen;
					break;
				case sf::Keyboard::R:
					depthStream.enable_registration(!depthStream.registration_enabled());
					break;
				case sf::Keyboard::M:
					depthStream.enable_mirroring(!depthStream.mirroring_enabled());
					break;
				default:
					break;
				}
				break;
			}
			default:
				break;
			}
		}

		// clear the window with black color
		window.clear(sf::Color::Black);

		listener.draw_to(window);
		window.display();
	}

	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	std::cout << "Time Spent: " << duration << '\n';

	std::cout << "Save Logging file" << endl;
	ofstream out(filename);
	out << "Time Spent: " << duration << '\n';
	out << "Robot Pos(X)\tRobot Pos(Y)\tRobot Pos(Z)\tRobot Pos(thX)\tRobot Pos(thY)\tRobot Pos(thZ)\tL_Hand Pos(X)\tL_Hand Pos(Y)\tL_Hand Pos(Z)\tR_Hand Pos(X)\tR_Hand Pos(Y)\tR_Hand Pos(Z)\n";
	for (int i = 0; i < t; i++) {
		out << Robot_Pos[(t * 6)] << '\t' << Robot_Pos[(t * 6) + 1] << '\t' << Robot_Pos[(t * 6) + 2] << '\t' << Robot_Pos[(t * 6)+3] << '\t' << Robot_Pos[(t * 6)+4] << '\t' << Robot_Pos[(t * 6)+5] << '\t'
			<< L_Hand_Pos[(t * 3)] << '\t' << L_Hand_Pos[(t * 3) + 1] << '\t' << L_Hand_Pos[(t * 3) + 2] << '\t'
			<< R_Hand_Pos[(t * 3)] << '\t' << R_Hand_Pos[(t * 3) + 1] << '\t' << R_Hand_Pos[(t * 3) + 2] << '\n';
	}
	out.close();

	std::cout << endl << "C L O S I N G   A P I" << endl;
	
	if (cameraDevice->stopCapture() != royale::CameraStatus::SUCCESS)
	{
		cerr << "Error stopping the capturing" << endl;
		return 1;
	}
	result = (*MyCloseAPI)();
	FreeLibrary(commandLayer_handle);
	astra::terminate();
	_getch();

	return 0;
}
