//#ifndef UNICODE
//#define UNICODE
//#endif
//#include "KinovaTypes.h"
//#include <iostream>
//#ifdef __linux__ 
//#include <dlfcn.h>
//#include <vector>
//#include "Kinova.API.CommLayerUbuntu.h"
//#include "Kinova.API.UsbCommandLayerUbuntu.h"
//#include <stdio.h>
//#include <unistd.h>
//#elif _WIN32
//#include <Windows.h>
//#include "CommunicationLayer.h"
//#include "CommandLayer.h"
//#include <conio.h>
//#include <SFML/Graphics.hpp>
//#include <astra/astra.hpp>
//#include <cstring>
//#endif
//
//using namespace std;
//
////A handle to the API.
//#ifdef __linux__ 
//void * commandLayer_handle;
//#elif _WIN32
//HINSTANCE commandLayer_handle;
//#endif
//
////Global Vars
//int NumofBodies = 0;
//int FirstDetect = 0;
//astra::Vector3f Right_Hand_Pos = astra::Vector3f();
//astra::Vector3f Left_Hand_Pos = astra::Vector3f();
//int Right_Hand_Grip = -1;
//int Left_Hand_Grip = -1;
//double rob_pos[3];
//double Dtogoal = 1000;
//double GUrep_bnd[] = { 0,0,0 };
//double GUrep_obs[] = { 0,0,0 };
//double GUrep[] = { 0,0,0 };
//double GUatt[3];
//double gradient[3];
//double D;
//double DtoCenter;
//double Cons;
//double norm_gradient;
//int numloop = 0;
//double bnd[2][3] = { { 0.07,-0.6,0.2 },{ 0.5,-0.2,0.7 } };	//boundary
//double bnd_center[] = { 0.5*(bnd[1][1] + bnd[2][1]),0.5*(bnd[1][2] + bnd[2][2]),0.5*(bnd[1][3] + bnd[2][3]) };
//int T_gap = 1200;
//int c_gap = -5000;
//int Bodyflag = 0;
//
//
////Function pointers to the functions we need
//int(*MyInitAPI)();
//int(*MyCloseAPI)();
//int(*MySendBasicTrajectory)(TrajectoryPoint command);
//int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
//int(*MySetActiveDevice)(KinovaDevice device);
//int(*MyMoveHome)();
//int(*MyInitFingers)();
//int(*MyGetCartesianCommand)(CartesianPosition &);
//
//#define PI 3.141592
//
//class sfLine : public sf::Drawable
//{
//public:
//	sfLine(const sf::Vector2f& point1, const sf::Vector2f& point2, sf::Color color, float thickness)
//		: color_(color)
//	{
//		const sf::Vector2f direction = point2 - point1;
//		const sf::Vector2f unitDirection = direction / std::sqrt(direction.x*direction.x + direction.y*direction.y);
//		const sf::Vector2f normal(-unitDirection.y, unitDirection.x);
//
//		const sf::Vector2f offset = (thickness / 2.f) * normal;
//
//		vertices_[0].position = point1 + offset;
//		vertices_[1].position = point2 + offset;
//		vertices_[2].position = point2 - offset;
//		vertices_[3].position = point1 - offset;
//
//		for (int i = 0; i<4; ++i)
//			vertices_[i].color = color;
//	}
//
//	void draw(sf::RenderTarget &target, sf::RenderStates states) const
//	{
//		target.draw(vertices_, 4, sf::Quads, states);
//	}
//
//private:
//	sf::Vertex vertices_[4];
//	sf::Color color_;
//};
//
//class BodyVisualizer : public astra::FrameListener
//{
//public:
//	static sf::Color get_body_color(std::uint8_t bodyId)
//	{
//		if (bodyId == 0)
//		{
//			// Handle no body separately - transparent
//			return sf::Color(0x00, 0x00, 0x00, 0x00);
//		}
//		// Case 0 below could mean bodyId == 25 or
//		// above due to the "% 24".
//		switch (bodyId % 6) {
//		case 0:
//			return sf::Color(0x00, 0x88, 0x00, 0xFF);
//		case 1:
//			return sf::Color(0x00, 0x00, 0xFF, 0xFF);
//		case 2:
//			return sf::Color(0x88, 0x00, 0x00, 0xFF);
//		case 3:
//			return sf::Color(0x00, 0xFF, 0x00, 0xFF);
//		case 4:
//			return sf::Color(0x00, 0x00, 0x88, 0xFF);
//		case 5:
//			return sf::Color(0xFF, 0x00, 0x00, 0xFF);
//		default:
//			return sf::Color(0xAA, 0xAA, 0xAA, 0xFF);
//		}
//	}
//
//	void init_depth_texture(int width, int height)
//	{
//		if (displayBuffer_ == nullptr || width != depthWidth_ || height != depthHeight_)
//		{
//			depthWidth_ = width;
//			depthHeight_ = height;
//			int byteLength = depthWidth_ * depthHeight_ * 4;
//
//			displayBuffer_ = BufferPtr(new uint8_t[byteLength]);
//			std::memset(displayBuffer_.get(), 0, byteLength);
//
//			texture_.create(depthWidth_, depthHeight_);
//			sprite_.setTexture(texture_, true);
//			sprite_.setPosition(0, 0);
//		}
//	}
//
//	void init_overlay_texture(int width, int height)
//	{
//		if (overlayBuffer_ == nullptr || width != overlayWidth_ || height != overlayHeight_)
//		{
//			overlayWidth_ = width;
//			overlayHeight_ = height;
//			int byteLength = overlayWidth_ * overlayHeight_ * 4;
//
//			overlayBuffer_ = BufferPtr(new uint8_t[byteLength]);
//			std::fill(&overlayBuffer_[0], &overlayBuffer_[0] + byteLength, 0);
//
//			overlayTexture_.create(overlayWidth_, overlayHeight_);
//			overlaySprite_.setTexture(overlayTexture_, true);
//			overlaySprite_.setPosition(0, 0);
//		}
//	}
//
//	void check_fps()
//	{
//		double fpsFactor = 0.02;
//
//		std::clock_t newTimepoint = std::clock();
//		long double frameDuration = (newTimepoint - lastTimepoint_) / static_cast<long double>(CLOCKS_PER_SEC);
//
//		frameDuration_ = frameDuration * fpsFactor + frameDuration_ * (1 - fpsFactor);
//		lastTimepoint_ = newTimepoint;
//		double fps = 1.0 / frameDuration_;
//
//		//printf("FPS: %3.1f (%3.4Lf ms)\n", fps, frameDuration_ * 1000);
//	}
//
//	void processDepth(astra::Frame& frame)
//	{
//		const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();
//
//		if (!depthFrame.is_valid()) { return; }
//
//		int width = depthFrame.width();
//		int height = depthFrame.height();
//
//		init_depth_texture(width, height);
//
//		const int16_t* depthPtr = depthFrame.data();
//		for (int y = 0; y < height; y++)
//		{
//			for (int x = 0; x < width; x++)
//			{
//				int index = (x + y * width);
//				int index4 = index * 4;
//
//				int16_t depth = depthPtr[index];
//				uint8_t value = depth % 255;
//				// Normalize depth
//				// uint8_t value = round((depth / 3400) * 255);			// Too Dark..				
//
//				displayBuffer_[index4] = value;
//				displayBuffer_[index4 + 1] = value;
//				displayBuffer_[index4 + 2] = value;
//				displayBuffer_[index4 + 3] = 255;
//			}
//		}
//
//		texture_.update(displayBuffer_.get());
//	}
//
//	void processBodies(astra::Frame& frame)
//	{
//		astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();
//
//		jointPositions_.clear();
//		circles_.clear();
//		circleShadows_.clear();
//		boneLines_.clear();
//		boneShadows_.clear();
//
//		if (!bodyFrame.is_valid() || bodyFrame.info().width() == 0 || bodyFrame.info().height() == 0)
//		{
//			clear_overlay();
//			NumofBodies = 0;
//			return;
//		}
//
//		const float jointScale = bodyFrame.info().width() / 120.f;
//
//		const auto& bodies = bodyFrame.bodies();
//
//		// Detect Human Body -> Immediately Stop
//		NumofBodies = bodies.size();
//		if (NumofBodies > 0 && FirstDetect == 0) {
//			FirstDetect = 1;
//			return;
//		}
//
//		for (auto& body : bodies)
//		{
//			/*printf("Processing frame #%d body %d left hand: %u\n",
//			bodyFrame.frame_index(), body.id(), unsigned(body.hand_poses().left_hand()))*/;
//		for (auto& joint : body.joints())
//		{
//			jointPositions_.push_back(joint.depth_position());
//		}
//
//		update_body(body, jointScale);
//		}
//
//		const auto& floor = bodyFrame.floor_info(); //floor
//		if (floor.floor_detected())
//		{
//			const auto& p = floor.floor_plane();
//
//		}
//
//		const auto& bodyMask = bodyFrame.body_mask();
//		const auto& floorMask = floor.floor_mask();
//
//		update_overlay(bodyMask, floorMask);
//	}
//
//	void update_body(astra::Body body,
//		const float jointScale)
//	{
//		const auto& joints = body.joints();
//
//		if (joints.empty())
//		{
//			return;
//		}
//
//		for (const auto& joint : joints)
//		{
//			astra::JointType type = joint.type();
//			const auto& pos = joint.depth_position();
//
//			if (joint.status() == astra::JointStatus::NotTracked)
//			{
//				continue;
//			}
//
//			auto radius = jointRadius_ * jointScale; // pixels
//			sf::Color circleShadowColor(0, 0, 0, 255);
//
//			auto color = sf::Color(0x00, 0xFF, 0x00, 0xFF);
//
//			if (type == astra::JointType::LeftHand)
//			{
//				if (astra::HandPose::Grip == body.hand_poses().left_hand()) {
//					Left_Hand_Grip = 1;
//					radius *= 1.5f;
//					circleShadowColor = sf::Color(255, 255, 255, 255);
//					color = sf::Color(0x00, 0xAA, 0xFF, 0xFF);
//				}
//				else {
//					Left_Hand_Grip = 0;
//				}
//			}
//
//			if (type == astra::JointType::RightHand)
//			{
//				if (astra::HandPose::Grip == body.hand_poses().right_hand()) {
//					Right_Hand_Grip = 1;
//					radius *= 1.5f;
//					circleShadowColor = sf::Color(255, 255, 255, 255);
//					color = sf::Color(0x00, 0xAA, 0xFF, 0xFF);
//				}
//				else {
//					Right_Hand_Grip = 0;
//				}
//			}
//
//			const auto shadowRadius = radius + shadowRadius_ * jointScale;
//			const auto radiusDelta = shadowRadius - radius;
//
//			sf::CircleShape circle(radius);
//
//			circle.setFillColor(sf::Color(color.r, color.g, color.b, 255));
//			circle.setPosition(pos.x - radius, pos.y - radius);
//			circles_.push_back(circle);
//
//			sf::CircleShape shadow(shadowRadius);
//			shadow.setFillColor(circleShadowColor);
//			shadow.setPosition(circle.getPosition() - sf::Vector2f(radiusDelta, radiusDelta));
//			circleShadows_.push_back(shadow);
//		}
//
//		update_bone(joints, jointScale, astra::JointType::Head, astra::JointType::ShoulderSpine);
//
//		update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::LeftShoulder);
//		update_bone(joints, jointScale, astra::JointType::LeftShoulder, astra::JointType::LeftElbow);
//		update_bone(joints, jointScale, astra::JointType::LeftElbow, astra::JointType::LeftHand);
//
//		update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::RightShoulder);
//		update_bone(joints, jointScale, astra::JointType::RightShoulder, astra::JointType::RightElbow);
//		update_bone(joints, jointScale, astra::JointType::RightElbow, astra::JointType::RightHand);
//
//		update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::MidSpine);
//		update_bone(joints, jointScale, astra::JointType::MidSpine, astra::JointType::BaseSpine);
//
//		update_bone(joints, jointScale, astra::JointType::BaseSpine, astra::JointType::LeftHip);
//		update_bone(joints, jointScale, astra::JointType::LeftHip, astra::JointType::LeftKnee);
//		update_bone(joints, jointScale, astra::JointType::LeftKnee, astra::JointType::LeftFoot);
//
//		update_bone(joints, jointScale, astra::JointType::BaseSpine, astra::JointType::RightHip);
//		update_bone(joints, jointScale, astra::JointType::RightHip, astra::JointType::RightKnee);
//		update_bone(joints, jointScale, astra::JointType::RightKnee, astra::JointType::RightFoot);
//	}
//
//	void update_bone(const astra::JointList& joints,
//		const float jointScale, astra::JointType j1,
//		astra::JointType j2)
//	{
//		const auto& joint1 = joints[int(j1)];
//		const auto& joint2 = joints[int(j2)];
//		const auto& jp1w = joint1.world_position();
//		const auto& jp2w = joint2.world_position();
//
//		switch (j1) {
//		case astra::JointType::LeftElbow:
//			switch (j2) {
//			case astra::JointType::LeftHand:
//				Left_Hand_Pos = jp2w;
//			}
//		case astra::JointType::RightElbow:
//			switch (j2) {
//			case astra::JointType::RightHand:
//				Right_Hand_Pos = jp2w;
//			}
//		}
//
//		if (joint1.status() == astra::JointStatus::NotTracked ||
//			joint2.status() == astra::JointStatus::NotTracked)
//		{
//			//don't render bones between untracked joints
//			return;
//		}
//
//		const auto& jp1 = joint1.depth_position();
//		const auto& jp2 = joint2.depth_position();
//
//		auto p1 = sf::Vector2f(jp1.x, jp1.y);
//		auto p2 = sf::Vector2f(jp2.x, jp2.y);
//
//		sf::Color color(255, 255, 255, 255);
//		float thickness = lineThickness_ * jointScale;
//		if (joint1.status() == astra::JointStatus::LowConfidence ||
//			joint2.status() == astra::JointStatus::LowConfidence)
//		{
//			color = sf::Color(128, 128, 128, 255);
//			thickness *= 0.5f;
//		}
//
//		boneLines_.push_back(sfLine(p1,
//			p2,
//			color,
//			thickness));
//		const float shadowLineThickness = thickness + shadowRadius_ * jointScale * 2.f;
//		boneShadows_.push_back(sfLine(p1,
//			p2,
//			sf::Color(0, 0, 0, 255),
//			shadowLineThickness));
//	}
//
//	void update_overlay(const astra::BodyMask& bodyMask,
//		const astra::FloorMask& floorMask)
//	{
//		const auto* bodyData = bodyMask.data();
//		const auto* floorData = floorMask.data();
//		const int width = bodyMask.width();
//		const int height = bodyMask.height();
//
//		init_overlay_texture(width, height);
//
//		const int length = width * height;
//
//		for (int i = 0; i < length; i++)
//		{
//			const auto bodyId = bodyData[i];
//			const auto isFloor = floorData[i];
//
//			sf::Color color(0x0, 0x0, 0x0, 0x0);
//
//			if (bodyId != 0)
//			{
//				color = get_body_color(bodyId);
//			}
//			else if (isFloor != 0)
//			{
//				color = sf::Color(0x0, 0x0, 0xFF, 0x88);
//			}
//
//			const int rgbaOffset = i * 4;
//			overlayBuffer_[rgbaOffset] = color.r;
//			overlayBuffer_[rgbaOffset + 1] = color.g;
//			overlayBuffer_[rgbaOffset + 2] = color.b;
//			overlayBuffer_[rgbaOffset + 3] = color.a;
//		}
//
//		overlayTexture_.update(overlayBuffer_.get());
//	}
//
//	void clear_overlay()
//	{
//		int byteLength = overlayWidth_ * overlayHeight_ * 4;
//		std::fill(&overlayBuffer_[0], &overlayBuffer_[0] + byteLength, 0);
//
//		overlayTexture_.update(overlayBuffer_.get());
//	}
//
//	virtual void on_frame_ready(astra::StreamReader& reader,
//		astra::Frame& frame) override
//	{
//		processDepth(frame);
//		processBodies(frame);
//
//		check_fps();
//	}
//
//	void draw_bodies(sf::RenderWindow& window)
//	{
//		const float scaleX = window.getView().getSize().x / overlayWidth_;
//		const float scaleY = window.getView().getSize().y / overlayHeight_;
//
//		sf::RenderStates states;
//		sf::Transform transform;
//		transform.scale(scaleX, scaleY);
//		states.transform *= transform;
//
//		for (const auto& bone : boneShadows_)
//			window.draw(bone, states);
//
//		for (const auto& c : circleShadows_)
//			window.draw(c, states);
//
//		for (const auto& bone : boneLines_)
//			window.draw(bone, states);
//
//		for (auto& c : circles_)
//			window.draw(c, states);
//
//	}
//
//	void draw_to(sf::RenderWindow& window)
//	{
//		if (displayBuffer_ != nullptr)
//		{
//			const float scaleX = window.getView().getSize().x / depthWidth_;
//			const float scaleY = window.getView().getSize().y / depthHeight_;
//			sprite_.setScale(scaleX, scaleY);
//
//			window.draw(sprite_); // depth
//		}
//
//		if (overlayBuffer_ != nullptr)
//		{
//			const float scaleX = window.getView().getSize().x / overlayWidth_;
//			const float scaleY = window.getView().getSize().y / overlayHeight_;
//			overlaySprite_.setScale(scaleX, scaleY);
//			window.draw(overlaySprite_); //bodymask and floormask
//		}
//
//		draw_bodies(window);
//	}
//
//private:
//	long double frameDuration_{ 0 };
//	std::clock_t lastTimepoint_{ 0 };
//	sf::Texture texture_;
//	sf::Sprite sprite_;
//
//	using BufferPtr = std::unique_ptr < uint8_t[] >;
//	BufferPtr displayBuffer_{ nullptr };
//
//	std::vector<astra::Vector2f> jointPositions_;
//
//	int depthWidth_{ 0 };
//	int depthHeight_{ 0 };
//	int overlayWidth_{ 0 };
//	int overlayHeight_{ 0 };
//
//	std::vector<sfLine> boneLines_;
//	std::vector<sfLine> boneShadows_;
//	std::vector<sf::CircleShape> circles_;
//	std::vector<sf::CircleShape> circleShadows_;
//
//	float lineThickness_{ 0.5f }; // pixels
//	float jointRadius_{ 1.0f };   // pixels
//	float shadowRadius_{ 0.5f };  // pixels
//
//	BufferPtr overlayBuffer_{ nullptr };
//	sf::Texture overlayTexture_;
//	sf::Sprite overlaySprite_;
//
//};
//
//astra::DepthStream configure_depth(astra::StreamReader& reader)
//{
//	auto depthStream = reader.stream<astra::DepthStream>();
//
//	//We don't have to set the mode to start the stream, but if you want to here is how:
//	astra::ImageStreamMode depthMode;
//
//	depthMode.set_width(640);
//	depthMode.set_height(480);
//	depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
//	depthMode.set_fps(30);
//
//	depthStream.set_mode(depthMode);
//
//	return depthStream;
//}
//
//int main(int argc, char* argv[])
//{
//	CartesianPosition currentCommand;
//	int programResult = 0;
//
//	//test case 1 - 2 obstacles, with moving goal (line 846~857), w/o hand tracking
//	//double Kappa = 0.4;  // Attractive Potential Gain
//	//double Nu = 1.0e-6;  // Repulsive Potential Gain
//	//double ObsTh = 0.05; // Obstacle
//	//double start[] = { 0.1,-0.3,0.5 };
//	//double start_theta[] = { -3.14,0.0,0.0 };
//	//double goal[] = { 0.306,-0.4971,-0.06 };
//	//double obs[2][4] = { { 0.15,-0.3,0.28,0.1 },{ 0.2,-0.5,0.22,0.09 } };
//	//double stepsize = 0.01;
//	//int obsnum = 2;
//
//	//test case 2 - 2 obstacles, PF+HT
//	double Kappa = 0.4;  // Attractive Potential Gain
//	double Nu = 1.0e-6;  // Repulsive Potential Gain
//	double ObsTh = 0.05; // Obstacle
//	double start[] = { 0.1,-0.3,0.5 };
//	double start_theta[] = { -3.14,0.0,0.0 };
//	double goal[] = { 0.0,0.0,0.0 };
//	double obs[2][4] = { { 0.15,-0.3,0.28,0.1 },{ 0.2,-0.5,0.22,0.09 } };
//	double stepsize = 0.01;
//	int obsnum = 2;
//
//#ifdef __linux__ 
//	//We load the API
//	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
//
//	//We load the functions from the library
//	MyInitAPI = (int(*)()) dlsym(commandLayer_handle, "InitAPI");
//	MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "CloseAPI");
//	MyMoveHome = (int(*)()) dlsym(commandLayer_handle, "MoveHome");
//	MyInitFingers = (int(*)()) dlsym(commandLayer_handle, "InitFingers");
//	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
//	MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");
//	MySendBasicTrajectory = (int(*)(TrajectoryPoint)) dlsym(commandLayer_handle, "SendBasicTrajectory");
//	MyGetCartesianCommand = (int(*)(CartesianPosition &)) dlsym(commandLayer_handle, "GetCartesianCommand");
//#elif _WIN32
//	//We load the API.
//	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");
//
//	//We load the functions from the library
//	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
//	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
//	MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
//	MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");
//	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) GetProcAddress(commandLayer_handle, "GetDevices");
//	MySetActiveDevice = (int(*)(KinovaDevice devices)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
//	MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
//	MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");
//#endif
//
//	//Verify that all functions has been loaded correctly
//	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
//		(MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetCartesianCommand == NULL) ||
//		(MyMoveHome == NULL) || (MyInitFingers == NULL))
//
//	{
//		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
//		programResult = 0;
//		return 0;
//	}
//	else
//	{
//		cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;
//	}
//	int result = (*MyInitAPI)();
//
//	cout << "Initialization's result :" << result << endl;
//
//	KinovaDevice list[MAX_KINOVA_DEVICE];
//
//	int devicesCount = MyGetDevices(list, result);
//
//
//	cout << "Found a robot on the USB bus (" << list[0].SerialNumber << ")" << endl;
//
//	//Setting the current device as the active device.
//	MySetActiveDevice(list[0]);
//
//	cout << "Send the robot to HOME position" << endl;
//	MyMoveHome();
//
//	cout << "Initializing the fingers" << endl;
//	MyInitFingers();
//
//
//	TrajectoryPoint pointToSend;
//	pointToSend.InitStruct();
//	pointToSend.Position.Type = CARTESIAN_POSITION;
//
//	//Sending to start position
//
//	MyGetCartesianCommand(currentCommand);
//	pointToSend.Position.CartesianPosition.Z = start[2];
//	pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y;
//	pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X;
//	pointToSend.Position.CartesianPosition.ThetaX = start_theta[0];
//	pointToSend.Position.CartesianPosition.ThetaY = start_theta[1];
//	pointToSend.Position.CartesianPosition.ThetaZ = start_theta[2];
//	MySendBasicTrajectory(pointToSend);
//	Sleep(3000);
//
//	MyGetCartesianCommand(currentCommand);
//	pointToSend.Position.CartesianPosition.X = start[0];
//	pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y;
//	pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z;
//	pointToSend.Position.CartesianPosition.ThetaX = start_theta[0];
//	pointToSend.Position.CartesianPosition.ThetaY = start_theta[1];
//	pointToSend.Position.CartesianPosition.ThetaZ = start_theta[2];
//	MySendBasicTrajectory(pointToSend);
//	Sleep(3000);
//
//	MyGetCartesianCommand(currentCommand);
//	pointToSend.Position.CartesianPosition.Y = start[1];
//	pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X;
//	pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z;
//	pointToSend.Position.CartesianPosition.ThetaX = start_theta[0];
//	pointToSend.Position.CartesianPosition.ThetaY = start_theta[1];
//	pointToSend.Position.CartesianPosition.ThetaZ = start_theta[2];
//	MySendBasicTrajectory(pointToSend);
//
//	Sleep(5000);
//
//	// Astra Camera Intialization [Start]
//	cout << "Start Camera Initialization" << endl;
//	astra::initialize();
//	const char* licenseString = "<INSERT LICENSE KEY HERE>";
//	orbbec_body_tracking_set_license(licenseString);
//
//	sf::RenderWindow window(sf::VideoMode(1280, 960), "Simple Body Viewer");
//
//#ifdef _WIN32
//	auto fullscreenStyle = sf::Style::None;
//#else
//	auto fullscreenStyle = sf::Style::Fullscreen;
//#endif
//
//	const sf::VideoMode fullScreenMode = sf::VideoMode::getFullscreenModes()[0];
//	const sf::VideoMode windowedMode(1280, 960);
//	bool isFullScreen = false;
//
//	astra::StreamSet sensor;
//	astra::StreamReader reader = sensor.create_reader();
//
//	BodyVisualizer listener;
//
//	auto depthStream = configure_depth(reader);
//	depthStream.start();
//
//	auto bodyStream = reader.stream<astra::BodyStream>();
//	bodyStream.start();
//	reader.add_listener(listener);
//
//	astra::SkeletonProfile profile = bodyStream.get_skeleton_profile();
//
//	// HandPoses includes Joints and Segmentation
//	astra::BodyTrackingFeatureFlags features = astra::BodyTrackingFeatureFlags::HandPoses;
//
//	// Astra Camera Intialization [End]
//
//	cout << "*********************START***************************" << endl;
//
//	while (Dtogoal > 0.02)
//	{
//		astra_update();
//
//		//reset PF
//		GUrep_bnd[0] = 0;
//		GUrep_bnd[1] = 0;
//		GUrep_bnd[2] = 0;
//
//		GUrep_obs[0] = 0;
//		GUrep_obs[1] = 0;
//		GUrep_obs[2] = 0;
//
//		MyGetCartesianCommand(currentCommand);
//		rob_pos[0] = currentCommand.Coordinates.X;
//		rob_pos[1] = currentCommand.Coordinates.Y;
//		rob_pos[2] = currentCommand.Coordinates.Z;
//
//		if (NumofBodies > 0)
//		{
//			if (Bodyflag == 0)
//			{
//				Bodyflag = 1;
//			}
//			else //Bodyflag == 1
//			{
//				//hand position in Kinova coordinate
//				float L_X = ((Left_Hand_Pos.x * 0.001) - 0.22);
//				float L_Y = (-sin(PI / 4.0f)*(Left_Hand_Pos.z * 0.001) - sin(PI / 4.0f)*(Left_Hand_Pos.y * 0.001) - 0.05);
//				float L_Z = (-sin(PI / 4.0f)*(Left_Hand_Pos.z * 0.001) + sin(PI / 4.0f)*(Left_Hand_Pos.y * 0.001) + 1.15);
//				float R_X = ((Right_Hand_Pos.x * 0.001) - 0.22);
//				float R_Y = (-sin(PI / 4.0f)*(Right_Hand_Pos.z * 0.001) - sin(PI / 4.0f)*(Right_Hand_Pos.y * 0.001) - 0.05);
//				float R_Z = (-sin(PI / 4.0f)*(Right_Hand_Pos.z * 0.001) + sin(PI / 4.0f)*(Right_Hand_Pos.y * 0.001) + 1.15);
//
//				printf("R_X : %f,\tR_Y : %f,\tR_Z : %f\n", R_X, R_Y, R_Z);
//				printf("L_X : %f,\tL_Y : %f,\tL_Z : %f\n", L_X, L_Y, L_Z);
//
//				//update goal position (hand pos)
//				if ((abs(currentCommand.Coordinates.X - R_X) > 0.3)
//					|| (abs(currentCommand.Coordinates.Y - R_Y) > 0.8)
//					|| (abs(currentCommand.Coordinates.Z - R_Z) > 0.3)
//					)
//				{
//					std::cout << "You moved too fast!" << endl;
//				}
//				else
//				{
//					goal[0] = R_X + 0.0;
//					goal[1] = R_Y + 0.6;
//					goal[2] = R_Z + 0.0;
//				}
//
//
//				//boundary PF
//				for (int i = 0; i < 3; i++)
//				{
//					D = min(abs(bnd[1][i] - rob_pos[i]), abs(bnd[2][i] - rob_pos[i]));
//					Cons = Nu*(1.0 / ObsTh - 1.0 / D)*pow(1.0 / D, 2);
//					DtoCenter = sqrt(pow(bnd_center[0] - rob_pos[0], 2) + pow(bnd_center[1] - rob_pos[1], 2) + pow(bnd_center[2] - rob_pos[2], 2));
//					if (D <= ObsTh)
//					{
//						GUrep_bnd[i] = Cons*((bnd_center[i] - rob_pos[i]) / DtoCenter);
//					}
//				}
//
//				//obstacles PF
//				for (int i = 0; i < obsnum; i++)
//				{
//					D = sqrt(pow(obs[i][0] - rob_pos[0], 2) + pow(obs[i][1] - rob_pos[1], 2) + pow(obs[i][2] - rob_pos[2], 2)) - obs[i][3];
//					if (D <= ObsTh)
//					{
//						DtoCenter = sqrt(pow(obs[i][0] - rob_pos[0], 2) + pow(obs[i][1] - rob_pos[1], 2) + pow(obs[i][2] - rob_pos[2], 2));
//						Cons = Nu*(1.0 / ObsTh - 1.0 / D)*pow(1.0 / D, 2);
//						GUrep_obs[0] = GUrep_obs[0] + Cons*((rob_pos[0] - obs[i][0]) / DtoCenter); // x direction
//						GUrep_obs[1] = GUrep_obs[1] + Cons*((rob_pos[1] - obs[i][1]) / DtoCenter); // y direction
//						GUrep_obs[2] = GUrep_obs[2] + Cons*((rob_pos[2] - obs[i][2]) / DtoCenter); // z direction
//					}
//				}
//
//				GUrep[0] = GUrep_bnd[0] + GUrep_obs[0];
//				GUrep[1] = GUrep_bnd[1] + GUrep_obs[1];
//				GUrep[2] = GUrep_bnd[2] + GUrep_obs[2];
//
//				GUatt[0] = Kappa * (rob_pos[0] - goal[0]);
//				GUatt[1] = Kappa * (rob_pos[1] - goal[1]);
//				GUatt[2] = Kappa * (rob_pos[2] - goal[2]);
//
//				gradient[0] = -GUrep[0] - GUatt[0];
//				gradient[1] = -GUrep[1] - GUatt[1];
//				gradient[2] = -GUrep[2] - GUatt[2];
//
//				norm_gradient = sqrt(pow(gradient[0], 2) + pow(gradient[1], 2) + pow(gradient[2], 2));
//
//				rob_pos[0] = rob_pos[0] + stepsize*gradient[0] / norm_gradient;
//				rob_pos[1] = rob_pos[1] + stepsize*gradient[1] / norm_gradient;
//				rob_pos[2] = rob_pos[2] + stepsize*gradient[2] / norm_gradient;
//
//				pointToSend.Position.CartesianPosition.X = rob_pos[0];
//				pointToSend.Position.CartesianPosition.Y = rob_pos[1];
//				pointToSend.Position.CartesianPosition.Z = rob_pos[2];
//				pointToSend.Position.CartesianPosition.ThetaX = currentCommand.Coordinates.ThetaX;
//				pointToSend.Position.CartesianPosition.ThetaY = currentCommand.Coordinates.ThetaY;
//				pointToSend.Position.CartesianPosition.ThetaZ = currentCommand.Coordinates.ThetaZ;
//
//				MyGetCartesianCommand(currentCommand);
//				Dtogoal = sqrt(pow(goal[0] - rob_pos[0], 2) + pow(goal[1] - rob_pos[1], 2) + pow(goal[2] - rob_pos[2], 2));
//				numloop = numloop + 1;
//				MySendBasicTrajectory(pointToSend);
//
//				cout << numloop << endl;
//				cout << "X : " << rob_pos[0] << "		Y: " << rob_pos[1] << "			Z: " << rob_pos[2] << endl;
//				cout << "X : " << goal[0] << "		Y: " << goal[1] << "			Z: " << goal[2] << endl << endl;
//
//
//				Sleep(180);
//
//				//to prevent infinite loop caused by local minima
//				if (numloop >= 600)
//				{
//					break;
//				}
//
//				////goal test case 1 (with test case 3 above)
//				//goal[0] = goal[0] - 0.001;
//				//goal[1] = goal[1] + 0.001;
//				//goal[2] = -0.05 + abs(0.1*sin(0.05*numloop));
//
//				////goal test case 2 (with test case 3 above)
//				//if (numloop >= 50)
//				//{
//				//	goal[0] = 0.2;
//				//	goal[1] = -0.3;
//				//	goal[2] = 0.1;
//				//}
//			}	
//		}
//		// clear the window with black color
//		window.clear(sf::Color::Black);
//
//		listener.draw_to(window);
//		window.display();
//	}
//
//	cout << endl << "C L O S I N G   A P I" << endl;
//	result = (*MyCloseAPI)();
//	astra::terminate();
//
//	programResult = 1;
//
//
//#ifdef __linux__ 
//	dlclose(commandLayer_handle);
//#elif _WIN32
//	FreeLibrary(commandLayer_handle);
//#endif
//
//	return programResult;
//
//}
