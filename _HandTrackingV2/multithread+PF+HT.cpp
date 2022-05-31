#define UNICODE
#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "KinovaTypes.h"
#include <Windows.h>
#include "CommunicationLayerWindows.h"
#include "CommandLayer.h"
#include <conio.h>
#include <SFML/Graphics.hpp>
#include <astra/astra.hpp>
#include <cstring>
#include <iostream>
#include <thread>
#include <atomic>

using namespace std;

HINSTANCE  commandLayer_handle;

//Function pointers to the functions we need
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MySendBasicTrajectory)(TrajectoryPoint command);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);
int(*MyMoveHome)();
int(*MyInitFingers)();
int(*MyGetCartesianCommand)(CartesianPosition &);


astra::Vector3f Right_Hand_Pos = astra::Vector3f();
astra::Vector3f Left_Hand_Pos = astra::Vector3f(); 


//global variables
int NumofBodies = 0;
int FirstDetect = 0;
int Right_Hand_Grip = -1;
int Left_Hand_Grip = -1;
int Bodyflag = 0;
double rob_pos[3];
double Dtogoal = 1000;
double GUrep_bnd[] = { 0,0,0 };
double GUrep_obs[] = { 0,0,0 };
double GUrep[] = { 0,0,0 };
double GUatt[3];
double gradient[3];
double D;
double DtoCenter;
double Cons;
double norm_gradient;
int numloop = 0;
double bnd[2][3] = { { -0.3,-0.6,-0.2 },{ 0.5,0.0,0.6 } };	//boundary
double bnd_center[] = { 0.5*(bnd[1][1] + bnd[2][1]),0.5*(bnd[1][2] + bnd[2][2]),0.5*(bnd[1][3] + bnd[2][3]) };
int T_gap = 1200;
int c_gap = -5000;
double norm_momentum = 0.0;

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

void thread_hand(atomic<bool>& flag, float* xgoal, float* ygoal, float* zgoal) 
{

	// Astra Camera Intialization [Start]
	std::cout << "Start Camera Initialization" << endl;
	astra::initialize();
	const char* licenseString = "<INSERT LICENSE KEY HERE>";
	orbbec_body_tracking_set_license(licenseString);

	sf::RenderWindow window(sf::VideoMode(1280, 960), "Simple Body Viewer");


	auto fullscreenStyle = sf::Style::None;
	const sf::VideoMode fullScreenMode = sf::VideoMode::getFullscreenModes()[0];
	const sf::VideoMode windowedMode(1280, 960);
	bool isFullScreen = false;

	astra::StreamSet sensor;
	astra::StreamReader reader = sensor.create_reader();

	BodyVisualizer listener;

	auto depthStream = configure_depth(reader);
	depthStream.start();

	auto bodyStream = reader.stream<astra::BodyStream>();
	bodyStream.start();
	reader.add_listener(listener);

	astra::SkeletonProfile profile = bodyStream.get_skeleton_profile();

	// HandPoses includes Joints and Segmentation
	astra::BodyTrackingFeatureFlags features = astra::BodyTrackingFeatureFlags::HandPoses;

	// Astra Camera Intialization [End]


	while (flag)
	{
			astra_update();

			if (NumofBodies > 0)
			{
				if (Bodyflag == 0)
				{
					Bodyflag = 1;
				}
				else //Bodyflag == 1
				{
					//hand position in Kinova coordinate
					float L_X = ((Left_Hand_Pos.x * 0.001) - 0.22);
					float L_Y = (-sin(PI / 4.0f)*(Left_Hand_Pos.z * 0.001) - sin(PI / 4.0f)*(Left_Hand_Pos.y * 0.001) - 0.05);
					float L_Z = (-sin(PI / 4.0f)*(Left_Hand_Pos.z * 0.001) + sin(PI / 4.0f)*(Left_Hand_Pos.y * 0.001) + 1.15);
					float R_X = ((Right_Hand_Pos.x * 0.001) - 0.22);
					float R_Y = (-sin(PI / 4.0f)*(Right_Hand_Pos.z * 0.001) - sin(PI / 4.0f)*(Right_Hand_Pos.y * 0.001) - 0.05);
					float R_Z = (-sin(PI / 4.0f)*(Right_Hand_Pos.z * 0.001) + sin(PI / 4.0f)*(Right_Hand_Pos.y * 0.001) + 1.15);

					//update goal position (hand pos)
					*xgoal = R_X + 0.0f;
					*ygoal = R_Y + 0.6f;
					*zgoal = R_Z + 0.0f;
				}
			}

			window.clear(sf::Color::Black);
			listener.draw_to(window);
			window.display();
	}
	
	
	return;
}

int main()
{
	//test case 1 - hand tracking w/o momentum
	//double Kappa = 0.4;  // Attractive Potential Gain
	//double Nu = 1.0e-6;  // Repulsive Potential Gain
	//double ObsTh = 0.05; // Obstacle
	//double ObsTh = 0.03;
	//double start[] = { 0.1,-0.3,0.5 };
	//double start_theta[] = { -3.14,0.0,0.0 };
	//double goal[] = { 0.0,0.0,0.0 };
	//double obs[2][4] = { { 0.15,-0.3,0.28,0.05 },{ 0.2,-0.5,0.22,0.04 } };
	//double stepsize = 0.01;
	//int obsnum = 0;

	//test case 2 - w/ momentum, 1 ball
	//double Kappa = 0.4;
	//double Nu = 1.0e-6;
	//double rate = 0.9;
	//double ObsTh = 0.05;
	//double start[] = { 0.034,-0.2,0.26 };
	//double temp[] = { 0,0,0 };
	//double start_theta[] = { -3.14,0.0,0.0 };
	//double goal[] = { 0.27,-0.60,-0.02 };
	//double momentum[] = { 0,0,0 };
	//double obs[1][4] = { { 0.18,-0.45,0.08,0.13 } };
	//double stepsize = 0.01;
	//int obsnum = 1;
	//double goal_theta[] = { 3.14,0.0,0.0 };

	//test case 3 - w/ momentum, 2 balls
	double Kappa = 0.4;
	double Nu = 1.0e-6;
	double rate = 0.9;
	double ObsTh = 0.05;
	double start[] = { 0.034,-0.2,0.26 };
	double temp[] = { 0,0,0 };
	double start_theta[] = { -3.14,0.0,0.0 };
	double goal[] = { 0.0,0.0,0.0 };
	double momentum[] = { 0,0,0 };
	double obs[2][4] = { { 0.237,-0.29,0.08,0.12 },{ 0.085,-0.49,0.02,0.12 } }; // x,y,z position + R radius
	double stepsize = 0.01;
	int obsnum = 2;
	double goal_theta[] = { 3.14,0.0,0.0 };


	int programResult = 0;

	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

	//We load the functions from the library
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
	MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");
	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) GetProcAddress(commandLayer_handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice devices)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
	MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
	MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");

	//Verify that all functions has been loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
		(MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetCartesianCommand == NULL) ||
		(MyMoveHome == NULL) || (MyInitFingers == NULL))

	{
		std::cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
		programResult = 0;
		return 0;
	}
	else
	{
		std::cout << "I N I T I A L I Z A T I O N   C O M P L E T E D - M A I N" << endl << endl;
	}
	int result = (*MyInitAPI)();

	std::cout << "Main Initialization's result :" << result << endl;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	std::cout << "Found a robot on the USB bus (" << list[0].SerialNumber << ")" << endl;

	//Setting the current device as the active device.
	MySetActiveDevice(list[0]);

	std::cout << "Send the robot to Home position" << endl;
	MyMoveHome();

	std::cout << "Initializing the fingers" << endl;
	MyInitFingers();

	TrajectoryPoint pointToSend;
	pointToSend.InitStruct();
	pointToSend.Position.Type = CARTESIAN_POSITION;

	atomic<bool> flag = true ;
	
	float xp, yp, zp = 0;
	float* xgoal = &xp;
	float* ygoal = &yp;
	float* zgoal = &zp;

	std::cout << "*********************START***************************" << endl;

	std::thread hand_t(&thread_hand, ref(flag), xgoal, ygoal, zgoal);

	CartesianPosition currentPosition;

	//Sending to start position
	MyGetCartesianCommand(currentPosition);
	pointToSend.Position.CartesianPosition.Z = start[2];
	pointToSend.Position.CartesianPosition.Y = currentPosition.Coordinates.Y;
	pointToSend.Position.CartesianPosition.X = currentPosition.Coordinates.X;
	pointToSend.Position.CartesianPosition.ThetaX = start_theta[0];
	pointToSend.Position.CartesianPosition.ThetaY = start_theta[1];
	pointToSend.Position.CartesianPosition.ThetaZ = start_theta[2];
	MySendBasicTrajectory(pointToSend);
	Sleep(3000);
	
	MyGetCartesianCommand(currentPosition);
	pointToSend.Position.CartesianPosition.X = start[0];
	pointToSend.Position.CartesianPosition.Y = start[1];
	pointToSend.Position.CartesianPosition.Z = currentPosition.Coordinates.Z;
	pointToSend.Position.CartesianPosition.ThetaX = start_theta[0];
	pointToSend.Position.CartesianPosition.ThetaY = start_theta[1];
	pointToSend.Position.CartesianPosition.ThetaZ = start_theta[2];
	pointToSend.Position.Fingers.Finger1 = 5700;
	pointToSend.Position.Fingers.Finger2 = 5700;
	pointToSend.Position.Fingers.Finger3 = 5700;

	MySendBasicTrajectory(pointToSend);
	Sleep(3000);
	

	while (true)
	{	
		//wait for next goal after reaching the goal
		if (Bodyflag == 1)
		{
			MyGetCartesianCommand(currentPosition);
			rob_pos[0] = currentPosition.Coordinates.X;
			rob_pos[1] = currentPosition.Coordinates.Y;
			rob_pos[2] = currentPosition.Coordinates.Z;
			Dtogoal = sqrt(pow(*xgoal - rob_pos[0] - momentum[0], 2) + pow(*ygoal - rob_pos[1] - momentum[1], 2) + pow(*zgoal - rob_pos[2] - momentum[2], 2));
		}

		while (Dtogoal > 0.02)
		{
			if (Bodyflag == 1)
			{
				//reset PF
				GUrep_bnd[0] = 0;
				GUrep_bnd[1] = 0;
				GUrep_bnd[2] = 0;
				GUrep_obs[0] = 0;
				GUrep_obs[1] = 0;
				GUrep_obs[2] = 0;

				//virtual position temp : position moved by momentum
				MyGetCartesianCommand(currentPosition);
				rob_pos[0] = currentPosition.Coordinates.X;
				rob_pos[1] = currentPosition.Coordinates.Y;
				rob_pos[2] = currentPosition.Coordinates.Z;

				temp[0] = rob_pos[0] + rate*momentum[0];
				temp[1] = rob_pos[1] + rate*momentum[1];
				temp[2] = rob_pos[2] + rate*momentum[2];

				//goal update
				if ((abs(currentPosition.Coordinates.X - *xgoal) > 0.3)
					|| (abs(currentPosition.Coordinates.Y - *ygoal) > 0.8)
					|| (abs(currentPosition.Coordinates.Z - *zgoal) > 0.3)
					)
				{
					std::cout << "You moved too fast!" << endl;
				}
				else
				{
					goal[0] = *xgoal;
					goal[1] = *ygoal;
					goal[2] = *zgoal;
				}

				goal[0] = *xgoal;
				goal[1] = *ygoal;
				goal[2] = *zgoal;


				//boundary PF at temp
				for (int i = 0; i < 3; i++)
				{
					D = min(abs(bnd[0][i] - temp[i]), abs(bnd[1][i] - temp[i]));
					Cons = Nu*(1.0 / ObsTh - 1.0 / D)*pow(1.0 / D, 2); //negative
					DtoCenter = sqrt(pow(bnd_center[0] - temp[0], 2) + pow(bnd_center[1] - temp[1], 2) + pow(bnd_center[2] - temp[2], 2));
					if (D <= ObsTh)
					{
						GUrep_bnd[i] = Cons*((bnd_center[i] - temp[i]) / DtoCenter); 
					}
				}

				//obstacles PF at temp
				for (int i = 0; i < obsnum; i++)
				{
					D = sqrt(pow(obs[i][0] - temp[0], 2) + pow(obs[i][1] - temp[1], 2) + pow(obs[i][2] - temp[2], 2)) - obs[i][3];
					if (D <= ObsTh)
					{
						DtoCenter = sqrt(pow(obs[i][0] - temp[0], 2) + pow(obs[i][1] - temp[1], 2) + pow(obs[i][2] - temp[2], 2));
						Cons = Nu*(1.0 / ObsTh - 1.0 / D)*pow(1.0 / D, 2); //negative
						GUrep_obs[0] = GUrep_obs[0] + Cons*((temp[0] - obs[i][0]) / DtoCenter); // x direction
						GUrep_obs[1] = GUrep_obs[1] + Cons*((temp[1] - obs[i][1]) / DtoCenter); // y direction
						GUrep_obs[2] = GUrep_obs[2] + Cons*((temp[2] - obs[i][2]) / DtoCenter); // z direction
					}
				}

				GUrep[0] = GUrep_bnd[0] + GUrep_obs[0];
				GUrep[1] = GUrep_bnd[1] + GUrep_obs[1];
				GUrep[2] = GUrep_bnd[2] + GUrep_obs[2];

				GUatt[0] = Kappa * (temp[0] - goal[0]);
				GUatt[1] = Kappa * (temp[1] - goal[1]);
				GUatt[2] = Kappa * (temp[2] - goal[2]);

				gradient[0] = -GUrep[0] - GUatt[0];
				gradient[1] = -GUrep[1] - GUatt[1];
				gradient[2] = -GUrep[2] - GUatt[2];

				norm_gradient = sqrt(pow(gradient[0], 2) + pow(gradient[1], 2) + pow(gradient[2], 2));

				//momentum(delta pos) = rate*previous momentum + PF at temp
				momentum[0] = rate * momentum[0] + stepsize*gradient[0] / norm_gradient;
				momentum[1] = rate * momentum[1] + stepsize*gradient[1] / norm_gradient;
				momentum[2] = rate * momentum[2] + stepsize*gradient[2] / norm_gradient;
				norm_momentum = sqrt(pow(momentum[0], 2) + pow(momentum[1], 2) + pow(momentum[2], 2));
				momentum[0] = stepsize * momentum[0] / norm_momentum;
				momentum[1] = stepsize * momentum[1] / norm_momentum;
				momentum[2] = stepsize * momentum[2] / norm_momentum;

				//send the robot to next pos
				pointToSend.Position.CartesianPosition.X = rob_pos[0] + momentum[0];
				pointToSend.Position.CartesianPosition.Y = rob_pos[1] + momentum[1];
				pointToSend.Position.CartesianPosition.Z = rob_pos[2] + momentum[2];
				pointToSend.Position.CartesianPosition.ThetaX = currentPosition.Coordinates.ThetaX;
				pointToSend.Position.CartesianPosition.ThetaY = currentPosition.Coordinates.ThetaY;
				pointToSend.Position.CartesianPosition.ThetaZ = currentPosition.Coordinates.ThetaZ;


				Dtogoal = sqrt(pow(goal[0] - rob_pos[0] - momentum[0], 2) + pow(goal[1] - rob_pos[1] - momentum[1], 2) + pow(goal[2] - rob_pos[2] - momentum[2], 2));
				numloop = numloop + 1;
				MySendBasicTrajectory(pointToSend);

				std::cout << numloop << endl;
				std::cout << "rob X : " << rob_pos[0] << "	rob Y : " << rob_pos[1] << "		rob Z : " << rob_pos[2] << endl;
				std::cout << "delta X : " << momentum[0] << "	delta Y : " << momentum[1] << "	delta Z : " << momentum[2] << endl;
				std::cout << "goal X : " << goal[0] << "	goal Y : " << goal[1] << "	goal Z : " << goal[2] << endl << endl;

				Sleep(80);
			}
		}
	}

	flag = false;
	hand_t.join();

	result = (*MyCloseAPI)();
	astra::terminate();
	FreeLibrary(commandLayer_handle);

	return programResult;
}