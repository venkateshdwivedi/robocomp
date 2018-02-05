#pragma once

#include "cameraI.h"
#include "differentialrobotI.h"
#include "omnirobotI.h"
#include "imuI.h"
#include "jointmotorI.h"
#include "laserI.h"
#include "rgbdI.h"
#include "touchsensorI.h"
#include "genericbaseI.h"

#include <CommonHead.h>

// Namespaces
// using namespace std;
// using namespace RoboCompCamera;
// using namespace RoboCompCommonHead;
// using namespace RoboCompDifferentialRobot;
// using namespace RoboCompLaser;
// using namespace RoboCompIMU;

/** XXXServer **/
/** XXXServer **/
/** XXXServer **/
class JointMotorServer
{
public:
	JointMotorServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker_, uint32_t _port);
	void add(InnerModel::JointPtr joint);
	void remove(InnerModel::JointPtr joint);
	void add(InnerModel::PrismaticJointPtr joint);
	void remove(InnerModel::PrismaticJointPtr joint);
	bool empty();
	void shutdown();

	uint32_t port;
	Ice::CommunicatorPtr comm;
	Ice::ObjectAdapterPtr adapter;
	JointMotorI *interface;
	std::vector<InnerModel::JointPtr> joints;
	std::vector<InnerModel::PrismaticJointPtr> prismaticjoints;
	SpecificWorker *worker;
};

class TouchSensorServer
{
public:
	TouchSensorServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker_, uint32_t _port);
	void add(InnerModel::TouchSensorPtr sensor);
	void remove(InnerModel::TouchSensorPtr sensor);
	bool empty();
	void shutdown();

	uint32_t port;
	Ice::CommunicatorPtr comm;
	Ice::ObjectAdapterPtr adapter;
	TouchSensorI *interface;
	std::vector<InnerModel::TouchSensorPtr> sensors;
	SpecificWorker *worker;
};


class LaserServer
{
public:
	LaserServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port);
	void add(InnerModel::LaserPtr laser);

	uint32_t port;
	Ice::ObjectAdapterPtr adapter;
	LaserI *interface;
	std::vector<InnerModel::LaserPtr> lasers;
};


class RGBDServer
{
public:
	RGBDServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port);
	void add(InnerModel::RGBDPtr rgbd);

	uint32_t port;
	Ice::ObjectAdapterPtr adapter;
	RGBDI *interface;
	std::vector<InnerModel::RGBDPtr> rgbds;
};


class IMUServer
{
public:
	IMUServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port);
	void add(InnerModel::IMUPtr imu);

	uint32_t port;
	Ice::ObjectAdapterPtr adapter;
	IMUI *interface;
	std::vector<InnerModel::IMUPtr> imus;
};


class DifferentialRobotServer
{
public:
	DifferentialRobotServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port);
	void add(InnerModel::DifferentialRobotPtr differentialrobot);

	uint32_t port;
	Ice::ObjectAdapterPtr adapter;
	DifferentialRobotI *interface;
	std::vector<InnerModel::DifferentialRobotPtr> differentialrobots;
};

class OmniRobotServer
{
public:
	OmniRobotServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port);
	void add(InnerModel::OmniRobotPtr omnirobot);

	uint32_t port;
	Ice::ObjectAdapterPtr adapter;
	OmniRobotI *interface;
	DifferentialRobotI *interfaceDFR;
	GenericBaseI *interfaceGB;
	std::vector<InnerModel::OmniRobotPtr> omnirobots;
};

