#include "servers.h"
#include "specificworker.h"

/** XXXServer **/
/** XXXServer **/
/** XXXServer **/
JointMotorServer::JointMotorServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker_, uint32_t _port)
{
	port = _port;
	worker = worker_;
	std::stringstream out1;
	out1 << port;
	comm = communicator;
	std::string name = std::string("JointMotor") + out1.str();
	std::string endp = std::string("tcp -p ")    + out1.str();

	adapter = communicator->createObjectAdapterWithEndpoints(name, endp);
	printf("Creating JointMotor adapter <<%s>> with endpoint <<%s>>\n", name.c_str(), endp.c_str());
	interface = new JointMotorI(worker);
	adapter->add(interface, communicator->stringToIdentity("jointmotor"));
	adapter->activate();
}


void JointMotorServer::add(InnerModel::JointPtr joint)
{
	joints.push_back(joint);
	interface->add(joint->getId());
	joint->setAngle(joint->getHome());
}

void JointMotorServer::add(InnerModel::PrismaticJointPtr joint)
{
	prismaticjoints.push_back(joint);
	interface->add(joint->getId());
}

void JointMotorServer::remove(InnerModel::JointPtr joint)
{
	interface->remove(joint->getId());
	joints.erase(std::remove(joints.begin(), joints.end(), joint), joints.end());
}

void JointMotorServer::remove(InnerModel::PrismaticJointPtr joint)
{
	interface->remove(joint->getId());
	prismaticjoints.erase(std::remove(prismaticjoints.begin(), prismaticjoints.end(), joint), prismaticjoints.end());
}

bool JointMotorServer::empty()
{
	if (joints.size()==0 and prismaticjoints.size()==0)
		return true;
	return false;
}

void JointMotorServer::shutdown()
{
	try
	{
		adapter->remove(comm->stringToIdentity("jointmotor"));
	}
	catch(Ice::ObjectAdapterDeactivatedException e)
	{
	}

	adapter->destroy();
}

TouchSensorServer::TouchSensorServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker_, uint32_t _port)
{
	port = _port;
	worker = worker_;
	std::stringstream out1;
	out1 << port;
	comm = communicator;
	std::string name = std::string("TouchSensor") + out1.str();
	std::string endp = std::string("tcp -p ")     + out1.str();

	adapter = communicator->createObjectAdapterWithEndpoints(name, endp);
	printf("Creating TouchSensor adapter <<%s>> with endpoint <<%s>>\n", name.c_str(), endp.c_str());
	interface = new TouchSensorI(worker);
	adapter->add(interface, communicator->stringToIdentity("touchsensor"));
	adapter->activate();
}
void TouchSensorServer::add(InnerModel::TouchSensorPtr sensor)
{
	sensors.push_back(sensor);
	interface->add(sensor->getId());
}
void TouchSensorServer::remove(InnerModel::TouchSensorPtr sensor)
{
	interface->remove(sensor->getId());
	sensors.erase(std::remove(sensors.begin(), sensors.end(), sensor), sensors.end());
}
bool TouchSensorServer::empty()
{
	if (sensors.size()==0)
		return true;
	return false;
}
void TouchSensorServer::shutdown()
{
	try
	{
		adapter->remove(comm->stringToIdentity("touchsensor"));
	}
	catch(Ice::ObjectAdapterDeactivatedException e)
	{
	}
	adapter->destroy();
}

LaserServer::LaserServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port)
{
	port = _port;
	std::stringstream out1;
	out1 << port;
	std::string name = std::string("Laser") + out1.str();
	std::string endp = std::string("tcp -p ")    + out1.str();
	adapter = communicator->createObjectAdapterWithEndpoints(name, endp);
	printf("Creating Laser adapter <<%s>> with endpoint <<%s>>\n", name.c_str(), endp.c_str());
	interface = new LaserI(worker);
	adapter->add(interface, communicator->stringToIdentity("laser"));
	adapter->activate();
}


void LaserServer::add(InnerModel::LaserPtr laser)
{
	lasers.push_back(laser);
	interface->add(laser->getId());
}


RGBDServer::RGBDServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port)
{
	port = _port;
	std::stringstream out1;
	out1 << port;
	std::string name = std::string("RGBD") + out1.str();
	std::string endp = std::string("tcp -p ") + out1.str();
	adapter = communicator->createObjectAdapterWithEndpoints(name, endp);
	printf("Creating RGBD adapter <<%s>> with endpoint <<%s>>\n", name.c_str(), endp.c_str());
	interface = new RGBDI(worker);
	adapter->add(interface, communicator->stringToIdentity("rgbd"));
	adapter->activate();
}

void RGBDServer::add(InnerModel::RGBDPtr rgbd)
{
	rgbds.push_back(rgbd);
	interface->add(rgbd->getId());
}

IMUServer::IMUServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port)
{
	port = _port;
	std::stringstream out1;
	out1 << port;
	std::string name = std::string("IMU") + out1.str();
	std::string endp = std::string("tcp -p ") + out1.str();
	adapter = communicator->createObjectAdapterWithEndpoints(name, endp);
	printf("Creating IMU adapter <<%s>> with endpoint <<%s>>\n", name.c_str(), endp.c_str());
	interface = new IMUI(worker);
	adapter->add(interface, communicator->stringToIdentity("imu"));
	adapter->activate();
}


void IMUServer::add(InnerModel::IMUPtr imu)
{
	imus.push_back(imu);
	interface->add(imu->getId());
}


DifferentialRobotServer::DifferentialRobotServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port)
{
	port = _port;
	std::stringstream out1;
	out1 << port;
	std::string name = std::string("DifferentialRobot") + out1.str();
	std::string endp = std::string("tcp -p ") + out1.str();
	adapter = communicator->createObjectAdapterWithEndpoints(name, endp);
	printf("Creating DifferentialRobot adapter <<%s>> with endpoint <<%s>>\n", name.c_str(), endp.c_str());
	interface = new DifferentialRobotI(worker);
	adapter->add(interface, communicator->stringToIdentity("differentialrobot"));
	adapter->activate();
}

void DifferentialRobotServer::add(InnerModel::DifferentialRobotPtr differentialrobot)
{
	differentialrobots.push_back(differentialrobot);
	interface->add(differentialrobot->getId());
	interface->start();
}

OmniRobotServer::OmniRobotServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port)
{
	port = _port;
	std::stringstream out1;
	out1 << port;
	std::string name = std::string("OmniRobot") + out1.str();
	std::string endp = std::string("tcp -p ") + out1.str();
	adapter = communicator->createObjectAdapterWithEndpoints(name, endp);

	printf("Creating OmniRobot adapter <<%s>> with endpoint <<%s>>\n", name.c_str(), endp.c_str());
	interface = new OmniRobotI(worker);
	adapter->add(interface, communicator->stringToIdentity("omnirobot"));
	adapter->activate();

	printf("Creating DifferentialRobot [[emulated from an OmniRobot interface]] adapter <<%s>> with endpoint <<%s>>\n", name.c_str(), endp.c_str());
	interfaceDFR = new DifferentialRobotI(worker, interface);
	adapter->add(interfaceDFR, communicator->stringToIdentity("differentialrobot"));
	adapter->activate();

	printf("Creating GenericBase adapter <<%s>> with endpoint <<%s>>\n", name.c_str(), endp.c_str());
	interfaceGB = new GenericBaseI(worker, interface);
	adapter->add(interfaceGB, communicator->stringToIdentity("genericbase"));
	adapter->activate();
}

void OmniRobotServer::add(InnerModel::OmniRobotPtr omnirobot)
{
	omnirobots.push_back(omnirobot);
	interface->add(omnirobot->getId());
	interface->start();
	interfaceGB->add(omnirobot->getId());
//	interfaceGB->start();
// 	interfaceDFR->add(omnirobot->getId());
// 	interfaceDFR->start();
}
