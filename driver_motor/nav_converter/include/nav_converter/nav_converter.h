#ifndef NAVI_H
#define NAVI_H
#pragma once

#include <ros/ros.h>
#include <unistd.h>
#include <stdint.h>
#include "libraryparam/agvparam.h"

class navigation_converter
{
public:
	agvParam param;
	/*
	 *	Convert from Twist message to speed_wheel message
	 * @param[in] msg struct of cmd_vel
	 */
	speedWheel navigationConverter(cmd_vel msg);

	/*
	*  Constructor.
	*  @param[in] nh ros::NodeHandle
	*/
	navigation_converter(ros::NodeHandle & nodeHandle);

	/*
	* Destructor.
	*/
	~navigation_converter();
private:
	void agvGetparam(agvParam &param);

	ros::NodeHandle nh_;
};

navigation_converter::navigation_converter(ros::NodeHandle & nodeHandle)
 : nh_(nodeHandle)
{
	agvGetparam(param);
}

navigation_converter::~navigation_converter() {}

speedWheel navigation_converter::navigationConverter(cmd_vel msg)
{
	speedWheel V;
	V.letf = -((2 * msg.linear.x - msg.angular.z * param.L) / (2 * param.R) * param.K * RAD_PER_RPM);
	V.right = (2 * msg.linear.x + msg.angular.z * param.L) / (2 * param.R) * param.K * RAD_PER_RPM;

	if (abs(V.letf) > param.speedMotor_max)
	{
		if (V.letf > 0)
			V.letf = param.speedMotor_max;
		else if (V.letf < 0)
			V.letf = -param.speedMotor_max;
	}

	if (abs(V.right) > param.speedMotor_max)
	{
		if (V.right > 0)
			V.right = param.speedMotor_max;
		else if (V.right < 0)
			V.right = -param.speedMotor_max;
	}
	if (abs(V.letf) < param.speedMotor_min)
		V.letf = 0;
	if (abs(V.right) < param.speedMotor_min)
		V.right = 0;
	return V;
}

void navigation_converter::agvGetparam(agvParam &param)
{
	char paramName[70];
	sprintf(paramName, "/L");
	nh_.getParam(paramName, param.L);
	ROS_INFO("%s = %f", paramName, param.L);
	sprintf(paramName, "/R");
	nh_.getParam(paramName, param.R);
	ROS_INFO("%s = %f", paramName, param.R);
	sprintf(paramName, "/K");
	nh_.getParam(paramName, param.K);
	ROS_INFO("%s = %f", paramName, param.K);
	sprintf(paramName, "/SpeedMotorMax");
	nh_.getParam(paramName, param.speedMotor_max);
	ROS_INFO("%s = %d", paramName, param.speedMotor_max);
	sprintf(paramName, "/SpeedMotorMin");
	nh_.getParam(paramName, param.speedMotor_min);
	ROS_INFO("%s = %d", paramName, param.speedMotor_min);
}
#endif