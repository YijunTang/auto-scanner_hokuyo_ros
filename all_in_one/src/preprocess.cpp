/*
 * Author : Yijun Tang
 * E-mail : yijun_tang@qq.com
 * Date   : 2018.7.24
 * Notes  : This source file contain the codes to preprocess the raw data captured by HOKUYO 2D-Lidar(UST-20LX), 
 *		 	the node named "preprocess_node" subscribes the topic of "/scan" published by "urg_node" node contained 
 *  		in official ros-package "urg_node", and flags each scan as true(some points on cuboid commodities) or false.
 *			Lastly, "preprocess_node" node publishes self-defined message "FlagedScan".
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <all_in_one/FlagedScan.h>
#include <string>
#include <cstddef>
#include <deque>
#include <cmath>

struct PointXYI
{
	float m_x;
	float m_y;
	float m_i;
	PointXYI(float x = 0.0, float y = 0.0, float i = 0.0) : m_x(x), m_y(y), m_i(i) {}
};

class RosNode
{
public:
	RosNode(){}

	inline void subscribe(const std::string &topicStr, int queueLen)
	{
		m_subscriber = m_nodeHandle.subscribe(topicStr, queueLen, scanCallback);
	}

	inline void setPublish(const std::string &topicStr, int queueLen)
	{
		m_publisher = m_nodeHandle.advertise<all_in_one::FlagedScan>(topicStr, queueLen);
	}

	inline ros::Publisher& getPublisher()
	{
		return m_publisher;
	}

	inline ros::NodeHandle& getNodeHandle()
	{
		return m_nodeHandle;
	}

	static void setPublish1(const std::string &topicStr, int queueLen, RosNode &node)
	{
		m_publisher1 = node.getNodeHandle().advertise<sensor_msgs::PointCloud>(topicStr, queueLen);
	}

protected:
	static void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
	{
		float angle = 0.0;			// angle between first point and current point
		PointXYI curPoint;

		sensor_msgs::PointCloud pointcloud;
		pointcloud.header.stamp = ros::Time::now();
		pointcloud.header.frame_id = "base";

		pointcloud.points.resize(m_endPointIndex - m_startPointIndex + 1);

		pointcloud.channels.resize(1);
		pointcloud.channels[0].name = "intensities";
		pointcloud.channels[0].values.resize(m_endPointIndex - m_startPointIndex + 1);

		for(int i = m_startPointIndex; i < m_endPointIndex; ++i)
		{
			angle = msg->angle_increment * i + msg->angle_min;
			curPoint.m_x = sin(angle) * msg->ranges[i];
			curPoint.m_y = cos(angle) * msg->ranges[i];
			curPoint.m_i = msg->intensities[i];

			if(msg->ranges[i] < 1.0)
			{
				pointcloud.points[i - m_startPointIndex].x = curPoint.m_x;
				pointcloud.points[i - m_startPointIndex].y = curPoint.m_y;
				pointcloud.points[i - m_startPointIndex].z = -m_numScans * m_timeUnit * m_rateOfBelt;
				pointcloud.channels[0].values[i - m_startPointIndex] = curPoint.m_i;

				// we only select points whoes ranges less than 1m and y less than 0.5m(interested ranges),
				// you can tune the value based on your hardware.
				if(curPoint.m_y < 0.5)
					m_scanBuffer.push_back(curPoint);
			}
		}

		m_publisher1.publish(pointcloud);

		all_in_one::FlagedScan curScan;
		curScan.isOnCommodity = (m_scanBuffer.size() == 0 ? 0 : 1);
		curScan.size = m_scanBuffer.size();
		curScan.x.resize(curScan.size);
		curScan.y.resize(curScan.size);
		curScan.i.resize(curScan.size);
		for(size_t i = 0; i < curScan.size; ++i)
		{
			curScan.x[i] = m_scanBuffer[i].m_x;
			curScan.y[i] = m_scanBuffer[i].m_y;
			curScan.i[i] = m_scanBuffer[i].m_i;
		}

		m_scanDeque.push_back(curScan);
		m_scanBuffer.clear();
	}

private:
	ros::NodeHandle m_nodeHandle;
	ros::Subscriber m_subscriber;
	ros::Publisher  m_publisher;

public:
	static int m_numScans;
	static std::vector<PointXYI> m_scanBuffer;
	static std::deque<all_in_one::FlagedScan> m_scanDeque;
	static ros::Publisher m_publisher1;

	// a scan contains 1081 points across 270 degrees, resolution of angle is 0.25 degree, 
	// we select points between 530 and 900 index cover the whole commodity. 
	const static int m_startPointIndex;
	const static int m_endPointIndex;
	const static float m_timeUnit;		// time of a scan consumed
	const static float m_rateOfBelt;	// rate of conveyor belt, this value need to calibrate by yourself
	const static float m_pi;
};

std::vector<PointXYI> RosNode::m_scanBuffer = std::vector<PointXYI>();
std::deque<all_in_one::FlagedScan> RosNode::m_scanDeque = std::deque<all_in_one::FlagedScan>();
ros::Publisher RosNode::m_publisher1 = ros::Publisher();
int RosNode::m_numScans = 0;
const int RosNode::m_startPointIndex = 530;
const int RosNode::m_endPointIndex = 900;
const float RosNode::m_timeUnit = 0.025;
const float RosNode::m_rateOfBelt = 0.0272;
const float RosNode::m_pi = 3.14159;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "preprocess_node");

	RosNode preprocess_node;
	preprocess_node.subscribe("/scan", 1000);
	preprocess_node.setPublish("/flaged_scans", 1000);
	RosNode::setPublish1("/raw_pointcloud", 1000, preprocess_node);

	tf::TransformBroadcaster broadcaster;

	ros::Rate loop_rate(40);
	while(ros::ok())
	{
		if(!RosNode::m_scanDeque.empty())
		{
			all_in_one::FlagedScan tmp = RosNode::m_scanDeque.front();
			RosNode::m_scanDeque.pop_front();

			preprocess_node.getPublisher().publish(tmp);
			RosNode::m_numScans++;
			ROS_INFO("Send scans: [%d]", RosNode::m_numScans);
		}

		broadcaster.sendTransform(tf::StampedTransform(
														tf::Transform(tf::Quaternion(cos(RosNode::m_pi/4), sin(RosNode::m_pi/4), 0, 0), 
														tf::Vector3(0.0, 0.0, 0.0)),
														ros::Time::now(),
														"base", 
														"laser"));

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
