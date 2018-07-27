/*
 * Author : Yijun Tang
 * E-mail : yijun_tang@qq.com
 * Date   : 2018.7.25
 * Notes  : This source file contain codes to process data preprocessed by "preprocess_node" node. "process_node" node
 *			subscribe "/flaged_scans" topic, and calculated z coordinate with rate of conveyor belt, height of commodity,
 *			number of commodities, and publish "sensor_msgs/PointCloud" messages.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <all_in_one/FlagedScan.h>
#include <vector>
#include <deque>
#include <cstddef>

struct PointXYZI
{
	float m_x;
	float m_y;
	float m_z;
	float m_i;
	PointXYZI(float x = 0.0, float y = 0.0, float z = 0.0, float i = 0.0) : m_x(x), m_y(y), m_z(z), m_i(i) {}
};

class RosNode
{
public:
	RosNode(){}

	inline void subscribe(const std::string &topicStr, int queueLen)
	{
		m_subscriber = m_nodeHandle.subscribe(topicStr, queueLen, flagedScanCallback);
	}

	inline void setPublish(const std::string &topicStr, int queueLen)
	{
		m_publisher = m_nodeHandle.advertise<sensor_msgs::PointCloud>(topicStr, queueLen);
	}

	inline ros::Publisher& getPublisher()
	{
		return m_publisher;
	}

protected:
	static void flagedScanCallback(const all_in_one::FlagedScan &msg)
	{
		bool isOnCommodityCur = ((msg.isOnCommodity == 1) ? true : false);

		if(!m_isOnCommodityLast && !isOnCommodityCur)
		{
			m_isOnCommodityLast = isOnCommodityCur;
		}
		else if(isOnCommodityCur)
		{
			int sizeOfScan = msg.size;
			PointXYZI point;

			for(int i = 0; i < sizeOfScan; ++i)
			{
				point.m_x = msg.x[i];
				point.m_y = msg.y[i];
				point.m_z = -m_numScans * m_timeUnit * m_rateOfBelt;
				point.m_i = msg.i[i];

				m_pointBuffer.push_back(point);
			}

			m_isOnCommodityLast = isOnCommodityCur;
		}
		else if(m_isOnCommodityLast && !isOnCommodityCur)
		{
			m_pointCloudDeque.push_back(m_pointBuffer);
			m_pointBuffer.clear();

			m_isOnCommodityLast = isOnCommodityCur;
		}

		m_numScans++;
	}

private:
	ros::NodeHandle m_nodeHandle;
	ros::Subscriber m_subscriber;
	ros::Publisher  m_publisher;

public:
	static int m_numScans;
	static bool m_isOnCommodityLast;	// save status of last flaged scan.

	static std::vector<PointXYZI> m_pointBuffer;	// contain the whole cloudpoint of a commodity
	static std::deque<std::vector<PointXYZI> > m_pointCloudDeque;	// pointcloud deque, each element represent a commodity

	const static float m_timeUnit;		// time of a scan consumed
	const static float m_rateOfBelt;	// rate of conveyor belt, this value need to calibrate by yourself
};

int RosNode::m_numScans = 0;
bool RosNode::m_isOnCommodityLast = false;
std::vector<PointXYZI> RosNode::m_pointBuffer = std::vector<PointXYZI>();
std::deque<std::vector<PointXYZI> > RosNode::m_pointCloudDeque = std::deque<std::vector<PointXYZI> >();
const float RosNode::m_timeUnit = 0.025;
const float RosNode::m_rateOfBelt = 0.0272;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "process_node");

	RosNode process_node;
	process_node.subscribe("/flaged_scans", 1000);
	process_node.setPublish("/recon_pointcloud", 2);

	ros::Rate loop_rate(3);
	while(ros::ok())
	{
		while(!RosNode::m_pointCloudDeque.empty())
		{
			std::vector<PointXYZI> tmp = RosNode::m_pointCloudDeque.front();
			RosNode::m_pointCloudDeque.pop_front();

			sensor_msgs::PointCloud pointCloud;
			pointCloud.header.stamp = ros::Time::now();
			pointCloud.header.frame_id = "base";

			pointCloud.points.resize(tmp.size());

			pointCloud.channels.resize(1);
			pointCloud.channels[0].name = "intensities";
			pointCloud.channels[0].values.resize(tmp.size());

			for(size_t sz = 0; sz < tmp.size(); ++sz)
			{
				pointCloud.points[sz].x = tmp[sz].m_x;
				pointCloud.points[sz].y = tmp[sz].m_y;
				pointCloud.points[sz].z = tmp[sz].m_z;
				pointCloud.channels[0].values[sz] = tmp[sz].m_i;
			}

			process_node.getPublisher().publish(pointCloud);
			ROS_INFO("Scanning a commodity.");
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
