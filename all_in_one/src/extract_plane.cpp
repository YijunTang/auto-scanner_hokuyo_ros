/*
 * Author : Yijun Tang
 * E-mail : yijun_tang@qq.com
 * Date   : 2018.7.25
 * Notes  : This source file contain codes to extract top plane of commodity. "extract_plane_node" node subscribe
 *			"/recon_pointcloud" topic published by "process_node" node, and calculate width and length of the top
 *			plane extracted previously.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <cstddef>
#include <algorithm>

using namespace std;

/**************************************************************************************************************************/
/**** this code segment contain functions to calculate minimum rectangle which can contain the given points in a plane ****/
/**** you can consider this part as a black box to use, if you are interested in details about minimum rectangle, you  ****/
/**** can get insights in tutorials of Computational Geometry.                                                         ****/
/**************************************************************************************************************************/
typedef double typev;  
const double eps = 1e-8;  
const int N = 50005;  
int sign(double d){  
    return d < -eps ? -1 : (d > eps);  
}  
struct point{  
    typev x, y;  
    point(typev _x = 0.0, typev _y = 0.0) : x(_x), y(_y) {}
    point operator-(point d){  
        point dd;  
        dd.x = this->x - d.x;  
        dd.y = this->y - d.y;  
        return dd;  
    }  
    point operator+(point d){  
        point dd;  
        dd.x = this->x + d.x;  
        dd.y = this->y + d.y;  
        return dd;  
    }  
    void read(){ scanf("%lf%lf", &x, &y); }  
}ps[N];  
int n, cn;  
double dist(point d1, point d2){  
    return sqrt(pow(d1.x - d2.x, 2.0) + pow(d1.y - d2.y, 2.0));  
}  
double dist2(point d1, point d2){  
    return pow(d1.x - d2.x, 2.0) + pow(d1.y - d2.y, 2.0);  
}  
bool cmp(point d1, point d2){  
    return d1.y < d2.y || (d1.y == d2.y && d1.x < d2.x);  
}  
//st1-->ed1叉乘st2-->ed2的值  
typev xmul(point st1, point ed1, point st2, point ed2){  
    return (ed1.x - st1.x) * (ed2.y - st2.y) - (ed1.y - st1.y) * (ed2.x - st2.x);  
}  
typev dmul(point st1, point ed1, point st2, point ed2){  
    return (ed1.x - st1.x) * (ed2.x - st2.x) + (ed1.y - st1.y) * (ed2.y - st2.y);  
}  
//多边形类  
struct poly{  
    static const int N = 50005; //点数的最大值  
    point ps[N+5]; //逆时针存储多边形的点,[0,pn-1]存储点  
    int pn;  //点数  
    poly() { pn = 0; }  
    //加进一个点  
    void push(point tp){  
        ps[pn++] = tp;  
    }  
    //第k个位置  
    int trim(int k){  
        return (k+pn)%pn;  
    }  
    void clear(){ pn = 0; }  
};  
//返回含有n个点的点集ps的凸包  
poly graham(point* ps, int n){  
    sort(ps, ps + n, cmp);  
    poly ans;  
    if(n <= 2){  
        for(int i = 0; i < n; i++){  
            ans.push(ps[i]);  
        }  
        return ans;  
    }  
    ans.push(ps[0]);  
    ans.push(ps[1]);  
    point* tps = ans.ps;  
    int top = -1;  
    tps[++top] = ps[0];  
    tps[++top] = ps[1];  
    for(int i = 2; i < n; i++){  
        while(top > 0 && xmul(tps[top - 1], tps[top], tps[top - 1], ps[i]) <= 0) top--;  
        tps[++top] = ps[i];  
    }  
    int tmp = top;  //注意要赋值给tmp！  
    for(int i = n - 2; i >= 0; i--){  
        while(top > tmp && xmul(tps[top - 1], tps[top], tps[top - 1], ps[i]) <= 0) top--;  
        tps[++top] = ps[i];  
    }  
    ans.pn = top;  
    return ans;  
}  
//求点p到st->ed的垂足，列参数方程  
point getRoot(point p, point st, point ed){  
    point ans;  
    double u=((ed.x-st.x)*(ed.x-st.x)+(ed.y-st.y)*(ed.y-st.y));  
    u = ((ed.x-st.x)*(ed.x-p.x)+(ed.y-st.y)*(ed.y-p.y))/u;  
    ans.x = u*st.x+(1-u)*ed.x;  
    ans.y = u*st.y+(1-u)*ed.y;  
    return ans;  
}  
//next为直线(st,ed)上的点，返回next沿(st,ed)右手垂直方向延伸l之后的点  
point change(point st, point ed, point next, double l){  
    point dd;  
    dd.x = -(ed - st).y;  
    dd.y = (ed - st).x;  
    double len = sqrt(dd.x * dd.x + dd.y * dd.y);  
    dd.x /= len, dd.y /= len;  
    dd.x *= l, dd.y *= l;  
    dd = dd + next;  
    return dd;  
}  
//求含n个点的点集ps的最小面积矩形，并把结果放在ds(ds为一个长度是4的数组即可,ds中的点是逆时针的)中，并返回这个最小面积。  
double getMinAreaRect(point* ps, int n, point* ds){  
    int cn, i;  
    double ans;  
    point* con;  
    poly tpoly = graham(ps, n);  
    con = tpoly.ps;  
    cn = tpoly.pn;  
    if(cn <= 2){  
        ds[0] = con[0]; ds[1] = con[1];  
        ds[2] = con[1]; ds[3] = con[0];  
        ans=0;  
    }else{  
        int  l, r, u;  
        double tmp, len;  
        con[cn] = con[0];  
        ans = 1e40;  
        l = i = 0;  
        while(dmul(con[i], con[i+1], con[i], con[l])  
            >= dmul(con[i], con[i+1], con[i], con[(l-1+cn)%cn])){  
                l = (l-1+cn)%cn;  
        }  
        for(r=u=i = 0; i < cn; i++){  
            while(xmul(con[i], con[i+1], con[i], con[u])  
                <= xmul(con[i], con[i+1], con[i], con[(u+1)%cn])){  
                    u = (u+1)%cn;  
            }  
            while(dmul(con[i], con[i+1], con[i], con[r])  
                <= dmul(con[i], con[i+1], con[i], con[(r+1)%cn])){  
                    r = (r+1)%cn;  
            }  
            while(dmul(con[i], con[i+1], con[i], con[l])  
                >= dmul(con[i], con[i+1], con[i], con[(l+1)%cn])){  
                    l = (l+1)%cn;  
            }  
            tmp = dmul(con[i], con[i+1], con[i], con[r]) - dmul(con[i], con[i+1], con[i], con[l]);  
            tmp *= xmul(con[i], con[i+1], con[i], con[u]);  
            tmp /= dist2(con[i], con[i+1]);  
            len = xmul(con[i], con[i+1], con[i], con[u])/dist(con[i], con[i+1]);  
            if(sign(tmp - ans) < 0){  
                ans = tmp;  
                ds[0] = getRoot(con[l], con[i], con[i+1]);  
                ds[1] = getRoot(con[r], con[i+1], con[i]);  
                ds[2] = change(con[i], con[i+1], ds[1], len);  
                ds[3] = change(con[i], con[i+1], ds[0], len);  
            }  
        }  
    }  
    return ans+eps;  
}
/**************************************************************************************************************************/

class RosNode
{
public:
	RosNode(){}

	inline void subscribe(const std::string &topicStr, int queueLen)
	{
		m_subscriber = m_nodeHandle.subscribe(topicStr, queueLen, pointCloudCallback);
	}

protected:
	static void pointCloudCallback(const sensor_msgs::PointCloud &msg)
	{
		m_numOfCommodities++;

		/**** part 1: convert the type of pointcloud from 'sensor_msgs::PointCloud' to 'pcl::PointCloud<pcl::PointXYZ>' ****/
		sensor_msgs::PointCloud2 msg2;
		bool flag = sensor_msgs::convertPointCloudToPointCloud2(msg, msg2);
		if(!flag)
		{
			std::cerr << "sensor_msgs::convertPointCloudToPointCloud2: error!!!" << std::endl;
			return;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(msg2, *cloud);

		std::cout << "PointCloud current commodity: " << cloud->width * cloud->height << " data points." << std::endl;

		/**** part 2: extract top plane of cuboid commodity, extracted pointcloud contained in 'cloud_p' ****/
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
		// Create the segmentation object
  		pcl::SACSegmentation<pcl::PointXYZ> seg;
  		// Optional
  		seg.setOptimizeCoefficients (true);
  		// Mandatory
  		seg.setModelType (pcl::SACMODEL_PLANE);
  		seg.setMethodType (pcl::SAC_RANSAC);
  		seg.setMaxIterations (1000);
  		seg.setDistanceThreshold (0.01);

  		// Create the filtering object
  		pcl::ExtractIndices<pcl::PointXYZ> extract;

  		// Segment the largest planar component from the remaining cloud
    	seg.setInputCloud (cloud);
    	seg.segment (*inliers, *coefficients);
    	if (inliers->indices.size () == 0)
    	{
      		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      		return;
    	}

    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

    	// Extract the inliers
    	extract.setInputCloud (cloud);
    	extract.setIndices (inliers);
    	extract.setNegative (false);
    	extract.filter (*cloud_p);
    	std::cout << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    	/**** part 3: calculate the minimum rectangle which can contain the whole top plane pointcloud ****/
    	n = 0;

    	double height = 0.0;
    	for(size_t i = 0; i < cloud_p->points.size(); ++i)
    	{
    		height += cloud_p->points[i].y;
    	}
    	height /= cloud_p->points.size();
    	height = 0.575 - height;
  		for(; n < cloud_p->points.size(); ++n)
  		{
    		ps[n].x = cloud_p->points[n].x;
    		ps[n].y = cloud_p->points[n].z;
  		}

  		point ds[4];

  		double area = getMinAreaRect(ps, n, ds);
  		double lenght1 = sqrt(pow(ds[0].x - ds[1].x, 2) + pow(ds[0].y - ds[1].y, 2));
  		double lenght2 = sqrt(pow(ds[1].x - ds[2].x, 2) + pow(ds[1].y - ds[2].y, 2));
  		/*std::cout << "1: (" << ds[0].x << ", " << ds[0].y << ")" << std::endl;
  		std::cout << "2: (" << ds[1].x << ", " << ds[1].y << ")" << std::endl;
  		std::cout << "3: (" << ds[2].x << ", " << ds[2].y << ")" << std::endl;
  		std::cout << "4: (" << ds[3].x << ", " << ds[3].y << ")" << std::endl;*/

  		cout << "******************************************************************" << endl;
  		cout << "Sum of Boxes: " << m_numOfCommodities << endl;
  		cout << "      length: " << (lenght1 > lenght2 ? lenght1 : lenght2) << endl;
  		cout << "       width: " << (lenght1 > lenght2 ? lenght2 : lenght1) << endl;
  		cout << "      height: " << height << endl;
  		cout << "******************************************************************" << endl;
	}

private:
	ros::NodeHandle m_nodeHandle;
	ros::Subscriber m_subscriber;

public:
	static int m_numOfCommodities;
};

int RosNode::m_numOfCommodities = 0;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "extract_plane_node");

	RosNode extract_plane_node;
	extract_plane_node.subscribe("/recon_pointcloud", 2);

	ros::spin();

	return 0;
}
