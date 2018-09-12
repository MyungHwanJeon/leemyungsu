#include <array>
#include <cmath>
#include <cfloat>
#include <queue>
#include <string>
#include <list>
#include <chrono>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <iostream>
#include <sstream>
#include <ctime>
#include <Eigen/Eigen>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <pthread.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>
#include <std_msgs/Float32MultiArray.h>

#ifndef _FOME_H_
#define _FOME_H_

#define THRS_1 0.37
#define THRS_0 0.48

#ifndef IMAGE_WIDTH
#define IMAGE_WIDTH 240
#define IMAGE_HEIGHT 180
#endif

#ifndef DECAY_RATE
#define DECAY_RATE 0.1;
#define FRAME_RATE 1;
#define SEGMT_SIZE 600;
#endif

class PointData {
  public :
	pthread_mutex_t mtx;
    int xpos;
    int ypos;
    bool status; // status = 0 (off), 1 (on)
    double eventlevel;
    double decay_time;
    ros::Time signal_time;
    ros::Time update_time;
    std::array<double,2> velocity; //velocity = {speed, direction}
    PointData(int x, int y, bool stat, double evlv, double decay, ros::Time updt, std::array<double,2> velo);
};

template <typename T>
struct DataQueue {
  	std::mutex qmutex_;
  	std::queue<T> data_queue_;
  	bool active_;

  	DataQueue() : active_(true){}

	void push(T data)
	{
		qmutex_.lock();
		data_queue_.push(data);
		qmutex_.unlock();
	}

  	T pop()
  	{
		T result;
		qmutex_.lock();
		result = data_queue_.front();
		data_queue_.pop();
		qmutex_.unlock();
		return result;
	}
};

struct XYpts{
	int x;
	int y;
	double value;
	XYpts() {}
	XYpts(int xpos, int ypos, double val)
	{
		x = xpos; y = ypos; value = val;
	}
	bool operator== (const std::vector<int> &xy)
	{
		return ((this->x == xy[0]) && (this->y == xy[1]));
	}
};

struct Cluster{
	std::list<XYpts> points = {};
	int num = 0;
	double min = 0;
	double max = 0;
	
	double minimum()
	{
		if (points.size() == 0) return 0;
		else
		{
			min = std::min_element(points.begin(),points.end())->value;
			return min;
		}
	}
	
	double maximum()
	{
		if (points.size() == 0) return 0;
		else
		{
			max = std::max_element(points.begin(),points.end())->value;
			return max;
		}
	}
	
    bool pushtest(double threshold, int numthreshold, XYpts curr_point)
    {
        if (points.size() >= numthreshold) return false;
        if (curr_point.value > 0)
		{
            if ( (curr_point.value < min*(1+threshold)) && (curr_point.value  > max*(1-threshold)) ) return true;
			else return false;
		}
		else
		{
            if ( (curr_point.value  < min*(1-threshold)) && (curr_point.value  > max*(1+threshold)) ) return true;
			else return false;
		}
	}
	
	void push(XYpts newpoint)
    {
        if (std::find(points.begin(),points.end(),newpoint) != points.end()) return;
        points.push_back(newpoint);
		minimum(); maximum();
	}
	
	Cluster(int number)
	{
		num = number;
	}
};

bool operator< (const XYpts &a, const XYpts &b);
bool operator== (const XYpts &a, const XYpts &b);
bool operator== (const Cluster &a, const int &b);

struct Clusterlist{
	std::list<Cluster> lists = {};
  	std::mutex lmutex_;

	std::array<std::array<int,IMAGE_HEIGHT>,IMAGE_WIDTH> map;
	Clusterlist()
	{
		std::array<int,IMAGE_HEIGHT> col;
		col.fill(0);
		map.fill(col);
	}
	void clean()
	{
  		lmutex_.lock();
		std::array<int,IMAGE_HEIGHT> col;
		col.fill(0);
		map.fill(col);
		lists = {};
  		lmutex_.unlock();
	}
	
	void push(Cluster newcluster)
	{
		std::list<Cluster>::iterator it = find(lists.begin(), lists.end(), newcluster.num);
		if (it == lists.end())
		{
  			lmutex_.lock();
			lists.push_back(newcluster);
  			lmutex_.unlock();
		}
		else
		{
			for (std::list<XYpts>::iterator iterP = newcluster.points.begin() ; iterP != newcluster.points.end() ; ++iterP)
				it->points.push_back(*iterP);
			it->minimum(); it->maximum();
		}
	}
};

class Estimator {
  public:
    Estimator();
    std::array<std::array<PointData*,IMAGE_HEIGHT>, IMAGE_WIDTH> Eventstack;
    ros::Time event_clock_now;
    cv_bridge::CvImagePtr cv_ptr;
    double decay_rate = DECAY_RATE;
	double frame_rate = FRAME_RATE;
	double segment_size = SEGMT_SIZE;

    void init();
    void pinit(int, int, ros::Time);
    void timedecay(int, int);
    void stack(dvs_msgs::Event);
    
    void estimate_flow_lk();
    bool ngbd_constancy(int, int);
    
    void calcuate();

    void static_express(cv::Mat&);
    void magnitude_express(cv::Mat&);
    void direction_express(cv::Mat&);
    void cluster_express(cv::Mat&);

    std::vector<float> build_magnitude();
    std::vector<float> build_direction();
    std::vector<float> build_eventlevel();

    Clusterlist clusterlist;
    DataQueue<XYpts> seedlist;
    
	void make_seed(int, int);
	void addto_cluster(Cluster*, int, int);
	~Estimator();
};

#endif
