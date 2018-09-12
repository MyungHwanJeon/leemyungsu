#include <fome/fome.h>
#include "fome/DataMsgContainer.h"

using namespace std;
using namespace cv;
Mat black(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,Scalar(0,0,0));

class ROS_fome
{
	private :
		ros::NodeHandle nh_;
		ros::NodeHandle nh_q;
		ros::CallbackQueue fomeque;

		ros::Subscriber eventsub_;
		ros::Subscriber param_decay_;
		ros::Subscriber param_frame_;
		ros::Subscriber param_segsz_;
		image_transport::Publisher mag_image_;
		image_transport::Publisher dir_image_;
		image_transport::Publisher cls_image_;
        ros::Publisher data_msg_node;
		
		cv::Mat mag_image_data;
		cv::Mat dir_image_data;
		cv::Mat cls_image_data;
        ros::Time next_publish_time;
        int seqnum;
		
	public :
		ROS_fome(ros::NodeHandle &nh1, ros::NodeHandle &nh2);
		Estimator fome;
		DataQueue<dvs_msgs::EventArray> dvs_queue_handle;
        boost::shared_ptr<ros::AsyncSpinner> q_spinner;
        void event_callback_queue(const dvs_msgs::EventArrayConstPtr&);
        void decay_rate_callback(const std_msgs::String::ConstPtr&);
        void frame_rate_callback(const std_msgs::String::ConstPtr&);
        void segment_size_callback(const std_msgs::String::ConstPtr&);
        void estimation();
        void image_publish();
        void data_publish();
        bool run();
        ~ROS_fome();
};

ROS_fome::ROS_fome(ros::NodeHandle &nh1, ros::NodeHandle &nh2)
{
    fome.init();
	fome.event_clock_now = ros::Time::now();
	image_transport::ImageTransport it_(nh_);
	mag_image_ = it_.advertise("/fome/magnitude",1);
	dir_image_ = it_.advertise("/fome/direction",1);
	cls_image_ = it_.advertise("/fome/cluster",1);

    data_msg_node = nh_.advertise<fome::DataMsgContainer>("/fome/data",10);
    seqnum = 0;
    next_publish_time = ros::Time::now() + ros::Duration(0.01);

	nh_ = nh1;
	nh_q = nh2;
	nh_q.setCallbackQueue(&fomeque);

    eventsub_ = nh_q.subscribe("/dvs/events", 2000000, &ROS_fome::event_callback_queue, this);
//	eventsub_ = nh_q.subscribe("/davis/left/events", 2000000, &ROS_fome::event_callback_queue, this);
	param_decay_ = nh_.subscribe("/fome/decay_rate", 1, &ROS_fome::decay_rate_callback, this);
	param_frame_ = nh_.subscribe("/fome/frame_rate", 1, &ROS_fome::frame_rate_callback, this);
	param_segsz_ = nh_.subscribe("/fome/segment_size", 1, &ROS_fome::segment_size_callback, this);
}

ROS_fome::~ROS_fome()
{
	dvs_queue_handle.active_ = false;
}

void ROS_fome::estimation()
{
	while (dvs_queue_handle.data_queue_.size() > 0)
	{
		if (dvs_queue_handle.active_==false) return;
		auto data = dvs_queue_handle.pop();
		for (int i=0;i < (int)data.events.size();i++)
		{
			fome.event_clock_now = data.events[i].ts;
			fome.stack(data.events[i]);
            if (next_publish_time.toSec() < ros::Time::now().toSec())
			{
				fome.calcuate();
                image_publish();
//                data_publish();
			}
		}
	}
}

void ROS_fome::data_publish()
{
    next_publish_time = ros::Time::now() + ros::Duration(1/fome.frame_rate);

    std_msgs::Float32MultiArray mag_msg;
    mag_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mag_msg.layout.dim[0].label = "height";
    mag_msg.layout.dim[0].size = IMAGE_HEIGHT;
    mag_msg.layout.dim[0].stride = IMAGE_WIDTH;
    mag_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mag_msg.layout.dim[1].label = "width";
    mag_msg.layout.dim[1].size = IMAGE_WIDTH;
    mag_msg.layout.dim[1].stride = 1;
    mag_msg.data.clear();
    mag_msg.data = fome.build_magnitude();

    std_msgs::Float32MultiArray dir_msg;
    dir_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dir_msg.layout.dim[0].label = "height";
    dir_msg.layout.dim[0].size = IMAGE_HEIGHT;
    dir_msg.layout.dim[0].stride = IMAGE_WIDTH;
    dir_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dir_msg.layout.dim[1].label = "width";
    dir_msg.layout.dim[1].size = IMAGE_WIDTH;
    dir_msg.layout.dim[1].stride = 1;
    dir_msg.data.clear();
    dir_msg.data = fome.build_direction();

    std_msgs::Float32MultiArray evv_msg;
    evv_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    evv_msg.layout.dim[0].label = "height";
    evv_msg.layout.dim[0].size = IMAGE_HEIGHT;
    evv_msg.layout.dim[0].stride = IMAGE_WIDTH;
    evv_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    evv_msg.layout.dim[1].label = "width";
    evv_msg.layout.dim[1].size = IMAGE_WIDTH;
    evv_msg.layout.dim[1].stride = 1;
    evv_msg.data.clear();
    evv_msg.data = fome.build_eventlevel();

    fome::DataMsgContainer data_msg;
    data_msg.header.seq = seqnum;
    data_msg.header.stamp = fome.event_clock_now;
    data_msg.header.frame_id = "0";
    data_msg.magnitude = mag_msg;
    data_msg.direction = dir_msg;
    data_msg.eventlevel = evv_msg;
    data_msg.height = IMAGE_HEIGHT;
    data_msg.width = IMAGE_WIDTH;
    data_msg_node.publish(data_msg);
    seqnum++;
}

void ROS_fome::image_publish()
{
    next_publish_time = ros::Time::now() + ros::Duration(1/fome.frame_rate);
	
	black.copyTo(mag_image_data);
	fome.magnitude_express(mag_image_data);
	sensor_msgs::ImagePtr mag_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mag_image_data).toImageMsg();
	mag_image_.publish(mag_msg);
	
	black.copyTo(dir_image_data);
	fome.direction_express(dir_image_data);
	sensor_msgs::ImagePtr dir_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dir_image_data).toImageMsg();
	dir_image_.publish(dir_msg);
	
	black.copyTo(cls_image_data);
	fome.cluster_express(cls_image_data);
	sensor_msgs::ImagePtr cls_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cls_image_data).toImageMsg();
	if (fome.clusterlist.lists.size() > 10)
	{
		cls_image_.publish(cls_msg);
		fome.clusterlist.clean();
    }
    fome.init();
}

void ROS_fome::event_callback_queue(const dvs_msgs::EventArrayConstPtr& msg_in)
{
    if (msg_in->events.size()>0)
		dvs_queue_handle.push(*msg_in);
	dvs_queue_handle.active_ = true;
}

void ROS_fome::decay_rate_callback(const std_msgs::String::ConstPtr& msg)
{
	string data = msg->data.c_str();
	if (data == "auto")
	{
		cout<<"Set decay rate fixed."<<endl;
    	fome.init();
	}		
	else
	{
        fome.decay_rate = pow(10,stod(data));
		cout<<"Set decay rate as : "<<fome.decay_rate<<"ms"<<endl;
	}
}

void ROS_fome::frame_rate_callback(const std_msgs::String::ConstPtr& msg)
{
	string data = msg->data.c_str();
	if (data == "auto")
	{
		cout<<"Set frame rate fixed."<<endl;
		fome.init();
	}		
	else
	{
        fome.frame_rate = stod(data);
		cout<<"Set frame rate as : "<<fome.frame_rate<<"hz"<<endl;
	}
}

void ROS_fome::segment_size_callback(const std_msgs::String::ConstPtr& msg)
{
	string data = msg->data.c_str();
	if (data == "auto")
	{
		cout<<"Set segment size fixed."<<endl;
		fome.init();
	}		
	else
	{
        fome.segment_size = stod(data);
		cout<<"Set segment size as : "<<fome.segment_size<<" (pixels)"<<endl;
	}
}

bool ROS_fome::run()
{
	ros::AsyncSpinner spinner(3);
	q_spinner.reset(new ros::AsyncSpinner(4, &fomeque));
	q_spinner->start();
	spinner.start();
	while(nh_q.ok()) estimation();
    return true;
}


int main(int argc, char* argv[])
{
    ros::init(argc,argv,"fome_ros_publisher");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_q;
	ROS_fome rosfome_node(nh_, nh_q);
	rosfome_node.run();
	return EXIT_SUCCESS;
}
