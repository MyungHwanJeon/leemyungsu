#include <dvsview.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

int main(int argc, char **argv)
{
// check cyusb link status with samsungdvs
	r = cyusb_open();
	if ( r < 0 )
	{
		printf("Error opening library\n");
		return -1;
	}
	else if ( r == 0 )
	{
		printf("No device found\n");
		return 0;
	}
	if ( r > 1 )
	{
		printf("More than 1 devices of interest found. Disconnect unwanted devices\n");
		return 0;
	}

	h1 = cyusb_gethandle(0);
	if ( cyusb_getvendor(h1) != 0x04b4 )
	{
		printf("Cypress chipset not detected\n");
		cyusb_close();
		return 0;
	}

	r = cyusb_kernel_driver_active(h1, 0);
	if ( r != 0 )
	{
		printf("kernel driver active. Exitting\n");
		cyusb_close();
		return 0;
	}

	r = cyusb_claim_interface(h1, 0);
	if ( r != 0 )
	{
		printf("Error in claiming interface\n");
		cyusb_close();
		return 0;
	}
//check cyusb end

	ros::init(argc, argv, "publishdvs");
	ros::NodeHandle nh_;
	ros::Publisher event_array_pub_ = nh_.advertise<dvs_msgs::EventArray>("/samdvs/events", 10);
	image_transport::ImageTransport it_(nh_);
	image_transport::Publisher image_pub_ = it_.advertise("/samdvs/image_raw", 1);

	dvs_msgs::EventArrayPtr event_array_msg;
	ros::Time inittime;

	buflen = cyusb_get_max_packet_size(h1, 0x81);
	printf("buffer size=%d\n", buflen);
	buf = (unsigned char *) malloc (buflen + 1);
	image.setTo(Scalar(128));

	ros::Duration publish_DT(PUB_DT*0.001);
	ros::Duration frame_DT(FRAME_DT*0.001);
	ros::Time next_msg_send_time = ros::Time::now() + publish_DT;
	ros::Time next_frame_send_time = ros::Time::now() + frame_DT;

	int initkey = 2;
	while (ros::ok())
	{
		r = cyusb_bulk_transfer(h1, 0x81, buf, buflen, &transferred, timeout);
		if (initkey == 2)
		{
			cout<<"Waiting sensor initialization..";
			initkey = 1;
			ros::Duration(0.5).sleep();
			cout<<"completed"<<endl;
		}
		//waiting for ros to initialize
		
		if (transferred % 4) // if transferred is not divided by 4, make it able.
			transferred = transferred - transferred % 4;

		if (!event_array_msg)
		{
			event_array_msg = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
			event_array_msg->height = 480;
			event_array_msg->width = 640;
		}

		for(int i=0;i<transferred; i+=4)
		{
			dvs_msgs::Event e;
			switch (buf[i])
			{
				case (0x66) :	// 0110 0110 | ---- TTTT | TTTT TTTT | TTTT TTTT	Reference Timestamp
					longTs = ((buf[i+1] & 0x0F) << 16) | ((buf[i+2] & 0xFF) << 8) | (buf[i+3] & 0xFF);
					break;

				case (0x99) :	// 1001 1001 | ---- TTTT | TTTT TTCC | CCCC CCCC	Column Address
					posX = (((buf[i+2] & 0x03) << 8) | (buf[i+3] & 0xFF)) - 1;	// Offset
					if ((posX < 0) || (posX >= 640)) continue;
					
					shortTs = ((buf[i+1] & 0x0F) << 6) | ((buf[i+2] & 0xFC) >> 2);
					timeStamp = (longTs << 10) | shortTs;
					if(longTs > 100000)
					{
						if (initkey == 1)
						{
							cout<<"Trying Timestamp synchronization...";
							inittime = ros::Time::now() - ros::Duration().fromNSec(1000*(longTs<<10)) - ros::Duration().fromNSec(shortTs);
							cout<<"completed"<<endl;
							initkey = 0;
							cout<<"current time :   "<<ros::Time::now()<<endl;
							cout<<"driver init time :   "<<inittime<<endl;
						}
					}
					else i=transferred;
					
					break;

				case (0xcc) :	// 1100 1100 | --GG GGGG | EEEE EEEE | EEEE EEEE	Group Event (Off / On)
					grpAddr = buf[i+1] & 0x3F;
					if ((grpAddr <= 0) || (grpAddr > 60)) continue;
					posY0 = (grpAddr -1) << 3;
					if (buf[i+2])// Off Event
					{
						pol = 1;
						for (n=0; n<8; n++)
						{
							if ((buf[i+2] >> n) & 0x01)
							{
								posY = posY0 + n;
								image.at<char> (posY, posX) = 

								e.x = posX;
								e.y = posY;
								e.ts = inittime + ros::Duration().fromNSec(1000*(longTs<<10)) + ros::Duration().fromNSec(shortTs);
								e.polarity = 0;
								if (e.ts.toSec() != 0)
								{
									event_array_msg->events.push_back(e);
								}
							}
						}
						continue;
					}
					if (buf[i+3])// On Event
					{
						pol = 0;
						for (n=0; n<8; n++)
						{
							if ((buf[i+3] >> n) & 0x01)
							{
								posY = posY0 + n;
								image.at<char> (posY, posX) = 255;
							
								e.x = posX;
								e.y = posY;
								e.ts = inittime + ros::Duration().fromNSec(1000*(longTs<<10)) + ros::Duration().fromNSec(shortTs);
								e.polarity = 1;
								if (e.ts.toSec() != 0)	event_array_msg->events.push_back(e);
							}
						}
						continue;
					}
					break;

				case (0x00):	// 0000 0000 | Padding
					break;

				default:
					break;

			} // switch:buf[i]

			if(i == 0)
			{
				event_array_msg->header.stamp = e.ts;
			}

		} // for:i
//start image update
		if((ros::Time::now()-next_frame_send_time).toSec() >FRAME_DT*0.001)
		{
			if(showImageFlag==1)
			{
//				imshow("/dvs_image",image);
				sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
				image_pub_.publish(msg);
			}
			
			char wk = waitKey(1);
			if(wk==32) showImageFlag = 1-showImageFlag;
			image.setTo(Scalar(128));
			next_frame_send_time += frame_DT;
		}
//end image update

//start dvs publish
		if ((ros::Time::now().toSec() > next_msg_send_time.toSec()) || (event_array_msg->events.size() > buflen))
		{
			event_array_pub_.publish(event_array_msg);

			if (event_array_msg->events.size() > buflen)	next_msg_send_time = ros::Time::now() + publish_DT;
			else next_msg_send_time += publish_DT;
			
			event_array_msg.reset();
		}
//end dvs publish

		ros::spinOnce();
	}// ros ok

	cyusb_close();
	return 0;
}
