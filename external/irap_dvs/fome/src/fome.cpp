#include <fome/fome.h>

using namespace std;
using namespace cv;
using namespace Eigen;
Estimator::Estimator() {}
Estimator::~Estimator()
{
	for(int cols=0;cols<IMAGE_WIDTH;cols++)
		for(int rows=0;rows<IMAGE_HEIGHT;rows++)
			delete Eventstack[cols][rows];
}

bool operator< (const XYpts &a,const XYpts &b)
{
	return a.value < b.value;
}
bool operator== (const XYpts &a, const XYpts &b)
{
	return ((a.x == b.x) && (a.y == b.y));
}
bool operator== (const Cluster &a, const int &b)
{
	return (a.num == b);
}

PointData::PointData(int x, int y, bool stat, double evlv, double decay, ros::Time updt, array<double,2> velo)
{
	xpos = x;
	ypos = y;
	status = stat;
	eventlevel = evlv;
	decay_time = decay;
	signal_time = updt;
	update_time = updt;
	velocity = velo;
}

void Estimator::init()
{
	event_clock_now = ros::Time::now();
	for(int cols=0;cols<IMAGE_WIDTH;cols++)
	{
		for(int rows=0;rows<IMAGE_HEIGHT;rows++)
		{
			Eventstack[cols][rows] = new PointData(rows,cols,0,0,decay_rate,event_clock_now,{0,DBL_MAX});
			Eventstack[cols][rows]->mtx = PTHREAD_MUTEX_INITIALIZER;
		}
	}
}

void Estimator::pinit(int x, int y, ros::Time timenow)
{
    Eventstack[x][y]->status = 0;
    Eventstack[x][y]->eventlevel = 0;
    Eventstack[x][y]->signal_time = timenow;
    Eventstack[x][y]->update_time = timenow;
    Eventstack[x][y]->decay_time = decay_rate;
    Eventstack[x][y]->velocity = {0,DBL_MAX};
}

void Estimator::timedecay(int x, int y)
{
	double dt = (event_clock_now - Eventstack[x][y]->update_time).toSec();
	Eventstack[x][y]->eventlevel*=exp(-dt/Eventstack[x][y]->decay_time);
	Eventstack[x][y]->update_time=event_clock_now;
}

void Estimator::stack(dvs_msgs::Event eventbuf)
{
	timedecay(eventbuf.x,eventbuf.y);
	Eventstack[eventbuf.x][eventbuf.y]->eventlevel+= eventbuf.polarity;
	Eventstack[eventbuf.x][eventbuf.y]->signal_time=eventbuf.ts;
	if (ngbd_constancy(eventbuf.x,eventbuf.y))
		make_seed(eventbuf.x,eventbuf.y);
}

void Estimator::calcuate()
{
	//build clusters from seeds
	seedlist.qmutex_.lock();
	while (seedlist.data_queue_.size() != 0)
	{
		XYpts newseed = seedlist.data_queue_.front();
		seedlist.data_queue_.pop();
		Cluster* newcluster = new Cluster(clusterlist.lists.size()+1);
		
		addto_cluster(newcluster, newseed.x, newseed.y);
		if (newcluster->points.size() > 1) clusterlist.push(*newcluster);
		delete newcluster;
	}
	seedlist.qmutex_.unlock();
	
	//calculate optical flow with event level & cluster info
	estimate_flow_lk();
}

void Estimator::estimate_flow_lk()
{ // estimate velocity by LK method
	//1. build a differential image, based on events, for 3x3 patch.
	MatrixXd diff_x(IMAGE_WIDTH,IMAGE_HEIGHT);
	MatrixXd diff_y(IMAGE_WIDTH,IMAGE_HEIGHT);
	MatrixXd diff_t(IMAGE_WIDTH,IMAGE_HEIGHT);
	
    for (int x = 0; x<IMAGE_WIDTH ; x++)
        for (int y = 0 ; y<IMAGE_HEIGHT ; y++)
			timedecay(x,y);
	for (int x = 2; x<IMAGE_WIDTH-2 ; x++)
	{
		for (int y = 2 ; y<IMAGE_HEIGHT-2 ; y++)
		{ //centeral finite difference with gaussian 3x3
			if (Eventstack[x][y]->eventlevel > 0)
			{
				diff_x(x,y) = (-1/THRS_1)*exp(-THRS_1*Eventstack[x][y]->eventlevel)
								*(Eventstack[x+2][y]->eventlevel - Eventstack[x-2][y]->eventlevel +
								2*Eventstack[x+1][y]->eventlevel - 2*Eventstack[x-1][y]->eventlevel)/4;
				diff_y(x,y) = (1/THRS_1)*exp(-THRS_1*Eventstack[x][y]->eventlevel)
								*(Eventstack[x][y+2]->eventlevel - Eventstack[x][y-2]->eventlevel +
								2*Eventstack[x][y+1]->eventlevel - 2*Eventstack[x][y-1]->eventlevel)/4;
				diff_t(x,y) = -(1/Eventstack[x][y]->decay_time)*abs(Eventstack[x][y]->eventlevel);
			}
			else
			{
				diff_x(x,y) = (1/THRS_0)*exp(-THRS_0*Eventstack[x][y]->eventlevel)
								*(Eventstack[x+2][y]->eventlevel - Eventstack[x-2][y]->eventlevel +
								2*Eventstack[x+1][y]->eventlevel - 2*Eventstack[x-1][y]->eventlevel)/4;
				diff_y(x,y) = (-1/THRS_0)*exp(-THRS_0*Eventstack[x][y]->eventlevel)
								*(Eventstack[x][y+2]->eventlevel - Eventstack[x][y-2]->eventlevel +
								2*Eventstack[x][y+1]->eventlevel - 2*Eventstack[x][y-1]->eventlevel)/4;
				diff_t(x,y) = -(1/Eventstack[x][y]->decay_time)*abs(Eventstack[x][y]->eventlevel);
			}
		}
	}

//	2. compute local patch optical flow with LK method
	for (list<Cluster>::iterator it = clusterlist.lists.begin() ; it != clusterlist.lists.end() ; ++it)
	{
		if (it->points.size() < 4) continue;
		MatrixXd xy(it->points.size(),2);
		VectorXd dt(it->points.size());
		int index = 0;
		for (list<XYpts>::iterator itp = it->points.begin(); itp != it->points.end() ; ++itp)
		{
            if ((itp->x < 2) || (itp->x > IMAGE_WIDTH-3) || (itp->y < 2) || (itp->y > IMAGE_HEIGHT-3)) continue;
			xy(index,0) = diff_x(itp->x,itp->y);
			xy(index,1) = diff_y(itp->x,itp->y);
			dt(index) = -diff_t(itp->x,itp->y);
			index++;
		}
        VectorXd nu = (xy.transpose() * xy).colPivHouseholderQr().solve(xy.transpose() * dt);
		if (isnan(nu.norm()) || nu.norm() == 0) continue; //checking NaN
		for (list<XYpts>::iterator itp = it->points.begin(); itp != it->points.end() ; ++itp)
		{
			Eventstack[itp->x][itp->y]->velocity[0] = nu.norm();
			Eventstack[itp->x][itp->y]->velocity[1] = 57.2958*atan2(nu(1),nu(0))+180;
			Eventstack[itp->x][itp->y]->status = 1;
		}
	}
}

bool Estimator::ngbd_constancy(int x, int y)
{
    if ( x<1 || x>IMAGE_WIDTH-2 || y<1 || y>IMAGE_HEIGHT-2 ) return false;
	int constancy = 0;
	double evlv = Eventstack[x][y]->eventlevel;
	for (int i = -1 ; i < 2 ; i++)
		for (int j = -1 ; j < 2 ; j++)
		{
			double ratio = Eventstack[x+i][y+j]->eventlevel / evlv;
			constancy += ( (ratio < 1.96) || (ratio > -1.96) );
		}
    if (constancy >= 2) return true;
	else return false;
}

void Estimator::make_seed(int x, int y)
{
	XYpts* newpt = new XYpts(x,y,Eventstack[x][y]->eventlevel);
	seedlist.push(*newpt);
}

void Estimator::addto_cluster(Cluster* newcluster, int x, int y)
{ //addto_cluster based on DBSCAN algorithm.
    if ( (x < 3) || (x > IMAGE_WIDTH-4) || (y < 3) || (y > IMAGE_HEIGHT-4) ) return;
    double threshold = 5;
    int numthreshold = 10*segment_size;
	
	clusterlist.map[x][y] = newcluster->num;
	newcluster->push(XYpts(x,y,Eventstack[x][y]->eventlevel));
	
    //predefined pattern, to efficiently find next points.
	array<int,12> xpos = {1,-1,2,-2,1,-1,0,0,-1,1,-2,2};
	array<int,12> ypos = {0,0,1,-1,2,-2,1,-1,2,-2,1,-1};
	for (int i = 0 ; i < 12; i++)
	{
		int xp = x+xpos[i];
        int yp = y+ypos[i];
		if (clusterlist.map[xp][yp] == newcluster->num) continue;
		else if (clusterlist.map[xp][yp] == 0)
		{
            if ( newcluster->pushtest(threshold, numthreshold, XYpts(xp,yp,Eventstack[xp][yp]->eventlevel)) )
				addto_cluster(newcluster,xp,yp);
		}
		else
		{
			std::list<Cluster>::iterator oldcluster = find(clusterlist.lists.begin(), clusterlist.lists.end(), clusterlist.map[xp][yp]);
			if (oldcluster == clusterlist.lists.end()) clusterlist.map[xp][yp] = 0;
			else
                if ( oldcluster->pushtest(threshold, numthreshold, XYpts(-1,-1,newcluster->min)) && oldcluster->pushtest(threshold, numthreshold, XYpts(-1,-1,newcluster->max)) )
				{
					newcluster->num = oldcluster->num;
					for (std::list<XYpts>::iterator iterP = newcluster->points.begin() ; iterP != newcluster->points.end() ; ++iterP)
						clusterlist.map[iterP->x][iterP->y] = oldcluster->num;
				}
		}
	}
}

array<int,3> hue2rgb(double hue)
{
    double p, q, t, ff, r, g, b;

    if(hue >= 360.0) hue -= 360*((int)(hue/360));
    hue /= 60.0;
    int i = (int)hue;
    ff = hue - i;
    p = 0.0;
    q = 1.0 - ff;
    t = ff;

    switch(i)
    {
		case 0:
		    r = 1; g = t; b = p;
		    break;
		case 1:
		    r = q; g = 1; b = p;
		    break;
		case 2:
		    r = p; g = 1; b = t;
		    break;
		case 3:
		    r = p; g = q; b = 1;
		    break;
		case 4:
		    r = t; g = p; b = 1;
		    break;
		case 5:
		    r = 1; g = p; b = q;
		    break;
		default:
		    r = 1; g = 1; b = 1;
		    break;
    }
    return {(int)(255*r),(int)(255*g),(int)(255*b)};  
}

void Estimator::direction_express(cv::Mat& image)
{
    for(int x=1; x<IMAGE_WIDTH-1; x++)
	{
        for(int y=1; y<IMAGE_HEIGHT-1; y++)
		{
			if (!Eventstack[x][y]->status) continue;
			array<int,3> rgb = hue2rgb(Eventstack[x][y]->velocity[1]);
			circle(image, cv::Point(x, y), 0, cv::Scalar(rgb[2],rgb[1],rgb[0]), 1, 4, 0);
		}
	}
}

void Estimator::magnitude_express(cv::Mat& image)
{
    for(int x=1; x<IMAGE_WIDTH-1; x++)
	{
        for(int y=1; y<IMAGE_HEIGHT-1; y++)
		{
			if (!Eventstack[x][y]->status) continue;
//            int i = 100*log(abs(Eventstack[x][y]->eventlevel[0])+1);
            int i = 1000*abs(Eventstack[x][y]->eventlevel);
			circle(image, cv::Point(x, y), 0, cv::Scalar(0.2*i,0.5*i,i), 1, 4, 0);
		}
	}
}

void Estimator::cluster_express(cv::Mat& image)
{
	if (clusterlist.lists.size() != 0)
		for (list<Cluster>::iterator iterC = clusterlist.lists.begin() ; iterC != clusterlist.lists.end() ; ++iterC)
		{
//            cout<<"size of cluster : "<<iterC->points.size()<<endl;
            array<int,3> rgb = hue2rgb(0.5*iterC->points.size());
			if (iterC->points.size() != 0)
				for (list<XYpts>::iterator iterP = iterC->points.begin() ; iterP != iterC->points.end() ; ++iterP)
                {
//                    cout<<"point : "<<iterP->x<<", "<<iterP->y<<endl;
					circle(image, cv::Point(iterP->x, iterP->y), 0, cv::Scalar(rgb[2],rgb[1],rgb[0]), 1, 4, 0);
                }
        }
	cv::waitKey(10);
}

std::vector<float> Estimator::build_magnitude()
{
    std::vector<float> output;
    for (int i = 0 ; i < IMAGE_WIDTH ; i++)
    {
        for (int j = 0 ; j < IMAGE_HEIGHT ; j++)
        {
            output.push_back(Eventstack[i][j]->velocity[0]);
        }
    }
    return output;
}
std::vector<float> Estimator::build_direction()
{
    std::vector<float> output;
    for (int i = 0 ; i < IMAGE_WIDTH ; i++)
    {
        for (int j = 0 ; j < IMAGE_HEIGHT ; j++)
        {
            output.push_back(Eventstack[i][j]->velocity[1]);
        }
    }
    return output;
}
std::vector<float> Estimator::build_eventlevel()
{
    std::vector<float> output;
    for (int i = 0 ; i < IMAGE_WIDTH ; i++)
    {
        for (int j = 0 ; j < IMAGE_HEIGHT ; j++)
        {
            output.push_back(Eventstack[i][j]->eventlevel);
        }
    }
    return output;
}
