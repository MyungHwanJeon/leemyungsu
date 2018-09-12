#include "fome_graph.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/stuff/sampler.h>
#include <g2o/stuff/command_args.h>
#include <g2o/core/factory.h>

using namespace std;
using namespace Eigen;
using namespace g2o;

Optimizer::Optimizer(){} //constructor
Optimizer::~Optimizer() {} //destructor

Frame::Frame(int id, int px, int py, ros::Time t, Eigen::MatrixXf evlv, Eigen::MatrixXf mag, Eigen::MatrixXf dir)
{
    frame_id = id; width = px; height = py; timestamp = t; eventlevel = evlv; _magnitude = mag; _direction = dir;
}

bool graph_fome::run()
{
    ros::AsyncSpinner spinner(0);
    spinner.start();
    while(graph_opt.ok()) Optimize();
    return true;
}

void graph_fome::Optimize()
{
    fome_graph.optimize_node();
}

void Optimizer::optimize_node()
{

}

Eigen::Vector3d graph_fome::unproject_point(double x, double y, double rho)
{
    Vector3d cam_3d_point = Vector3d::Zero();
    Vector3d img_3d_point = Vector3d(x,y,1);
    cam_3d_point = (1/rho)*Proj_inv*img_3d_point;
    return cam_3d_point;
}

void graph_fome::initial_estimate(Frame newframe, double dt)
{
    //build initial inverse depth matrix, based on event level
    MatrixXf idepth = MatrixXf::Zero(IMAGE_WIDTH,IMAGE_HEIGHT);
    newframe.depth_nz = 0;
    for (int i = 0 ; i < IMAGE_WIDTH-1 ; i++)
        for (int j = 0 ; j < IMAGE_HEIGHT-1 ; j++)
            if (newframe._magnitude(i,j) != 0)
            {
                idepth(i,j)= 1/newframe._magnitude(i,j);
                newframe.depth_nz++;
            }
    newframe.idepth = idepth;

    //build initial rotation & translation for given time interval(dt)

    //prepare original 3d point
    Matrix3d rot = T_imu2cam.block(0,0,3,3)*imu_rot; // initial value from imu
    Vector3d trn = Vector3d::Zero();
    map_3d original_3d;
    for (int i = 0 ; i < IMAGE_WIDTH-1 ; i++)
        for (int j = 0 ; j < IMAGE_HEIGHT-1 ; j++)
        {
            if (idepth(i,j) != 0)
                original_3d[i][j] = unproject_point((double) i,(double) j,idepth(i,j));
            else original_3d[i][j] = Vector3d::Zero();
        }
    //prepare estimated flow (u*dt)
    map_3d optiflow_3d;
    for (int i = 0 ; i < IMAGE_WIDTH-1 ; i++)
        for (int j = 0 ; j < IMAGE_HEIGHT-1 ; j++)
        {
            double flow_x = i + newframe._magnitude(i,j)*cos(newframe._direction(i,j));
            double flow_y = j + newframe._magnitude(i,j)*sin(newframe._direction(i,j));
            double flow_r = newframe.idepth(i,j);
            if (idepth(i,j) != 0)
                optiflow_3d[i][j] = unproject_point(flow_x, flow_y, flow_r);
            else optiflow_3d[i][j] = Vector3d::Zero();
        }
    //compute rotation & translation based on difference
    for (int i = 0; i < 10 ; i++)
    {
        map_3d trn_residual;
        for (int i = 0 ; i < IMAGE_WIDTH-1 ; i++)
            for (int j = 0 ; j < IMAGE_HEIGHT-1 ; j++)
            {
                if (optiflow_3d[i][j](2) == 0) trn_residual[i][j] = optiflow_3d[i][j];
                else trn_residual[i][j] = dt*optiflow_3d[i][j] - (rot - Matrix3d::Identity(3,3))*original_3d[i][j];
            }
        trn = trn_squaremin(newframe.depth_nz, trn_residual);
        map_3d rot_residual;
        for (int i = 0 ; i < IMAGE_WIDTH-1 ; i++)
            for (int j = 0 ; j < IMAGE_HEIGHT-1 ; j++)
            {
                if (optiflow_3d[i][j](2) == 0) rot_residual[i][j] = optiflow_3d[i][j];
                else rot_residual[i][j] = trn - dt*optiflow_3d[i][j] - original_3d[i][j];
            }
        rot = rot_squaremin(newframe.depth_nz, rot_residual, original_3d);
        if ( rot.determinant() > 2 || rot.determinant() < 0.5 )
        {
            rot = Matrix3d::Identity(3,3);
            trn = Vector3d(0,0,0);
        }
    }
    newframe._trn = trn;
    newframe._rot = rot;

    Frames.push_back(newframe);
}

Vector3d graph_fome::trn_squaremin(int nonzero, map_3d trn_residual)
{
    Vector3d trn = Vector3d::Zero();
    for (int i = 0 ; i < IMAGE_WIDTH-1 ; i++)
        for (int j = 0 ; j < IMAGE_HEIGHT-1 ; j++)
        {
            if (trn_residual[i][j](2) == 0) continue;
            else trn += trn_residual[i][j];
        }
    trn = (1/(double)nonzero)*trn;
    return trn;
}

Matrix3d graph_fome::rot_squaremin(int nonzero, map_3d rot_residual, map_3d original_3d)
{
    MatrixXd rot_res(3, nonzero);
    MatrixXd orignal(3, nonzero);
    int index = 0;
    for (int i = 0 ; i < IMAGE_WIDTH-1 ; i++)
        for (int j = 0 ; j < IMAGE_HEIGHT-1 ; j++)
            if (rot_residual[i][j](2) != 0)
            {
                rot_res(0,index) = rot_residual[i][j](0);
                rot_res(1,index) = rot_residual[i][j](1);
                rot_res(2,index) = rot_residual[i][j](2);
                orignal(0,index) = original_3d[i][j](0);
                orignal(1,index) = original_3d[i][j](1);
                orignal(2,index) = original_3d[i][j](2);
                index++;
            }
    Matrix3d rot = -rot_res * orignal.transpose() * (orignal * orignal.transpose()).inverse();
    if (isnan(rot.norm())) rot = Matrix3d::Identity(3,3);
    return rot;
}

void graph_fome::update_imu_data(const sensor_msgs::Imu::ConstPtr& imumsg)
{
    double q0 = imumsg->orientation.w;
    double q1 = imumsg->orientation.x;
    double q2 = imumsg->orientation.y;
    double q3 = imumsg->orientation.z;
    imu_ori << q0*q0+q1*q1-q2*q2-q3*q3, 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3),
            2*(q1*q2+q0*q3), q0*q0-q1*q1+q2*q2-q3*q2, 2*(q2*q3-q0*q1),
            2*(q1*q3-q0*q2), 2*(q0*q1+q2*q3), q0*q0-q1*q1-q2*q2+q3*q3;

    double tx = 0.01*imumsg->angular_velocity.x;
    double ty = 0.01*imumsg->angular_velocity.y;
    double tz = 0.01*imumsg->angular_velocity.z;
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
          0, cos(tx), -sin(tx),
          0, sin(tx), cos(tx);
    Eigen::Matrix3d Ry;
    Ry << cos(ty), 0, sin(ty),
          0, 1, 0,
          -sin(ty), 0, cos(ty);
    Eigen::Matrix3d Rz;
    Rz << cos(tz), -sin(tz), 0,
          sin(tz), cos(tz), 0,
          0, 0, 1;
    imu_rot = Rx*Ry*Rz;
    imu_rot_cov = imumsg->angular_velocity_covariance;
}

void graph_fome::create_raw_data(const fome::DataMsgContainer::ConstPtr& datamsg)
{
    //save raw data into Frame
    vector<float>::const_iterator mag_iter = datamsg->magnitude.data.begin();
    vector<float>::const_iterator dir_iter = datamsg->direction.data.begin();
    vector<float>::const_iterator evv_iter = datamsg->eventlevel.data.begin();
    Eigen::MatrixXf mag_map = zeromap;
    Eigen::MatrixXf dir_map = zeromap;
    Eigen::MatrixXf evv_map = zeromap;

    for (int i = 0 ; i < IMAGE_WIDTH-1 ; i++)
    {
        for (int j = 0 ; j < IMAGE_HEIGHT-1 ; j++)
        {
            mag_iter++;
            dir_iter++;
            evv_iter++;
            mag_map(i,j)= *mag_iter;
            dir_map(i,j)= *dir_iter;
            evv_map(i,j)= *evv_iter;
        }
    }

    //initializing new frame with given data
    Frame newframe(datamsg->header.seq, datamsg->width, datamsg->height, datamsg->header.stamp, evv_map, mag_map, dir_map);

    //update initial estimate
    initial_estimate(newframe, 0.01);
}


int main (int argc, char** argv)
{
    ros::init(argc,argv,"fome_graph");
    ros::NodeHandle graph_;
    ros::NodeHandle graph_opt;
    graph_fome graphfome_node(graph_, graph_opt);
//    graphfome_node.run();

    if (USE_ROSBAG)
    {
        rosbag::Bag bag;
        std::vector<std::string> topics;
        bag.open("/home/jhlee/data/fome_data/carttocar.bag", rosbag::bagmode::Read);
        topics.push_back(std::string("/fome/data"));
        topics.push_back(std::string("/dvs/imu"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        BOOST_FOREACH(rosbag::MessageInstance const message, view)
        {
            sensor_msgs::Imu::ConstPtr imudata = message.instantiate<sensor_msgs::Imu>();
            if (imudata != NULL)    graphfome_node.update_imu_data(imudata);
            fome::DataMsgContainer::ConstPtr fomedata = message.instantiate<fome::DataMsgContainer>();
            if (fomedata != NULL)   graphfome_node.create_raw_data(fomedata);
        }
        bag.close();
        cout<<"Rosbag file read complete"<<endl;
    }
    return EXIT_SUCCESS;
}
