#include <fome/fome.h>
#include <fome/DataMsgContainer.h>
#include <sensor_msgs/Imu.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>
#ifndef _FOME_GRAPH_H_
#define _FOME_GRAPH_H_

#define USE_ROSBAG 1

class Optimizer {
  public:
    Optimizer();
    void optimize_node();
    ~Optimizer();
};

class Frame {
    public :
        int frame_id;
        int width;
        int height;
        ros::Time timestamp;

        Eigen::Vector3d _trn;
        Eigen::Matrix3d _rot;

        Eigen::MatrixXf idepth;
        int depth_nz;
        Eigen::MatrixXf eventlevel;
        Eigen::MatrixXf _magnitude;
        Eigen::MatrixXf _direction;
        Frame(int id, int px, int py, ros::Time t, Eigen::MatrixXf evlv, Eigen::MatrixXf mag, Eigen::MatrixXf dir);
};

class graph_fome
{
    private :
        typedef std::array<std::array<Eigen::Vector3d,IMAGE_HEIGHT>,IMAGE_WIDTH> map_3d;
        Eigen::MatrixXd Proj;
        Eigen::Matrix3d Proj_inv;
        Eigen::Matrix4d T_imu2cam;
        Eigen::Matrix4d T_cam2imu;
        Eigen::Matrix3d imu_ori;
        Eigen::Matrix3d imu_rot;
        boost::array<double,9> imu_rot_cov;

        ros::NodeHandle graph_;
        ros::NodeHandle graph_opt;
        ros::Subscriber data_sub_;
        ros::Subscriber imu_sub_;

        Eigen::MatrixXf zeromap;

    public :
        std::vector<Frame> Frames;
        Optimizer fome_graph;
        boost::shared_ptr<ros::AsyncSpinner> graph_spinner;

        graph_fome(ros::NodeHandle &nh1, ros::NodeHandle &nh2)
        {
            zeromap = Eigen::MatrixXf::Zero(IMAGE_WIDTH,IMAGE_HEIGHT);
            if (USE_ROSBAG) std::cout<<"Using Rosbag file.."<<std::endl;
            else
            {
                data_sub_ = graph_.subscribe("/fome/data", 1, &graph_fome::create_raw_data,this);
                imu_sub_ = graph_.subscribe("/dvs/imu", 1, &graph_fome::update_imu_data,this);
            }
            Proj = Eigen::MatrixXd(3,4);
            imu_rot = Eigen::Matrix3d::Zero();
            Proj << 199.092366542 , 0.0, 132.192071378, 0.0,
                    0.0, 198.82882047, 110.712660011, 0.0,
                    0.0, 0.0, 1.0, 0.0;
            Proj_inv << 0.00502279, 0, -0.663974,
                    0,0.00502945,-0.556824,
                    0,0,1;
            T_imu2cam << 0.99068559, -0.00436895, -0.13609911,  0.11357021,
                    0.00040228,  0.9995747,  -0.02915932,  0.0322925,
                    0.13616862,  0.02883297,  0.99026601, -0.28642442,
                    0.,          0.,          0.,          1.       ;
            T_cam2imu << 0.99068559,  0.00040228,  0.13616862, -0.07352334,
                    -0.00436895,  0.9995747,   0.02883297, -0.02352412,
                    -0.13609911, -0.02915932,  0.99026601,  0.30003479,
                     0.        ,  0.        ,  0.        ,  1.        ;
        }


        void create_raw_data(const fome::DataMsgContainer::ConstPtr&);
        void update_imu_data(const sensor_msgs::Imu::ConstPtr&);
        void initial_estimate(Frame newframe, double dt);

        Eigen::Vector2d project_point(Frame);
        Eigen::Vector3d unproject_point(double x, double y, double rho);
        Eigen::Vector3d trn_squaremin(int nz, map_3d trn_residual);
        Eigen::Matrix3d rot_squaremin(int nz, map_3d rot_residual, map_3d original_3d);
        void Optimize();
        bool run();
        ~graph_fome() {}
};

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    signalMessage(msg);
  }
};
#endif
