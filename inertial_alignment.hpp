
/*---------------------------- INCLUDES/MACROS -------------------------------*/

#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/StdVector>
#include <Eigen/Dense>

#define PI 3.14159265
#define G  9.80665


/*-------------------------------- CLASSES -----------------------------------*/
//--- Codependent classes declaration
class pose;

//--- Classes definition
typedef struct alignment_data{
    double pitch;              // 静基座下可以确定pitch和roll,从而确定b系到R系的旋转矩阵
    double roll;
    Eigen::Matrix3d R;                  // wRb, Update to quaternions
} alignment;

typedef struct sensor_accelerometer{
    double nd;
    double rw;
    Eigen::Vector3d bias;
} sensor_accelerometer;

typedef struct sensor_gyroscope{
    double nd;
    double rw;
    Eigen::Vector3d bias;
} sensor_gyroscope;

class sensor_extrinsics{
    public:
        Eigen::Matrix4d T;
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        void print();
};

class ground_truth{
    public:
        //--- 'ground_truth' info
        std::string type;               // "vicon", "mle"
        sensor_extrinsics extrinsics;
        //--- 'ground_truth' data
        Eigen::MatrixXd data;
        //--- 'ground_truth' methods
        ground_truth(std::string t);
        int  get_pose(pose &pose_gt, int row);
        void print();
        void print(int index);
};

class vicon{
    public:
        Eigen::Matrix4d extrinsics;
        void print();
        Eigen::Vector3d t();
        Eigen::Matrix3d R();
};

class imu{
    public:
        //--- 'imu' info
        int rate;
        sensor_accelerometer accelerometer;     // 噪声密度 随机游走 随机常值偏移
        sensor_gyroscope gyroscope;
        Eigen::Matrix4d extrinsics;             // imu的外参?是什么?
        alignment_data alignment;               // imu所确定的姿态,roll pitch和姿态阵? 
        //--- 'imu' data
        Eigen::MatrixXd data;                   // imu观测值,矩阵的每一行表示一帧数据吧
        //--- 'imu' methods
        void print();
        void initialize(ground_truth &gt, int samples);
};

class camera{
    public:
        int rate;
        Eigen::Vector2d resolution;      // 分辨率
        Eigen::Vector4d distortion;      // 畸变
        Eigen::Vector4d intrinsics;      // 标定产生的内参文件
        Eigen::Matrix4d extrinsics;      // 标定产生的外参文件
        void print();
};

class pose{
    public:
        double timestamp;
        Eigen::Vector3d position;       // 状态量,位置速度姿态阵
        Eigen::Vector3d velocity;
        Eigen::Matrix3d orientation;    // wRb, Update to quaternions
        int initialize(ground_truth &gt, imu &s);     // pose赋初值    gt给位置,初始速度为0,imu给时间戳和姿态阵
        int update(imu &s, int row);
        void print();
};


/*------------------------ FUNCTION PROTOTYPES -------------------------------*/

/* File I/O */
int loadCSV(std::string &filename, Eigen::MatrixXd &data);

template<class sensor> int loadYAML(std::string &filename, sensor &s);
template<> int loadYAML<vicon>(std::string &filename, vicon &s);
template<> int loadYAML<imu>(std::string &filename, imu &s);
template<> int loadYAML<camera>(std::string &filename, camera &s);

/* Misc */
void compute_error(pose &p_error, pose &p, pose & p_gt);

void align_datasets(ground_truth &gt, imu &s);
int  find_index(double t, ground_truth &gt);
void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);
void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove);
