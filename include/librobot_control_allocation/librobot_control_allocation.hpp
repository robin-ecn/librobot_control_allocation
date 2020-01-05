#ifndef Allocation_H
#define Allocation_H
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

class Allocation
{

public:

    typedef Eigen::Matrix<double,4,4> Matrixd44;
    typedef Eigen::Matrix<double,6,1> Vector6d;

    // constructor 
    Allocation(Eigen::Vector3d r_1, Eigen::Vector3d r_2, Eigen::Vector3d r_3, Eigen::Vector3d r_4, Eigen::Vector3d eta_1, Eigen::Vector3d eta_2, Eigen::Vector3d eta_3, Eigen::Vector3d eta_4);

    //~Allocation();
    
    // get required wrench and current pose of robot
    void getWrechDesAndPose(const Allocation::Vector6d wrench_des_, const Allocation::Vector6d q);

    // get the required drone wrench u_ 16X1
    void getDroneWrench(Eigen::VectorXd &u_1, Eigen::VectorXd &u_2,Eigen::VectorXd &u_3, Eigen::VectorXd &u_4);

    void getAllocationMatrix();

    Eigen::MatrixXd  getTransferMatrix4LocalWrench(Eigen::Vector3d r_i, Eigen::Vector3d eta_i, Eigen::Vector3d eta);

    void getDroneMappingMatrix();

    void solveAllocation();

    Eigen::Matrix3d getRotationMatrix(Eigen::Vector3d angle_v);

    Eigen::Matrix3d getSkewMatrix(Eigen::Vector3d position_v);



    /**************************Pose of robot**************************/
private:
    // attitude vector
    Eigen::Vector3d qa_;



    /**************************Controller**************************/
    // required wrench
    Allocation::Vector6d wrench_des;    //6X1

    // control input
    Eigen::VectorXd u_;                  //16X1

    // final control allocation matrix
    Eigen::MatrixXd AllocationMatrix;   //6X16

    // matrix mapping motor speed 2 to local wrench
    Allocation::Matrixd44 gama; //4X4

    // matrix mapping 16 motor speed 2 to local wrench
    Eigen::MatrixXd Gama;       //16X16

    // matirx mapping local wrench of drone to world frame for forces and body frame for torques
    Eigen::MatrixXd T_1;        //6X4
    Eigen::MatrixXd T_2;        //6X4
    Eigen::MatrixXd T_3;        //6X4
    Eigen::MatrixXd T_4;        //6X4

    Eigen::MatrixXd T;          //6X16


    /**************************Deisn parameters**************************/
    // position of drone in the body frame
    //double r_=1; // distance between the drone and the CoM of structure
    Eigen::Vector3d r_1_;
    Eigen::Vector3d r_2_;
    Eigen::Vector3d r_3_;
    Eigen::Vector3d r_4_;

    // relative angle of drone in the body frame
    //double alpha = 30; // degrees: drone rotating angle in the body frame of robot    
    Eigen::Vector3d eta_1_;
    Eigen::Vector3d eta_2_;
    Eigen::Vector3d eta_3_;
    Eigen::Vector3d eta_4_;

    /**************************Aerodynamics parameters**************************/
    // aerodyanmic coffients
    double Kt=3.65*pow(10,-6);
    double Kc=5.40*pow(10,-9);    
    double Ld=0.17;









};







#endif