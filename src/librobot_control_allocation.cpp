#include "librobot_control_allocation/librobot_control_allocation.hpp"


Allocation::Allocation(Eigen::Vector3d r_1, Eigen::Vector3d r_2, Eigen::Vector3d r_3, Eigen::Vector3d r_4, Eigen::Vector3d eta_1, Eigen::Vector3d eta_2, Eigen::Vector3d eta_3, Eigen::Vector3d eta_4)
    {
       
        // drone relation position in the robot frame
        r_1_ = r_1;
        r_2_ = r_2;
        r_3_ = r_3;
        r_4_ = r_4;

       
        // drone relative angle in the robot frame
        eta_1_ =  eta_1;
        eta_2_ =  eta_2;
        eta_3_ =  eta_3;
        eta_4_ =  eta_4;

        /* drone rotating angle                        
        eta_1_<<-alpha/180*M_PI,0,0;
         
        eta_2_<<alpha/180*M_PI,0,0;
        
        eta_3_<<0,-alpha/180*M_PI,0;
        eta_4_<<0,alpha/180*M_PI,0;


        r_1_<<0,r_,0;
        r_2_<<0,-r_,0;
        r_3_<<-r_,0,0;
        r_4_<<r_,0,0;
        */

        T = Eigen::MatrixXd::Zero(6,16);

        Gama= Eigen::MatrixXd::Zero(16,16);

        u_ =  Eigen::MatrixXd::Zero(16,1);




    }


void Allocation::getWrechDesAndPose(const Allocation::Vector6d wrench_des_, const Allocation::Vector6d q)
    {
        // get current attitude
            qa_ = q.tail<3>();

        // get required wrench    
            wrench_des = wrench_des_;
    }


void Allocation::getDroneWrench(Eigen::VectorXd &u_1, Eigen::VectorXd &u_2,Eigen::VectorXd &u_3, Eigen::VectorXd &u_4)
{
    //1. get allocation matrix  AllocationMatrix
    //
    //std::cout<<"point 1"<<std::endl;
    getAllocationMatrix();


    // 2. solve allocation using peuso inver
    //std::cout<<"point 2"<<std::endl;
    solveAllocation();

    // 3. output the control input
    std::cout<<"point 3"<<std::endl;
    u_1 = u_.segment(0,4);

    std::cout<<"point 4"<<std::endl;
    u_2 = u_.segment(4,4);

    std::cout<<"point 5"<<std::endl;
    std::cout<<u_.transpose()<<std::endl;
    std::cout<<u_1.transpose()<<std::endl;
    std::cout<<u_2.transpose()<<std::endl;
    u_3 = u_.segment(8,4);
    
    std::cout<<u_3.transpose()<<std::endl;
    std::cout<<"point 6"<<std::endl;
    u_4 = u_.segment(12,4);
    std::cout<<u_4.transpose()<<std::endl;

    
}


void Allocation::getAllocationMatrix()
    {
        //  1. T mapping drone wrench to final wrench 
        T_1 = getTransferMatrix4LocalWrench(r_1_, eta_1_, qa_);
        T_2 = getTransferMatrix4LocalWrench(r_2_, eta_2_, qa_);
        T_3 = getTransferMatrix4LocalWrench(r_3_, eta_3_, qa_);
        T_4 = getTransferMatrix4LocalWrench(r_4_, eta_4_, qa_);
        // allocation matrix for force and torque
        // since the f = [0 0 T]
        //std::cout<<"position 1"<<std::endl;
        T<<T_1,T_2,T_3,T_4;

        //std::cout<<"position 2"<<std::endl;
        // 2. Gama mapping drone wrench to motor speed
        /*
        //std::cout<<"position 3"<<std::endl;
        Gama.block<4,4>(0,0) = gama;
         //std::cout<<"Gama"<<Gama<<std::endl;
        //std::cout<<"position 4"<<std::endl;
        Gama.block<4,4>(4,4) = gama;
        //std::cout<<"position 5"<<std::endl;
        Gama.block<4,4>(8,8) = gama;
        //std::cout<<"position 6"<<std::endl;
        Gama.block<4,4>(12,12) = gama;
        //std::cout<<"position 7"<<std::endl;
        // 3. get control allocation matrix 
        
        AllocationMatrix = T * Gama;
        */
        AllocationMatrix = T;
        
    }


Eigen::MatrixXd  Allocation::getTransferMatrix4LocalWrench(Eigen::Vector3d r_i, Eigen::Vector3d eta_i, Eigen::Vector3d qa_)
{
        // r_i: position of the drone i local frame expressed in body frame
        // eta_i: euler angle of drone i local frame respect to body frame
        // qa_: euler angle of body frame respect to world frame
        Eigen::Matrix3d Si;

        Si = getSkewMatrix(r_i);

        // rotation matrix from drona local frame to body frame
        Eigen::Matrix3d R_i2b;
        R_i2b = getRotationMatrix(eta_i);

        // rotation matrix from body frame to world frame
        Eigen::Matrix3d R_b20;
        R_b20 = getRotationMatrix(qa_);     

        // rotation matrix from local frame to world frame   
        Eigen::Matrix3d R_i20;
        R_i20 = R_b20 * R_i2b;

        // get 6*6 transfer matrix
        Eigen::MatrixXd A_(6,6);
        // A = [R_i2o  O3;  Skew(r_i)*R_i2b  R_i2b] 
        A_<<R_i20,Eigen::MatrixXd::Zero(3,3),Si*R_i2b,R_i2b;
        

        // get the 3:6 colms of the A1 matrix
        Eigen::MatrixXd A = A_.block<6,4>(0,2);

        return A;

}

void Allocation::getDroneMappingMatrix()
{
    // intilize the output
        //std::cout<<"Kt"<<Kt<<std::endl;
        //std::cout<<"Kc "<<Kc<<std::endl;
        //std::cout<<"Ld "<<Ld<<std::endl;
        gama(0,0) = Kt;
        gama(0,1) = Kt;
        gama(0,2) = Kt;
        gama(0,3) = Kt;
        // 2nd row
        gama(1,0) = -Kt * Ld;
        gama(1,1) = Kt * Ld;
        gama(1,2) = 0;
        gama(1,3) = 0;
        // 3nd row
        gama(2,0) = 0;
        gama(2,1) = 0;
        gama(2,2) = -Kt * Ld;
        gama(2,3) = Kt * Ld;
        // 4nd row
        gama(3,0) = -Kc;
        gama(3,1) = -Kc;
        gama(3,2) = Kc;
        gama(3,3) = Kc;

}


void Allocation::solveAllocation()
{
     // get the desired wrench
     // w_des =  B*u_^2;

     // using LDLT to solve this linear equation
     //std::cout<<"desired wrench "<<wrench_des.transpose()<<std::endl;
     //u_ = AllocationMatrix.ldlt().solve(wrench_des).array().sqrt();   // as the real input is the motor speed

     Eigen::MatrixXd AllocationMatrix_Inv;

     AllocationMatrix_Inv = AllocationMatrix.completeOrthogonalDecomposition().pseudoInverse();

     u_ = AllocationMatrix_Inv*wrench_des;

     if((u_.array() < 0).all())
      {
        std::cout<<"WARMING some wrench element is larger than zero"<<std::endl;
      }

}



Eigen::Matrix3d Allocation::getRotationMatrix(Eigen::Vector3d angle_v)
    {
        // get angle vector element
        double phi = angle_v(0);
        //std::cout<< "phi angle " << phi<<std::endl;
        double theta = angle_v(1);
        //std::cout<< "theta angle " << theta<<std::endl;
        double psi = angle_v(2);   
        //std::cout<< "psi angle " << psi<<std::endl;
        // get rotation matrix based on euler angles 3-2-1
        Eigen::AngleAxisd rollAngle(phi, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(theta, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(psi, Eigen::Vector3d::UnitZ());

        // transform that to rotation matrix
        //Eigen::Quaterniond Q =  rollAngle *pitchAngle*yawAngle;
        //Eigen::Matrix3d R = Q.toRotationMatrix();
        //T = R;
        Eigen::Matrix3d R;
        // Rotation order 1 2 3
        //R = rollAngle*pitchAngle*yawAngle;
        // rotation order 3 2 1 
        R = yawAngle*pitchAngle*rollAngle;
        //R = rollAngle;

        return R;
    }

Eigen::Matrix3d Allocation::getSkewMatrix(Eigen::Vector3d position_v)
    {
        // get elment of the position vector
        double x = position_v(0);
        double y = position_v(1);
        double z = position_v(2);

        // intilize output
        Eigen::Matrix3d S;
        S << 0, -z,  y,
             z, 0,  -x,
            -y, x, 0;

        return S;
    }    