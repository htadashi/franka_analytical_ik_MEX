/* franka_ik_EE_CC
 * IK = franka_ik_EE_CC(O_T_EE_array, q7, q_actual_array);
 * Different from franka_IK_EE(), this function only returns the solution belonging to the same "case" as the actual joint configuration q_actual_array. 
 * The purpose is to prevent unexpected switching between different solution cases (e.g. elbow-up and -down) during continuous motion planning.
 * 
 * The input O_T_EE_array is the desired Cartesian pose, represented as a transformation matrix relative to the robot base frame, 
 * stored in an array in column-major format. 
 * q7 is the last joint angle as the redundant parameter, specified by the user in radian. 
 * q_actual_array is the actual joint configuration. At singularity, the actual angle of the first joint will be assigned to the final output.
 * 
 * Original C++ code by Yanhao He, February 2020 (https://github.com/ffall007/franka_analytical_ik)
 * Wrapper by Hugo Tadashi and Martin Schonger, November 2023
*/

#define _USE_MATH_DEFINES
#include <array>
#include <cmath>
#include "Eigen/Dense"

#include "mex.hpp"
#include "mexAdapter.hpp"

class MexFunction : public matlab::mex::Function {
public:
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        
        matlab::data::TypedArray<double> O_T_EE_array_MAT   = inputs[0];
        matlab::data::TypedArray<double> q_actual_array_MAT = inputs[2];

        std::array<double, 16> O_T_EE_array;
        double q7;
        std::array<double, 7> q_actual_array;

        for(int i = 0; i < 16; i++){
            O_T_EE_array[i] = O_T_EE_array_MAT[i];
        }
        q7 = inputs[1][0];
        for(int i = 0; i < 7; i++){
            q_actual_array[i] = q_actual_array_MAT[i];
        }
        
        std::array<double, 7> IK = franka_IK_EE_CC(O_T_EE_array, q7, q_actual_array);

        matlab::data::ArrayFactory factory;        
        matlab::data::TypedArray<double> result = factory.createArray<double>({1,7});
        for(int i = 0; i < 7; i++){            
            result[i] = IK[i];
        }
        outputs[0] = result;
    }
private:
    // "Case-Consistent" inverse kinematics w.r.t. End Effector Frame (using Franka Hand data)
    std::array<double, 7> franka_IK_EE_CC ( std::array<double, 16> O_T_EE_array,
                                            double q7,
                                            std::array<double, 7> q_actual_array )
    {
        const std::array<double, 7> q_NAN = {{NAN, NAN, NAN, NAN, NAN, NAN, NAN}};
        
        std::array<double, 7> q;
        
        Eigen::Map< Eigen::Matrix<double, 4, 4> > O_T_EE(O_T_EE_array.data());
        
        // constants
        const double d1 = 0.3330;
        const double d3 = 0.3160;
        const double d5 = 0.3840;
        const double d7e = 0.2104;
        const double a4 = 0.0825;
        const double a7 = 0.0880;

        const double LL24 = 0.10666225; // a4^2 + d3^2
        const double LL46 = 0.15426225; // a4^2 + d5^2
        const double L24 = 0.326591870689; // sqrt(LL24)
        const double L46 = 0.392762332715; // sqrt(LL46)
        
        const double thetaH46 = 1.35916951803; // atan(d5/a4);
        const double theta342 = 1.31542071191; // atan(d3/a4);
        const double theta46H = 0.211626808766; // acot(d5/a4);
        
        const std::array<double, 7> q_min = {{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
        const std::array<double, 7> q_max = {{ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973}};
        
        // return NAN if input q7 is out of range
        if (q7 <= q_min[6] || q7 >= q_max[6])
            return q_NAN;
        else
            q[6] = q7;

        // FK for getting current case id
        double c1_a = std::cos(q_actual_array[0]); double s1_a = std::sin(q_actual_array[0]);
        double c2_a = std::cos(q_actual_array[1]); double s2_a = std::sin(q_actual_array[1]);
        double c3_a = std::cos(q_actual_array[2]); double s3_a = std::sin(q_actual_array[2]);
        double c4_a = std::cos(q_actual_array[3]); double s4_a = std::sin(q_actual_array[3]);
        double c5_a = std::cos(q_actual_array[4]); double s5_a = std::sin(q_actual_array[4]);
        double c6_a = std::cos(q_actual_array[5]); double s6_a = std::sin(q_actual_array[5]);

        std::array< Eigen::Matrix<double, 4, 4>, 7> As_a;
        As_a[0] <<   c1_a, -s1_a,  0.0,  0.0,    // O1
                    s1_a,  c1_a,  0.0,  0.0,
                    0.0,   0.0,  1.0,   d1,
                    0.0,   0.0,  0.0,  1.0;
        As_a[1] <<   c2_a, -s2_a,  0.0,  0.0,    // O2
                    0.0,   0.0,  1.0,  0.0,
                    -s2_a, -c2_a,  0.0,  0.0,
                    0.0,   0.0,  0.0,  1.0;
        As_a[2] <<   c3_a, -s3_a,  0.0,  0.0,    // O3
                    0.0,   0.0, -1.0,  -d3,
                    s3_a,  c3_a,  0.0,  0.0,
                    0.0,   0.0,  0.0,  1.0;
        As_a[3] <<   c4_a, -s4_a,  0.0,   a4,    // O4
                    0.0,   0.0, -1.0,  0.0,
                    s4_a,  c4_a,  0.0,  0.0,
                    0.0,   0.0,  0.0,  1.0;
        As_a[4] <<    1.0,   0.0,  0.0,  -a4,    // H
                    0.0,   1.0,  0.0,  0.0,
                    0.0,   0.0,  1.0,  0.0,
                    0.0,   0.0,  0.0,  1.0;
        As_a[5] <<   c5_a, -s5_a,  0.0,  0.0,    // O5
                    0.0,   0.0,  1.0,   d5,
                    -s5_a, -c5_a,  0.0,  0.0,
                    0.0,   0.0,  0.0,  1.0;
        As_a[6] <<   c6_a, -s6_a,  0.0,  0.0,    // O6
                    0.0,   0.0, -1.0,  0.0,
                    s6_a,  c6_a,  0.0,  0.0,
                    0.0,   0.0,  0.0,  1.0;
        std::array< Eigen::Matrix<double, 4, 4>, 7> Ts_a;
        Ts_a[0] = As_a[0];
        for (unsigned int j = 1; j < 7; j++)
            Ts_a[j] = Ts_a[j - 1]*As_a[j];

        // identify q6 case
        Eigen::Vector3d V62_a = Ts_a[1].block<3, 1>(0, 3) - Ts_a[6].block<3, 1>(0, 3);
        Eigen::Vector3d V6H_a = Ts_a[4].block<3, 1>(0, 3) - Ts_a[6].block<3, 1>(0, 3);
        Eigen::Vector3d Z6_a = Ts_a[6].block<3, 1>(0, 2);
        bool is_case6_0 = ((V6H_a.cross(V62_a)).transpose()*Z6_a <= 0);

        // identify q1 case
        bool is_case1_1 = (q_actual_array[1] < 0);
        
        // IK: compute p_6
        Eigen::Matrix3d R_EE = O_T_EE.topLeftCorner<3, 3>();
        Eigen::Vector3d z_EE = O_T_EE.block<3, 1>(0, 2);
        Eigen::Vector3d p_EE = O_T_EE.block<3, 1>(0, 3);
        Eigen::Vector3d p_7 = p_EE - d7e*z_EE;
        
        Eigen::Vector3d x_EE_6;
        x_EE_6 << std::cos(q7 - M_PI_4), -std::sin(q7 - M_PI_4), 0.0;
        Eigen::Vector3d x_6 = R_EE*x_EE_6;
        x_6 /= x_6.norm(); // visibly increases accuracy
        Eigen::Vector3d p_6 = p_7 - a7*x_6;
        
        // IK: compute q4
        Eigen::Vector3d p_2;
        p_2 << 0.0, 0.0, d1;
        Eigen::Vector3d V26 = p_6 - p_2;
        
        double LL26 = V26[0]*V26[0] + V26[1]*V26[1] + V26[2]*V26[2];
        double L26 = std::sqrt(LL26);
        
        if (L24 + L46 < L26 || L24 + L26 < L46 || L26 + L46 < L24)
            return q_NAN;
        
        double theta246 = std::acos((LL24 + LL46 - LL26)/2.0/L24/L46);
        q[3] = theta246 + thetaH46 + theta342 - 2.0*M_PI;
        if (q[3] <= q_min[3] || q[3] >= q_max[3])
            return q_NAN;
        
        // IK: compute q6
        double theta462 = std::acos((LL26 + LL46 - LL24)/2.0/L26/L46);
        double theta26H = theta46H + theta462;
        double D26 = -L26*std::cos(theta26H);
        
        Eigen::Vector3d Z_6 = z_EE.cross(x_6);
        Eigen::Vector3d Y_6 = Z_6.cross(x_6);
        Eigen::Matrix3d R_6;
        R_6.col(0) = x_6;
        R_6.col(1) = Y_6/Y_6.norm();
        R_6.col(2) = Z_6/Z_6.norm();
        Eigen::Vector3d V_6_62 = R_6.transpose()*(-V26);

        double Phi6 = std::atan2(V_6_62[1], V_6_62[0]);
        double Theta6 = std::asin(D26/std::sqrt(V_6_62[0]*V_6_62[0] + V_6_62[1]*V_6_62[1]));
        
        if (is_case6_0)
            q[5] = M_PI - Theta6 - Phi6;
        else
            q[5] = Theta6 - Phi6;
        
        if (q[5] <= q_min[5])
            q[5] += 2.0*M_PI;
        else if (q[5] >= q_max[5])
            q[5] -= 2.0*M_PI;
        
        if (q[5] <= q_min[5] || q[5] >= q_max[5])
            return q_NAN;

        // IK: compute q1 & q2
        double thetaP26 = 3.0*M_PI_2 - theta462 - theta246 - theta342;
        double thetaP = M_PI - thetaP26 - theta26H;
        double LP6 = L26*sin(thetaP26)/std::sin(thetaP);
        
        Eigen::Vector3d z_6_5;
        z_6_5 << std::sin(q[5]), std::cos(q[5]), 0.0;
        Eigen::Vector3d z_5 = R_6*z_6_5;
        Eigen::Vector3d V2P = p_6 - LP6*z_5 - p_2;
        
        double L2P = V2P.norm();
        
        if (std::fabs(V2P[2]/L2P) > 0.999)
        {
            q[0] = q_actual_array[0];
            q[1] = 0.0;
        }
        else
        {
            q[0] = std::atan2(V2P[1], V2P[0]);
            q[1] = std::acos(V2P[2]/L2P);
            if (is_case1_1)
            {
                if (q[0] < 0.0)
                    q[0] += M_PI;
                else
                    q[0] -= M_PI;
                q[1] = -q[1];
            }
        }
        
        if ( q[0] <= q_min[0] || q[0] >= q_max[0]
        || q[1] <= q_min[1] || q[1] >= q_max[1] )
            return q_NAN;
        
        // IK: compute q3
        Eigen::Vector3d z_3 = V2P/V2P.norm();
        Eigen::Vector3d Y_3 = -V26.cross(V2P);
        Eigen::Vector3d y_3 = Y_3/Y_3.norm();
        Eigen::Vector3d x_3 = y_3.cross(z_3);
        Eigen::Matrix3d R_1;
        double c1 = std::cos(q[0]);
        double s1 = std::sin(q[0]);
        R_1 <<   c1,  -s1,  0.0,
                s1,   c1,  0.0,
                0.0,  0.0,  1.0;
        Eigen::Matrix3d R_1_2;
        double c2 = std::cos(q[1]);
        double s2 = std::sin(q[1]);
        R_1_2 <<   c2,  -s2, 0.0,
                0.0,  0.0, 1.0,
                -s2,  -c2, 0.0;
        Eigen::Matrix3d R_2 = R_1*R_1_2;
        Eigen::Vector3d x_2_3 = R_2.transpose()*x_3;
        q[2] = std::atan2(x_2_3[2], x_2_3[0]);
        
        if (q[2] <= q_min[2] || q[2] >= q_max[2])
            return q_NAN;
        
        // IK: compute q5
        Eigen::Vector3d VH4 = p_2 + d3*z_3 + a4*x_3 - p_6 + d5*z_5;
        Eigen::Matrix3d R_5_6;
        double c6 = std::cos(q[5]);
        double s6 = std::sin(q[5]);
        R_5_6 <<   c6,  -s6,  0.0,
                0.0,  0.0, -1.0,
                s6,   c6,  0.0;
        Eigen::Matrix3d R_5 = R_6*R_5_6.transpose();
        Eigen::Vector3d V_5_H4 = R_5.transpose()*VH4;
        
        q[4] = -std::atan2(V_5_H4[1], V_5_H4[0]);
        if (q[4] <= q_min[4] || q[4] >= q_max[4])
            return q_NAN;
        
        return q;
    }
};