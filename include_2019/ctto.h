/* Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/)
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 *
 * AUTHOR: Bruce JK Huang (bjhuang@umich.edu)
 * WEBSITE: https://www.BrucebotStudio.com/
 */
#pragma once
#ifndef CTTO_H
#define CTTO_H

#include <iostream>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR> 
#include <random>
#include "gp/gp.h"
#include "Dstar.h"

using namespace std;

namespace bipedlab{
struct Node {
    public:
        Node() {}

        Node(geometry_msgs::Point p) : point(p) {}

        template <class T,class U >
            Node(T t, U u) {point.x=t; point.y=u;};

        template <class T,class U >
            Node(T t, U u, int id, int pId) {
                this->point.x=t; 
                this->point.y=u; 
                this->id=id;
                this->parentId=pId;
            };

        int id;
        geometry_msgs::Point point;
        std::vector<Node> children;
        int parentId;
};

typedef struct PotentialCostMap{
    Eigen::MatrixXd costMap;
    Eigen::MatrixXd costMapInverse;
    Eigen::MatrixXd signedCostMap;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} PotentialCostMap_t;

typedef struct Obstacle{
    Obstacle(geometry_msgs::Point p, int id): point(p), id(id) {}
    Obstacle(int id): id(id) {}
    geometry_msgs::Point point;
    int id;
} Obstacle_t;

typedef struct BoundaryPoint{
    /*
     *   p4          p3
     *    o----------o
     *    |          |
     *    |          |
     *    |          |
     *    |          |
     *    o----------o
     *   p1          p2
     *   
     */
    BoundaryPoint(){
        p1.x = 1000; 
        p1.y = 1000; 
        p1.z = 0.5; 

        p2.x = -1000; 
        p2.y = 1000; 
        p2.z = 0.5; 

        p3.x = -1000; 
        p3.y = -1000; 
        p3.z = 0.5; 

        p4.x = 1000; 
        p4.y = -1000; 
        p4.z = 0.5; 
    }
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    geometry_msgs::Point p3;
    geometry_msgs::Point p4;
}BoundaryPoint_t;

typedef struct Constraint{
    double x_init;
    double y_init;
    double x_goal;
    double y_goal;

    double vx_init;
    double vy_init;
    double vx_goal;
    double vy_goal;

    double theta_init;
    double theta_goal;
} Constraint_t; 

typedef struct ConversionMatrix{
    Eigen::MatrixXd M;
    Eigen::MatrixXd A;
    Eigen::MatrixXd AH;
    Eigen::VectorXd dF;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::VectorXd dp_star;
    Eigen::VectorXd p;
} ConversionMatrix_t;

typedef struct PolynomialTraj{
    //PolynomialTraj(float x_init, float y_init, float x_goal, float y_goal, 
    //               float vx_init, float vy_init, float vx_goal, float vy_goal, 
    //               float theta_init, float theta_goal): x_init(x_init), y_init(y_init) {}
                   // x_goal(x_goal), y_goal(y_goal),
                   // vx_init(x_init), vy_init(y_init),
                   // vx_goal(x_goal), vy_goal(y_goal),
                   // theta_init(theta_init),
                   // theta_goal(theta_goal) {}
    Constraint_t constraints;
    ConversionMatrix_t conversion;
    Eigen::VectorXd px;
    Eigen::VectorXd py;
    Eigen::VectorXd ptheta;
}PolynomialTraj_t;

typedef struct State{
    geometry_msgs::Point pRobot;
    geometry_msgs::Point vRobot;
    double yaw;
} State_t;


class CTTO {
public:
    CTTO(Node init, Node goal, int gridToCostMapRatio, float epsilon, int order,
         float sigma, int x_max, int x_min, int y_max, int y_min) : 
            _init(init), _goal(goal), _gridToCostMapRatio(gridToCostMapRatio), _polynomialOrder(order),
            _epsilon(epsilon), _sigma(sigma), x_max(x_max), x_min(x_min), 
            y_max(y_max), y_min(y_min) {

        initPolynomial();
        // cout << "M: \n" << _polynomial.conversion.M << endl;
        // cout << "------- A0------- " << endl; derivative(1,7,0);
        // cout << "------- A1------- " << endl; derivative(1,7,1);
        // cout << "------- A2------- " << endl; derivative(1,7,2);
        // cout << "------- A3------- " << endl; derivative(1,7,3);
        // cout << "------- A4------- " << endl; derivative(1,7,4);
        // cout << "------- A5------- " << endl; derivative(1,7,5);
        // cout << "------- A0------- " << endl; derivative(3,7,0);
        // cout << "------- A1------- " << endl; derivative(3,7,1);
        // cout << "------- A2------- " << endl; derivative(3,7,2);
        // cout << "------- A3------- " << endl; derivative(3,7,3);
        // cout << "------- A4------- " << endl; derivative(3,7,4);
        // cout << "------- A5------- " << endl; derivative(3,7,5);
        

        _robot = init.point;
	    _robotState.pRobot = _robot;	
        geometry_msgs::Point v;
        v.x = 0.5;
        v.y = 0.5;
        v.z = 0;
	    _robotState.vRobot = v;	
	    _robotState.yaw = 45*M_PI/180;	
        // feasiblePath(1, 3, _robot, 4, 5, 0.5,0.5,0.5,0.5, 0.785, 0.785);
        // feasiblePathNew(0, 3, _robot, 12, 17, 0.5, 0.5, 1, 1, 0.785, 0.785);
        // feasiblePathSeperate(1, 3, _robot, 4, 5, 0.5,0.5,0.5,0.5, 6, 7);
        // exit(0);

        _potentialCostMap.costMap = 1*Eigen::MatrixXd::Ones(_gridToCostMapRatio*y_max, 
                                                            _gridToCostMapRatio*x_max);
        _potentialCostMap.costMapInverse = 0*Eigen::MatrixXd::Ones(_gridToCostMapRatio*y_max, 
                                                            _gridToCostMapRatio*x_max);

        _potentialCostMap.signedCostMap;

        nodesList.reserve(1000);
        nodesList.push_back(init);
    }


    void initPolynomial(){
        _polynomial.constraints  = {_init.point.x, _init.point.y, 
                                    _goal.point.x, _goal.point.y, 
                                    0, 0, 0, 0, 0, 0};

        // contruct M
        _polynomial.conversion.M = Eigen::MatrixXd::Zero(36, 36);

        // d11 -> d11
        _polynomial.conversion.M.block(0,0,4,4) = Eigen::MatrixXd::Identity(4,4);

        // d21 -> d21
        _polynomial.conversion.M.block(4,10,8,8) = Eigen::MatrixXd::Identity(8,8);

        // d31 -> d21
        _polynomial.conversion.M.block(12,4,4,4) = Eigen::MatrixXd::Identity(4,4);

        // d12 -> d22
        _polynomial.conversion.M.block(16,18,8,8) = Eigen::MatrixXd::Identity(8,8);

        // d22 -> d31
        _polynomial.conversion.M.block(24,8,2,2) = Eigen::MatrixXd::Identity(2,2);

        // d32 -> d32
        _polynomial.conversion.M.block(26,26,10,10) = Eigen::MatrixXd::Identity(10,10);

        // initialize A
        // _polynomial.conversion.A = Eigen::MatrixXd::Zero(36, 3*(_polynomialOrder+1));
        _polynomial.conversion.A = Eigen::MatrixXd::Zero(36, 3*(_polynomialOrder+1));

        // initialize AH
        _polynomial.conversion.AH = Eigen::MatrixXd::Zero(3*(_polynomialOrder+1), 36);

        // initialize dF
        _polynomial.conversion.dF = Eigen::VectorXd::Zero(10);

        // initialize Q
        _polynomial.conversion.Q = Eigen::MatrixXd::Zero(3*(_polynomialOrder+1), 3*(_polynomialOrder+1));
    }

	template <class MatT>
	Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
	pseudoInverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
	{
		typedef typename MatT::Scalar Scalar;
		auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
		const auto &singularValues = svd.singularValues();
		Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
		singularValuesInv.setZero();
		for (unsigned int i = 0; i < singularValues.size(); ++i) {
			if (singularValues(i) > tolerance)
			{
				singularValuesInv(i, i) = Scalar{1} / singularValues(i);
			}
			else
			{
				singularValuesInv(i, i) = Scalar{0};
			}
		}
		return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
	}



    unsigned int factorial(unsigned int n) {
        if (n == 0)
            return 1;
        return n * factorial(n - 1);
    }


    Eigen::VectorXd
    derivative(double t, int order, int derivativeOrder){
        Eigen::VectorXd v = Eigen::VectorXd::Zero(order);
        for (int i=derivativeOrder; i<order; ++i){
           v[i] = std::pow(t, i-derivativeOrder);
            for (int j=0; j<derivativeOrder; ++j){
                v[i] = v[i]*(i-j);
            }
        }
        // for (int i=0; i<order; ++i){
        //    cout << "v[" << i << "]: " << v[i] << endl;
        // }
        return v;
    }

    Eigen::MatrixXd 
    constructPolynomailA(double t_0, double t_m){
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12, _polynomialOrder+1);
        A.row(0) = derivative(t_0, _polynomialOrder+1, 0);
        A.row(1) = derivative(t_m, _polynomialOrder+1, 0);

        A.row(2) = derivative(t_0, _polynomialOrder+1, 1);
        A.row(3) = derivative(t_m, _polynomialOrder+1, 1);

        A.row(4) = derivative(t_0, _polynomialOrder+1, 2);
        A.row(5) = derivative(t_m, _polynomialOrder+1, 2);

        A.row(6) = derivative(t_0, _polynomialOrder+1, 3);
        A.row(7) = derivative(t_m, _polynomialOrder+1, 3);

        A.row(8) = derivative(t_0, _polynomialOrder+1, 4);
        A.row(9) = derivative(t_m, _polynomialOrder+1, 4);

        A.row(10) = derivative(t_0, _polynomialOrder+1, 5);
        A.row(11) = derivative(t_m, _polynomialOrder+1, 5);

        return A;
    }

    Eigen::VectorXd 
    constructPolynomailb(double t_0, double t_m){
        double duration = t_m - t_0;
        Eigen::VectorXd b = Eigen::VectorXd::Zero(_polynomialOrder+1);
        b[3] = 6;
        b[4] = 24*duration;
        b[5] = 60*std::pow(duration, 2);
        b[6] = 120*std::pow(duration, 3);

        return b;
    }

    double constructPolynomailQ_3(double t_0, double t_m){
        double duration = t_m - t_0;

        return 36*duration + 192*(std::pow(t_m, 3) - std::pow(t_0, 3)) + 
               720*(std::pow(t_m, 5) - std::pow(t_0, 5)) + 
               2057*(std::pow(t_m, 7) - std::pow(t_0, 7));
    }

    double constructPolynomailQ_0(double t_0, double t_m){
        double duration = t_m - t_0;

        return 1*duration + 
               (1/3)*(std::pow(t_m, 3) - std::pow(t_0, 3)) + 
               (1/5)*(std::pow(t_m, 5) - std::pow(t_0, 5)) + 
               (1/7)*(std::pow(t_m, 7) - std::pow(t_0, 7)) + 
               (1/9)*(std::pow(t_m, 9) - std::pow(t_0, 9)) + 
               (1/11)*(std::pow(t_m, 11) - std::pow(t_0, 11)) + 
               (1/13)*(std::pow(t_m, 13) - std::pow(t_0, 13));
    }

    Eigen::MatrixXd 
    constructPolynomailQ(double t0, double tm){
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(7, 7);
        Q(3,3) = 36 * (tm - t0);
        Q(3,4) = 72 * (std::pow(tm, 2) - std::pow(t0, 2));
        Q(3,5) = 120 * (std::pow(tm, 3) - std::pow(t0, 3));
        Q(3,6) = 180 * (std::pow(tm, 4) - std::pow(t0, 4));

        Q(4,3) = 72 * (std::pow(tm, 2) - std::pow(t0, 2));
        Q(4,4) = 192 * (std::pow(tm, 3) - std::pow(t0, 3));
        Q(4,5) = 360 * (std::pow(tm, 4) - std::pow(t0, 4));
        Q(4,6) = 576 * (std::pow(tm, 5) - std::pow(t0, 5));

        Q(5,3) = 120 * (std::pow(tm, 3) - std::pow(t0, 3));
        Q(5,4) = 360 * (std::pow(tm, 4) - std::pow(t0, 4));
        Q(5,5) = 720 * (std::pow(tm, 5) - std::pow(t0, 5));
        Q(5,6) = 1200 * (std::pow(tm, 6) - std::pow(t0, 6));

        Q(6,3) = 180 * (std::pow(tm, 4) - std::pow(t0, 4));
        Q(6,4) = 576 * (std::pow(tm, 5) - std::pow(t0, 5));
        Q(6,5) = 1200 * (std::pow(tm, 6) - std::pow(t0, 6));
        Q(6,6) = 2057 * (std::pow(tm, 7) - std::pow(t0, 7));

        return Q;

    }

    void feasiblePathSeperate(double t_0, double t_m,
                              const geometry_msgs::Point &robot, 
                              double x_goal, double y_goal, 
                              double vx_init, double vy_init, 
                              double vx_goal, double vy_goal, 
                              double theta_init, double theta_goal){
        Eigen::MatrixXd A = constructPolynomailA(t_0, t_m);
        Eigen::MatrixXd AH;
        // Eigen::MatrixXd AH = A.completeOrthogonalDecomposition().pseudoInverse();
        // Eigen::MatrixXd Q = constructPolynomailQ(t_0, t_m);
        double Q = constructPolynomailQ_3(t_0, t_m);
        Eigen::MatrixXd R = AH.transpose()*Q*(AH);

        Eigen::VectorXd dF_x(4);
        dF_x[0] = robot.x;
        dF_x[1] = x_goal;
        dF_x[2] = vx_init;
        dF_x[3] = vx_goal;
        Eigen::MatrixXd R_FP = R.topRightCorner(4,8);
        Eigen::MatrixXd R_PP = R.bottomRightCorner(8,8);
        _polynomial.px = -R_PP.inverse()*R_FP.transpose()*dF_x;

        Eigen::VectorXd dF_y(4);
        dF_y[0] = robot.y;
        dF_y[1] = y_goal;
        dF_y[2] = vy_init;
        dF_y[3] = vy_goal;
        _polynomial.py = -R_PP.inverse()*R_FP.transpose()*dF_y;

        Eigen::VectorXd dF_theta(2);
        _polynomial.conversion.dF[0] = theta_init;
        _polynomial.conversion.dF[1] = theta_goal;
        Eigen::MatrixXd Rtheta_FP = R.topRightCorner(2,10);
        Eigen::MatrixXd Rtheta_PP = R.bottomRightCorner(10,10);
        _polynomial.ptheta = -Rtheta_PP.inverse()*Rtheta_FP.transpose()*dF_theta;

        _robotState.vRobot.x = (x_goal - _robot.x)/ (t_m-t_0);
        _robotState.vRobot.y = (y_goal - _robot.y)/ (t_m-t_0);
        _robotState.yaw = theta_goal;
    }

    void feasiblePathNew(double t_0, double t_m,
                         const geometry_msgs::Point &robot, 
                         double x_goal, double y_goal, 
                         double vx_init, double vy_init, 
                         double vx_goal, double vy_goal, 
                         double theta_init, double theta_goal){
		cout << "t_0:" << t_0 << endl;
		cout << "t_m:" << t_m << endl;
		cout << "robot:\n" << robot << endl;
		cout << "x_goal:" << x_goal << endl;
		cout << "y_goal:" << y_goal << endl;
		cout << "vx_int:" << vx_init << endl;
		cout << "vy_int:" << vy_init << endl;
		cout << "vx_goal:" << vx_goal << endl;
		cout << "vy_goal:" << vy_goal << endl;
		cout << "theta_init:" << theta_init << endl;
		cout << "theta_goal:" << theta_goal << endl;
        cout << "===========================" << endl;


        Eigen::MatrixXd A0 = constructPolynomailA(0, 0);
        Eigen::MatrixXd A1 = constructPolynomailA(0, 1);
        Eigen::MatrixXd A2 = constructPolynomailA(0, 2);
        cout << "A0: \n" << A0 << endl;
        cout << "A1: \n" << A1 << endl;
        cout << "A2: \n" << A2 << endl;
        Eigen::MatrixXd A = constructPolynomailA(t_0, t_m);
        Eigen::MatrixXd AH;
        // Eigen::MatrixXd AH = A.completeOrthogonalDecomposition().pseudoInverse();
        // Eigen::MatrixXd Q = constructPolynomailQ(t_0, t_m);
        double Q0 = constructPolynomailQ_0(t_0, t_m);
        double Q3 = constructPolynomailQ_3(t_0, t_m);
        // cout << "Q: " << Q0 << ", " << Q3 << endl;
        double Q = 1*Q0 + 1*Q3;
        Eigen::MatrixXd R = AH.transpose()*Q*(AH);

        Eigen::VectorXd dF_x(4);
        dF_x[0] = robot.x;
        dF_x[1] = x_goal;
        dF_x[2] = vx_init;
        dF_x[3] = vx_goal;
        Eigen::MatrixXd R_FP = R.topRightCorner(4,8);
        Eigen::MatrixXd R_PP = R.bottomRightCorner(8,8);
        Eigen::VectorXd dP_x = -(R_PP.inverse()).transpose()*R_FP.transpose()*dF_x;
        Eigen::VectorXd px(12);
        px << dF_x, dP_x;
        cout << "px:\n" << px << endl;
        _polynomial.px = AH*px;

        Eigen::VectorXd dF_y(4);
        dF_y[0] = robot.y;
        dF_y[1] = y_goal;
        dF_y[2] = vy_init;
        dF_y[3] = vy_goal;
        Eigen::VectorXd dP_y= -(R_PP.inverse()).transpose()*R_FP.transpose()*dF_y;
        Eigen::VectorXd py(12);
        py << dF_y, dP_y;
        cout << "py:\n" << py << endl;
        _polynomial.py = AH*py;


        Eigen::VectorXd dF_theta(2);
        dF_theta[0] = theta_init;
        dF_theta[1] = theta_goal;
        Eigen::MatrixXd Rtheta_FP = R.topRightCorner(2,10);
        Eigen::MatrixXd Rtheta_PP = R.bottomRightCorner(10,10);
        Eigen::VectorXd dP_theta = -(Rtheta_PP.inverse()).transpose()*Rtheta_FP.transpose()*dF_theta;
        Eigen::VectorXd ptheta(12);
        ptheta << dF_theta, dP_theta;
        cout << "ptheta:\n" << ptheta << endl;
        _polynomial.ptheta = AH*ptheta;

        double xInit = 0;
        double yInit = 0;
        double thetaInit = 0;
        double xGoal = 0;
        double yGoal = 0;
        double thetaGoal = 0;
        for (int i=0; i<7; ++i){
            xInit = xInit + _polynomial.px[i]*std::pow(t_0, i);
            yInit = yInit + _polynomial.py[i]*std::pow(t_0, i);
            thetaInit = thetaInit + _polynomial.ptheta[i]*std::pow(t_0, i);
        }

        for (int i=0; i<7; ++i){
            xGoal = xGoal + _polynomial.px[i]*std::pow(t_m, i);
            yGoal = yGoal + _polynomial.py[i]*std::pow(t_m, i);
            thetaGoal = thetaGoal + _polynomial.ptheta[i]*std::pow(t_m, i);
        }

		cout << "init: " << xInit << ", " << yInit << ", " << thetaInit << endl;
		cout << "goal: " << xGoal << ", " << yGoal << ", " << thetaGoal << endl;


        _robotState.vRobot.x = (x_goal - _robot.x)/ (t_m-t_0);
        _robotState.vRobot.y = (y_goal - _robot.y)/ (t_m-t_0);
        _robotState.yaw = theta_goal;
    }

    void feasiblePath(double t_0, double t_m,
                      const geometry_msgs::Point &robot, 
                      double x_goal, double y_goal, 
                      double vx_init, double vy_init, 
                      double vx_goal, double vy_goal, 
                      double theta_init, double theta_goal){
		// cout << "t_0:" << t_0 << endl;
		// cout << "t_m:" << t_m << endl;
		// cout << "robot:\n" << robot << endl;
		// cout << "x_goal:" << x_goal << endl;
		// cout << "y_goal:" << y_goal << endl;
		// cout << "vx_int:" << vx_init << endl;
		// cout << "vy_int:" << vy_init << endl;
		// cout << "vx_goal:" << vx_goal << endl;
		// cout << "vy_goal:" << vy_goal << endl;
		// cout << "theta_init:" << theta_init << endl;
		// cout << "theta_goal:" << theta_goal << endl;
        // cout << "===========================" << endl;

        // 
        Eigen::MatrixXd Ax = constructPolynomailA(t_0, t_m);
        Eigen::VectorXd bx = constructPolynomailb(t_0, t_m);
        // Eigen::MatrixXd Qx = bx.transpose()*bx;
        Eigen::MatrixXd Qx = bx*bx.transpose();
        // double Q0 = constructPolynomailQ_0(t_0, t_m);
        // double Q3 = constructPolynomailQ_3(t_0, t_m);
        // cout << "Q: " << Q0 << ", " << Q3 << endl;
        // double Q = 1*Q0 + 1*Q3;
        // cout << "Qx: \n" << Qx << endl;
        // cout << "Qx: " << Qx.rows() << ", " << Qx.cols() << endl;

        Eigen::MatrixXd Ay = constructPolynomailA(t_0, t_m);
        Eigen::VectorXd by = constructPolynomailb(t_0, t_m);
        // Eigen::MatrixXd Qy = by.transpose()*by;
        Eigen::MatrixXd Qy = by*by.transpose();

        Eigen::MatrixXd Atheta = constructPolynomailA(t_0, t_m);
        Eigen::VectorXd btheta = constructPolynomailb(t_0, t_m);
        // Eigen::MatrixXd Qtheta = btheta.transpose()*btheta;
        Eigen::MatrixXd Qtheta = btheta*btheta.transpose();

        // A 
        // _polynomial.conversion.A.topLeftCorner(7, 12) = Ax;
        // _polynomial.conversion.A.bottomRightCorner(7, 12) = Atheta;
        // _polynomial.conversion.A.block(7, 12, 7, 12) = Ay;

        // AH
        _polynomial.conversion.AH.topLeftCorner(7, 12) = pseudoInverse(Ax);
        _polynomial.conversion.AH.bottomRightCorner(7, 12) = pseudoInverse(Atheta);
        _polynomial.conversion.AH.block(7, 12, 7, 12) = pseudoInverse(Ay);

        // Q
        _polynomial.conversion.Q.topLeftCorner(7, 7) = Qx;
        _polynomial.conversion.Q.block(7, 7, 7, 7) = Qy;
        _polynomial.conversion.Q.bottomRightCorner(7, 7) = Qtheta;
        _polynomial.conversion.R = _polynomial.conversion.M.transpose()*
                                   ((_polynomial.conversion.AH).transpose())*
                                   // _polynomial.conversion.Q*
                                   _polynomial.conversion.Q*
                                   (_polynomial.conversion.AH)*
                                   _polynomial.conversion.M;



        // 
        _polynomial.conversion.dF[0] = robot.x;
        _polynomial.conversion.dF[1] = x_goal;
        _polynomial.conversion.dF[2] = vx_init;
        _polynomial.conversion.dF[3] = vx_goal;

        _polynomial.conversion.dF[4] = robot.y;
        _polynomial.conversion.dF[5] = y_goal;
        _polynomial.conversion.dF[6] = vy_init;
        _polynomial.conversion.dF[7] = vy_goal;

        _polynomial.conversion.dF[8] = theta_init;
        _polynomial.conversion.dF[9] = theta_goal;

        _polynomial.conversion.dp_star = -_polynomial.conversion.R.bottomRightCorner(26,26).inverse()*
                                          _polynomial.conversion.R.topRightCorner(10,26).transpose()*
                                          _polynomial.conversion.dF;

        Eigen::VectorXd d(36);
        d << _polynomial.conversion.dF, _polynomial.conversion.dp_star;
        // cout << "AxH:\n" << pseudoInverse(Ax) << endl;
        // cout << "AxH:\n" << pseudoInverse(Ax).rows() << ", " << pseudoInverse(Ax).cols()<< endl;
        // cout << "AyH:\n" << pseudoInverse(Ay) << endl;
        // cout << "AyH:\n" << pseudoInverse(Ay).rows() << ", " << pseudoInverse(Ay).cols()<< endl;
        // cout << "AthetaH:\n" << pseudoInverse(Atheta) << endl;
        // cout << "AthetaH:\n" << pseudoInverse(Atheta).rows() << ", " << pseudoInverse(Atheta).cols()<< endl;
        // cout << "AH:\n" << _polynomial.conversion.AH << endl;
        // cout << "AH:\n" << _polynomial.conversion.AH.rows() << ", " << _polynomial.conversion.AH.cols()<< endl;
        // cout << "M:\n" << _polynomial.conversion.M.rows() << ", " << _polynomial.conversion.M.cols()<< endl;
        // cout << "d:\n" << d.rows() << ", " << d.cols()<< endl;

        _polynomial.conversion.p = _polynomial.conversion.AH*
                                   _polynomial.conversion.M*d;
        
        _polynomial.px = _polynomial.conversion.p.head(7);
        _polynomial.ptheta = _polynomial.conversion.p.tail(7);
        _polynomial.py = _polynomial.conversion.p.segment(7,7);
        double xInit = 0;
        double yInit = 0;
        double thetaInit = 0;
        double xGoal = 0;
        double yGoal = 0;
        double thetaGoal = 0;
        for (int i=0; i<7; ++i){
            xInit = xInit + _polynomial.px[i]*std::pow(t_0, i);
            yInit = yInit + _polynomial.py[i]*std::pow(t_0, i);
            thetaInit = thetaInit + _polynomial.ptheta[i]*std::pow(t_0, i);
        }

        for (int i=0; i<7; ++i){
            xGoal = xGoal + _polynomial.px[i]*std::pow(t_m, i);
            yGoal = yGoal + _polynomial.py[i]*std::pow(t_m, i);
            thetaGoal = thetaGoal + _polynomial.ptheta[i]*std::pow(t_m, i);
        }

		cout << "init: " << xInit << ", " << yInit << ", " << thetaInit << endl;
		cout << "goal: " << xGoal << ", " << yGoal << ", " << thetaGoal << endl;
        // cout << "v1: " << _robotState.vRobot.x << ", " << _robotState.vRobot.y << endl;
        // cout << "yaw1: " << _robotState.yaw << endl;
        _robotState.vRobot.x = x_goal - _robot.x;
        _robotState.vRobot.y = y_goal - _robot.y;
        _robotState.yaw = theta_goal;
        // cout << "v2: " << _robotState.vRobot.x << ", " << _robotState.vRobot.y << endl;
        // cout << "yaw2: " << _robotState.yaw << endl;




        // cout << "A:\n" << _polynomial.conversion.A << endl;
        // cout << "A:\n" << _polynomial.conversion.A.rows() << ", " 
        //                << _polynomial.conversion.A.cols() << endl;
        // cout << "AH:\n" << pseudoInverse(_polynomial.conversion.A).rows() << ", " 
        //                 << pseudoInverse(_polynomial.conversion.A).cols() << endl;
        // cout << "Ax:\n" << _polynomial.conversion.A.topRows(12) << endl;
        // cout << "Ay:\n" << _polynomial.conversion.A.block(12,0,12,_polynomialOrder+1) << endl;
        // cout << "Atheta:\n" << _polynomial.conversion.A.bottomRows(12) << endl;
        // cout << "Q:\n" << _polynomial.conversion.Q << endl;
        // cout << "Qx:\n" << Qx << endl;
        // cout << "Qy:\n" << Qy << endl;
        // cout << "Qtheta:\n" << Qtheta << endl;
        // cout << "R:\n" << _polynomial.conversion.R << endl;
        // cout << "dp_star:\n" << _polynomial.conversion.dp_star << endl;
        // cout << "p:\n" << _polynomial.conversion.p << endl;
        // cout << "px:\n" << _polynomial.px << endl;
        // cout << "py:\n" << _polynomial.py << endl;
        // cout << "ptheta:\n" << _polynomial.ptheta << endl;
    }

    void polynomial(int step, vector<state> path){
        vector<float> x_v;
        vector<float> y_v;

        for (int i=0; i<path.size(); i+=step){
            x_v.push_back(path[i].x);
            y_v.push_back(path[i].y);
        }
            x_v.push_back(path.back().x);
            y_v.push_back(path.back().y);
        
    }

    template <class T>
    int toCostMap(T x){
        return _gridToCostMapRatio*x;
    }

    template <class T>
    double toWorldMap(T x){
        return x/_gridToCostMapRatio;
    }

    /* update nine-cell of anobstacle*/
    void nineCellUpdate(Eigen::MatrixXd &map, int x, int y, int value){
        if (x==x_max || y==y_max) return;

        map(_gridToCostMapRatio*x, _gridToCostMapRatio*y) = value;
        map(_gridToCostMapRatio*x, _gridToCostMapRatio*y+1) = value;
        map(_gridToCostMapRatio*x+1, _gridToCostMapRatio*y+1) = value;
        map(_gridToCostMapRatio*x+1, _gridToCostMapRatio*y) = value;

        if (y>0){
            map(_gridToCostMapRatio*x, _gridToCostMapRatio*y-1) = value;
            map(_gridToCostMapRatio*x+1, _gridToCostMapRatio*y-1) = value;
        }

        if (x>0){
            map(_gridToCostMapRatio*x-1, _gridToCostMapRatio*y) = value;
            map(_gridToCostMapRatio*x-1, _gridToCostMapRatio*y+1) = value;
        }

        if (x>0 && y>0)
            map(_gridToCostMapRatio*x-1, _gridToCostMapRatio*y-1) = value;
    }


    void updatePotentialCostMap(const std::vector<Obstacle_t> &currObsVec,
            PotentialCostMap_t &costMap, float sparsity=1){
        _findPotentialBoundary(currObsVec);
        _updatePotentialMap(sparsity, currObsVec);
    };

    void updatePotentialGPCostMap(float sparsity, libgp::GaussianProcess &gp, 
                                  std::vector<Obstacle_t> &datapoint,
                                  const std::vector<Obstacle_t> &currObsVec,
                                  PotentialCostMap_t &costMap){
        _findPotentialBoundary(currObsVec);
        _updatePotentialGPMap(sparsity, gp, datapoint, currObsVec);
    };

    std::map<float, Node> distance_map;

    Node getNearestNode(geometry_msgs::Point p) {
        int n_nodes = nodesList.size();
        if (n_nodes == 1) {
            return (nodesList[0]);
        }
        distance_map.clear();

        for (int i = 0; i < n_nodes; i++) {
            Node treeNode = nodesList[i];
            float d = getEuclideanDistance(p, treeNode.point);
            distance_map[d] = treeNode;
        }

        return distance_map.begin()->second;
    }

    float getEuclideanDistance(float x1, float x2) {
        return std::abs(x1-x2);
    }

    double getEuclideanDistance(double x1, double y1, double x2, double y2) {
        return std::sqrt(std::pow((x1 - x2), 2) + std::pow((y1 - y2), 2));
    }

    /**
     *
     * @param p1
     * @param p2
     * @return euclidean distance between p1 and p2
     */
    float getEuclideanDistance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
        return std::sqrt(std::pow((p1.x - p2.x), 2) + std::pow((p1.y - p2.y), 2));
    }

    /**
     *
     * @param p1 nearest point
     * @param p2 random config
     * @return point P which is in sigma distance to p1 in the direction of p2
     */
    Node expand(Node p1, Node p2, std::vector<visualization_msgs::Marker> obsVec, 
            int frameid) {
        //calculate the slope
        float m, nume, denom;
        if (p1.point.x != p2.point.x) {
            nume = (p2.point.y - p1.point.y);
            denom = (p2.point.x - p1.point.x);
            m = nume / denom;
        }
        float theta = atan(m);
        if (theta < 0) {
            if (denom < 0) {
                theta = theta + M_PI;
            } else {
                theta = theta + 2 * M_PI;
            }
        } else {
            if ((nume < 0) && (denom < 0)) {
                theta = theta + M_PI;
            }
        }
        float sin_theta = sin(theta);
        float cos_theta = cos(theta);

        //calculate P
        Node p;
        p.point.y = _sigma * sin_theta + p1.point.y;
        p.point.x = _sigma * cos_theta + p1.point.x;
        p.point.z = 0;
        p.id = frameid;

        // calculate if the point is within an obstacle
        if (!intersectsObs(p1.point, p.point, obsVec) && isWithinWorld(p.point)) {
            std::vector<Node>::iterator it = parentList.begin();
            it = parentList.insert(it, p1);

            p.parentId = p1.id;
            p1.children.push_back(p); //children of init is not in the nodeslist

            nodesList.push_back(p);
            return p;
        }
        return p1;
    }

    bool isWithinWorld(geometry_msgs::Point p) {
        return (p.x > this->x_min && p.x < this->x_max && p.y > this->y_min && p.y < this->y_max);
    }

    template <class T>
    bool intersectsObs(T p1, T p2, std::vector<visualization_msgs::Marker> obsVec) {
        // bool intersectsObs(geometry_msgs::Point p1, geometry_msgs::Point p2, 
        //                    std::vector<visualization_msgs::Marker> obsVec) {
        float x1 = p1.x;
        float y1 = p1.y;
        float x2 = p2.x;
        float y2 = p2.y;

        for (int i = 0; i < obsVec.size(); i++) {
            visualization_msgs::Marker obs = obsVec[i];

            float obs_xl = (obs.pose.position.x - obs.scale.x / 2) - 0.01;
            float obs_xr = (obs.pose.position.x + obs.scale.x / 2) + 0.01;
            float obs_yb = (obs.pose.position.y - obs.scale.y / 2) - 0.01;
            float obs_yt = (obs.pose.position.y + obs.scale.y / 2) + 0.01;

            //check for the bottom intersection
            bool bottom = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xr, obs_yb);
            //left intersect
            bool left = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xl, obs_yt);
            //right intersect
            bool right = lineIntersect(x1, y1, x2, y2, obs_xr, obs_yb, obs_xr, obs_yt);
            //top intersect
            bool top = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yt, obs_xr, obs_yt);

            if (bottom || left || right || top) {
                return true;
            }
        }
        return false;
    }

    bool lineIntersect(float x1, float y1, float x2, float y2, 
                       float x3, float y3, float x4, float y4) {
        // calculate the distance to intersection point
        float uA = ((x4-x3) * (y1-y3) - (y4-y3) * (x1-x3)) / 
                    ((y4-y3) * (x2-x1) - (x4-x3) * (y2-y1));
        float uB = ((x2-x1) * (y1-y3) - (y2-y1) * (x1-x3)) / 
                    ((y4-y3) * (x2-x1) - (x4-x3) * (y2-y1));

        // if uA and uB are between 0-1, lines are colliding
        if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) {
            float intersectionX = x1 + (uA * (x2 - x1));
            float intersectionY = y1 + (uA * (y2 - y1));

            return true;
        }
        return false;
    }

    void getCurrObstacles(const geometry_msgs::Point &robot, 
                          const std::vector<visualization_msgs::Marker> &stateObsVec,
                          // std::vector<geometry_msgs::Point> &currObsVec){
                          std::vector<Obstacle_t> &currObsVec){
         float offset = 0.05;
         for (int i=0; i<stateObsVec.size(); i++) {
             visualization_msgs::Marker curObs = stateObsVec[i];
             Obstacle_t obs(i);
             geometry_msgs::Point p;
             float curObs_x1 = curObs.pose.position.x - curObs.scale.x/2 - offset;
             float curObs_y1 = curObs.pose.position.y - curObs.scale.y/2 - offset;
             p.x = curObs_x1;
             p.y = curObs_y1;
             p.z = 0;

             if(!intersectsObs(robot, p, stateObsVec)){
                 obs.point = p;
                 currObsVec.push_back(obs);
             }
             // currObsVec.push_back(p);

             float curObs_x2 = curObs.pose.position.x + curObs.scale.x/2 + offset;
             float curObs_y2 = curObs.pose.position.y - curObs.scale.y/2 - offset;
             p.x = curObs_x2;
             p.y = curObs_y2;
             p.z = 0;

             if(!intersectsObs(robot, p, stateObsVec)){
                 obs.point = p;
                 currObsVec.push_back(obs);
             }
             // currObsVec.push_back(p);

             float curObs_x3 = curObs.pose.position.x + curObs.scale.x/2 + offset;
             float curObs_y3 = curObs.pose.position.y + curObs.scale.y/2 + offset;
             p.x = curObs_x3;
             p.y = curObs_y3;
             p.z = 0;

             if(!intersectsObs(robot, p, stateObsVec)){
                 obs.point = p;
                 currObsVec.push_back(obs);
             }
             // currObsVec.push_back(p);

             float curObs_x4 = curObs.pose.position.x - curObs.scale.x/2 - offset;
             float curObs_y4 = curObs.pose.position.y + curObs.scale.y/2 + offset;
             p.x = curObs_x4;
             p.y = curObs_y4;
             p.z = 0;

             if(!intersectsObs(robot, p, stateObsVec)){
                 obs.point = p;
                 currObsVec.push_back(obs);
             }
             // currObsVec.push_back(p);
         }
     }


    void getCurrObstacles(const geometry_msgs::Point &robot, 
                          const std::vector<visualization_msgs::Marker> &worldObsVec,
                          std::vector<Obstacle_t> &stateObsVec,
                          // std::vector<geometry_msgs::Point> &currObsVec){
                          std::vector<Obstacle_t> &currObsVec){
         float offset = 0.05;
         for (int i=0; i<worldObsVec.size(); i++) {
             visualization_msgs::Marker curObs = worldObsVec[i];
             Obstacle_t obs(i);
             geometry_msgs::Point p;
             float curObs_x1 = curObs.pose.position.x - curObs.scale.x/2 - offset;
             float curObs_y1 = curObs.pose.position.y - curObs.scale.y/2 - offset;
             p.x = curObs_x1;
             p.y = curObs_y1;
             p.z = 0;

             if(!intersectsObs(robot, p, worldObsVec)){
                 obs.point = p;
                 currObsVec.push_back(obs);
                 stateObsVec.push_back(obs);
             }
             // currObsVec.push_back(p);

             float curObs_x2 = curObs.pose.position.x + curObs.scale.x/2 + offset;
             float curObs_y2 = curObs.pose.position.y - curObs.scale.y/2 - offset;
             p.x = curObs_x2;
             p.y = curObs_y2;
             p.z = 0;

             if(!intersectsObs(robot, p, worldObsVec)){
                 obs.point = p;
                 currObsVec.push_back(obs);
                 stateObsVec.push_back(obs);
             }
             // currObsVec.push_back(p);

             float curObs_x3 = curObs.pose.position.x + curObs.scale.x/2 + offset;
             float curObs_y3 = curObs.pose.position.y + curObs.scale.y/2 + offset;
             p.x = curObs_x3;
             p.y = curObs_y3;
             p.z = 0;

             if(!intersectsObs(robot, p, worldObsVec)){
                 obs.point = p;
                 currObsVec.push_back(obs);
                 stateObsVec.push_back(obs);
             }
             // currObsVec.push_back(p);

             float curObs_x4 = curObs.pose.position.x - curObs.scale.x/2 - offset;
             float curObs_y4 = curObs.pose.position.y + curObs.scale.y/2 + offset;
             p.x = curObs_x4;
             p.y = curObs_y4;
             p.z = 0;

             if(!intersectsObs(robot, p, worldObsVec)){
                 obs.point = p;
                 currObsVec.push_back(obs);
                 stateObsVec.push_back(obs);
             }
             // currObsVec.push_back(p);
         }
     }


    void moveRobot(float x, float y){
        _robot.x = x;
        _robot.y = y;
        _robotState.pRobot.x = x;
        _robotState.pRobot.y = y;
    }

    bool checkGoal(){
        float d = getEuclideanDistance(_robot, _goal.point);
        // bool status = (d < 0.6) ? true : false;
        return (d < 0.6) ? true : false;;
    }

    std::vector<Node> getNodesList() {
        return this->nodesList;
    }

    geometry_msgs::Point getRobot(){
        return this->_robot;
    }

    BoundaryPoint_t getPotentialBoundaryPoints(){
        return this->_potentialBoundaryPoints;
    }

    PotentialCostMap_t getPotentialMap(){
        return this->_potentialCostMap;
    }

    PolynomialTraj_t getTraj(){
        return this->_polynomial;
    }
    
    State_t getRobotState(){
        return this->_robotState;
    }

private:
    Node _init, _goal;
    geometry_msgs::Point _robot;
	State_t _robotState;
    double _robotYaw;
    BoundaryPoint_t _potentialBoundaryPoints;
    PotentialCostMap_t _potentialCostMap;
    PolynomialTraj_t _polynomial;
    int _gridToCostMapRatio;
    int _polynomialOrder;
    float _sigma;
    float _epsilon;
    int x_max;
    int x_min;
    int y_max;
    int y_min;
    std::vector<Node> nodesList;
    std::vector<Node> parentList;
    std::vector<Node> currentObsList;

    /* find potential map boundary to update*/
    void _findPotentialBoundary(const std::vector<Obstacle_t> &currObsVec){
        float xMin = 1000;
        float xMax = -1000;
        float yMin = 1000;
        float yMax = -1000;
        for (int i=0; i<currObsVec.size(); ++i){
            xMin = std::min((float) currObsVec[i].point.x, xMin);
            xMax = std::max((float) currObsVec[i].point.x, xMax);
            yMin = std::min((float) currObsVec[i].point.y, yMin);
            yMax = std::max((float) currObsVec[i].point.y, yMax);
        }
        _potentialBoundaryPoints.p1.x = xMin;
        _potentialBoundaryPoints.p1.y = yMin;

        _potentialBoundaryPoints.p2.x = xMax;
        _potentialBoundaryPoints.p2.y = yMin;

        _potentialBoundaryPoints.p3.x = xMax;
        _potentialBoundaryPoints.p3.y = yMax;

        _potentialBoundaryPoints.p4.x = xMin;
        _potentialBoundaryPoints.p4.y = yMax;
    }

    void _updatePotentialGPMap(float sparsity, libgp::GaussianProcess &gp, 
                        std::vector<Obstacle_t> &datapoint,
                        const std::vector<Obstacle_t> &currObsVec){
        // update boundary
        for (int i=0; i<currObsVec.size(); ++i){
            int x = std::round(currObsVec[i].point.x);
            int y = std::round(currObsVec[i].point.y);
            nineCellUpdate(_potentialCostMap.costMap, x, y , 0);
            nineCellUpdate(_potentialCostMap.costMapInverse, x, y , 1);
        }

        // updata whole map between current boundary
        double d_max = 0;
        // cout << "adding" << endl;
        int t = 0;
        for (float i=_potentialBoundaryPoints.p1.x; 
             i<_potentialBoundaryPoints.p3.x; 
             i+=sparsity){
            for (float j=_potentialBoundaryPoints.p1.y; 
                 j<_potentialBoundaryPoints.p3.y; 
                 j+=sparsity){
                double d = 100000;
                for (int k=0; k<currObsVec.size(); ++k){
                    d = std::min(getEuclideanDistance(i, j, 
                                      currObsVec[k].point.x, currObsVec[k].point.y), 
                                 d);
                }
                d_max = std::max(d, d_max);
                double x[] = {(double) toCostMap(i), (double) toCostMap(j)};
                double y = d/2;
                Obstacle_t obs(t);
                geometry_msgs::Point p;
                p.x = i;
                p.y = j;
                p.z = 0;
                obs.point = p;
                datapoint.push_back(obs);
                
                gp.add_pattern(x, y);
                t++;
            } 
        }
        // cout << "added" << endl;

        // cout << "inferencing" << endl;
        for (int i=toCostMap(_potentialBoundaryPoints.p1.x); 
             i<toCostMap(_potentialBoundaryPoints.p3.x); 
             ++i){
            for (int j=toCostMap(_potentialBoundaryPoints.p1.y); 
                 j<toCostMap(_potentialBoundaryPoints.p3.y); 
                 ++j){
                double x[] = {(double) i, (double) j};
                double d = gp.f(x);
                _potentialCostMap.costMap(i, j) = d/2;
                _potentialCostMap.costMapInverse(i, j) = 1/((d)+1e-6);

                if ((i>toCostMap(12)) && (i<toCostMap(15)) && 
                    (j>toCostMap(15)) && (j<toCostMap(20)))
                    _potentialCostMap.costMap(i, j) = -1;

                if ((i>toCostMap(12)) && (i<toCostMap(15)) && 
                    (j<toCostMap(17)) && (j>toCostMap(8)))
                    _potentialCostMap.costMap(i, j) = -1;
            } 
        }
        _potentialCostMap.signedCostMap = _potentialCostMap.costMap -
                                          _potentialCostMap.costMapInverse;
        // cout << "inferenced" << endl;


    }

    void _updatePotentialMap(float sparsity, const std::vector<Obstacle_t> &currObsVec){
        // update boundary
        for (int i=0; i<currObsVec.size(); ++i){
            int x = std::round(currObsVec[i].point.x);
            int y = std::round(currObsVec[i].point.y);
            nineCellUpdate(_potentialCostMap.costMap, x, y , 0);
            nineCellUpdate(_potentialCostMap.costMapInverse, x, y , 1);
        }

        // updata whole map between current boundary
        double d_max = 0;
        for (int i=toCostMap(_potentialBoundaryPoints.p1.x); 
             i<toCostMap(_potentialBoundaryPoints.p3.x); 
             i+= sparsity){
            for (int j=toCostMap(_potentialBoundaryPoints.p1.y); 
                 j<toCostMap(_potentialBoundaryPoints.p3.y); 
                 j+=sparsity){
                double cell_x = toWorldMap(i);
                double cell_y = toWorldMap(j);
                double d = 100000;
                for (int k=0; k<currObsVec.size(); ++k){
                    d = std::min(getEuclideanDistance(cell_x, cell_y, 
                                      currObsVec[k].point.x, currObsVec[k].point.y), 
                                 d);
                }
                d_max = std::max(d, d_max);
                _potentialCostMap.costMap(i, j) = d/2;
                _potentialCostMap.costMapInverse(i, j) = 1/((d)+1e-6);
                if ((i>toCostMap(12)) && (i<toCostMap(15)) && (j>toCostMap(16)) && (j<toCostMap(20)))
                    _potentialCostMap.costMap(i, j) = -1;
            } 
        }
        _potentialCostMap.signedCostMap = _potentialCostMap.costMap -
                                          _potentialCostMap.costMapInverse;

        // this->_potentialCostMap.costMap = ((this->_potentialCostMap.costMap)/d_max).eval;
        // Eigen::MatrixXf *tmp = &((this->_potentialCostMap.costMap)/d_max);
        // (this->_potentialCostMap.costMap) = *tmp;
    }
};
} // bipedlab

#endif
