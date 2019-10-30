#pragma once

#include "ukf.hpp"
#include "../util/util.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>

/*
 *  This is a state estimator 
 *  My final goal is to estimator many states, but now just start with orientation and velocity
 *  2019-05-12 move on to second stage: add position velocity and foot pose
 *  2019-06-18 move to next stage: incorperate visual slam vins-mono
 *  need to extend current framework to allow different measure function
 */

using namespace Eigen;

namespace robot {

	void process_func_gyro(VectorXd& x_t, const VectorXd& x_t_1, const Vector3d a, const Vector3d w, const double dt);
	void measure_func_acc(VectorXd& z_t, const VectorXd& x_t);
    void measure_func_visual(VectorXd& z_t, const VectorXd& x_t);

	class Estimator
	{
		public:
			Estimator(ros::NodeHandle _nh);
			Estimator(ros::NodeHandle _nh, int state_size_, int measure_size_);
			~Estimator();

			void initialize(VectorXd& _b_g, VectorXd& _b_a, const std::vector<Eigen::VectorXd> ee_pos_c_list);
			bool isInitialized(){return filter_initialized_;}

			void getState(VectorXd & estimate_state);
			Quaterniond getRotation();
			Eigen::VectorXd getPosition();
			Eigen::VectorXd getVelocity();
			Eigen::VectorXd getFootPosition(int i);
			SRUKF* getUKF();

			// these two functions switch different measurement function
			void setMeasurementLegOdom();  // corresponding to measure_func_acc
			void setMeasurementVisual();   // corresponding to measure_func_visual

			void update(const Vector3d& acc_b, const Vector3d& gyro_b, 
				const std::vector<Eigen::VectorXd> ee_pos_c_list, const std::vector<Eigen::VectorXd> ee_vel_c_list, const std::vector<int> contact_list,
				const double dt, const float phase, const float covarianceScale = 1.0);
      
      void visualUpdate(const Vector3d& pos, const Vector3d& vel, const Eigen::Quaterniond& meas_q, const float phase);

			void setQ(int idx_start, int idx_end, double value); // process
			void setR(int idx_start, int idx_end, double value); // measure

		private:
      ros::NodeHandle nh;
			SRUKF* ukf;
			// implementation in second version:
			// state size 43

			// 3 position
			// 3 velocity
			// 4 rotation
      // 3 angular velocity
			// 3 gyro bias
			// 3 acc bias
			// 3 foot pos 1
			// 3 foot pos 2
			// 3 foot pos 3
			// 3 foot pos 4
			// 3 foot pos 5
			// 3 foot pos 6
			// 1 foot 1 contact state
			// 1 foot 2 contact state
			// 1 foot 3 contact state
			// 1 foot 4 contact state
			// 1 foot 5 contact state
			// 1 foot 6 contact state

			int state_size;

			// measure  31
			// 3 FK1
			// 3 FK2
			// 3 FK3
			// 3 FK4
			// 3 FK5
			// 3 FK6
			// 3 vel
			// 1 foot 1 contact state
			// 1 foot 2 contact state
			// 1 foot 3 contact state
			// 1 foot 4 contact state
			// 1 foot 5 contact state
			// 1 foot 6 contact state
			// 3 acc
			int measure_size;
			double dt;

			bool filter_initialized_;

			Vector3d state_pos_xyz;
			Vector3d state_vel_xyz;
			Vector3d state_acc_xyz;
			Quaterniond state_rotation;
			Vector3d state_vel_angular;
			Vector3d state_bias_acc;
			Vector3d state_bias_gyro; 
			VectorXd state;

			MatrixXd R; // measurement noise
			MatrixXd Q; // process noise

			void (*f_func)(VectorXd& x_t, const VectorXd& x_t_1, 
    			const Vector3d a, const Vector3d w, const std::vector<Eigen::VectorXd> ee_vel_c_list, const double dt);
			void (*h_func)(VectorXd& z_t, const VectorXd& x_t);

      // record bias
      bool bias_record_start;
      std::vector<Eigen::VectorXd> acc_bias_list;
      std::vector<Eigen::VectorXd> gyro_bias_list;
	};



} // namespace robot