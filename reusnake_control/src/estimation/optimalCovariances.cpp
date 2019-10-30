#include "optimalCovariances.hpp"

float computeCost(const VectorXd x_ukf, const VectorXd x_gt)
{

	float root =  pow((x_ukf(0)-x_gt(0)),2) + pow((x_ukf(1)-x_gt(1)),2) + pow((x_ukf(2)-x_gt(2)),2);
	//std::cout << "root" << root << std::endl;
	if (root < .00000001){
		return 0.0;
	} else{
    return sqrt(root);
	}
}

float gradFunction(robot::Estimator* estimator, const VectorXd x_gt, const Vector3d& acc_b, const Vector3d& gyro_b, 
    const std::vector<Eigen::VectorXd> ee_pos_c_list, const std::vector<Eigen::VectorXd> ee_vel_c_list, const std::vector<int> contact_list,
    const double dt, const float phase, const float covarianceScale) {

	VectorXd temp_state_pre; 
	VectorXd temp_state_post;
	MatrixXd temp_S_pre;
	MatrixXd temp_S_post;
	SRUKF* ourUKF = estimator->getUKF();

	temp_state_pre = ourUKF->state_pre; 
    temp_state_post = ourUKF->state_post;
    temp_S_pre = ourUKF->S_pre;
    temp_S_post = ourUKF->S_post;

    estimator -> update(acc_b, gyro_b, ee_pos_c_list, ee_vel_c_list, contact_list, dt,phase,covarianceScale);
	VectorXd estimated_pos = estimator -> getPosition();

    //Revert relevant variables back
    ourUKF->state_pre = temp_state_pre; 
    ourUKF->state_post = temp_state_post;
    ourUKF->S_pre = temp_S_pre;
    ourUKF->S_post = temp_S_post;
    
    //return estimated_pos(0);
	return computeCost(estimated_pos, x_gt);
}



float getOptimalScale(robot::Estimator* estimator, int num_iters, const VectorXd x_gt, const Vector3d& acc_b, const Vector3d& gyro_b, 
    const std::vector<Eigen::VectorXd> ee_pos_c_list, const std::vector<Eigen::VectorXd> ee_vel_c_list, const std::vector<int> contact_list,
    const double dt, const float phase, const float init_scale){

	
	float scale = init_scale;
	int iter;
	double alpha = 10.0;
	float h = .0001;
	float x1; float x2; float delta;



	for (iter = 0; iter < num_iters; iter++)
    {

        x1 = gradFunction(estimator, x_gt, acc_b, gyro_b,ee_pos_c_list,ee_vel_c_list,contact_list,dt, phase, scale + h); 
        x2 = gradFunction(estimator, x_gt, acc_b, gyro_b,ee_pos_c_list,ee_vel_c_list,contact_list,dt, phase, scale - h); 
        
        delta = (x1 - x2)/(2*h);

        
        
        scale = scale-alpha*delta ;
        if (isnan(scale)){
              terminate();
            }

        //std::cout << "delta" << delta << std::endl;
        //std::cout << "root" << scale << std::endl;
    }

    return scale;

}

 
