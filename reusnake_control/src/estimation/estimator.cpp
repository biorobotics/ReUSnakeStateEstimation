#include "estimator.hpp"


namespace robot {


  // state index init			
  // 3 position
  // 3 velocity
  // 4 rotation
  // 3 foot pos 1
  // 3 foot pos 2
  // 3 foot pos 3
  // 3 foot pos 4
  // 3 foot pos 5
  // 3 foot pos 6
  // 3 gyro bias
  // 3 acc bias
  int state_pos_idx =  0;     //  0  1  2
  int state_vel_idx =  3;     //  3  4  5
  int state_ort_idx =  6;     //  6  7  8 9 
  int state_foot_idx= 10;     // 10 11 12
                              // 13 14 15
                              // 16 17 18 
                              // 19 20 21 
                              // 22 23 24 
                              // 25 26 27
  int state_bgyro_idx = 28;   // 28 29 30
  int state_bacc_idx = 31;    // 31 32 33 
  // measurement idx 
  // 3 FK1
  // 3 FK2
  // 3 FK3
  // 3 FK4
  // 3 FK5
  // 3 FK6
  int meas_FK1_idx  =  0;     //  0  1  2
  int meas_FK2_idx  =  3;     //  3  4  5
  int meas_FK3_idx  =  6;     //  6  7  8
  int meas_FK4_idx  =  9;     //  9 10 11
  int meas_FK5_idx  = 12;     // 12 13 14
  int meas_FK6_idx  = 15;     // 15 16 17


  void process_func_gyro(VectorXd& x_t, const VectorXd& x_t_1, 
    const Vector3d a, const Vector3d w, const std::vector<Eigen::VectorXd> ee_vel_c_list, const double dt)
  {

    static Eigen::Vector3d old_acc(0,0,0);

    // cout <<"reach " << __FILE__ << __LINE__ << endl;
    Quaterniond q_1(x_t_1(state_ort_idx+3),x_t_1(state_ort_idx),x_t_1(state_ort_idx+1),x_t_1(state_ort_idx+2));
    // Estimated gyro bias
    Vector3d b_g(x_t_1.segment<3>(state_bgyro_idx));
    // Esitmated accelerometer bias
    Vector3d b_a(x_t_1.segment<3>(state_bacc_idx));

    // current body angular velocity
    Vector3d w_b = w - b_g;

    // this is q_eb
    //  q.coeffs (x y z w)
    // add a omeage_e state, averge with imu
    VectorXd q_w(4); q_w << w_b(0)*dt, w_b(1)*dt, w_b(2)*dt, 0;
    VectorXd q_coeffs = q_1.coeffs() + 0.5 * Omega(q_1.coeffs()) * q_w;
    Quaterniond q = Quaterniond(q_coeffs(3),q_coeffs(0),q_coeffs(1),q_coeffs(2));
    q.normalize();

    std::cout << "ori " << a.transpose() << std::endl;
    std::cout << "nob " << a.transpose() - b_a.transpose() << std::endl;
    old_acc = 0.999999*old_acc + 0.000001*(q_1.toRotationMatrix()*(a-b_a-Eigen::Vector3d(0,0,9.8)));
    std::cout << "fil " << old_acc.transpose() << std::endl;

    // cout <<"reach " << __FILE__ << __LINE__ << endl;
    x_t.segment<4>(state_ort_idx) = q.coeffs();

    // r_t   =  r_t_1 + v_t_1*dt
    x_t.segment<3>(state_pos_idx) = x_t_1.segment<3>(state_pos_idx) + x_t_1.segment<3>(state_vel_idx)*dt + 0.5*dt*dt*(old_acc) ;
    // v_t   =  v_t_1 + a_t_1*dt
    x_t.segment<3>(state_vel_idx) = x_t_1.segment<3>(state_vel_idx) + dt*(old_acc);

    x_t.segment<18>(state_foot_idx) = x_t_1.segment<18>(state_foot_idx);

    x_t.segment<3>(state_bgyro_idx) = b_g;

    x_t.segment<3>(state_bacc_idx) = b_a;

  }

  // measurement fucntion for leg odometry
  void measure_func_acc(VectorXd& z_t, const VectorXd& x_t)
  {
    z_t = Eigen::VectorXd(18); z_t.setZero();

    Quaterniond q(x_t(state_ort_idx+3),x_t(state_ort_idx),x_t(state_ort_idx+1),x_t(state_ort_idx+2));
    // measure FK
    z_t.segment<3>(meas_FK1_idx) = q.toRotationMatrix().transpose() * (x_t.segment<3>(state_foot_idx)    - x_t.segment<3>(state_pos_idx));
    z_t.segment<3>(meas_FK2_idx) = q.toRotationMatrix().transpose() * (x_t.segment<3>(state_foot_idx+3)  - x_t.segment<3>(state_pos_idx));
    z_t.segment<3>(meas_FK3_idx) = q.toRotationMatrix().transpose() * (x_t.segment<3>(state_foot_idx+6)  - x_t.segment<3>(state_pos_idx));
    z_t.segment<3>(meas_FK4_idx)=  q.toRotationMatrix().transpose() * (x_t.segment<3>(state_foot_idx+9)  - x_t.segment<3>(state_pos_idx));
    z_t.segment<3>(meas_FK5_idx)=  q.toRotationMatrix().transpose() * (x_t.segment<3>(state_foot_idx+12) - x_t.segment<3>(state_pos_idx));
    z_t.segment<3>(meas_FK6_idx)=  q.toRotationMatrix().transpose() * (x_t.segment<3>(state_foot_idx+15) - x_t.segment<3>(state_pos_idx));

  }

  // measurement fucntion for visual odometry
  void measure_func_visual(VectorXd& z_t, const VectorXd& x_t)
  {
    // from visual odometry: posx posy posz velx vely velz orientx orienty orientz orientw
    z_t = Eigen::VectorXd(7); z_t.setZero();

    z_t.segment<3>(0) = x_t.segment<3>(state_pos_idx);
    z_t.segment<4>(3) = x_t.segment<4>(state_ort_idx);

  }

  Estimator::Estimator(ros::NodeHandle _nh):
    nh(_nh),
    filter_initialized_(false)
  {
    state_size = 34;
    measure_size = 18;
 

    f_func = &process_func_gyro;
    h_func = &measure_func_acc;

    state.resize(state_size);
    Q = 1e-3*MatrixXd::Identity(state_size, state_size);
    R = 1e-12*MatrixXd::Identity(measure_size, measure_size);

  }

  // not used 
	Estimator::Estimator(ros::NodeHandle _nh, int state_size_, int measure_size_):
    nh(_nh),
    filter_initialized_(false)
  {
    state_size = state_size_;
    measure_size = measure_size_;

    f_func = &process_func_gyro;
    h_func = &measure_func_acc;

    state.resize(state_size);
    // TODO: read noise from constructor input
    Q = 0.1*MatrixXd::Identity(state_size, state_size);
    R = 1*MatrixXd::Identity(measure_size, measure_size);
  }

  Estimator::~Estimator(){}

  // must provide initial bias
  void Estimator::initialize(VectorXd& _b_g, VectorXd& _b_a, const std::vector<Eigen::VectorXd> ee_pos_c_list)
  {
    state.setZero();
    Eigen::Matrix3d init_rot; init_rot.setZero();
    Eigen::Vector3d zeroV; zeroV.setZero();
    // this is for flat dataset 
    // double angle = -0.1/180.0*3.1415926;
    // init_rot <<  cos(angle),  -sin(angle),  0,
    //              sin(angle),  cos(angle),  0,
    //              0,  0,  1;

    // for ramp
    double angle = 0.0/180.0*3.1415926;
    init_rot <<  cos(angle),    0,    sin(angle),
                          0,    1,             0,  
                -sin(angle),    0,    cos(angle);

    state.segment<4>(state_ort_idx) = Quaterniond(init_rot).coeffs();
    state.segment<3>(state_bgyro_idx) = _b_g;

    // state.segment<3>(state_bacc_idx) = _b_a;
    // for ramp, set it to be 0
    state.segment<3>(state_bacc_idx) = _b_a;

    state.segment<3>(state_foot_idx) = ee_pos_c_list[0];
    state.segment<3>(state_foot_idx+3) = ee_pos_c_list[1];
    state.segment<3>(state_foot_idx+6) = ee_pos_c_list[2];
    state.segment<3>(state_foot_idx+9) = ee_pos_c_list[3];
    state.segment<3>(state_foot_idx+12) = ee_pos_c_list[4];
    state.segment<3>(state_foot_idx+15) = ee_pos_c_list[5];

    // // delete previous filter if exists
    // if (ukf != NULL)
    // {
    //   delete ukf;
    // }

    ukf = new SRUKF(state_size, measure_size, 0.1, 0.3, f_func, h_func);
  
    ukf -> setQ(Q);
    ukf -> setR(R);

    ukf -> state_pre = state;
    ukf -> state_post = state;

    ukf->S_pre = 1e-5*MatrixXd::Identity(state_size,state_size);
		ukf->S_post = 1e-5*MatrixXd::Identity(state_size,state_size);


		filter_initialized_ = true;
  }

  
  void Estimator::setMeasurementLegOdom() {
    measure_size = 18;
    // R = 0.001*MatrixXd::Identity(measure_size, measure_size);
    // ukf -> setR(R);
    // setR(meas_FK1_idx,meas_FK6_idx+2,0.005);  // FK should be very accurate
    ukf -> setMeasurementFunc(&measure_func_acc, measure_size);
  }

	void Estimator::setMeasurementVisual() {
    measure_size = 7;
    R = 0.1*MatrixXd::Identity(measure_size, measure_size);
    ukf -> setR(R);
    setR(0,0,0.095);
    setR(1,1,0.095);
    setR(2,2,0.007);


    setR(3,6,0.001);
    ukf -> setMeasurementFunc(&measure_func_visual, measure_size);
  }

  void Estimator::setQ(int idx_start, int idx_end, double value) {
    for (int i = idx_start; i <= idx_end; i++) {
      ukf -> Q(i,i) = value;
    }
  }

	void Estimator::setR(int idx_start, int idx_end, double value) {
    for (int i = idx_start; i <= idx_end; i++) {
      ukf -> R(i,i) = value;
    }
  }

  void Estimator::update(const Vector3d& acc_b, const Vector3d& gyro_b, 
    const std::vector<Eigen::VectorXd> ee_pos_c_list, const std::vector<Eigen::VectorXd> ee_vel_c_list, const std::vector<int> contact_list,
    const double dt, const float phase, float covarianceScale)
  {
    //std::cout << "phase of Gait" << phase << std::endl;
    
    // velocity noise
    // setQ(state_vel_idx,state_vel_idx+2,0.1);

    // // change Q for foot
    for (int i = 0; i < 6; i++) {
      // for leg on the ground, very small Q
      if (contact_list[i] >= 0.9f) {
          setQ(state_foot_idx+i*3, state_foot_idx+i*3+2, 1e-3);
      } else {
          setQ(state_foot_idx+i*3, state_foot_idx+i*3+2, 1e-1);
      }
    }

   /* if (0 < phase && phase < .1){
      std::cout << "phase of Gait" << phase << std::endl;
      setQ(7,12,0.001); //  imu measure, not that accurate
      //ukf -> Q = ukf -> Q*10;
    } else {
      setQ(7,12,0.001); 
      ukf -> predict(gyro_b, ee_vel_c_list, dt, phase);
      //ukf -> Q = ukf -> Q*1;
    }*/
    // old_acc = 0.8*old_acc + 0.2*acc_b;
    ukf -> predict(acc_b, gyro_b, ee_vel_c_list, dt, 0);

    // for (int i = 0; i < 6; i++) {
    //   if (contact_list[i] < 0.1f) {
    //       double large = 99;
    //       ukf -> S_pre(state_foot_idx+i*3, state_foot_idx+i*3) =  large;
    //       for (int j = state_foot_idx+i*3+1; j < ukf -> S_pre.cols(); j++) ukf -> S_pre(state_foot_idx+i*3, j) = 0;
    //       ukf -> S_pre(state_foot_idx+i*3+1, state_foot_idx+i*3+1)= large;
    //       for (int j = state_foot_idx+i*3+2; j < ukf -> S_pre.cols(); j++) ukf -> S_pre(state_foot_idx+i*3+1, j) = 0;
    //       ukf -> S_pre(state_foot_idx+i*3+2, state_foot_idx+i*3+2)= large;
    //       for (int j = state_foot_idx+i*3+3; j < ukf -> S_pre.cols(); j++) ukf -> S_pre(state_foot_idx+i*3+2, j) = 0;
    //   }
    // }
    

    setMeasurementLegOdom();
    Eigen::VectorXd measure = Eigen::VectorXd(measure_size); measure.setZero();
    // measure FK
    for (int i = 0; i < 6; i++) {
      measure.segment<3>(meas_FK1_idx+i*3) = ee_pos_c_list[i];
      // measure.segment<3>(meas_FK1_idx+i*3) = ukf -> state_pre.segment<3>(state_foot_idx+i*3);
    }
    
    ukf -> correct(measure,0);
    // ukf -> state_post = ukf -> state_pre;
    

    // normalize quaternion
    Quaterniond q(ukf -> state_post(state_ort_idx+3),ukf -> state_post(state_ort_idx),ukf -> state_post(state_ort_idx+1),ukf -> state_post(state_ort_idx+2));
    q.normalize();
    ukf -> state_post.segment<4>(state_ort_idx) = q.coeffs();


  }

  void Estimator::visualUpdate(const Vector3d& pos, const Vector3d& vel, const Eigen::Quaterniond& meas_q, const float phase) {
    
    // no predict, only correct
    setMeasurementVisual();
    Eigen::VectorXd measure = Eigen::VectorXd(measure_size); measure.setZero();
    
    measure.segment<3>(0) = pos;
    measure.segment<4>(3) = meas_q.coeffs();

    ukf -> correct(measure,phase);

    // normalize quaternion
    Quaterniond q(ukf -> state_post(state_ort_idx+3),ukf -> state_post(state_ort_idx),ukf -> state_post(state_ort_idx+1),ukf -> state_post(state_ort_idx+2));
    q.normalize();
    ukf -> state_post.segment<4>(state_ort_idx) = q.coeffs();    
  }

  void Estimator::getState(VectorXd & estimate_state)
  {
    estimate_state = ukf -> state_post;
  }

	Quaterniond Estimator::getRotation()
  {
    Quaterniond q(ukf -> state_post(state_ort_idx+3),ukf -> state_post(state_ort_idx),ukf -> state_post(state_ort_idx+1),ukf -> state_post(state_ort_idx+2));
    return q;
  }

  Eigen::VectorXd Estimator::getPosition()
  {
    return ukf -> state_post.segment<3>(state_pos_idx);
  }

  Eigen::VectorXd Estimator::getVelocity()
  {
    return ukf -> state_post.segment<3>(state_vel_idx);
  }

  Eigen::VectorXd Estimator::getFootPosition(int i)
  {
    return ukf -> state_post.segment<3>(state_foot_idx+i*3);
  }

  SRUKF* Estimator::getUKF()
  {
    return ukf;
  }




}