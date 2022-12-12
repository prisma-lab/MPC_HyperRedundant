// ROS libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "gazebo_msgs/ContactsState.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/Empty.h"

// Custom library
#include "utils.h"


// C++ libraries
#include "boost/thread.hpp"
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <random>

// Eigen
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/QR>
#include <unsupported/Eigen/MatrixFunctions>

//Execution Time
#include <time.h>
#include <algorithm>
#include <chrono>

// Acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"
#include "/home/simone/Scrivania/MATLAB/acados/examples/acados_matlab_octave/rover_on_a_pipe/c_generated_code/acados_solver_ocp_rover.h"

// Namespaces
using namespace std;
using namespace Eigen;
using namespace std::chrono;
// Acados params
#define NX     OCP_ROVER_NX
#define NZ     OCP_ROVER_NZ
#define NU     OCP_ROVER_NU
#define NP     OCP_ROVER_NP
#define NBX    OCP_ROVER_NBX
#define NBX0   OCP_ROVER_NBX0
#define NBU    OCP_ROVER_NBU
#define NSBX   OCP_ROVER_NSBX
#define NSBU   OCP_ROVER_NSBU
#define NSH    OCP_ROVER_NSH
#define NSG    OCP_ROVER_NSG
#define NSPHI  OCP_ROVER_NSPHI
#define NSHN   OCP_ROVER_NSHN
#define NSGN   OCP_ROVER_NSGN
#define NSPHIN OCP_ROVER_NSPHIN
#define NSBXN  OCP_ROVER_NSBXN
#define NS     OCP_ROVER_NS
#define NSN    OCP_ROVER_NSN
#define NG     OCP_ROVER_NG
#define NBXN   OCP_ROVER_NBXN
#define NGN    OCP_ROVER_NGN
#define NY0    OCP_ROVER_NY0
#define NY     OCP_ROVER_NY
#define NYN    OCP_ROVER_NYN
#define NH     OCP_ROVER_NH
#define NPHI   OCP_ROVER_NPHI
#define NHN    OCP_ROVER_NHN
#define NPHIN  OCP_ROVER_NPHIN
#define NR     OCP_ROVER_NR

// Class definition
class CONTROLLER {
	public:
		CONTROLLER();
		void new_plan();
		void arm_invdyn_control();
		void run();
		void cmd_wheel_ctrl();
		void cmd_arm_ctrl();
        void model_state_cb( const gazebo_msgs::ModelStates gazebo_msg );
        void joint_state_cb( const sensor_msgs::JointState arm_state_msg );
        void traj_set_cb( const std_msgs::Float64MultiArray traj_set_msg );
        void ft_sens_cb( const gazebo_msgs::ContactsState ft_sens_msg );
		void feedback();
		Eigen::Matrix4d dir_kin(Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd, int);
		Eigen::MatrixXd diff_kin(Eigen::VectorXd);
		Eigen::VectorXd recursive_inv_dyn(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::Vector3d, Eigen::VectorXd );
		Eigen::MatrixXd inertia_matrix(Eigen::VectorXd);
		Eigen::MatrixXd traj_generation(Eigen::Matrix4d, Eigen::Matrix4d, float, float, int);
		void coll_avoidance_param_computation(Eigen::VectorXd, Eigen::Matrix4d, Eigen::Matrix4d*, int*, float*, Eigen::Matrix4d*, int*, float*, Eigen::Matrix4d*, Eigen::Matrix4d*, int*, int*, float*);
		void AV_MPC_param_computation();
		void fileclose();
        void writedata(const Vector3d & , std::ofstream & ofs);
        void writedata(const Vector2d & , std::ofstream & ofs);
		void writedata(const VectorXd & , std::ofstream & ofs);

	private:
		ros::NodeHandle _nh;
		ros::Publisher  _cmd_tau1_pub, _cmd_tau2_pub, _cmd_tau3_pub, _cmd_tau4_pub, _cmd_tau5_pub, _cmd_tau6_pub;
		ros::Publisher  _cmd_tau7_pub, _cmd_tau8_pub, _cmd_tau9_pub, _cmd_tau10_pub, _cmd_tau11_pub, _cmd_tau12_pub;
		ros::Publisher _cmd_tau13_pub, _cmd_tau14_pub, _cmd_tau15_pub, _cmd_tau16_pub, _cmd_tau17_pub, _cmd_tau18_pub;
		ros::Publisher _cmd_tau19_pub, _cmd_tau20_pub, _cmd_tau21_pub;
        ros::Publisher  _cmd_wheel1_pub, _cmd_wheel2_pub, _cmd_wheel3_pub, _cmd_wheel4_pub;
		ros::Subscriber _model_state_sub;
        ros::Subscriber _joint_state_sub;
		ros::Subscriber _traj_setpoint_sub;
		ros::Subscriber _ft_sensor_sub;
		ros::ServiceClient _pauseGazebo;
		ros::ServiceClient _unpauseGazebo;

		Eigen::Vector4d _wheel_cmd;
		Eigen::VectorXd _arm_cmd;
		gazebo_msgs::ModelStates _gaz_odom;
        sensor_msgs::JointState _arm_state;
		std_msgs::Float64MultiArray _traj_setpoints;
		gazebo_msgs::ContactsState _ft_sens;

		Eigen::VectorXd _traj_set;
		Eigen::MatrixXd _2d_traj_setpoints;
		int _len_traj;

        Eigen::Vector3d _Eta;
        bool _first_odom_1, _first_odom_2, _setpoints_received;
		Eigen::VectorXd _arm_state_pos, _arm_state_vel;
		Eigen::Matrix3d _R_uav;
		Eigen::Matrix4d _T_uav, _T_arm;
		Eigen::MatrixXd _T_traj, _Td_traj, _T_tot_traj, _Td_tot_traj;
		Eigen::Vector3d _p_uav;
		Eigen::MatrixXd _J_arm;
		Eigen::VectorXd  _arm_vel_mes;

		// Kinematic/Dynamic modeling
        int _N_JOINTS;
		Eigen::MatrixXd _T_rest, _Inertia, _S, _inertial_dist, _rot_mat, _rot_angle;
		Eigen::VectorXd _x_distances, _y_distances, _z_distances, _mass;
		Eigen::VectorXd _curr_rot, _rot_vec;
		Eigen::Vector3d _gravity;
		int _rate, _ctrl_rate;
		bool _traj_ready, _goal_reached, _start_controller;
		double _theta, _theta_d;


		float _min_dist_self, _min_dist_1_coll, _min_dist_2_coll;
        int _nearest_j_index_1_self, _nearest_j_index_1_coll;
        int _nearest_j_index_2_self, _nearest_j_index_2_coll;
        Eigen::Matrix4d _nearest_j_pose_1_self, _nearest_j_pose_1_coll;
        Eigen::Matrix4d _nearest_j_pose_2_self, _nearest_j_pose_2_coll;
		Eigen::Vector3d _pipe1, _pipe2;
		float _d0, _d_safety, _radius, _safety;

		//inv_dyn params
		Eigen::Matrix4d _M_prev, _M;
		Eigen::MatrixXd _A, _G, _M_save;
		Eigen::MatrixXd _d_base;
		Eigen::VectorXd _F_ee;

		//MPC param
		Eigen::Matrix4d _T_u_com, _T_wuav_init, _T_fix_com_drone;
		int _ctrl_hor;
		double _T_step, _sigma, _delta;
		double _mpc_mu, _mpc_m, _mpc_L, _mpc_Rw, _mpc_gamma_s, _mpc_mu0, _mpc_th_s, _mpc_th_s_dx, _mpc_th_s_sx, _mpc_Rp, _mpc_Ts, _mpc_g;
		double _mpc_L_sx, _mpc_L_dx;
		Eigen::Vector3d _pos_com;
		Eigen::MatrixXd _T_com;
		double _arm_mass, _ang_mpc, _mpc_tot_m, _dist;
		Eigen::Vector2d _p_com_tot, _pos_sx,_pos_dx, _intersection, _gamma_mpc,_theta_s_mpc;
		Eigen::Vector4d _arm_fix_com, _pos_w1_sx_fix, _pos_w2_dx_fix,_pos_w1_sx,_pos_w2_dx;

	    double _p[NP];
		Eigen::VectorXd _F_mpc;

		// Time param
		clock_t _start, _end;
		double _time;
		ros::WallTime start_, end_;
		ros::WallTime start_1, end_1, start_2, end_2, start_3, end_3, start_4, end_4, start_5, end_5;
		string _key_input;

		std::ofstream _epFile;
        std::ofstream _evFile;
        std::ofstream _erpyFile;
        std::ofstream _ewFile;
        std::ofstream _f_extFile;
        std::ofstream _tauFile;
        std::ofstream _mpcparamFile;
        std::ofstream _xmpcFile;
        std::ofstream _umpcFile;

		Eigen::Vector3d _f_ext, _m_ext;

};

// Class constructor
CONTROLLER::CONTROLLER() : _first_odom_1(false), _first_odom_2(false), _setpoints_received(false) {

	// ROS Services
	_pauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    _unpauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

	// ROS Publishers
    _cmd_tau1_pub  = _nh.advertise< std_msgs::Float64 >("/prisma_snake/jointFrame_position_controller/command", 0);
    _cmd_tau2_pub  = _nh.advertise< std_msgs::Float64 >("/prisma_snake/jointMotor_position_controller/command", 0);
    _cmd_tau3_pub  = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint2_position_controller/command", 0);
    _cmd_tau4_pub  = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint3_position_controller/command", 0);
    _cmd_tau5_pub  = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint4_position_controller/command", 0);
    _cmd_tau6_pub  = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint5_position_controller/command", 0);
    _cmd_tau7_pub  = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint6_position_controller/command", 0);
    _cmd_tau8_pub  = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint7_position_controller/command", 0);
    _cmd_tau9_pub  = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint8_position_controller/command", 0);
    _cmd_tau10_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint9_position_controller/command", 0);
    _cmd_tau11_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint10_position_controller/command", 0);
    _cmd_tau12_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint11_position_controller/command", 0);
    _cmd_tau13_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint12_position_controller/command", 0);
    _cmd_tau14_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint13_position_controller/command", 0);
    _cmd_tau15_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint14_position_controller/command", 0);
    _cmd_tau16_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint15_position_controller/command", 0);
    _cmd_tau17_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint16_position_controller/command", 0);
    _cmd_tau18_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint17_position_controller/command", 0);
    _cmd_tau19_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint18_position_controller/command", 0);
    _cmd_tau20_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/joint20_position_controller/command", 0);
    _cmd_tau21_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/jointprobe_position_controller/command", 0);

    _cmd_wheel1_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/JointWheel_F_dx_position_controller/command", 0);
    _cmd_wheel2_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/JointWheel_F_sx_position_controller/command", 0);
    _cmd_wheel3_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/JointWheel_R_dx_position_controller/command", 0);
    _cmd_wheel4_pub = _nh.advertise< std_msgs::Float64 >("/prisma_snake/JointWheel_R_sx_position_controller/command", 0);

    // rOS subscribers
    _model_state_sub   = _nh.subscribe("/gazebo/model_states", 1, &CONTROLLER::model_state_cb, this);
    _joint_state_sub   = _nh.subscribe("/prisma_snake/joint_states", 1, &CONTROLLER::joint_state_cb, this);
    _traj_setpoint_sub = _nh.subscribe("/prisma_snake/traj", 1, &CONTROLLER::traj_set_cb, this);
    _ft_sensor_sub = _nh.subscribe("/prisma_snake/ft_sensor", 1, &CONTROLLER::ft_sens_cb, this);

	// Params initialization
	_rate = 100;
	_ctrl_rate = 100;
	_N_JOINTS = 21;

	_wheel_cmd << 0,0,0,0;

	_arm_cmd.resize(21);
	_arm_cmd << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

	_first_odom_1 = false;
	_first_odom_2 = false;
	_setpoints_received = false;

	_arm_state_pos.resize(21);
	_arm_state_pos<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
	_arm_state_vel.resize(21);
	_arm_state_vel<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

	_T_arm.setIdentity();
	_R_uav.setIdentity();
	_p_uav<<0,0,0;
	_J_arm.resize(6,_N_JOINTS);
	_J_arm.setIdentity();

	_arm_vel_mes.resize(6);
	_arm_vel_mes<<0,0,0,0,0,0;

    _theta =-0.0872;//*2;
	_theta_d = 0.0;

	//+++++++++++++++++ kinematic/dynamic modeling params ++++++++++++++++++++++++++++

	_T_rest.resize(4*_N_JOINTS,4*_N_JOINTS);

	_T_rest.block<4,4>(0,0) <<

	-0.999999999974344,  -0.000002653587619,   0.000006653590660,  -0.010012551106078,
	 0.000006653589793,   0.000000326811685,   0.999999999977811,   0.007727967007767,
	-0.000002653589793,   0.999999999996426,  -0.000000326794029,   0.155000106110157,
					 0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(4,4) <<

	-0.000006653589900,   0.000002000000000,   0.999999999975865,  -0.010012551106078,
	-0.999999999977865,  -0.000000000012440,  -0.000006653589900,   0.007727967007767,
	-0.000000000000867,  -0.999999999998000,   0.000002000000000,   0.155000106110157,
					 0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(8,8) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.190057595997444,
	 0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007729147796908,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177447546020112,
					 0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(12,12) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.242557595996288,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007729479953645,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177447441020112,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(16,16) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.295057595995133,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007729812110383,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177447336020112,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(20,20) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.347557595993977,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007730144267121,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177447231020112,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(24,24) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.400057595992821,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007730476423859,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177447126020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(28,28) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.452557595991665,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007730808580596,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177447021020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(32,32) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.505057595990510,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007731140737334,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177446916020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(36,36) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.557557595989354,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007731472894072,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177446811020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(40,40) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.610057595988198,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007731805050809,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177446706020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(44,44) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.662557595987042,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007732137207547,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177446601020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(48,48) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.715057595985887,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007732469364285,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177446496020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(52,52) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.767557595984731,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007732801521022,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177446391020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(56,56) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.820057595983575,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007733133677760,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177446286020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(60,60) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.872557595982419,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007733465834498,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177446181020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(64,64) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.925057595981264,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007733797991235,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177446076020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(68,68) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -0.977557595980108,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007734130147973,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177445971020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(72,72) <<

	-0.999999999977986,  -0.000002000016789,   0.000006326789696,  -1.030057595978952,
	0.000006326795003,  -0.000002653577353,   0.999999999976465,   0.007734462304711,
	-0.000002000000000,   0.999999999994479,   0.000002653590007,   0.177445866020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(76,76) <<

	0.000006653584593,  -0.000002000016789,   0.999999999975865,  -1.082557595977796,
	0.999999999974344,  -0.000002653576486,  -0.000006653589900,   0.007734794461448,
	0.000002653589793,   0.999999999994479,   0.000001999999133,   0.177445761020111,
					0,                   0,                   0,   1.000000000000000;


	_T_rest.block<4,4>(80,80) <<

	0.000006653584593,  -0.000002000016789,   0.999999999975865,  -1.115557643977203,
	0.999999999974344,  -0.000002653576486,  -0.000006653589900,   0.007734950344345,
	0.000002653589793,   0.999999999994479,   0.000001999999133,   0.201445595020007,
					0,                   0,                   0,   1.000000000000000;

    _x_distances.resize(21);
	_x_distances << -0.0101000000000000,	0,	-0.180045000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0525000000000000,	-0.0330000000000000;

	_y_distances.resize(21);
	_y_distances << 0.00772822000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0;

	_z_distances.resize(21);
	_z_distances << 0.155000000000000,	0,	0.0224478000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0.0239999000000000;
	_mass.resize(21);
	_mass << 0.0518023610121400, 0.254048214605011, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000, 0.0200000000000000;

	_S.resize(21,6);
	_S << 	0,	0,	1,	0.00772822000000000,   0.0101000000000000,	                   0,
			1,	0,	0,	                  0,    0.155000000000000,	-0.00772822000000000,
			0,	0,	1,	0.00772822000000000,	0.190145000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.242645000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.295145000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.347645000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.400145000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.452645000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.505145000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.557645000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.610145000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.662645000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.715145000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.767645000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.820145000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.872645000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.925145000000000,	                   0,
			0,	0,	1,	0.00772822000000000,	0.977645000000000,	                   0,
			0,	0,	1,	0.00772822000000000,     1.03014500000000,	                   0,
			0,	0,	1,	0.00772822000000000,	 1.08264500000000,	                   0,
			0,	1,	0,   -0.201447700000000,	                0,	   -1.11564500000000;
	_Inertia.resize(3*_N_JOINTS,3*_N_JOINTS);

	_Inertia.block<3,3>(0,0) <<

		0,     0,     0,
		0,     0,     0,
		0,     0,     0;

	_Inertia.block<3,3>(3,3) <<

	0.001812954286736,                   0,                   0,
					0,   0.001431881964829,                   0,
					0,                   0,   0.001431881964829;


	_Inertia.block<3,3>(6,6) <<

	0.000611519947657607,                0,                      0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(9,9) <<

	0.000611519947657607,                0,                      0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(12,12) <<

	0.000611519947657607,                0,                      0,
				    0,   0.000613416100174628,                0,
			        0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(15,15) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(18,18) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(21,21) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(24,24) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(27,27) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(30,30) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(33,33) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(36,36) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(39,39) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(42,42) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(45,45) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(48,48) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(51,51) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(54,54) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(57,57) <<

	0.000611519947657607,                   0,                   0,
					0,   0.000613416100174628,                   0,
					0,                   0,   0.000624936047832236;

	_Inertia.block<3,3>(60,60) <<

	0.000600000000000000,                   0,                   0,
					0,   0.000600000000000000,                   0,
					0,                   0,   0.000600000000000000;

    _inertial_dist.resize(21,3);
	_inertial_dist <<  -0.0248469510078303,	-0.0563614377028206,	0.0100000000000000,
					   -0.0100000000000000,	-0.0400000000000000,   -0.0400000000000000,
					    0.0258999036432844,	0.0239999454766124,	0,
					    0.0258999036432844,	0.0239999454766124,	0,
					    0.0258999036432844,	0.0239999454766124,	0,
					    0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0.0258999036432844,	0.0239999454766124,	0,
						0,	0,	0;

	_rot_mat.resize(21,2);
	_rot_mat << 1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2,
				1,	2;
	_rot_angle.resize(21,2);
	_rot_angle <<   1.57079632679490,	3.14159265358979,
				   -1.57079632679490,	1.57079632679490,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	3.14159265358979,
					1.57079632679490,	1.57079632679490,
					1.57079632679490,	1.57079632679490;

    _curr_rot.resize(21);
    _rot_vec.resize(21);
    // Rotation axis of the previous (i-1) frame for each joint
    _rot_vec  << 0, 3, 1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;
    // Rotation axis of the actual (i) frame for each joint
    _curr_rot << 3, 1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2;
	_gravity<<0,0,-9.8;

	_traj_ready = false;
	_goal_reached = true;
	_start_controller = false;

	_min_dist_self = 0.0;
	_min_dist_1_coll = 0.0;
	_min_dist_2_coll = 0.0;
    _nearest_j_index_1_self = 0;
    _nearest_j_index_2_self = 0;
    _nearest_j_pose_1_self.setIdentity();
    _nearest_j_pose_2_self.setIdentity();
    _nearest_j_index_1_coll = 0;
    _nearest_j_index_2_coll = 0;
    _nearest_j_pose_1_coll.setIdentity();
    _nearest_j_pose_2_coll.setIdentity();

	_pipe1 << 0.0, -1.23, 2.027572;
    _pipe2 << 0.0, -1.71, 2.027572;
	_d_safety =  0.01;
	_radius = 0.136728;
	_d0 = 0.055/2 + _radius;
	_safety = 0.05;

	_len_traj = 0;

	_M_prev.setIdentity();
	_A.resize(6,21);
	_G.resize(6*21,6*21);
	_M_save.resize(4*21,4*21);

	_d_base.resize(_N_JOINTS,3);
	_d_base.block<21,1>(0,0) = _x_distances;
	_d_base.block<21,1>(0,1) = _y_distances;
	_d_base.block<21,1>(0,2) = _z_distances;

	for (int i=0;i<_N_JOINTS;i++) {
		_M = utilities::M_f(_rot_mat.block<1,2>(i,0), _rot_angle.block<1,2>(i,0), _d_base, i);
		_M_save.block<4,4>(i*4,i*4) = _M_prev.inverse()*_M;
		_A.block<6,1>(0,i) = utilities::Ad_f(_M.inverse())*(_S.block<1,6>(i,0)).transpose();

		for (int j=0;j<6;j++){
			if (_A(j,i)<0.5){
				_A(j,i) = 0;
			}
		}
		_G.block<6,6>(6*i,6*i) = utilities::G_f(_mass[i],_Inertia.block<3,3>(i*3,i*3),_inertial_dist.block<1,3>(i,0));
		_M_prev = _M;
	}

	_arm_mass = 0.685850575617151;
	_T_u_com << 1,0,0,0, 0,1,0,0, 0,0,1,-0.0736, 0,0,0,1;
	_ctrl_hor = 10;
	_T_step = 0.01;
	_sigma = 50;
	_delta = 0.05;
	_mpc_mu = 0.85;
	_mpc_m = 10.6;
	_mpc_L = 0.233674;
	_mpc_Rw = 0.035;
	_mpc_gamma_s = 1.1456;
	_mpc_mu0 = tan(_mpc_gamma_s);
	_mpc_th_s = 0.4252;
	_mpc_th_s_dx = _mpc_th_s;
	_mpc_th_s_sx = _mpc_th_s;
	_mpc_Rp = 0.136728;
	_mpc_Ts = 0.01;
	_mpc_g = 9.8;
	_ang_mpc = -0,174533/2;
	_T_wuav_init <<  0,0,0,2, 0,0,0,-1.185, 0,0,0,2.285, 0,0,0,1;
	_T_wuav_init.block<3,3>(0,0)=utilities::RpyToMat(Eigen::Vector3d(-0.1745, 0.0, 0.0));
	_T_uav = _T_wuav_init;
	_T_com.resize(4*_N_JOINTS,4*_N_JOINTS);
	_T_com.block<4,4>(0,0) << 1,0,0,-0.0248469510078303, 0,1,0,-0.0563614377028206, 0,0,1,0.01, 0,0,0,1;
    _T_com.block<4,4>(4,4) << 1,0,0,-0.01, 0,1,0,-0.04, 0,0,1,-0.04, 0,0,0,1;
    _T_com.block<4,4>(8,8) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(12,12) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(16,16) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(20,20) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(24,24) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(28,28) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(32,32) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(36,36) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(40,40) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(44,44) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(48,48) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(52,52) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(56,56) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(60,60) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(64,64) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(68,68) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(72,72) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(76,76) <<  1,0,0,0.0258999036432844, 0,1,0,0.0239999454766124, 0,0,1,0, 0,0,0,1;
    _T_com.block<4,4>(4*(_N_JOINTS-1),4*(_N_JOINTS-1)) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1;

	_mpc_tot_m = _arm_mass+_mpc_m;
	_pos_w1_sx << 0, 0.087805, -0.113358, 1;
    _pos_w2_dx << 0, -0.087805, -0.113358, 1;

	_F_mpc.resize(6);
	_F_mpc << 0,0,0,0,0,0;

	_tauFile.open("/home/simone/uav_ws/src/prisma_snake_control/log/tau.txt", std::ios::out);
    _epFile.open("/home/simone/uav_ws/src/prisma_snake_control/log/ep.txt", std::ios::out);
    _evFile.open("/home/simone/uav_ws/src/prisma_snake_control/log/ev.txt", std::ios::out);
    _ewFile.open("/home/simone/uav_ws/src/prisma_snake_control/log/ew.txt", std::ios::out);
    _erpyFile.open("/home/simone/uav_ws/src/prisma_snake_control/log/erpy.txt", std::ios::out);
    _f_extFile.open("/home/simone/uav_ws/src/prisma_snake_control/log/f_ext.txt", std::ios::out);
    _xmpcFile.open("/home/simone/uav_ws/src/prisma_snake_control/log/xmpc.txt", std::ios::out);
    _umpcFile.open("/home/simone/uav_ws/src/prisma_snake_control/log/umpc.txt", std::ios::out);
    _mpcparamFile.open("/home/simone/uav_ws/src/prisma_snake_control/log/param.txt", std::ios::out);

	_f_ext << 0,0,0;
	_m_ext << 0,0,0;
	_F_ee.resize(6);
	_F_ee << 0,0,0,0,0,0;
	_dist = 0;
}

// Close log-file
void CONTROLLER::fileclose(){

  _tauFile.close();
  _epFile.close();
  _evFile.close();
  _ewFile.close();
  _erpyFile.close();
  _xmpcFile.close();
  _umpcFile.close();
  _f_extFile.close();
  _mpcparamFile.close();

}

void CONTROLLER::writedata(const Vector3d & data,std::ofstream & ofs){

    ofs<<data(0)<<','<<data(1)<<','<<data(2);
    ofs<<'\n';
}

void CONTROLLER::writedata(const Vector2d & data,std::ofstream & ofs){

    ofs<<data(0)<<','<<data(1);
    ofs<<'\n';
}

void CONTROLLER::writedata(const VectorXd & data,std::ofstream & ofs){

    ofs<<data(0)<<','<<data(1)<<','<<data(2)<<','<<data(3)<<','<<data(4)<<','<<data(5)<<','<<data(6)<<','<<data(7)<<','<<data(8)<<','<<data(9)<<','<<data(10)<<','<<data(11)<<','<<data(12)<<','<<data(13)<<','<<data(14)<<','<<data(15)<<','<<data(16)<<','<<data(17)<<','<<data(18)<<','<<data(19)<<','<<data(20);
    ofs<<'\n';
}

// ------ CALLBACK ROS SUBSCRIBER ------

// Callback arm feedback
void CONTROLLER::joint_state_cb( const sensor_msgs::JointState arm_state_msg ) {

    _arm_state = arm_state_msg;
	Eigen::VectorXd arm_pos_temp, arm_vel_temp;
	arm_pos_temp.resize(_N_JOINTS+4);
	arm_vel_temp.resize(_N_JOINTS+4);
	for(int i=0; i<_N_JOINTS+4;i++){
		arm_pos_temp[i] = _arm_state.position[i];
		arm_vel_temp[i] = _arm_state.velocity[i];
	}
	_arm_state_pos(0) = arm_pos_temp(18);
	_arm_state_pos(1) = arm_pos_temp(19);
	_arm_state_pos(2) = arm_pos_temp(9);
	_arm_state_pos(3) = arm_pos_temp(11);
	_arm_state_pos(4) = arm_pos_temp(12);
	_arm_state_pos(5) = arm_pos_temp(13);
	_arm_state_pos(6) = arm_pos_temp(14);
	_arm_state_pos(7) = arm_pos_temp(15);
	_arm_state_pos(8) = arm_pos_temp(16);
	_arm_state_pos(9) = arm_pos_temp(17);
	_arm_state_pos(10) = arm_pos_temp(0);
	_arm_state_pos(11) = arm_pos_temp(1);
	_arm_state_pos(12) = arm_pos_temp(2);
	_arm_state_pos(13) = arm_pos_temp(3);
	_arm_state_pos(14) = arm_pos_temp(4);
	_arm_state_pos(15) = arm_pos_temp(5);
	_arm_state_pos(16) = arm_pos_temp(6);
	_arm_state_pos(17) = arm_pos_temp(7);
	_arm_state_pos(18) = arm_pos_temp(8);
	_arm_state_pos(19) = arm_pos_temp(10);
	_arm_state_pos(20) = arm_pos_temp(24);

	_arm_state_vel(0) = arm_vel_temp(18);
	_arm_state_vel(1) = arm_vel_temp(19);
	_arm_state_vel(2) = arm_vel_temp(9);
	_arm_state_vel(3) = arm_vel_temp(11);
	_arm_state_vel(4) = arm_vel_temp(12);
	_arm_state_vel(5) = arm_vel_temp(13);
	_arm_state_vel(6) = arm_vel_temp(14);
	_arm_state_vel(7) = arm_vel_temp(15);
	_arm_state_vel(8) = arm_vel_temp(16);
	_arm_state_vel(9) = arm_vel_temp(17);
	_arm_state_vel(10) = arm_vel_temp(0);
	_arm_state_vel(11) = arm_vel_temp(1);
	_arm_state_vel(12) = arm_vel_temp(2);
	_arm_state_vel(13) = arm_vel_temp(3);
	_arm_state_vel(14) = arm_vel_temp(4);
	_arm_state_vel(15) = arm_vel_temp(5);
	_arm_state_vel(16) = arm_vel_temp(6);
	_arm_state_vel(17) = arm_vel_temp(7);
	_arm_state_vel(18) = arm_vel_temp(8);
	_arm_state_vel(19) = arm_vel_temp(10);
	_arm_state_vel(20) = arm_vel_temp(24);

	_first_odom_1 = true;
}

// Callback rover feedback
void CONTROLLER::model_state_cb( const gazebo_msgs::ModelStates gazebo_msg ) {

    _gaz_odom = gazebo_msg;
	_p_uav << _gaz_odom.pose[5].position.x, _gaz_odom.pose[5].position.y, _gaz_odom.pose[5].position.z;
	_Eta = utilities::R2XYZ( utilities::QuatToMat ( Vector4d( _gaz_odom.pose[5].orientation.w, _gaz_odom.pose[5].orientation.x, _gaz_odom.pose[5].orientation.y, _gaz_odom.pose[5].orientation.z ) ) );
	_theta_d = _gaz_odom.twist[5].angular.x;
	_theta = _Eta(0);
	_first_odom_2 = true;

}

// Callback setpoints from navigation function
void CONTROLLER::traj_set_cb( const std_msgs::Float64MultiArray traj_set_msg ){
    _traj_setpoints = traj_set_msg;
	_len_traj = _traj_setpoints.data[0];
    _traj_set.resize(_len_traj);
	int k = 0;
	for(int i=0;i<_len_traj;i++){
		_traj_set(i) = _traj_setpoints.data[i+1];
	}
	_2d_traj_setpoints.resize(_len_traj/2,2);
	for(int j=0;j<2;j++){
		for (int i=0;i<_len_traj/2;i++){
			_2d_traj_setpoints(i,j) = _traj_set(k);
			k++;
		}
	}
	_setpoints_received = true;
}

// Callback force sensor feedback
void CONTROLLER::ft_sens_cb( const gazebo_msgs::ContactsState ft_sens_msg ){

	_ft_sens = ft_sens_msg;
}

// Compute actual arm kinematics from feedback and avoidance parameters
void CONTROLLER::feedback(){
	std_srvs::Empty pauseSrv;
	float s_minimum_dist;
	int s_nearest_joint_index_1, s_nearest_joint_index_2;
	Eigen::Matrix4d s_nearest_joint_pose_1, s_nearest_joint_pose_2;
	float min_dist_1, min_dist_2;
	int nearest_joint_index_1, nearest_joint_index_2;
	Eigen::Matrix4d nearest_joint_pose_1, nearest_joint_pose_2;
	Eigen::Vector3d pos_com;

	_T_arm = dir_kin(_arm_state_pos,_S,_T_rest.block<4,4>(80,80),_N_JOINTS);
	_J_arm = diff_kin(_arm_state_pos);
	_arm_vel_mes = _J_arm*_arm_state_vel;

    // _pauseGazebo.call(pauseSrv);
	if (!_ft_sens.states.empty()){
		_f_ext << _ft_sens.states[0].total_wrench.force.x,_ft_sens.states[0].total_wrench.force.y,_ft_sens.states[0].total_wrench.force.z;
		_m_ext << _ft_sens.states[0].total_wrench.torque.x,_ft_sens.states[0].total_wrench.torque.y,_ft_sens.states[0].total_wrench.torque.z;

		_F_ee <<0,0,0,_f_ext;
	}
	else {
		// _f_ext << 0,0,0;
		// _F_ee.setZero();
	}

	_R_uav = utilities::RpyToMat(_Eta);
	_T_uav.setIdentity();
	_T_uav.block<3,3>(0,0) = _R_uav;
	_T_uav.block<3,1>(0,3) = _p_uav;

	coll_avoidance_param_computation( _arm_state_pos, _T_uav, &nearest_joint_pose_1, &nearest_joint_index_1, &min_dist_1, &nearest_joint_pose_2, &nearest_joint_index_2, &min_dist_2, &s_nearest_joint_pose_1, &s_nearest_joint_pose_2, &s_nearest_joint_index_1, &s_nearest_joint_index_2, &s_minimum_dist);

	_min_dist_self = s_minimum_dist;
	_nearest_j_index_1_self = s_nearest_joint_index_1;
	_nearest_j_index_2_self = s_nearest_joint_index_2;
	_nearest_j_pose_1_self = s_nearest_joint_pose_1;
	_nearest_j_pose_2_self  = s_nearest_joint_pose_2;

	_min_dist_1_coll = min_dist_1;
	_min_dist_2_coll = min_dist_2;
	_nearest_j_index_1_coll = nearest_joint_index_1;
	_nearest_j_index_2_coll = nearest_joint_index_2;
	_nearest_j_pose_1_coll = nearest_joint_pose_1;
	_nearest_j_pose_2_coll  = nearest_joint_pose_2;

	pos_com = _pos_com;
	_arm_fix_com = _T_uav*(Eigen::Vector4d(pos_com(0),pos_com(1),pos_com(2), 1));
    _T_fix_com_drone = _T_uav*_T_u_com;
    _p_com_tot = (_arm_mass*_arm_fix_com.block<2,1>(1,0)+_mpc_m*_T_fix_com_drone.block<2,1>(1,3))/(_arm_mass+_mpc_m);

	//robustness test
	unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator(seed);
    normal_distribution<double> distribution(0, 0.01);
	_dist = + distribution(generator);
	_ang_mpc = atan2(-(_p_com_tot(0)-_pipe1(1)),+(_p_com_tot(1)-_pipe1(2))) + _dist;
	
	// Update MPC param from rover COM knowledge
    _mpc_L = sqrt(pow((_p_com_tot(0)-_pipe1(1)),2)+pow((_p_com_tot(1)-_pipe1(2)),2));

    _pos_w1_sx_fix = _T_uav*_pos_w1_sx;
    _pos_sx = _pos_w1_sx_fix.block<2,1>(1,0);
    _mpc_L_sx = sqrt(pow((_p_com_tot(0)-_pos_sx(0)),2)+pow((_p_com_tot(1)-_pos_sx(1)),2));

    _pos_w2_dx_fix = _T_uav*_pos_w2_dx;
    _pos_dx = _pos_w2_dx_fix.block<2,1>(1,0);
    _mpc_L_dx = sqrt(pow((_p_com_tot(0)-_pos_dx(0)),2)+pow((_p_com_tot(1)-_pos_dx(1)),2));

    _intersection = utilities::find_intersect(_p_com_tot,_pipe1.block<2,1>(1,0), _pos_dx, _pos_sx);
    _gamma_mpc = utilities::model_angles(_intersection, _pos_dx, _pos_sx, _p_com_tot);
    _theta_s_mpc = utilities::model_angles(_intersection, _pos_dx, _pos_sx, _pipe1.block<2,1>(1,0));
    _mpc_th_s_dx = _theta_s_mpc(0);
    _mpc_th_s_sx = _theta_s_mpc(1);

    _mpc_gamma_s = (_gamma_mpc(0)+_gamma_mpc(1))/2;
    _mpc_mu0 = tan(_mpc_gamma_s);
	_p[0] = _mpc_L_dx;
    _p[1] = _mpc_L_sx;
    _p[2] = _mpc_th_s_dx;
    _p[3] = _mpc_th_s_sx;
    _p[4] = _mpc_gamma_s;
    _p[5] = _mpc_L;
    _p[6] = _ang_mpc;
	_p[7] = _theta_d;
	writedata(Eigen::Vector3d(_p[0],_p[1],_p[2]),_mpcparamFile);
	writedata(Eigen::Vector3d(_p[3],_p[4],_p[5]),_mpcparamFile);
}

// Request new plan
void CONTROLLER::new_plan() {

	std_srvs::Empty pauseSrv;
    std_srvs::Empty unpauseSrv;
	ros::Rate r(_rate);
	Eigen::VectorXd theta_init;
	theta_init.resize(21);
	theta_init << 1.617291898068025, 0,  -1.048558443070210,  -0.685436693758534,  -0.685599771825468,  -0.187653686972250,   0.145119675834729,   0.819119912480271,   0.684238725497609,   0.673976354840312,   0.611772808716284,   0.476893764851552,   0.103672557568423,   0.145141580595989,  -0.093410021566777,   0.414690230273994,   0.041469023027346,   0.031206487025799,  -0.031206487025699,   0.146240646533758,                   0;
	Eigen::Matrix4d T1, T2, Tfinal;
	Eigen::MatrixXd Traj_i, T_i, Td_i, T_i_traj, Td_i_traj, Traj_f, Tf_traj, Tdf_traj;
	Eigen::Vector3d goal_lin;
	while (ros::ok()) {

	    // while( !_first_odom_1 && !_first_odom_2 ) usleep(0.1*1e6);
		string input;
		if(_goal_reached){
		cout<<"Want new trajectory?"<<endl;
		getline( cin, input);
		_goal_reached = false;
		_start_controller = false;
		}

		if( input == "y" && !_goal_reached) {
			_traj_ready = false;
		}
		else {
			_traj_ready = true;
		}

		if(!_traj_ready){
			// _pauseGazebo.call(pauseSrv);
			// First trajectory
			int N = 600;
			float Ti = 0;
			float Tf = 6;
			Eigen::MatrixXd Traj;
			Traj.resize(N*8,N*8);
			Eigen::Matrix4d T_s, T_g;
			T_s = dir_kin(theta_init,_S,_T_rest.block<4,4>(80,80),_N_JOINTS);
			Eigen::VectorXd theta_goal;
			theta_goal.resize(21);
			theta_goal<< 1.617291898068025,   1.570796326794900,  -1.048558443070210,  -0.685436693758534,  -0.685599771825468,  -0.187653686972250,   0.145119675834729,   0.819119912480271,   0.684238725497609,   0.673976354840312,   0.611772808716284,   0.476893764851552,   0.103672557568423,   0.145141580595989,  -0.093410021566777,   0.414690230273994,   0.041469023027346,   0.031206487025799,  -0.031206487025699,   0.146240646533758,                   0;
			T_g = dir_kin(theta_goal, _S,_T_rest.block<4,4>(80,80),_N_JOINTS);

			Traj = traj_generation(T_s,T_g,Ti,Tf,N);
			_T_traj.resize(4*N,4*N);
			_Td_traj.resize(4*N,4*N);
			for (int i=0;i<4*N;i++){
				for(int j=0;j<4*N;j++){
					_T_traj(i,j) = Traj(i,j);
					_Td_traj(i,j) = Traj(4*N+i,j+4*N);
				}
			}
			if(_setpoints_received){
			// Second traj from matlab setpoints
				cout<<"setpoint from matlab arrived"<<endl;
				cout<<"Starting computing whole trajectory..."<<endl;
				T_i_traj.resize(4*200,4*200);
				Td_i_traj.resize(4*200,4*200);
				for (int k=0;k<2;k++){
					T1.block<3,3>(0,0) = T_g.block<3,3>(0,0);
					T1.block<3,1>(0,3) << T_g(0,3), _2d_traj_setpoints(k,0), _2d_traj_setpoints(k,1);
					T2.block<3,3>(0,0) = T_g.block<3,3>(0,0);
					T2.block<3,1>(0,3) << T_g(0,3), _2d_traj_setpoints(k+1,0), _2d_traj_setpoints(k+1,1);
					Traj_i.resize(2*4*100,2*4*100);
					Traj_i = traj_generation(T1,T2,0,1,100);
					T_i.resize(4*100,4*100);
					Td_i.resize(4*100,4*100);
					for (int i=0;i<4*100;i++){
						for(int j=0;j<4*100;j++){
							T_i(i,j) = Traj_i(i,j);
							Td_i(i,j) = Traj_i(4*100+i,j+4*100);
						}
					}
					T_i_traj.block<4*100,4*100>(400*k,400*k) = T_i;
					Td_i_traj.block<4*100,4*100>(400*k,400*k) = Td_i;
				}

				for (int i=0;i<200;i++){
					T_i_traj.block<3,3>(4*i,4*i) = _T_traj.block<3,3>(4*(N-1),4*(N-1));
					Td_i_traj.block<3,3>(4*i,4*i) = _Td_traj.block<3,3>(4*(N-1),4*(N-1));
				}

				// Last segment to reach desired orientation
				Tfinal.setIdentity();
				Tfinal.block<3,3>(0,0) = utilities::RpyToMat(Vector3d(3.142, -0.025, 1.571));
				// Tfinal.block<3,1>(0,3) << 0.0528873, -0.43, -0.48;
				// Tfinal.block<3,1>(0,3) << 0.0528873, -0.43, -0.3;
				Tfinal.block<3,1>(0,3) << 0.0528873, -0.485, -0.4;

				Traj_f.resize(2*4*150,2*4*150);
				Traj_f = traj_generation(T_i_traj.block<4,4>(796,796),Tfinal,0,1.5,150);
				Tf_traj.resize(4*150,4*150);
				Tdf_traj.resize(4*150,4*150);
				for (int i=0;i<4*150;i++){
					for(int j=0;j<4*150;j++){
						Tf_traj(i,j) = Traj_f(i,j);
						Tdf_traj(i,j) = Traj_f(4*150+i,j+4*150);
					}
				}
			}
			_T_tot_traj.resize(4*1100,4*1100);
			_Td_tot_traj.resize(4*1100,4*1100);
			_T_tot_traj.block<4*600,4*600>(0,0) = _T_traj;
			_T_tot_traj.block<4*200,4*200>(4*600,4*600) = T_i_traj;
			_T_tot_traj.block<4*150,4*150>(4*800,4*800) = Tf_traj;
			_Td_tot_traj.block<4*600,4*600>(0,0) = _Td_traj;
			_Td_tot_traj.block<4*200,4*200>(4*600,4*600) = Td_i_traj;
			_Td_tot_traj.block<4*150,4*150>(4*800,4*800) = Tdf_traj;

			// Regulation
			for(int i=0;i<150;i++){
				_T_tot_traj.block<4,4>(4*(950+i),4*(950+i)) = _T_tot_traj.block<4,4>(4*950-4,4*950-4);
				_Td_tot_traj.block<4,4>(4*(950+i),4*(950+i)) = _Td_tot_traj.block<4,4>(4*950-4,4*950-4);
			}

			_traj_ready = true;
			_goal_reached = false;
			_start_controller = true;
			cout<<"+++++Planning Completed+++++"<<endl;
		}

		r.sleep();
	}
}

// Collision avoidance anc rover COM param computation (NOW USED)
void CONTROLLER::coll_avoidance_param_computation(Eigen::VectorXd theta_fb, Eigen::Matrix4d WUAV_pose, Eigen::Matrix4d* nearest_joint_pose_1, int* nearest_joint_index_1, float* min_dist_1, Eigen::Matrix4d* nearest_joint_pose_2, int* nearest_joint_index_2, float* min_dist_2, Eigen::Matrix4d* s_nearest_joint_pose_1, Eigen::Matrix4d* s_nearest_joint_pose_2, int* s_nearest_joint_index_1, int* s_nearest_joint_index_2, float* s_minimum_dist){
    Eigen:Matrix4d joint_pose_wuav, joint_pose;
	float dist_pipe1, dist_pipe2;
	*min_dist_1 = 100000;
    *min_dist_2 = 100000;

	*s_minimum_dist = 100000;
	Eigen::Matrix4d s_joint_pose_wuav,s_joint_pose,s_joint_pose_wuav_temp, s_joint_pose_temp;
    float s_actual_dist = 0.0f;

	Eigen::MatrixXd T_uav_com;
	Eigen::Vector3d num;
	T_uav_com.resize(4*_N_JOINTS,4*_N_JOINTS);
	num << 0,0,0;

    for (int i=0;i<_N_JOINTS-1;i++) {
        joint_pose_wuav = dir_kin(theta_fb, _S,_T_rest.block<4,4>(4*i,4*i),i);
        joint_pose = WUAV_pose*joint_pose_wuav;
        dist_pipe1 = sqrt( pow( (joint_pose(1,3) - _pipe1(1)),2 ) + pow( (joint_pose(2,3) - _pipe1(2)),2 ) );
        dist_pipe2 = sqrt( pow( (joint_pose(1,3) - _pipe2(1)),2 ) + pow( (joint_pose(2,3) - _pipe2(2)),2 ) );

        if (dist_pipe1 < *min_dist_1) {
            *min_dist_1  = dist_pipe1;
            *nearest_joint_index_1 = i;
            *nearest_joint_pose_1 = joint_pose;
        }

        if (dist_pipe2 < *min_dist_2) {
            *min_dist_2  = dist_pipe2;
            *nearest_joint_index_2 = i;
            *nearest_joint_pose_2 = joint_pose;
        }

		s_joint_pose_wuav = joint_pose_wuav;
        s_joint_pose = s_joint_pose_wuav;


        for (int j=i+5;j<_N_JOINTS;j++){

            s_joint_pose_wuav_temp = dir_kin(theta_fb, _S,_T_rest.block<4,4>(4*j,4*j),j);
            s_joint_pose_temp = s_joint_pose_wuav_temp;

            s_actual_dist = sqrt( pow((s_joint_pose(0,3)-s_joint_pose_temp(0,3)),2) + pow((s_joint_pose(1,3)-s_joint_pose_temp(1,3)),2) + pow((s_joint_pose(2,3)-s_joint_pose_temp(2,3)),2) );

            if (s_actual_dist < *s_minimum_dist) {
                *s_minimum_dist = s_actual_dist;
                *s_nearest_joint_index_1 = i;
                *s_nearest_joint_index_2 = j;
                *s_nearest_joint_pose_1 = s_joint_pose;
                *s_nearest_joint_pose_2 = s_joint_pose_temp;
            }
        }
		// COM arm computation
		T_uav_com.block<4,4>(i*4,i*4) = dir_kin(theta_fb, _S,_T_rest.block<4,4>(4*i,4*i),i) * _T_com.block<4,4>(i*4,i*4);
		num = num + _mass(i)*T_uav_com.block<3,1>(4*i,4*i+3);
    }
	_pos_com = num/_arm_mass;
}

// Hierarchical Inverse Dynamic Controller
void CONTROLLER::arm_invdyn_control() {

	// Acados problem init
    ocp_rover_solver_capsule *acados_ocp_capsule = ocp_rover_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = OCP_ROVER_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = ocp_rover_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("ocp_rover_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = ocp_rover_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = ocp_rover_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = ocp_rover_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = ocp_rover_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = ocp_rover_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = ocp_rover_acados_get_nlp_opts(acados_ocp_capsule);

    // Initial condition for x
    int idxbx0[NBX0];
    idxbx0[0] = 0;
    idxbx0[1] = 1;

    double lbx0[NBX0];
    double ubx0[NBX0];
    lbx0[0] = -0.0872;
    ubx0[0] = -0.0872;
    lbx0[1] = 0;
    ubx0[1] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

	// Const function
    double W_0[NX+NU][NX+NU] = {0}; 				// weight on first horizon of each iteration
    double W[NX+NU][NX+NU] = {0};   				// weight on 2 to N-1 horizon
    double W_e[NX][NX] = {0};                       // weight on N-th horizon of each iteration

    W_0[0][0] = 0.1;
    W_0[1][1] = 0.1;
    W_0[2][2] = 0.1;
    W_0[3][3] = 0.001;
    W_0[4][4] = 0.001;

    for (int i = 0; i < (NX + NU); ++i)
        W[i][i] = W_0[i][i];
    for (int i = 0; i < NX; ++i)
        W_e[i][i] = W_0[i][i];

	// Lower & Upper bounds on u and x
    double lbu[NU],ubu[NU],lbx[NX-1],ubx[NX-1];
    lbu[0] = -10;
    lbu[1] = -1;
    lbu[2] = -1;
    ubu[0] = 10;
    ubu[1] = 1;
    ubu[2] = 1;
    ubx[0] = 0.3491;
    lbx[0] = -0.3491;

    for(int i=0; i<N; i++){
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);

    // Initialization for state values
    double x_init[NX];
    x_init[0] = 0.0;
    x_init[1] = 0.0;

    // Initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;

    // Set parameters
    double x1_0, x2_0,u1_0,u2_0,u3_0;
    x1_0 = -0.0872;//*2;
    x2_0 = 0;
    u1_0 = 0;
    u2_0 = 0;
    u3_0 = 0;

	std_srvs::Empty pauseSrv;
	std_srvs::Empty unpauseSrv;

	ros::Rate r(_ctrl_rate);
	Eigen::Matrix4d T_eff, X_error_, Td_T, T_traj_act, Td_traj_act;
	Eigen::Matrix3d Identity_3_3;
	Eigen::Matrix3d R_eff;
	Eigen::Vector3d p_eff;
	Eigen::MatrixXd J_be,B_in_mat_inverse,J_wpi,J_be_prev,J_be_d, Kp, Kd, J_aug, J_aug_wpi, kd3, kp3, Identity;
	Eigen::VectorXd V_eff, X_error, V_error, X_err_temp, V_d, V_des, V_des_prev, A_des, u_v, u_q, f_ee, u_sec, u_third, f_zero,c_dyn,g_dyn;
	Eigen::VectorXd theta_init, initial_conf, e_p3, e_v3, u_fourth;
	Eigen::VectorXd f_pipe1, tau_pipe1, f_pipe2, tau_pipe2, f_j1, f_j2, tau_j1, tau_j2, Fp_ee, tau_mpc;
	float h_pipe1, h_pipe2, h_j, kp1,kp2,kj1,kj2;
	int i = 0;
	float Ts = 0.01;
	int N_ = 1100;
	int slack;
	double lambda_k, lambda_d, lambda_e, c_lambda;
	Eigen::VectorXd u_mot, F_err, F_int, F_des, u_force;
	Eigen::MatrixXd A_f, Lambda_inv, P, Identity_6_6, Ki_f, Kp_f, J_tr, J_tr_wpi, J_wp, J_be_body;
	Eigen::Vector3d temp_p, temp_pd;

	Identity_3_3.setIdentity();

	
     kp1 = 0.8;
     kp2 = 0.8;
     kj1 = 1.5;
     kj2 = 35;

	f_pipe1.resize(6);
	f_pipe2.resize(6);
	f_j1.resize(6);
	f_j2.resize(6);
	f_pipe1.setZero();
	f_pipe2.setZero();
	f_j1.setZero();
	f_j2.setZero();
	Fp_ee.resize(6);
	Fp_ee.setZero();

	tau_pipe1.resize(_N_JOINTS);
	tau_pipe2.resize(_N_JOINTS);
	tau_j1.resize(_N_JOINTS);
	tau_j2.resize(_N_JOINTS);
	tau_pipe1.setZero();
	tau_pipe2.setZero();
	tau_j1.setZero();
	tau_j2.setZero();
	tau_mpc.resize(_N_JOINTS);
	tau_mpc.setZero();

	V_eff.resize(6);
	X_err_temp.resize(6);
	X_err_temp<<0,0,0,0,0,0;
	V_error.resize(6);
	V_d.resize(6);
	V_des.resize(6);
	V_des_prev.resize(6);
	A_des.resize(6);
	u_v.resize(6);
	u_q.resize(21);
	u_sec.resize(21);
	u_third.resize(21);
	B_in_mat_inverse.resize(_N_JOINTS,_N_JOINTS);
	J_be.resize(6,_N_JOINTS);
	J_be_d.resize(6,_N_JOINTS);
	J_be_prev.resize(6,_N_JOINTS);
	J_wpi.resize(_N_JOINTS,6);
	f_ee.resize(6);
	f_ee << 0,0,0,0,0,0;
	f_zero.resize(21);
	f_zero << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
	c_dyn.resize(21);
	c_dyn << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
	g_dyn.resize(21);
	g_dyn << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

	Kp.resize(6,6);
	Kd.resize(6,6);
	Kp.setIdentity();
	Kd.setIdentity();
	Kp = 275*Kp;
	Kd = 35*Kd;
	J_aug.resize(6*2,_N_JOINTS);
	J_aug_wpi.resize(_N_JOINTS,6*2);

	kp3.resize(_N_JOINTS,_N_JOINTS);
	kd3.resize(_N_JOINTS,_N_JOINTS);
	kp3.setIdentity();
	kd3.setIdentity();
	kp3 = 2*kp3;
	kd3 = 1*kd3;

	initial_conf.resize(21);
	initial_conf << 1.61729189806803,0,-1.04855844307021,-0.685436693758534,-0.685599771825468,-0.187653686972250,0.145119675834729,0.819119912480271,0.684238725497609,0.673976354840312,0.611772808716284,0.476893764851552,0.103672557568423,0.145141580595989,-0.0934100215667767,0.414690230273994,0.0414690230273456,0.0312064870257989,-0.0312064870256987,0.146240646533758,0;
	theta_init.resize(21);
	theta_init << 1.617291898068025, 0,  -1.048558443070210,  -0.685436693758534,  -0.685599771825468,  -0.187653686972250,   0.145119675834729,   0.819119912480271,   0.684238725497609,   0.673976354840312,   0.611772808716284,   0.476893764851552,   0.103672557568423,   0.145141580595989,  -0.093410021566777,   0.414690230273994,   0.041469023027346,   0.031206487025799,  -0.031206487025699,   0.146240646533758,                   0;
	e_p3.resize(21);
	e_v3.resize(21);
	u_fourth.resize(21);
	Identity.resize(21,21);
	Identity.setIdentity();

	_arm_state_pos = theta_init;
	_arm_state_vel.setZero();
	_T_arm = dir_kin(theta_init,_S,_T_rest.block<4,4>(80,80),_N_JOINTS);
	_J_arm = diff_kin(theta_init);
	_arm_vel_mes = _J_arm*_arm_state_vel;

	Eigen::MatrixXd J_pipe1, J_pipe2, J_joint1, J_joint2;
	J_pipe1.resize(6,_N_JOINTS);
	J_pipe2.resize(6,_N_JOINTS);
	J_joint1.resize(6,_N_JOINTS);
	J_joint2.resize(6,_N_JOINTS);

	Eigen::VectorXd arm_cmd_prev;
	arm_cmd_prev.resize(21);

	u_mot.resize(21);
	A_f.resize(1,6);
	F_des.resize(6);
	F_err.resize(6);
	F_int.resize(6);
	u_force.resize(21);
	F_err.setZero();
	F_int.setZero();

	Lambda_inv.resize(6,6);
	P.resize(6,6);
	Identity_6_6.resize(6,6);
	Ki_f.resize(6,6);
	Kp_f.resize(6,6);
	J_tr.resize(21,6);
	J_be_body.resize(6,21);
	J_tr_wpi.resize(6,21);
	J_wp.resize(21,6);

	A_f.setZero();
	u_force.setZero();
	F_des << 0,0,0,3,3,3;
	Identity_6_6.setIdentity();
	Ki_f.setIdentity();
	Kp_f.setIdentity();
	Ki_f = 2.0*Ki_f;
	Kp_f = 2.5*Kp_f;
	slack = 0;

	_p[0] = _mpc_L_dx;
    _p[1] = _mpc_L_sx;
    _p[2] = _mpc_th_s_dx;
    _p[3] = _mpc_th_s_sx;
    _p[4] = _mpc_gamma_s;
    _p[5] = _mpc_L;
    _p[6] = _theta;
	_p[7] = _theta_d;
	_p[8] = u1_0;
    _p[9] = u2_0;
    _p[10] = u3_0;
	writedata(Eigen::Vector3d(_p[0],_p[1],_p[2]),_mpcparamFile);
	writedata(Eigen::Vector3d(_p[3],_p[4],_p[5]),_mpcparamFile);
	
    for (int ii = 0; ii <= N; ii++)
    {
        ocp_rover_acados_update_params(acados_ocp_capsule, ii, _p, NP);
    }


    // Prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];
    double x_next[NX],x_act[NX],u_opt[NU];
    Eigen::MatrixXd x_sol, u_sol;
    int max_iter = 800;
    x_sol.resize(2,max_iter);
    u_sol.resize(3,max_iter);
    double act_state[NX];

    // Solve ocp in loop
    int rti_phase = 0;
    int ii = 0;
	Eigen::VectorXd ex_time_1, ex_time_2, ex_time_3, ex_time_4, ex_time_5;
	ex_time_1.resize(N_+4000);
	ex_time_2.resize(N_+4000);
	ex_time_3.resize(N_+4000);
	ex_time_4.resize(N_+4000);
	ex_time_5.resize(N_+4000);

	while( !_traj_ready ) usleep(0.1*1e6);

	while ( ros::ok() && i < N_+4000 ) {

		if(_start_controller){
			if(i==1){
			_start=clock();
			start_ = ros::WallTime::now();
			}
			start_1 = ros::WallTime::now();
			//compute dir and diff kin and collision avoidance params
			if (i>1) {
			feedback();
			}
			// Direct kinematic to compute effective pose of the ee from the sensed data
			T_eff = _T_arm;
			R_eff = T_eff.block<3,3>(0,0);
			p_eff = T_eff.block<3,1>(0,3);
			// Differential kinematic to compute the actual velocity of the ee from sensed data
			J_be = _J_arm;
			V_eff = _arm_vel_mes;

			_f_ext << _F_ee(3),_F_ee(4),_F_ee(5);
			writedata(_f_ext,_f_extFile);

			// switch modalities
			P = Identity_6_6;
			c_lambda = 0.05;
			
			if((i>900 && abs(X_error(5)))>0.03 && abs(X_error(5))<=0.1){
				lambda_d = 0.5*(1+cos((X_error(5)-0.02)/(0.2-0.02)*M_PI));
			}
			else if(i>900 && abs(X_error(5))<=0.03){
				lambda_d = 1;
			}
			else{
				lambda_d = 0;
				// _F_ee.setZero();
				// _f_ext.setZero();
			}
			lambda_e = 1;
			if(F_des.norm()>0){
				lambda_k = c_lambda*lambda_d*lambda_e+(1-c_lambda)*lambda_k;
			}
			else{
				lambda_k = 0;
			}
			P(5,5) = 1 - lambda_k;
			end_1 = ros::WallTime::now();
			ex_time_1(i) = (end_1 - start_1).toNSec() * 1e-6;
			// -------------------------------------- MPC FORMULATION -----------------------------------------
			start_2 = ros::WallTime::now();
			act_state[0] =_ang_mpc;
			act_state[1] =_theta_d;
			writedata(Eigen::Vector2d(_theta + _dist, act_state[1]),_xmpcFile);
			x1_0 = act_state[0];
			x2_0 = act_state[1];
			x_init[0] = x1_0;
			x_init[1] = x2_0;

			lbx0[0] = x1_0;
			ubx0[0] = x1_0;
			lbx0[1] = x2_0;
			ubx0[1] = x2_0;

			// Needed to close the loop here
			ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
			ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
			ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

			// Initialize solution
			for (int i = 0; i <= nlp_dims->N; i++)
			{
				ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
				ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
				ocp_rover_acados_update_params(acados_ocp_capsule, i, _p, NP);
			}
			ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
			// Solve Optimization Problem
			status = ocp_rover_acados_solve(acados_ocp_capsule);
			ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
			min_time = MIN(elapsed_time, min_time);
			// Get actual optimal input and new state
			ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "x", x_act);
			ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", x_next);
			ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u_opt);

			u1_0 = u_opt[0];
			u2_0 = u_opt[1];
			u3_0 = u_opt[2];
			_p[8] = u1_0;
			_p[9] = u2_0;
			_p[10] = u3_0;
			u0[0] = u1_0;
			u0[1] = u2_0;
			u0[2] = u3_0;

			_F_mpc(4) = -u1_0;
			_wheel_cmd[0] = -u2_0/2;
			_wheel_cmd[1] = -u3_0/2;
			_wheel_cmd[2] = -u2_0/2;
			_wheel_cmd[3] = -u3_0/2;
			writedata(Eigen::Vector3d(u1_0,u2_0,u3_0),_umpcFile);

			// Referring _F_mpc to the base frame of the manipulator
			Fp_ee = utilities::Ad_f(_T_u_com.inverse()).transpose()*_F_mpc;

			// Computing torques with the statics theory
			tau_mpc = J_be.transpose()*P*Fp_ee;
			end_2 = ros::WallTime::now();
			ex_time_2(i) = (end_2 - start_2).toNSec() * 1e-6;
			// ------------------------------------Trajectory tracking task------------------------------------
			start_3 = ros::WallTime::now();
			if(i<N_){
				T_traj_act = _T_tot_traj.block<4,4>(4*i,4*i);
				Td_traj_act = _Td_tot_traj.block<4,4>(4*i,4*i);
			}
			X_error_ = (T_eff.inverse()*T_traj_act).log();                                 //4x4
			X_err_temp << X_error_(2,1), X_error_(0,2), X_error_(1,0), X_error_.block<3,1>(0,3);
			X_error = utilities::Ad_f(T_eff) * X_err_temp;     								 					//6x1
			Td_T = (T_traj_act).inverse()*Td_traj_act;				//4x4
			V_d << Td_T(2,1), Td_T(0,2), Td_T(1,0), Td_T.block<3,1>(0,3);    									//6x1
			V_des = utilities::Ad_f(T_eff) * (utilities::Ad_f(T_eff.inverse() * T_traj_act) * V_d);
			V_error = V_des - V_eff;																			//6x1

			if (i == 1){
				V_des_prev = V_des;
			}

			A_des = (V_des - V_des_prev)/Ts;
			V_des_prev = V_des;

			// Computation of the inverse Inertia Matrix
			B_in_mat_inverse = (inertia_matrix(_arm_state_pos)).inverse();

			// Computation of the Dinamically consistent pseudoinverse space Jacobian
			J_wpi = (B_in_mat_inverse*J_be.transpose())*(J_be*B_in_mat_inverse*J_be.transpose()).inverse(); //nx6

			// Initialization for numerical differentiation
			if (i == 1){
				J_be_prev = J_be;
			}

			// Numerical differentiation of the space Jacobian
			J_be_d = (J_be - J_be_prev)/Ts;
			J_be_prev = J_be;

			// Virtual acceleration cartesian space
			u_v = (A_des + Kd*V_error + Kp*X_error);

			writedata(Eigen::Vector3d(X_error(0),X_error(1),X_error(2)),_erpyFile);
			writedata(Eigen::Vector3d(X_error(3),X_error(4),X_error(5)),_epFile);
			writedata(Eigen::Vector3d(V_error(0),V_error(1),V_error(2)),_ewFile);
			writedata(Eigen::Vector3d(V_error(3),V_error(4),V_error(5)),_evFile);
			end_3 = ros::WallTime::now();
			ex_time_3(i) = (end_3 - start_3).toNSec() * 1e-6;
			// ------------------------------------Obstacle avoidance task------------------------------------
			start_4 = ros::WallTime::now();
			// Augmented Jacobian composed by the other 2 task Jacobians
			J_aug.block<6,21>(0,0) = J_be;
			J_aug.block<6,21>(6,0) = (B_in_mat_inverse*J_be.transpose()).completeOrthogonalDecomposition().pseudoInverse();

			// Computation of the Dinamically consistent pseudoinverse augmented Jacobian
			J_aug_wpi = (B_in_mat_inverse*J_aug.transpose())*(J_aug*B_in_mat_inverse*J_aug.transpose()).inverse();
			if(i>10){
			J_pipe1.setZero();
			J_pipe2.setZero();
			J_joint1.setZero();
			J_joint2.setZero();
			J_pipe1.block(0,0,6,_nearest_j_index_1_coll) = J_be.block(0,0,6,_nearest_j_index_1_coll);
			J_pipe2.block(0,0,6,_nearest_j_index_2_coll) = J_be.block(0,0,6,_nearest_j_index_2_coll);
			J_joint1.block(0,0,6,_nearest_j_index_1_self) = J_be.block(0,0,6,_nearest_j_index_1_self);
			J_joint2.block(0,0,6,_nearest_j_index_2_self) = J_be.block(0,0,6,_nearest_j_index_2_self);

			if(_min_dist_1_coll < _d_safety + _d0){
				h_pipe1 = kp1 * (exp(kp2*(_d_safety+ _d0 - _min_dist_1_coll))-1);
				_pipe1 << _nearest_j_pose_1_coll.block<1,1>(0,3), _pipe1(1), _pipe1(2);
				f_pipe1.tail(3) = h_pipe1 / _min_dist_1_coll *(_nearest_j_pose_1_coll.block<3,1>(0,3) - _pipe1);
				f_pipe1 = utilities::Ad_f(_T_uav).transpose()*f_pipe1;
				tau_pipe1 = J_pipe1.transpose()*P*f_pipe1;
			}
			else{
				f_pipe1.setZero();
				tau_pipe1.setZero();
			}

			if(_min_dist_2_coll < _d_safety + _d0){
				h_pipe2 = kp1 * (exp(kp2*(_d_safety+ _d0 - _min_dist_2_coll))-1);
				_pipe2 << _nearest_j_pose_2_coll.block<1,1>(0,3), _pipe2(1), _pipe2(2);
				f_pipe2.tail(3) = h_pipe2 / _min_dist_2_coll *(_nearest_j_pose_2_coll.block<3,1>(0,3) - _pipe2);
				f_pipe2 = utilities::Ad_f(_T_uav).transpose()*f_pipe2;
				tau_pipe2 = J_pipe2.transpose()*P*f_pipe2;
			}
			else{
				f_pipe2.setZero();
				tau_pipe2.setZero();
			}
			if(_min_dist_self < 0.115){ 
				h_j = kj1 * (1/(1+(exp(kj2*(_min_dist_self-_safety-0.055)+0.847)))-0.3);
				f_j1.tail(3) = h_j / _min_dist_self * (_nearest_j_pose_1_self.block<3,1>(0,3)- _nearest_j_pose_2_self.block<3,1>(0,3));
                f_j2 = -f_j1;
				tau_j1 = J_joint1.transpose()*P*f_j1;
				tau_j2 = J_joint2.transpose()*P*f_j2;
			}
			else{
				f_j1.setZero();
				tau_j1.setZero();
				f_j2.setZero();
				tau_j2.setZero();
			}
			u_sec = (B_in_mat_inverse*(tau_pipe1+tau_pipe2+tau_j1+tau_j2));
			u_third = (Identity- J_aug_wpi*J_aug)*u_sec;
			}
			end_4 = ros::WallTime::now();
			ex_time_4(i) = (end_4 - start_4).toNSec() * 1e-6;
			
			// ----------------------------Final aux input-------------------------------
			start_5 = ros::WallTime::now();
			if (i<10) {
				u_q = J_wpi*(u_v - J_be_d*_arm_state_vel) + (Identity- J_wpi*J_be)*B_in_mat_inverse*tau_mpc;
			}
			else {
				u_q = J_wpi*(P*u_v - J_be_d*_arm_state_vel) + (Identity- J_wpi*J_be)*B_in_mat_inverse*tau_mpc + u_third;
			}

			u_mot = CONTROLLER::recursive_inv_dyn(_arm_state_pos, _arm_state_vel, u_q, _gravity, f_ee);
			for(int i=0;i<21;i++){
				if(u_mot(i)>5){
					u_mot(i)=5;
				}
				if(u_mot(i)<-5){
					u_mot(i)=-5;
				}
			}
			writedata(u_mot,_tauFile);
			if (i> 900+1400 ) { 
				F_des << 0,0,0,6,6,6;
			}	
			if (i> 900+1400+1400 ) {
				F_des << 0,0,0,9,9,9;
			}																					
			F_err = - F_des + _F_ee;																						
			F_int = F_int + F_err*Ts;																					

			J_be_body = utilities::Ad_f(T_eff.inverse())*J_be; //Body Jacobian
			Lambda_inv = J_be_body*(inertia_matrix(_arm_state_pos)).inverse()*J_be_body.transpose(); 						//6x6
			
			_arm_cmd = u_mot + J_be_body.transpose()*(Lambda_inv.inverse())*(Identity_6_6-P)*(-F_des+Kp_f*F_err+Ki_f*F_int);
			end_5 = ros::WallTime::now();
			ex_time_5(i) = (end_5 - start_5).toNSec() * 1e-6;
			i++;
			_unpauseGazebo.call(unpauseSrv);
		    r.sleep();
		}
		else{
			_pauseGazebo.call(pauseSrv);
			r.sleep();
		}
	}
	_pauseGazebo.call(pauseSrv);
	if(i==N_+4000){
		_end=clock();
		end_ = ros::WallTime::now();
		_time=((double)(_end-_start))/CLOCKS_PER_SEC;
		double execution_time = (end_ - start_).toNSec() * 1e-6;
		ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
		
	}

	printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("ocp_rover_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("ocp_rover_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    ocp_rover_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    // free solver
    status = ocp_rover_acados_free(acados_ocp_capsule);
    if (status) {
        printf("ocp_rover_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = ocp_rover_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("ocp_rover_acados_free_capsule() returned status %d. \n", status);
    }
}

// Dir Kin
Eigen::Matrix4d CONTROLLER::dir_kin(Eigen::VectorXd theta_fb, Eigen::MatrixXd S_axis, Eigen::MatrixXd rest_M, int num_j){
	Eigen::MatrixXd T;
	T.resize(4,4);
	T=rest_M;
	Eigen::Matrix4d S_bracket;
	for (int i=num_j-1;i>=0;i--){
		S_bracket.block<3,3>(0,0) = utilities::skew(S_axis.block<1,3>(i,0));
		S_bracket.block<3,1>(0,3) = _S.block<1,3>(i,3);
		S_bracket.block<1,4>(3,0) <<0,0,0,0;
		T = ((S_bracket*theta_fb[i]).exp())*T;
	}
	return T;
}

// Diff Kin (Space Jacobian)
Eigen::MatrixXd CONTROLLER::diff_kin(Eigen::VectorXd theta_fb){
    Eigen::MatrixXd J;
	Eigen::Matrix3d R_temp, R_curr;
	Eigen::Vector3d omega_vec, omega, v, q, q_disp, base_disp;

	R_curr.setIdentity();
	base_disp << 0,0,0;
	J.resize(6,_N_JOINTS);
	J.setIdentity();

	for(int i=0;i<_N_JOINTS;i++){
		if (_rot_vec[i]==0){
			R_temp.setIdentity();
		}
		else if (_rot_vec[i]==1){
			R_temp = utilities::rotx(theta_fb[i-1]);
		}
		else if (_rot_vec[i]==2){
			R_temp = utilities::roty(theta_fb[i-1]);
		}
		else if (_rot_vec[i]==3){
		    R_temp = utilities::rotz(theta_fb[i-1]);
		}

		R_curr = R_curr * R_temp;
		q_disp << _x_distances(i), _y_distances(i), _z_distances(i);
		q = base_disp + R_curr*q_disp;
		base_disp = q;

		if (_curr_rot[i] == 1){
			omega_vec << 1,0,0;
		}
		else if (_curr_rot[i] == 2){
			omega_vec << 0,1,0;
		}
		else if (_curr_rot[i] == 3){
			omega_vec << 0,0,1;
		}
        omega = R_curr * omega_vec;
		v = -omega.cross(q);

        J.block<3,1>(0,i) << omega;
        J.block<3,1>(3,i) << v;
	}
	return J;
	//Body Jacobian: J_b=Ad_f(T_eff.inverse())*J_s
}

// Inv Dyn
Eigen::VectorXd CONTROLLER::recursive_inv_dyn(Eigen::VectorXd theta_fb, Eigen::VectorXd theta_d_fb, Eigen::VectorXd theta_dd_fb, Eigen::Vector3d gravity, Eigen::VectorXd f_ee){

	// param definition
	Eigen::VectorXd V_prev,Vd_prev, F_prev, tau;
	Vd_prev.resize(6);
	Vd_prev<<0,0,0,-gravity;
	V_prev.resize(6);
	V_prev<<0,0,0,0,0,0;

	F_prev.resize(6);
	F_prev = f_ee;
	tau.resize(_N_JOINTS);
	tau<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;


	Eigen::Matrix4d M_prev, M_mutual, T_mutual;
	M_prev.setIdentity();
    Eigen::MatrixXd A_bracket, A_i, G_i, T, G, A, V, Vd, F;
	A_i.resize(6,1);
	G_i.resize(6,6);
	A_bracket.resize(4,4);
	A_bracket.setZero();
	T.resize(4*(_N_JOINTS+1),4*(_N_JOINTS+1));
	G.resize(6*_N_JOINTS,6*_N_JOINTS);
	A.resize(6*_N_JOINTS,_N_JOINTS);
	V.resize(6,_N_JOINTS);
	Vd.resize(6,_N_JOINTS);
	F.resize(6,_N_JOINTS);
	F.setZero();

	Eigen::MatrixXd d_base;

	for (int i=0;i<_N_JOINTS;i++){
		M_mutual = _M_save.block<4,4>(i*4,i*4);															//4x4
		A_i = _A.block<6,1>(0,i);
		A_bracket.block<3,3>(0,0) = utilities::skew(A_i.block<3,1>(0,0)); 								//4x4
		A_bracket.block<3,1>(0,3) = A_i.block<3,1>(3,0);
        T_mutual = (-(A_bracket*theta_fb(i))).exp()*M_mutual.inverse();									//4x4

		T.block<4,4>(i*4,i*4) = T_mutual;																//4(n+1)x4(n+1)
		V.block<6,1>(0,i) = utilities::Ad_f(T_mutual)*V_prev + A_i*theta_d_fb[i];						//6xn
		V_prev = V.block<6,1>(0,i);																		//6x1

        Vd.block<6,1>(0,i) = utilities::Ad_f(T_mutual)*Vd_prev + utilities::ad_f_(V_prev)*A_i*theta_d_fb[i] + A_i*theta_dd_fb[i];
		Vd_prev = Vd.block<6,1>(0,i);																	//6x1
	}
    (T.block<4,4>(4*_N_JOINTS,4*_N_JOINTS)).setIdentity();

	//backward cycle
	for (int i=_N_JOINTS-1;i>=0;i--){

		F.block<6,1>(0,i) =  utilities::Ad_f(T.block<4,4>((i+1)*4,(i+1)*4)).transpose()*F_prev
											+ _G.block<6,6>(i*6,i*6)*Vd.block<6,1>(0,i)
										    	    - utilities::ad_f_(V.block<6,1>(0,i)).transpose()*(_G.block<6,6>(i*6,i*6)*V.block<6,1>(0,i));
		F_prev = F.block<6,1>(0,i);																		//6x1
        tau[i] = F_prev.transpose()*_A.block<6,1>(0,i);												//1x6*6x1
	}
	return tau;
}

// Inertia Matrix computation
Eigen::MatrixXd CONTROLLER::inertia_matrix(Eigen::VectorXd theta_fb){
    Eigen::MatrixXd M;
	Eigen::VectorXd fake_acc, zero_vel, f_ee;
	Eigen::Vector3d gravity;
	f_ee.resize(6);
	f_ee<<0,0,0,0,0,0;
	gravity<<0,0,0;
	fake_acc.resize(_N_JOINTS);
	M.resize(_N_JOINTS,_N_JOINTS);
	M.setZero();
	zero_vel.resize(_N_JOINTS);
	zero_vel<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

    for (int i=0;i<_N_JOINTS;i++){
        fake_acc << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
        fake_acc[i] = 1;
        M.block<21,1>(0,i) = recursive_inv_dyn(theta_fb, zero_vel, fake_acc, gravity, f_ee);
    }
	return M;
}

// Quintic Polynomial Planner
Eigen::MatrixXd CONTROLLER::traj_generation(Eigen::Matrix4d T_s, Eigen::Matrix4d T_g, float ti, float tf, int N){

	// Starting and goal position and orientation
    Eigen::Matrix3d Rs = T_s.block<3,3>(0,0);
    Eigen::Vector3d ps = T_s.block<3,1>(0,3);

    Eigen::Matrix3d Rg = T_g.block<3,3>(0,0);
    Eigen::Vector3d pg = T_g.block<3,1>(0,3);

	float t, s, sd, sdd, th, thd, thdd;
	Eigen::Vector3d p, pd, pdd;
	Eigen::Vector3d omega_i, omegad_i, omega_e, omegad_e;
	Eigen::Matrix3d Re, Re_dot, Ri;
	Eigen::MatrixXd Te_dot, Te, Traj;

	Te.resize(4*N,4*N);
	Te.setIdentity();
	Te_dot.resize(4*N,4*N);
	Te_dot.setZero();
	Traj.resize(8*N,8*N);
	Traj.setZero();

    // Time interval
    float T = tf - ti;

    // Coefficients of the fifth order polinomial
    float a1, a2, a3;
	a1 = 10/pow(T,3);
    a2 = -15/pow(T,4);
    a3 = 6/pow(T,5);

    // Displacements parameters among Rs and Rg
    Eigen::Matrix3d Rsg;
	Rsg = Rs.transpose()*Rg;
    // These two equations can be used instead of rotm2axang
	float th_f;
	Eigen::Vector3d r, r_t;
    th_f = acos((Rsg(0,0) + Rsg(1,1) + Rsg(2,2) -1)/2);
	r_t << Rsg(2,1) - Rsg(1,2), Rsg(0,2) - Rsg(2,0), Rsg(1,0) - Rsg(0,1);
    r = 1/(2*sin(th_f)) * r_t;

    for (int i=0;i<N;i++){
        t = T/(N-1)*(i);

        // Arc length for the rectilinear path
        s = a1*pow(t, 3.0) + a2*pow(t, 4.0) + a3*pow(t, 5.0);
        sd = 3*a1*pow(t, 2.0) + 4*a2*pow(t, 3.0) + 5*a3*pow(t, 4.0);
        sdd = 6*a1*t + 12*a2*pow(t, 2.0) + 20*a3*pow(t, 3.0);

        // Rectilinear path
        p = ps + s*(pg-ps);
        pd = sd*(pg-ps);
        pdd = sdd*(pg-ps);

        // Angle between Rs and Rg
        th = a1*pow(t, 3.0)*th_f + a2*pow(t, 4.0)*th_f + a3*pow(t, 5.0)*th_f;
        thd = 3*a1*pow(t, 2.0)*th_f + 4*a2*pow(t, 3.0)*th_f + 5*a3*pow(t, 4.0)*th_f;
        thdd = 6*a1*t*th_f + 12*a2*pow(t, 2.0)*th_f + 20*a3*pow(t, 3.0)*th_f;

        // Angular velocity and acceleration of the "middle frame" R^i
        omega_i = thd*r;
        omegad_i = thdd*r;

        // Angular path
		Ri << pow(r[0],2)*(1-cos(th))+cos(th), r[0]*r[1]*(1-cos(th))-r[2]*sin(th), r[0]*r[2]*(1-cos(th))+r[1]*sin(th), r[0]*r[1]*(1-cos(th))+r[2]*sin(th), pow(r[1],2)*(1-cos(th))+cos(th), r[1]*r[2]*(1-cos(th))-r[0]*sin(th), r[0]*r[2]*(1-cos(th))-r[1]*sin(th), r[1]*r[2]*(1-cos(th))+r[0]*sin(th), pow(r[2],2)*(1-cos(th))+cos(th);
        Re = Rs*Ri;
        omega_e = Rs*omega_i;
        omegad_e = Rs*omegad_i;

        // Te_dot and Te
        Re_dot = utilities::skew(omega_e)*Re;

        Te_dot.block<3,3>(4*i,4*i)=Re_dot;
		Te_dot.block<3,1>(4*i,3+4*i) = pd;

		Te.block<3,3>(4*i,4*i)=Re;
		Te.block<3,1>(4*i,4*i+3) = p;
	}

	for (int k=0;k<4*N;k++){
			for(int j=0;j<4*N;j++){
				Traj(k,j) = Te(k,j);
				Traj(4*N+k,4*N+j) = Te_dot(k,j);
			}
		}
	return Traj;
}

// ----- ROS PUBLISHERS -----

// Arm command ROS Publisher
void CONTROLLER::cmd_wheel_ctrl() {

	std_srvs::Empty pauseSrv;
    std_srvs::Empty unpauseSrv;

	ros::Rate r(_rate);
	std_msgs::Float64 ctrl_input_1, ctrl_input_2 ,ctrl_input_3, ctrl_input_4;
    int debug = 0;
    while( ros::ok() ) {

        ctrl_input_1.data = _wheel_cmd[0];
        ctrl_input_2.data = _wheel_cmd[1];
        ctrl_input_3.data = _wheel_cmd[2];
        ctrl_input_4.data = _wheel_cmd[3];

        _cmd_wheel1_pub.publish(ctrl_input_1);
        _cmd_wheel2_pub.publish(ctrl_input_2);
        _cmd_wheel3_pub.publish(ctrl_input_3);
        _cmd_wheel4_pub.publish(ctrl_input_4);

    	r.sleep();
    }

}

// Wheels command ROS Publisher
void CONTROLLER::cmd_arm_ctrl() {

	ros::Rate r(_rate);
	std_msgs::Float64 ctrl_input_1, ctrl_input_2, ctrl_input_3, ctrl_input_4, ctrl_input_5, ctrl_input_6;
	std_msgs::Float64 ctrl_input_7, ctrl_input_8, ctrl_input_9, ctrl_input_10, ctrl_input_11, ctrl_input_12;
	std_msgs::Float64 ctrl_input_13, ctrl_input_14, ctrl_input_15, ctrl_input_16, ctrl_input_17, ctrl_input_18;
	std_msgs::Float64 ctrl_input_19, ctrl_input_20, ctrl_input_21;

    while( ros::ok() ) {

        ctrl_input_1.data = _arm_cmd[0];
        ctrl_input_2.data = _arm_cmd[1];
        ctrl_input_3.data = _arm_cmd[2];
        ctrl_input_4.data = _arm_cmd[3];
        ctrl_input_5.data = _arm_cmd[4];
        ctrl_input_6.data = _arm_cmd[5];
		ctrl_input_7.data = _arm_cmd[6];
        ctrl_input_8.data = _arm_cmd[7];
        ctrl_input_9.data = _arm_cmd[8];
        ctrl_input_10.data = _arm_cmd[9];
        ctrl_input_11.data = _arm_cmd[10];
        ctrl_input_12.data = _arm_cmd[11];
        ctrl_input_13.data = _arm_cmd[12];
        ctrl_input_14.data = _arm_cmd[13];
        ctrl_input_15.data = _arm_cmd[14];
        ctrl_input_16.data = _arm_cmd[15];
		ctrl_input_17.data = _arm_cmd[16];
        ctrl_input_18.data = _arm_cmd[17];
        ctrl_input_19.data = _arm_cmd[18];
        ctrl_input_20.data = _arm_cmd[19];
        ctrl_input_21.data = _arm_cmd[20];

        _cmd_tau1_pub.publish(ctrl_input_1);
        _cmd_tau2_pub.publish(ctrl_input_2);
        _cmd_tau3_pub.publish(ctrl_input_3);
        _cmd_tau4_pub.publish(ctrl_input_4);
        _cmd_tau5_pub.publish(ctrl_input_5);
        _cmd_tau6_pub.publish(ctrl_input_6);
        _cmd_tau7_pub.publish(ctrl_input_7);
        _cmd_tau8_pub.publish(ctrl_input_8);
        _cmd_tau9_pub.publish(ctrl_input_9);
        _cmd_tau10_pub.publish(ctrl_input_10);
        _cmd_tau11_pub.publish(ctrl_input_11);
        _cmd_tau12_pub.publish(ctrl_input_12);
        _cmd_tau13_pub.publish(ctrl_input_13);
        _cmd_tau14_pub.publish(ctrl_input_14);
        _cmd_tau15_pub.publish(ctrl_input_15);
        _cmd_tau16_pub.publish(ctrl_input_16);
        _cmd_tau17_pub.publish(ctrl_input_17);
        _cmd_tau18_pub.publish(ctrl_input_18);
        _cmd_tau19_pub.publish(ctrl_input_19);
        _cmd_tau20_pub.publish(ctrl_input_20);
        _cmd_tau21_pub.publish(ctrl_input_21);

    	r.sleep();
    }

}

// Multithread
void CONTROLLER::run() {
	boost::thread cmd_input_t( &CONTROLLER::new_plan, this );
	boost::thread ctrl_law_t( &CONTROLLER::arm_invdyn_control, this );
	boost::thread cmd_vel_ctrl_t( &CONTROLLER::cmd_wheel_ctrl, this );
	boost::thread cmd_arm_ctrl_t( &CONTROLLER::cmd_arm_ctrl, this );

	ros::spin();
}

//MAIN
int main(int argc, char** argv ) {

	ros::init(argc, argv, "cmd_vel_ctrl");

	CONTROLLER Prisma_Snake;
	Prisma_Snake.run();

	return 0;
}
