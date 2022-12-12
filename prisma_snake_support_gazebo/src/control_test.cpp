#include "ros/ros.h"
#include <boost/thread.hpp>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>


#include "../snake_lib/dirkin.hpp"

#define JOINTS 									23

#define JFRAME_ID 							0
#define JMOTOR_ID 							1
#define J1_ID 									2
#define J2_ID 									3
#define J3_ID 									4
#define J4_ID 									5
#define J5_ID 									6
#define J6_ID 									7
#define J7_ID 									8
#define J8_ID 									9
#define J9_ID 									10
#define J10_ID 									11
#define J11_ID 									12
#define J12_ID 									13
#define J13_ID 									14
#define J14_ID 									15
#define J15_ID 									16
#define J16_ID 									17
#define J17_ID 									18
#define J18_ID 									19
#define J19_ID 									20
#define J20_ID 									21


#define JFRAME_NAME 							"JFrame"
#define JMOTOR_NAME 							"JMotor"
#define J1_NAME 									"J1"
#define J2_NAME 									"J2"
#define J3_NAME 									"J3"
#define J4_NAME 									"J4"
#define J5_NAME 									"J5"
#define J6_NAME 									"J6"
#define J7_NAME 									"J7"
#define J8_NAME 									"J8"
#define J9_NAME 									"J9"
#define J10_NAME 									"J10"
#define J11_NAME 									"J11"
#define J12_NAME 									"J12"
#define J13_NAME 									"J13"
#define J14_NAME 									"J14"
#define J15_NAME 									"J15"
#define J16_NAME 									"J16"
#define J17_NAME 									"J17"
#define J18_NAME 									"J18"
#define J19_NAME 									"J19"
#define J20_NAME 									"J20"


using namespace std;

class PRISMA_SNAKE_CTEST {
	public:
		PRISMA_SNAKE_CTEST();
		void run();
		void j_state_cb( sensor_msgs::JointState j_state );
		void ctrl();

	private:
		ros::NodeHandle _nh;
		ros::Subscriber _j_sub;
		ros::Publisher _j_pub[ JOINTS ];

		vector< float > _j_pos;
		vector< float > _j_min;
		vector< float > _j_max;

		bool _first_js;

};

PRISMA_SNAKE_CTEST::PRISMA_SNAKE_CTEST() {

	_j_sub = _nh.subscribe("/prisma_snake/joint_states", 0, &PRISMA_SNAKE_CTEST::j_state_cb, this);
	_j_pos.resize( JOINTS );
	_j_min.resize( JOINTS );
	_j_max.resize( JOINTS );



	_j_pub[JFRAME_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/jointFrame_position_controller/command", 0);
	_j_pub[JMOTOR_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/jointMotor_position_controller/command", 0);
	_j_pub[J1_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint1_position_controller/command", 0);
	_j_pub[J2_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint2_position_controller/command", 0);
	_j_pub[J3_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint3_position_controller/command", 0);
	_j_pub[J4_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint4_position_controller/command", 0);
	_j_pub[J5_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint5_position_controller/command", 0);
	_j_pub[J6_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint6_position_controller/command", 0);
	_j_pub[J7_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint7_position_controller/command", 0);
	_j_pub[J8_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint8_position_controller/command", 0);
	_j_pub[J9_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint9_position_controller/command", 0);
	_j_pub[J10_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint10_position_controller/command", 0);
	_j_pub[J11_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint11_position_controller/command", 0);
	_j_pub[J12_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint12_position_controller/command", 0);
	_j_pub[J13_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint13_position_controller/command", 0);
	_j_pub[J14_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint14_position_controller/command", 0);
	_j_pub[J15_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint15_position_controller/command", 0);
	_j_pub[J16_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint16_position_controller/command", 0);
	_j_pub[J17_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint17_position_controller/command", 0);
	_j_pub[J18_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint18_position_controller/command", 0);
	_j_pub[J19_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint19_position_controller/command", 0);
	_j_pub[J20_ID] = _nh.advertise<std_msgs::Float64>("/prisma_snake/joint20_position_controller/command", 0);




	_j_min[JFRAME_ID] = -10000;
	_j_min[JMOTOR_ID] = -1.5707963267949;
	for(int i=JMOTOR_ID+1; i<JOINTS-1; i++ ) {
		_j_min[i] = -1.0471975511966;
	}

	_j_max[JFRAME_ID] = 10000;
	_j_max[JMOTOR_ID] = 1.5707963267949;
	for(int i=JMOTOR_ID+1; i<JOINTS-1; i++ ) {
		_j_max[i] = 1.0471975511966;
	}

	_j_min[J20_ID] = -0.7;
	_j_max[J20_ID] = 0.7;

	_first_js = false;
}



void PRISMA_SNAKE_CTEST::j_state_cb( sensor_msgs::JointState j_state ) {
	for(int i=0; i<j_state.name.size(); i++ ) {
		if( j_state.name[i] == JFRAME_NAME ) {
			_j_pos[JFRAME_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == JMOTOR_NAME ) {
			_j_pos[JMOTOR_ID] = j_state.position[i];
		}

		else if( j_state.name[i] == J1_NAME ) {
			_j_pos[J1_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J2_NAME ) {
			_j_pos[J2_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J3_NAME ) {
			_j_pos[J3_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J4_NAME ) {
			_j_pos[J4_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J5_NAME ) {
			_j_pos[J5_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J6_NAME ) {
			_j_pos[J6_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J7_NAME ) {
			_j_pos[J7_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J8_NAME ) {
			_j_pos[J8_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J9_NAME ) {
			_j_pos[J9_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J10_NAME ) {
			_j_pos[J10_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J11_NAME ) {
			_j_pos[J11_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J12_NAME ) {
			_j_pos[J12_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J13_NAME ) {
			_j_pos[J13_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J14_NAME ) {
			_j_pos[J14_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J15_NAME ) {
			_j_pos[J15_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J16_NAME ) {
			_j_pos[J16_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J17_NAME ) {
			_j_pos[J17_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J18_NAME ) {
			_j_pos[J18_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J19_NAME ) {
			_j_pos[J19_ID] = j_state.position[i];
		}
		else if( j_state.name[i] == J20_NAME ) {
			_j_pos[J20_ID] = j_state.position[i];
		}
	}

	_first_js = true;
}

void PRISMA_SNAKE_CTEST::ctrl() {

	while( !_first_js ) {
		usleep(0.5*1e6);

	}
	
	for(int i=0; i<JOINTS; i++) {



		std_msgs::Float64 cmd;
		cmd.data = ( _j_min[i] < -1.57 ) ? -1.57 : _j_min[i];

		while( fabs ( _j_pos[i] - cmd.data ) > 0.06 ) {			

			_j_pub[i].publish( cmd );
			usleep(0.01*1e6);
		} 

		cmd.data = ( _j_max[i] > 1.57 ) ? 1.57 : _j_max[i];
		while( fabs ( _j_pos[i] - cmd.data ) > 0.06 ) {			
			_j_pub[i].publish( cmd );
			usleep(0.01*1e6);
		} 

		cmd.data = 0.0; 
		while( fabs ( _j_pos[i] ) > 0.06 ) {			
			_j_pub[i].publish( cmd );
			usleep(0.01*1e6);
		} 
		

	}
	

	for(int i=2; i<JOINTS; i++) {
		std_msgs::Float64 cmd;
		cmd.data = _j_min[i];

		while( fabs ( _j_pos[i] - cmd.data ) > 0.06 ) {			

			_j_pub[i].publish( cmd );
			usleep(0.01*1e6);
		} 

	}

}

void PRISMA_SNAKE_CTEST::run() {
	boost::thread ctrl_t( &PRISMA_SNAKE_CTEST::ctrl, this);
	ros::spin();
}

int main(int argc, char** argv) {

	/*
	ros::init( argc, argv, "prisma_snake_ctest");

	PRISMA_SNAKE_CTEST psc;

	psc.run();
	*/

	
	//cout << "Dirkin: " << endl<<q_dirKin_Tbe( q ) << endl;
	//q_Jacobian( q );
	
	Vector<12> q = Zeros;
	Matrix<4> Tbe = q_dirKin_Tbe( q );
	Matrix<4> Td = q_dirKin_Tbe( q );
	Td[1][3] += 0.1;
	Matrix<12> W_inv = Identity;
	clik( Td, Zeros, q, 100, 1e-3, 1e-3, 0.005, 1, 0.1, Identity);


	return 0;
}
