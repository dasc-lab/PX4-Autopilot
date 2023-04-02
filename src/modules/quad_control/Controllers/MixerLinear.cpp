#include "MixerLinear.hpp"


MixerLinear::MixerLinear()
{
	
  _k_thrust = 4.0; // N produced at cmd = 1.0
  _k_torque = 0.05; // Nm produced at cmd = 1.0

	// default: assumes the quadrotor is arranged as follows:
	//     +x
	//      ^
	//      |
	//      |
	//   2     0
	//      x       --> +y
	//   1     3
	//
	// where 1 is spinning ccw
	//
	// Use an FRD frame
	float L = 0.33 / 2.0; // arm_length 
  float LX = L / sqrt(2.0f);
  float LY = L / sqrt(2.0f);

	Vector3f rotor_pos_0 ( LX,  LY, 0);
	Vector3f rotor_pos_1 (-LX, -LY, 0);
	Vector3f rotor_pos_2 ( LX, -LY, 0);
	Vector3f rotor_pos_3 (-LX,  LY, 0);

	set_rotor(0, rotor_pos_0, -1);
	set_rotor(1, rotor_pos_1, -1);
	set_rotor(2, rotor_pos_2, 1);
	set_rotor(3, rotor_pos_3, 1);


	construct_G_matrix();

}

void MixerLinear::set_thrust_coeff(float k){
	_k_thrust = k;
}

void MixerLinear::set_torque_coeff(float k){
	_k_torque = k;
}


// dir: +1 if it produces torque in +z direction (i.e., down). -1 else
void MixerLinear::set_rotor(size_t ind, Vector3f pos, int dir)
{
	rotor_pos[ind] = pos;
	rotor_dir[ind] = dir;
}


void MixerLinear::construct_G_matrix()
{
  
	// construct the G, Ginv matrix,
	//
	// assume relationship is 
	// [thrust, torque] = G * cmd
	// 
	// where 
	//   thrust is in N
	//   torque is in Nm
	//   cmd is in [0, 1]

	Vector3f iz (0,0,1);

	SquareMatrix<float, 4> G;

	for (size_t ind = 0; ind < 4; ind ++){
		G(0, ind) = _k_thrust;
	}
	
	for (size_t ind=0; ind<4; ind++){
		Vector3f torque = -_k_thrust * rotor_pos[ind].cross(iz) - (float)rotor_dir[ind] * Vector3f(0,0,_k_torque);
		for (size_t j=0; j<3; j++){
			G(j+1, ind) = torque(j);
		}
	}

	// store inverse G matrix
	_invG = G.I();

}

Vector4f MixerLinear::mix(float thrust_cmd, Vector3f torque_cmd)
{

	Vector4f fM;
	fM(0) = thrust_cmd;
	for (size_t i=0; i<3; i++){
		fM(i+1) = torque_cmd(i);
	}

	Vector4f omega;

	// they assume linear model PWM -> thrust	
	omega = _invG * fM;

	// constrain
	for (size_t i=0; i<4;i++){
		omega(i) = (omega(i) > 1) ? 1 : omega(i);
		omega(i) = (omega(i) < 0) ? 0 : omega(i);
	}

	// convert to PWM
	const float PWM_MIN = 1000;
	const float PWM_MAX = 2000;
	Vector4f pwm;
	for (size_t i=0; i<4; i++){
		pwm(i) = PWM_MIN + (PWM_MAX - PWM_MIN) * omega(i);
	}

	return pwm;

}

SquareMatrix<float, 4> MixerLinear::get_G_matrix(){

	return _invG.I();
}
