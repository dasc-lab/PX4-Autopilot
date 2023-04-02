// This file has the main control loops

#include <IndiControl.hpp>

using namespace matrix;

Vector3f acceleration_cmd(Vector3f x, Vector3f v, Vector3f a_filt, 
		Vector3f x_ref, Vector3f v_ref, Vector3f a_ref,
		float ka, float kv, float kx)
{

	Vector3f a_cmd = a_ref + ka * (a_ref - a_filt) + kv * (v_ref - v) + kx * (x_ref - x);
	return a_cmd;
}


Vector3f thrust_cmd()
{

	Vector3f tau_bz_cmd = tau_bz_filt + a_cmd - a_filt;


}


