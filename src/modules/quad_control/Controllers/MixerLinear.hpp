#pragma once

#include <lib/matrix/matrix/math.hpp>

using namespace matrix;

class MixerLinear
{

  public:

    MixerLinear();

    // TODO: add some docs on what these are, and what the assumed model is
    void set_thrust_coeff(float k_thrust);
    void set_torque_coeff(float k_torque);
    void set_rotor(size_t ind, Vector3f pos, int dir);
    void construct_G_matrix();
    SquareMatrix<float, 4> get_G_matrix();

    Vector4f mix(float thrust_cmd, Vector3f torque_cmd);

  private:

    //Parameters
    float _k_thrust, _k_torque;

    Vector3f rotor_pos[4];
    int rotor_dir[4];


    //
    SquareMatrix<float, 4> _invG;
  
};
