/**
 * @file geometric_control_params.c
 * Parameters for multicopter geometric controller.
 *
 * @author Devansh Agrawal <devansh@umich.edu>
 */

/**
 * Pos Error Gain
 *
 * kx
 *
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_KX, 1.0f);


/**
 * Vel Error Gain
 *
 * kv
 *
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_KV, 2.0f);


/**
 * Rotation Error Gain
 *
 * kR
 *
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_KR, 0.35f);


/**
 * AngVel Error Gain
 *
 * kOmega
 *
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_KOMEGA, 0.15f);


/**
 * Quadrotor Mass
 *
 * mass
 *
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_M, 1.5f);


/**
 * Quad Moment of Inertia xx
 *
 * Jxx
 *
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_JXX, 0.03f);


/**
 * Quad Moment of Inertia yy
 *
 * Jyy
 *
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_JYY, 0.03f);


/**
 * Quad Moment of Inertia zz
 *
 * Jzz
 *
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_JZZ, 0.06f);

// todo: add other moments of inertia


/**
 * Motor Force Constant (N / kRPM^2)
 *
 * k_f
 *
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_K_F, 0.02f);

// 1.91e-6 N / (rad/s)^2 = 1.91e-6 * 1e6 /(9.54^2) N / kRPM^2 = 0.02  N / kRPM^2

/**
 * Motor Torque Constant (Nm / kRPM^2)
 *
 * k_mu
 *
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_K_MU, 0.00296f); // TODO: UPDATE
// 2.7e-7 Nm / (rad/s)^2 = 2.7e-7 * 1e6 / (9.54)^2 Nm / kRPM^2 = 0.00296 Nm / kRPM^2

/**
 * Max Motor Speed (kRPM)
 *
 * omega_max
 *
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_OMEGA_MAX, 20.0f); // TODO: UPDATE


/**
 * Motor 1 position x (m)
 *
 * M1 X
 *
 * @min -0.25
 * @max 0.25
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_M1X, 0.08f); // TODO: UPDATE


/**
 * Motor 2 position x (m)
 *
 * M2 X
 *
 * @min -0.25
 * @max 0.25
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_M2X, -0.08f); // TODO: UPDATE



/**
 * Motor 3 position x (m)
 *
 * M3 X
 *
 * @min -0.25
 * @max 0.25
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_M3X, 0.08f); // TODO: UPDATE


/**
 * Motor 4 position x (m)
 *
 * M4 X
 *
 * @min -0.25
 * @max 0.25
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_M4X, -0.08f); // TODO: UPDATE




/**
 * Motor 1 position y (m)
 *
 * M1 y
 *
 * @min -0.25
 * @max 0.25
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_M1Y, -0.08f); // TODO: UPDATE


/**
 * Motor 2 position y (m)
 *
 * M2 Y
 *
 * @min -0.25
 * @max 0.25
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_M2Y, 0.08f); // TODO: UPDATE



/**
 * Motor 3 position y (m)
 *
 * M3 Y
 *
 * @min -0.25
 * @max 0.25
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_M3Y, 0.08f); // TODO: UPDATE


/**
 * Motor 4 position y (m)
 *
 * M4 Y
 *
 * @min -0.25
 * @max 0.25
 * @decimal 3
 * @increment 0.01
 * @group Geometric Control
 */
PARAM_DEFINE_FLOAT(GEO_M4Y, -0.08f); // TODO: UPDATE
