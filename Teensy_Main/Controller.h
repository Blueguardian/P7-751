#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "Arduino.h"
#include "Print.h"
#include <PID_v1.h>

//PID Y
# define Kp_y 0.0352971737456039
# define Kd_y 0.140000825221533
# define Ki_y 0.000877932917802346
//PID phi
# define Kp_phi 4.76078120501853
# define Kd_phi 1.76915436082347
# define Ki_phi 0.
//PID X
# define Kp_x 0.0354014133693988
# define Kd_x 0.140522674904209
# define Ki_x 0.000880735512599299
//PID theta
# define Kp_theta 4.76078120501853
# define Kd_theta 1.76915436082347
# define Ki_theta 0.
//PID psi
# define Kp_psi 0.520783247027697
# define Kd_psi 0.40929272881607
# define Ki_psi 0.165660889683683
//PID alt
# define Kp_z 672.111379297916/100
# define Kd_z 838.711429535055/100
# define Ki_z 134.651111894447/100
// Controll Matrix components
#define n1 26.322773189895987
#define n2 1441.846293426507
#define n3 4.935519973105490
// Motor data
#define MAX_RPM 109495296 // RPM^2
#define MAX_PWM 250

# define GRAVITY 9.80665

#endif


class PID_Controller{
    private:
        double EKF_x, EKF_y, EKF_z, EKF_roll, EKF_pitch, EKF_yaw;   // Variables from EKF.
        double pitch, roll;                                         // Variables from PID.
        double Mx, My, Mz, Fz;                                      // Variables from PID (OUTPUT).
        double zero = 0;                                            // this is the set point of the PID. Is the desired angle.

        float drone_mass = 0.905; // Kg


        float ControllMatrix[4][4]= { 
                                        {  n1, -n1,  n2,  n3 },
                                        { -n1, -n1, -n2,  n3 },
                                        {  n1,  n1, -n2,  n3 },
                                        { -n1,  n1,  n2,  n3 }
                                    };

        PID *PID_x;
        PID *PD_pitch;
        PID *PID_y;
        PID *PD_roll;
        PID *PID_yaw;
        PID *PID_alt;

        void Compute_PIDs();
        void get_pwm();

    public:
        double alt = 0;

        float pwm_matrix[4] = {0,0,0,0};

        PID_Controller();
        void begin();
        void update(float EKF_x, float EKF_y, float EKF_z, float EKF_pitch, float EKF_yaw, float EKF_roll);
        void print();

};















