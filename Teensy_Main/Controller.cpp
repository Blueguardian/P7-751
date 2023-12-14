#include "Controller.h"

PID_Controller::PID_Controller(){
                    // &input,&output,&setpoint,kp,ki,kd,MODE
    PID_x    = new PID(&EKF_x, &pitch, &zero, Kp_x, Ki_x, Kd_x, DIRECT);
    PD_pitch = new PID(&EKF_pitch, &My, &pitch, Kp_theta, 0, Kd_theta, DIRECT);   // pitch - theta

    PID_y    = new PID(&EKF_y, &roll, &zero, Kp_y, Ki_y, Kd_y, DIRECT);
    PD_roll  = new PID(&EKF_roll, &Mx, &roll, Kp_phi, 0, Kd_phi, DIRECT);      // roll - phi

    PID_yaw  = new PID(&EKF_yaw, &Mz, &zero, Kp_psi, Ki_psi, Kd_psi, DIRECT);         // yaw -  psi
    PID_alt  = new PID(&EKF_z, &Fz, &alt, Kp_z, Ki_z, Kd_z, DIRECT);


}

void PID_Controller::begin(){    

	//turn the PIDs on
	PID_x->SetMode(AUTOMATIC);
	PD_pitch->SetMode(AUTOMATIC);

	PID_y->SetMode(AUTOMATIC);
	PD_roll->SetMode(AUTOMATIC);

	PID_yaw->SetMode(AUTOMATIC);

	PID_alt->SetMode(AUTOMATIC);
}

void PID_Controller::Compute_PIDs(){
    PID_x->Compute();
    PD_pitch->Compute();

    PID_y->Compute();
    PD_roll->Compute();

    PID_yaw->Compute();
    PID_alt->Compute();

    Fz+=drone_mass*GRAVITY*0.9;

}

void PID_Controller::get_pwm(){
    double mom_matrix[4] = {Mx, My, Mz, Fz};

    for(int m = 0; m<4; m++){
        pwm_matrix[m] = 0;
        for(int n=0; n<4; n++){
            pwm_matrix[m] +=  ControllMatrix[m][n] * mom_matrix[n]; // get desired RPM^2
        }
        pwm_matrix[m] = constrain(pwm_matrix[m], 0, MAX_PWM); // convert RPM^2 to PWM pulse and constrain it [0,250].
    }

}

void PID_Controller::update(float EKF_x, float EKF_y, float EKF_z, float EKF_pitch, float EKF_yaw, float EKF_roll){
    this->EKF_x     = EKF_x;
    this->EKF_y     = EKF_y;
    this->EKF_z     = EKF_z;
    this->EKF_roll  = EKF_roll;
    this->EKF_pitch = EKF_pitch;
    this->EKF_yaw   = EKF_yaw;

    Compute_PIDs(); // Calculates the outputs of the pid (Mx, My, Mz and Fz)

    get_pwm();      // Translates PID outputs into a PWM signal that can be used by the motors.
}

void PID_Controller::print(){
    double mom_matrix[4] = {Mx, My, Mz, Fz};
    for(int i=0; i<4; i++){
        Serial.print(String(mom_matrix[i]) + " ");
    }
    Serial.print(" -- ");
    
    for(int i=0; i<4; i++){
        Serial.print(String(pwm_matrix[i]) + " ");
    }
}



