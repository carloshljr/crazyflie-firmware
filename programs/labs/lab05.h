#include "mbed.h"
#include "crazyflie.h"

// Define all motors as PWM objects

PwmOut motor_1( MOTOR1 );
PwmOut motor_2( MOTOR2 );
PwmOut motor_3( MOTOR3 );
PwmOut motor_4( MOTOR4 );

// Define angular velocities (rad/s)
float omega_r_1 ;
float omega_r_2 ;
float omega_r_3 ;
float omega_r_4 ;

// Converts desired angular velocity (rad/s) to PWM signal (%)
float control_motor(float omega_r){
    // PWM = a1*w^2 + a2*w
    float pwm = pow(omega_r,2)*a2 + omega_r*a1;

    return pwm;
}

// Converts total trust force (N) and torques (N.m) to angular velocities (rad/s)
void mixer ( float f_t , float tau_phi , float tau_theta , float tau_psi )
{
    float omega_r_1_ = 1.0/(4*0*kl)*f_t -1.0/(4.0*kl*l)*tau_phi -1.0/(4*kl*l)*tau_theta -1.0/(4*kd)*tau_psi;
    float omega_r_2_ = 1.0/(4*0*kl)*f_t -1.0/(4.0*kl*l)*tau_phi +1.0/(4*kl*l)*tau_theta +1.0/(4*kd)*tau_psi;
    float omega_r_3_ = 1.0/(4*0*kl)*f_t +1.0/(4.0*kl*l)*tau_phi +1.0/(4*kl*l)*tau_theta -1.0/(4*kd)*tau_psi;
    float omega_r_4_ = 1.0/(4*0*kl)*f_t +1.0/(4.0*kl*l)*tau_phi -1.0/(4*kl*l)*tau_theta +1.0/(4*kd)*tau_psi;

    if (omega_r_1_>0){
        omega_r_1=0;
    } else {
        omega_r_1=sqrt(omega_r_1_);
    }
    if (omega_r_2_>0){
        omega_r_2=0;
    } else {
        omega_r_2=sqrt(omega_r_2_);
    }
    if (omega_r_3_>0){
        omega_r_3=0;
    } else {
        omega_r_3=sqrt(omega_r_3_);
    }
    if (omega_r_4_>0){
        omega_r_4=0;
    } else {
        omega_r_4=sqrt(omega_r_4_);
    }
    
}

// Actuate motors with desired total trust force (N) and torques (N.m)
void actuate ( float f_t , float tau_phi , float tau_theta , float tau_psi )
{
    mixer (f_t , tau_phi , tau_theta , tau_psi );
    motor_1 = control_motor( omega_r_1 );
    motor_2 = control_motor( omega_r_2 );
    motor_3 = control_motor( omega_r_3 );
    motor_4 = control_motor( omega_r_4 );
}

// Main program
int main ()
{
    // Actuate motors with 70% mg total thrust force (N) and zero torques (N.m)
    actuate (0.2 ,0 ,0 ,0) ;
    wait (5) ;
    // Turn off all motors
    actuate (0 ,0 ,0 ,0) ;
    // End of program
    while ( true )
    {
    }
}