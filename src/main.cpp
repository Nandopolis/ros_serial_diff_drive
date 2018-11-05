#include <Arduino.h>

#include "STM32Encoder.h"
#include "TB6612FNG.h"
//#define USE_STM32_HW_SERIAL
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"

#define ARM_MATH_CM3
#include "arm_math.h"

#define RPM210

enum {
  LEFT,
  RIGHT,

  MOTOR_MAX
};
enum PID {
	P,
	I,
	D,
	
	PID_MAX
};

TB6612FNG *motor[MOTOR_MAX];
STM32Encoder *encoder[MOTOR_MAX];

float32_t u_n_global[MOTOR_MAX][2] = {{0.0, 0.0}, {0.0, 0.0}};
float32_t u[MOTOR_MAX] = {0.0, 0.0};
float32_t r[MOTOR_MAX] = {0.0, 0.0};
float32_t aux1, aux2, aux3;
float32_t r_global[MOTOR_MAX];	// vel desired in rad/s
float32_t d_global[MOTOR_MAX];	// vel estimation in rad/s
float32_t y_est[MOTOR_MAX];
float32_t pid_k[MOTOR_MAX][PID_MAX];
float32_t P_n[MOTOR_MAX][4] = {	{	10.0,	0.0,			\
                                  0.0, 	10.0	},	\
                                {	10.0,	0.0,			\
                                  0.0, 	10.0	}	};
float32_t u_n[MOTOR_MAX][2], k_n[2], aux41[2], aux14[2], aux44[4];
float32_t a = 0.999, aux, e_n, y_n, d[MOTOR_MAX];
float32_t ainv = 1/a;
float32_t pid_e[MOTOR_MAX][PID_MAX] = {	{0.0, 0.0, 0.0}, \
																				{0.0, 0.0, 0.0}	};
float32_t w_n[MOTOR_MAX][2] =	{	{0.188, 0.776}, \
																{0.188, 0.776}	};
float32_t L = 0.167;
float32_t R = 0.0325;
float32_t PPR = 341.2;

uint8_t i, j, rls_count = 0, dist_sensor_count = 0;
arm_matrix_instance_f32 mP_n[MOTOR_MAX];
arm_matrix_instance_f32 mk_n;
arm_matrix_instance_f32 mu_n[MOTOR_MAX];
arm_matrix_instance_f32 muT_n[MOTOR_MAX];
arm_matrix_instance_f32 maux41;
arm_matrix_instance_f32 maux14;
arm_matrix_instance_f32 maux44;

ros::NodeHandle nh;
std_msgs::Float32MultiArray wheel_vel;
ros::Publisher wheel_vel_pub("wheel_vel", &wheel_vel);
float32_t wheel_vel_array[2];
std_msgs::Float32MultiArray motor_param;
ros::Publisher motor_param_pub("motor_param", &motor_param);
float32_t motor_param_array[4];
std_msgs::Float32MultiArray dist_sensor;
ros::Publisher dist_sensor_pub("dist_sensor", &dist_sensor);
float32_t dist_sensor_array[3];

void twist_messageCb( const geometry_msgs::Twist& twist_msg){
  float32_t v, w;

  v = twist_msg.linear.x;
  w = twist_msg.angular.z;
  r_global[RIGHT] = (2 * v + w * L) / (2 * R);
  r_global[LEFT] = (2 * v - w * L) / (2 * R);
}
ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", &twist_messageCb );

uint32_t last_time;
int32_t dir;

void setup() {
  // put your setup code here, to run once:
  pinMode(PC15, INPUT_FLOATING);
  pinMode(PC14, INPUT_FLOATING);
  pinMode(PB0, INPUT_FLOATING);
  pinMode(PB1, INPUT_FLOATING);
  encoder[LEFT] = new STM32Encoder(TIMER2, COUNT_BOTH_CHANNELS, 1, 65535);
  encoder[RIGHT] = new STM32Encoder(TIMER3, COUNT_BOTH_CHANNELS, 1, 65535);
  encoder[LEFT]->setFilter(15);
  encoder[RIGHT]->setFilter(15);
  // left encoder
  pinMode(PA0, INPUT_PULLUP);
  pinMode(PA1, INPUT_PULLUP);
  // right encoder
  pinMode(PA6, INPUT_PULLUP);
  pinMode(PA7, INPUT_PULLUP);

  // motor init
  motor[LEFT] = new TB6612FNG(PB6, PB5, PA15, PB3, 1);
  motor[RIGHT] = new TB6612FNG(PB7, PB8, PB9, PB3, 1);
  motor[LEFT]->Setup();
  motor[RIGHT]->Setup();

  nh.initNode();
  wheel_vel.data_length = 2;
  motor_param.data_length = 4;
  dist_sensor.data_length = 3;
  nh.advertise(wheel_vel_pub);
  nh.advertise(motor_param_pub);
  nh.advertise(dist_sensor_pub);
  nh.subscribe(twist_sub);

  while(!nh.connected()){
    nh.spinOnce();
  }
  if (!nh.getParam("ppr", &PPR, 1)) {
    PPR = 341.2;
  }
  if (!nh.getParam("r", &R, 1)) {
    R = 0.0325;
  }
  if (!nh.getParam("l", &L, 1)) {
    L = 0.167;
  }
  float32_t pid[] = {1.0, 75.0, 0.005};
  if (!nh.getParam("pid_left", pid_k[LEFT], PID_MAX)) {
	  arm_copy_f32 (pid, pid_k[LEFT], PID_MAX);
  }
  if (!nh.getParam("pid_right", pid_k[RIGHT], PID_MAX)) {
	  arm_copy_f32 (pid, pid_k[RIGHT], PID_MAX);
  }

  arm_fill_f32(0.0, r_global, MOTOR_MAX);
	arm_fill_f32(0.0, d_global, MOTOR_MAX);
	arm_fill_f32(0.0, y_est, MOTOR_MAX);

  for (i = 0; i < MOTOR_MAX; i++) {
		arm_mat_init_f32(&mP_n[i], 2, 2, P_n[i]);
		arm_mat_init_f32(&mu_n[i], 2, 1, u_n[i]);
		arm_mat_init_f32(&muT_n[i], 1, 2, u_n[i]);
	}
	arm_mat_init_f32(&mk_n, 2, 1, k_n);
	arm_mat_init_f32(&maux41, 2, 1, aux41);
	arm_mat_init_f32(&maux14, 1, 2, aux14);
  arm_mat_init_f32(&maux44, 2, 2, aux44);

  encoder[LEFT]->reset();
  encoder[RIGHT]->reset();

  last_time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (millis() - last_time >= 10) {
    last_time = millis();
    
    // data update
    for (i = 0; i < MOTOR_MAX; i++) {
			u_n_global[i][1] = d_global[i];
			u_n_global[i][0] = u[i];
    }

    // encoder reading
    for(i = 0; i < MOTOR_MAX; i++) {
      if (encoder[i]->getDirection() == POSITIVE) {
        wheel_vel_array[i] = (float32_t)encoder[i]->value();
      }
      else{
        wheel_vel_array[i] = (float32_t)encoder[i]->value() - 65535;
      }
      encoder[i]->reset();
      if (wheel_vel_array[i] == -65535.0) {
        wheel_vel_array[i] = 0.0;
      }
    }
    wheel_vel_array[0] = -wheel_vel_array[0] * 100 * TWO_PI / (PPR * 4);
    wheel_vel_array[1] = -wheel_vel_array[1] * 100 * TWO_PI / (PPR * 4);

    // control loop
    for (i = 0; i < MOTOR_MAX; i++) {
			aux1 = d_global[i];
			
			d_global[i] = wheel_vel_array[i];
			arm_dot_prod_f32(w_n[i], u_n_global[i], 2, &y_est[i]);
			
			r[i] = r_global[i];
			
			pid_e[i][P] = r[i] - y_est[i];
			u[i] = pid_k[i][P] * pid_e[i][P];
			pid_e[i][D] = (aux1 - d_global[i]) / 0.01;
			u[i] += pid_k[i][D] * pid_e[i][D];
			pid_e[i][I] += pid_e[i][P] * 0.01;
			aux2 = pid_k[i][I] * pid_e[i][I];
			if (aux2 > 31.416) {
				pid_e[i][I] = 31.416;
			}
			if (aux2 < 0.0) {
				pid_e[i][I] = 0.0;
			}
			u[i] += aux2;
			
			aux3 = d_global[i] * 1.0;
			if (u[i] > 31.416) {
				u[i] = 31.416;
			}
			if (u[i] < -31.416) {
				u[i] = -31.416;
			}
			u[i] = r[i];
			motor[i]->Move(u[i] * 255 / 31.416);
    }
    wheel_vel.data = wheel_vel_array;
    wheel_vel_pub.publish(&wheel_vel);

    dist_sensor_count++;
    if (dist_sensor_count == 2) {
      dist_sensor_count = 0;
      dist_sensor_array[0] = analogRead(PA3);
      dist_sensor_array[1] = analogRead(PA4);
      dist_sensor_array[2] = analogRead(PA5);
      dist_sensor.data = dist_sensor_array;
      dist_sensor_pub.publish(&dist_sensor);
    }
    
    rls_count++;
    if (rls_count == 5) {
      rls_count = 0;
      for (i = 0; i < MOTOR_MAX; i++) {
        for (j = 0; j < 2; j++) {
          u_n[i][j] = u_n_global[i][j];
        }
        d[i] = d_global[i];
        
        //RLS motor i
        arm_mat_mult_f32(&mP_n[i], &mu_n[i], &maux41);
        arm_dot_prod_f32(u_n[i], aux41, 2, &aux);
        aux = 1/(a + aux);
        arm_scale_f32(aux41, aux, k_n, 2);
        
        arm_dot_prod_f32(w_n[i], u_n[i], 2, &y_n);
        
        e_n = d[i] - y_n;
        
        arm_scale_f32(k_n, e_n, aux41, 2);
        arm_add_f32(w_n[i], aux41, w_n[i], 2);
        
        arm_mat_mult_f32(&muT_n[i], &mP_n[i], &maux14);
        arm_mat_mult_f32(&mk_n, &maux14, &maux44);
        arm_mat_sub_f32(&mP_n[i], &maux44, &maux44);
        arm_mat_scale_f32(&maux44, ainv, &mP_n[i]);
      }
      //motor_param_array[0] = w_n[LEFT][0];
      //motor_param_array[1] = w_n[LEFT][1];
      //motor_param_array[2] = w_n[RIGHT][0];
      //motor_param_array[3] = w_n[RIGHT][1];
      motor_param.data = (float32_t *)w_n;
      motor_param_pub.publish(&motor_param);
    }
  }
  nh.spinOnce();
}
