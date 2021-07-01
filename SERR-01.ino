//#include <Arduino.h>
#include <TimerOne.h>
#include <Motor.h>
#include <Encoder.h>
#include "ros.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

//Encoder esquerdo
#define encA_Lh 19
#define encB_Lh 21

//motor esquerdo
#define en_l_Lh 2
#define en_r_Lh 3
#define pwm_l_Lh 4
#define pwm_r_Lh 9

//Encoder direito
#define encA_Rh 18
#define encB_Rh 20

//motor direito
#define en_l_Rh 5
#define en_r_Rh 6
#define pwm_l_Rh 7
#define pwm_r_Rh 8

#define deltaT 100000 //1000000us = 1s
#define tics 1600
#define wheelCirc 510 //Conrcunferencia das rodas
#define wheelDist 360 //distância entre as rodas 

ros::NodeHandle  nh;

Encoder lhEncoder(encA_Lh, encB_Lh, deltaT, tics);
Motor lhMotor(en_l_Lh, en_r_Lh, pwm_l_Lh, pwm_r_Lh, true);

Encoder rhEncoder(encA_Rh, encB_Rh, deltaT, tics);
Motor rhMotor(en_l_Rh, en_r_Rh, pwm_l_Rh, pwm_r_Rh, true);

class ObjMov {
  private:
    int pwmL;
    int pwmR;
    float movAng;
    String tipo; //R = Girar; M = Movimentar
    bool flag;
  public:
    ObjMov() {
      this->pwmL = 0;
      this->pwmR = 0;
      this->movAng = 0.0;
      this->tipo = "";
      this->flag = false;
    }
    
    void setPwmL(int value){
      pwmL = value;
    }
    void setPwmR(int value){
      pwmR = value;
    }
    void setMovAng(float value){
      movAng = value;
    }
    void setTipo(String value){
      tipo = value;
    }
    void setExecutar(bool value){
      flag = value;
    }
    int getPwmL(){
      return pwmL;
    }
    int getPwmR(){
      return pwmR;
    }
    float getMovAng(){
      return movAng;
    }
    String getTipo(){
      return tipo;
    }
    bool getExecutar(){
      return flag;
    }
};

boolean pararLh = true;
boolean pararRh = true;
boolean flagRotate = false;
boolean flagMover = false;
int countDist = 0;
float distanciaLh = 0;
float distanciaRh = 0;
float countLh = 0;
float countRh = 0;
float distanciaFinal = 0;
float anguloRobo = 0;
int _pwmLh = 100;
int _pwmRh = 100;
int _angRobo, _distancia;
float countDir = 0;
float countEsq = 0;
float angTot = 0;
double x = 0;
double y = 0;
double dTheta = 0;
double theta = 0;
ObjMov objMov[4];


void messageMover(const std_msgs::Float64& msg);
void messageRotate(const std_msgs::Float64& msg);
void parar(const std_msgs::Empty& msg);
void setPwmLh(const std_msgs::Float64& msg);
void setPwmRh(const std_msgs::Float64& msg);
void setPwmLhRh(const std_msgs::Float64& msg);

ros::Subscriber<std_msgs::Float64> sub_mover("mover", &messageMover);
ros::Subscriber<std_msgs::Float64> sub_rotate("girar", &messageRotate);
ros::Subscriber<std_msgs::Empty> sub_parar("parar", &parar);
ros::Subscriber<std_msgs::Float64> sub_setPwmLh("setPwmLh", &setPwmLh);
ros::Subscriber<std_msgs::Float64> sub_setPwmRh("setPwmRh", &setPwmRh);
ros::Subscriber<std_msgs::Float64> sub_setPwmLhRh("setPwmLhRh", &setPwmLhRh);


void setup() {  
  Timer1.initialize(deltaT);
  Timer1.attachInterrupt(encoderUpTime);
  attachInterrupt(digitalPinToInterrupt(encA_Lh), readLHEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA_Rh), readRHEncoder, CHANGE);
  nh.initNode();
  nh.subscribe(sub_mover);
  nh.subscribe(sub_rotate);
  nh.subscribe(sub_setPwmLh);
  nh.subscribe(sub_setPwmRh);
  nh.subscribe(sub_setPwmLhRh);
  nh.spinOnce();
}

void loop() {
  nh.spinOnce();
}

void encoderUpTime(){
  
  for(int idx = 0; idx < 4; idx++){
    if(pararLh && pararRh){
      if(objMov[idx].getExecutar()){
        if(objMov[idx].getTipo() == String("R")){
          girar(objMov[idx].getPwmL(), objMov[idx].getPwmR(), objMov[idx].getMovAng());
        }
        if(objMov[idx].getTipo() == String("M")){
          mover(objMov[idx].getPwmL(), objMov[idx].getPwmR(), objMov[idx].getMovAng());
        }
        objMov[idx].setExecutar(false);
      }
    }
  }
  
  ObjMov objMovTemp[4];
  int atual = 0;
  for(int idx = 0; idx < 4; idx++){
    if(objMov[idx].getExecutar()){
      objMovTemp[atual] = objMov[idx];
      atual = atual++;
    }
  }
  for(int idx = 0; idx < 4; idx++){
    objMov[idx] = objMovTemp[idx];
  }
  
}

void readLHEncoder(){
  lhEncoder.updateCount();
  countLh = countLh+1;
  if(countLh == 10 && !pararLh){
    countLh = 0;
    distanciaLh += lhEncoder.getDistanceRT(); //Retorna a distancia percorrida em contagens de pulsos do encoder, sem interferir na odometria.
    motorVerificaDist(&lhMotor, distanciaLh, distanciaFinal, &pararLh); 
    if(pararLh){
      distanciaLh = 0; 
    }
  }
  if(countLh > 10)
    countLh = 0;
}

void readRHEncoder(){
  rhEncoder.updateCount();
  countRh = countRh+1;
  if(countRh == 10 && !pararRh){
    countRh = 0;
    distanciaRh += rhEncoder.getDistanceRT(); //Retorna a distancia percorrida em contagens de pulsos do encoder, sem interferir na odometria.
    motorVerificaDist(&rhMotor, distanciaRh, distanciaFinal, &pararRh);
    if(pararRh){
      distanciaRh = 0; 
    }
  }
  if(countRh > 10)
    countRh = 0;
}

void motorVerificaDist(Motor *motor, float distanciaAtual, float distFinal, boolean *parar){
  if(distanciaAtual < 0)
    distanciaAtual = distanciaAtual*-1;
  if(distanciaAtual >= distFinal){
    motor->setPWM(0);
    *parar = true;
  }
}

void girar(int pwmLh, int pwmRh, float angRobo){
    distanciaLh = 0;
    distanciaRh = 0;
    anguloRobo = angRobo;
    theta += double(angRobo);
    definirDistAngFinal(angRobo);
    pararLh = false;
    pararRh = false;
    if(anguloRobo > 0){
      lhMotor.setBack();
      lhMotor.setPWM(pwmLh);
      rhMotor.setFwd();
      rhMotor.setPWM(pwmRh);
    }else{
      lhMotor.setFwd();
      lhMotor.setPWM(pwmLh);
      rhMotor.setBack();
      rhMotor.setPWM(pwmRh);
    }
}

void mover(int pwmLh, int pwmRh, float distancia){ // Distância em mm
    distanciaLh = 0;
    distanciaRh = 0;
    definirDistFinal(distancia);
    pararLh = false;
    pararRh = false;
    if(distancia > 0){
      lhMotor.setFwd();
      lhMotor.setPWM(pwmLh);
      rhMotor.setFwd();
      rhMotor.setPWM(pwmRh);
    }else{
      lhMotor.setBack();
      lhMotor.setPWM(pwmLh);
      rhMotor.setBack();
      rhMotor.setPWM(pwmRh);
    }
}

void parar(const std_msgs::Empty& msg){
  distanciaLh = distanciaFinal;
  distanciaRh = distanciaFinal;
}

void definirDistFinal(float dist){
  if(dist > 0){
    distanciaFinal = (dist/wheelCirc)*tics;
  }else{
    distanciaFinal = -1*(dist/wheelCirc)*tics;
  }
}
void definirDistAngFinal(float angulo){
  float conv = 0;
  if(angulo > 0){
    conv = (440*angulo)/90;
    distanciaFinal = (angulo/wheelCirc)*tics;
  }else{
    conv = -1*(440*angulo)/90;
  }
  definirDistFinal(conv);
}

void messageMover(const std_msgs::Float64& msg){
  for(int i = 0; i < 3; i++){
    if(!objMov[i].getExecutar()){
      objMov[i].setPwmL(_pwmLh);
      objMov[i].setPwmR(_pwmRh);
      objMov[i].setMovAng(msg.data);
      objMov[i].setTipo("M");
      objMov[i].setExecutar(true);
      i = 4;
    }
  }
  //publicarMensagem("publicou mover: " + String(msg.data));
}

void messageRotate(const std_msgs::Float64& msg){
  for(int i = 0; i < 3; i++){
    if(!objMov[i].getExecutar()){
      objMov[i].setPwmL(_pwmLh);
      objMov[i].setPwmR(_pwmRh);
      objMov[i].setMovAng(msg.data);
      objMov[i].setTipo("R");
      objMov[i].setExecutar(true);
      i = 4;
    }
  }
  //publicarMensagem("publicou girar: " + String(msg.data));
}

void setPwmLh(const std_msgs::Float64& msg){
  _pwmLh = msg.data;
}

void setPwmRh(const std_msgs::Float64& msg){
  _pwmRh = msg.data;
}

void setPwmLhRh(const std_msgs::Float64& msg){
  _pwmLh = msg.data;
  _pwmRh = msg.data;
}
