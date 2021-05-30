#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include <webots/accelerometer.h>
#include <webots/supervisor.h>




static WbDeviceTag left_motor, right_motor, right_sensor, left_sensor, right_motor_sensor, left_motor_sensor;

double val_sensor_direita;
double val_sensor_esquerda;
const double *posicao;

int quantCaixasMoveis = 0, quantCaixasFixas = 0;

#define MAX_SPEED 6.28
#define TIME_STEP 64

void delay(int time_milisec)
{
  double currentTime, initTime, Timeleft;
  double timeValue = (double)time_milisec/1000;
  initTime = wb_robot_get_time();
  Timeleft =0.00;
  while (Timeleft < timeValue)
  {
    currentTime = wb_robot_get_time();
    Timeleft=currentTime-initTime;
    wb_robot_step(TIME_STEP);
  }
}

void virarDireita(){
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
  delay(700);
}

void virarEsquerda(){
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
  delay(700);
}

void irAtras(){
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
}

void irReto(){
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void resetVelocidades(){
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");

  WbNodeRef robot_node = wb_supervisor_node_get_from_def("ePuck"); 
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  

  right_sensor = wb_robot_get_device("ps0");
  left_sensor = wb_robot_get_device("ps7");
  
  right_motor_sensor = wb_robot_get_device("right wheel sensor");
  left_motor_sensor = wb_robot_get_device("left wheel sensor");


  wb_distance_sensor_enable(right_sensor, TIME_STEP);
  wb_distance_sensor_enable(left_sensor, TIME_STEP);

  
  wb_position_sensor_enable(right_motor_sensor, TIME_STEP);
  wb_position_sensor_enable(left_motor_sensor, TIME_STEP);

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */

  while (wb_robot_step(TIME_STEP) != -1) {
    val_sensor_direita = wb_distance_sensor_get_value(right_sensor);
    val_sensor_esquerda = wb_distance_sensor_get_value(left_sensor);

    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    irReto();
    posicao = wb_supervisor_field_get_sf_vec3f(trans_field);
    int limiarSensor = 100;
    if(val_sensor_direita > limiarSensor || val_sensor_esquerda > limiarSensor){
        posicao = wb_supervisor_field_get_sf_vec3f(trans_field);
        double posAntX = posicao[0];
        double posAntZ = posicao[2];
        irReto();
        delay(1000);
        posicao = wb_supervisor_field_get_sf_vec3f(trans_field);
        double posAtualX = posicao[0];
        double posAtualZ = posicao[2];
        
        printf("posAntX: %f, posAtualX: %f, posAntZ: %f, posAtualZ: %f\n", posAntX, posAtualX, posAntZ, posAtualZ); 
        
        double tolerancia = 0.025;
        if(posAntX != posAtualX || posAntZ != posAtualZ){
          if(posAtualX > 0 && posAtualZ > 0){
            if((posAtualX < posAntX+tolerancia && posAtualX > posAntX-tolerancia) || (posAtualZ < posAntZ+tolerancia && posAtualZ > posAntZ-tolerancia)){
            
            }else{
              quantCaixasMoveis = quantCaixasMoveis+1;
            }  
           }
           /*
          else if(posAtualX < 0 && posAtualZ > 0){
            if((posAtualX > posAntX+tolerancia && posAtualX < posAntX-tolerancia) || (posAtualZ < posAntZ+tolerancia && posAtualZ > posAntZ-tolerancia)){
            
            }else{
              quantCaixasMoveis = quantCaixasMoveis+1;
            }  
          }
          else if(posAtualX > 0 && posAtualZ < 0){
            if((posAtualX < posAntX+tolerancia && posAtualX > posAntX-tolerancia) || (posAtualZ > posAntZ+tolerancia && posAtualZ < posAntZ-tolerancia)){
            
            }else{
              quantCaixasMoveis = quantCaixasMoveis+1;
            }  
          }
          else if(posAtualX < 0 && posAtualZ < 0){
            if((posAtualX > posAntX+tolerancia && posAtualX < posAntX-tolerancia) || (posAtualZ > posAntZ+tolerancia && posAtualZ < posAntZ-tolerancia)){
            
            }else{
              quantCaixasMoveis = quantCaixasMoveis+1;
            }  
          }
          */
        
        }
        
      if(val_sensor_direita > limiarSensor){
        while(val_sensor_direita>limiarSensor){
          val_sensor_direita = wb_distance_sensor_get_value(right_sensor);
          //printf("val sensor direita: %f\n", val_sensor_direita);
          virarEsquerda();
          irReto();
        }

      }
      if(val_sensor_esquerda > limiarSensor){
  
        while(val_sensor_esquerda>limiarSensor){
            val_sensor_esquerda = wb_distance_sensor_get_value(left_sensor);
            //printf("val sensor direita: %f\n", val_sensor_esquerda);
            virarDireita();
            irReto();
        }
      }
    }

    
    //posicao = wb_supervisor_field_get_sf_vec3f(trans_field);    
    //printf("PosX: %f, PosY: %f, PosZ: %f, Direita: %f, Esquerda: %f\n", posicao[0], posicao[1], posicao[2], val_sensor_direita, val_sensor_esquerda);
    printf("Quantidade de caixas moveis: %d\n", quantCaixasMoveis);
    
  };
  
  wb_robot_cleanup();

  return 0;
}
