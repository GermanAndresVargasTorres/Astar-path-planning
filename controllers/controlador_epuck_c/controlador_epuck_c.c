#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/differential_wheels.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <webots/robot.h>

#define TIME_STEP 16



int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  // Variables requeridas
  int time_step = wb_robot_get_basic_time_step();
  //int max_speed = 1000;
  
  // Crear receptor para comunicacion inalambrica
  WbDeviceTag receptor = wb_robot_get_device("receiver");  
  
  // Declarar las funciones asociadas al receptor
  void wb_receiver_enable(WbDeviceTag tag, int ms);
  int wb_receiver_get_queue_length(WbDeviceTag tag);
  const void *wb_receiver_get_data(WbDeviceTag tag);
  void wb_receiver_next_packet(WbDeviceTag tag);
  
  // Activar el receptor
  wb_receiver_enable(receptor, time_step);
  
  
  // MAIN
  while (wb_robot_step(TIME_STEP) != -1) {
  
    // Variable para almacenar el mensaje que se recibira
    const char *mensaje;
    
    // Variable para almacenar las velocidades que se asignaran
    float v1 = 0.0;
    float v2 = 0.0;
    
    // Leer el mensaje enviado por el supervisor
    if (wb_receiver_get_queue_length(receptor) > 0) {
      mensaje = wb_receiver_get_data(receptor);  
      //printf("Orden recibida: %s\n", mensaje);   
      
      // Si el caracter recibido es F, debemos avanzar hacia adelante
      if (mensaje[0] == 'F') {
        v1 = 700.0;
        v2 = 700.0;
      }
      
      // Si el caracter recibido es A, debemos hacer una rotación en sentido antihorario
      else if (mensaje[0] == 'A') {
        v1 = -500.0;
        v2 = 500.0;
      }
      
      // Si el caracter recibido es H, debemos hacer una rotación en sentido horario
      else if (mensaje[0] == 'H') {
        v1 = 500.0;
        v2 = -500.0;
      }
      
      // Si el caracter recibido es S, debemos detener el robot
      else if (mensaje[0] == 'S') {
        v1 = 0.0;
        v2 = 0.0;
      }
      
      wb_receiver_next_packet(receptor);
    }    
    
    // Mover el robot de acuerdo a las velocidades configuradas
    wb_differential_wheels_set_speed(v1, v2);          
  };
  
  
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  
  return 0;
}