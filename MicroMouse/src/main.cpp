#include <Arduino.h>
#include "ROBOT.h"
//#include "FloodFill.h"
//90
//-88
 ROBOT robot = ROBOT(9.7, 6.5, 970);

void setup()
{
  // while(!Serial){
  //   ;
  // }

   Serial.begin(9600);
   //initialize();
   robot.init();
  
  for(int i = 0; i < 3; i++){
   
    robot.move_distance(19.2);
     
   //  Serial.println("-------------------------------------------------------------------");
    //  robot.move_distance(912);
   
    //  robot.move_distance(912);
     
  //   robot.rotate_angle(90);
    
  //   robot.move_distance(912.5);
  //   robot.move_distance(912.5);
  //   robot.rotate_angle(90);
  //    robot.move_distance(912.5);
  //   robot.move_distance(912.5);
  //   robot.move_distance(912.5);
  //   robot.rotate_angle(90);
  //   robot.move_distance(912.5);
  //   robot.move_distance(912.5);
  //   robot.rotate_angle(90);


   //robot.move_distance(19.2);
  
   // robot.rotate_angle(90);
  }
   
  //  robot.move_distance(19.2);
  //  robot.move_distance(19.2);
   robot.rotate_angle(90); 
  // robot.rotate_angle(-88);
  
}

 

void loop()
{
 // solve();
 //robot.Rotaion_move_imu(90);
  

  // robot.rotate_angle(90);
  
}