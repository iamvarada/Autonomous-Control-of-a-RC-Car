///////////////////// PSU ME 445  - GROUP PROJECT ////////////////////////////////
///////////////////// Greg Brulo, Krishna P. Varadarajan S. //////////////////
//////////////////// Final code for the project //////////////////////////////
//////////////////// Penn State University, University Park //////////////////////////////


// Add Servo and Pololu IR sensor libraries

#include <Servo.h>
#include <PololuQTRSensors.h>

Servo myspeedservo; // create speed servo object
Servo mysteerservo; //create steering servo object

PololuQTRSensorsRC qtr((char[]) {13,12,11,10,9,8,7,6}, 8); // define the IR sensor pins (in the order from 1st to last) and the number of pins = n

// define all the necessary variables here

int prevpos = 0; // initialize the variable to store the previous value of the IR sensor
int error = 0; // intialize the variable to store the error
int reference = 3500; // initialize reference variable corresponding to the value of the middle IR sensor array = (n-1)/2
float Kp = 0.017; // set proportional gain
int Kd = 0.007; // set derivative gain
float P = 0; // initialize variable to store (error*proportional gain)
int D = 0; // initialize variable to store (error*derivative gain)
float total = 0; // to store the total error
int Huston=0; // set Huston to 0.  When Huston is 1, the vehicle goes
int val_4=0; // set the value fo the button to start the vehicle to LOW
int vehicle_speed=103; // set the forward driving speed of the vehicle


void setup() // the code in this function runs only once
{
    
      // define pins for the speed and the steering servo
      
      mysteerservo.attach(1); // set steering servo to pin 1
      myspeedservo.attach(2); // set ESC/speed servo to pin 2 
    
      // reset the two servos before calibration
      
      myspeedservo.write(90); 
      mysteerservo.write(90);
    
      // initialize pins for push button and calibration LED
    
      pinMode(4,INPUT); // push button pin set to input
      pinMode(5,OUTPUT); // LED pin set to output
    
      digitalWrite(5,HIGH); // turn ON the calibration LED
    
      while(0==val_4){  // when pin 4 is low, keep reading the pin
    
          val_4=digitalRead(4); // read the button which is on pin 4
          delay(250);           // a nominal delay
        
          if(1==val_4){  // begin calibrating  
        
              // calibrate the IR sensors - the calibration LED starts blinking at a high frequency to indicate that it is calibrating
              
              digitalWrite(5, HIGH);
              delay(750);
              digitalWrite(5, LOW);
              delay(250);
              digitalWrite(5, HIGH);
              delay(750);
              digitalWrite(5, LOW);
              delay(250);
              digitalWrite(5, HIGH);
              delay(750);
              digitalWrite(5, LOW);
              delay(250);
              digitalWrite(5, HIGH);
              delay(750);
              digitalWrite(5, LOW);
              delay(250);
              digitalWrite(5, HIGH);
              delay(750);
              digitalWrite(5, LOW);
              delay(250);
              
              int i;
                    for (i = 0; i < 250; i++) // make the calibration take about 5 seconds
                    {
                    qtr.calibrate(); // in-built function to calibrate the sensor array
                    digitalWrite(5, HIGH); // indication to turn the LED ON and OFF
                    delay(20);
                    digitalWrite(5, LOW);
                    }
             }
        }
}



void loop() // the code in this function runs over and over again
{
      
      myspeedservo.write(90); // reset the ESC to netural position
      
      mysteerservo.write(90); // reset the steering servo to neutral position (facing straight)
      
      delay(500); // a nominal delay for the above commands to take effect
    
      val_4=digitalRead(4); // read if the push button is pressed
    
      digitalWrite(5, HIGH); // blink the LED on pin 5
      delay(2000);
      digitalWrite(5, LOW);
      delay(1500);
    
      if(1==val_4){ // check to see if the push buton is pressed
        delay(500);
          if(1==val_4){
            Huston=1; // assign the variable Huston to 1 if the push button is pressed
          }
      }
    
      while(1==Huston){ // while Huston is 1
          digitalWrite(5, HIGH); // LED on pin 5 is ON
          myspeedservo.write(vehicle_speed); // now drive the vehicle forward
          
          unsigned int sensors[5]; // define the number of sensors you wanna use
    
          // store the position sensor array reading in an array and store it in a variable which would represent he current position of the vehicle
          int position_GSB = qtr.readLine(sensors); 
          
          error = reference - position_GSB; //  calculate error as the difference of reference and current sensor value
        
          P = error*Kp; // proportional gain multiplied by error
          D = (prevpos - position_GSB)*Kd; // derivative gain multiplied by the change in the sensor value
       
          total = P + D; // calculate the sum of the above two quantities
          
          prevpos = position_GSB; // set the current position to previous position
        
          mysteerservo.write(90+total); // the total error after multiplication with the respective gains, add to the steering value 
          
      }
}



////////////////////////////// END OF CODE ////////////////////////////////
 
