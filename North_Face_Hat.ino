/* Arduino Flora Wearable Project for Compass Hat with Haptic (Vibrating) Feedback When Facing North
 * Instructions: Switch on hat, then turn and tilt the hat in all directions to record the min/max magnetic field strengths. 
 * Hat will vibrate when you face north. 
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h> //for 9DOF sensor
#include <Adafruit_LSM9DS0.h> //for 9DOF sensor 
#include <LSM303_mod.h> //for 9DOF sensor - Modified library to account for direction of XYZ coordinates
#include <Adafruit_DRV2605.h> //for haptic controller 

// Initialize Variable for Buzz Count
int BuzzCount;

//For Compass Calibration
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
LSM303::vector<int16_t> running_min_previous = {32766, 32766, 32766}, running_max_previous = {-32767, -32767, -32767};
LSM303::vector<int16_t> running_min_converted = {32766, 32766, 32766}, running_max_converted = {-32767, -32767, -32767};
char report[80];


// Assign a unique base ID for this sensor  
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

//Configures the gain and integration time for the TSL2561
void configureSensor(void)
{
  
  // Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

}

//Declare  drv
Adafruit_DRV2605 drv;


void setup(void) 
{
  delay(5000); // Delay 5 seconds for serial port to get ready

  Serial.begin(9600);
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");

  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
// Select library for haptic controller 

  drv.begin();
  
  drv.selectLibrary(1);
  
  // I2C trigger by sending 'go' command 
  // default, internal trigger when sending GO command
  drv.setMode(DRV2605_MODE_INTTRIG); 

  //Initialize compass before calibration
  Wire.begin();
  compass.init();
  compass.enableDefault();

  //Read compass and determine min and max
  //This routine requires the hat to be turned across all axes so that the minimum and maximum magnetic field can be calibrated 
  //Calibration will continue until no more changes to min and max are recorded 

  int CalibrationCounter; //Counts how many measurements are taken before comparing to previous min/max
  int CalibrationAccuracy = 30; //Higher values increase calibration accuracy but delay start  
  
while (running_min.x != running_min_previous.x  || running_min.y != running_min_previous.y || running_min.z != running_min_previous.z) 
  {
    //Set new previous minimum for comparison after next completed for loop
    running_min_previous.x = running_min.x;
    running_min_previous.y = running_min.y;
    running_min_previous.z = running_min.z;
    
    for (CalibrationCounter = 0; CalibrationCounter < CalibrationAccuracy; CalibrationCounter++)   
    { 
      compass.read();
  
      Serial.println("CalibrationCounter=");
      Serial.println(CalibrationCounter);

      running_min.x = min(running_min.x, compass.m.x);
      running_min.y = min(running_min.y, compass.m.y);
      running_min.z = min(running_min.z, compass.m.z);
    
      running_max.x = max(running_max.x, compass.m.x);
      running_max.y = max(running_max.y, compass.m.y);
      running_max.z = max(running_max.z, compass.m.z);
      
      snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
        running_min.x, running_min.y, running_min.z,
        running_max.x, running_max.y, running_max.z);
      Serial.println(report);
      
      delay(100);
    }
  }


  //Setting the min/max values to calibrate the compass
  Serial.println("After While\n");
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){running_min.x, running_min.y, running_min.z};
  compass.m_max = (LSM303::vector<int16_t>){running_max.x, running_max.y, running_max.z};
    
}


void loop(void) 

{  

  compass.read();
  float heading = compass.heading();
  Serial.println(heading);  
  delay(100);
  
if ((heading > 350 ) || (heading >= 0 && heading <= 15))
  {  
  if (BuzzCount < 1)
    {
    // buzz vibrating motor 
    drv.setWaveform(0, 110);  // Transition ramp up short smooth, 0% to 50%, see datasheet part 11.2
    drv.setWaveform(1, 3);  // strong click 30%, see datasheet part 11.2
    drv.setWaveform(2, 98);  // Transition ramp down short smooth, 50% to 0%, see datasheet part 11.2
    drv.setWaveform(3, 0);  // end of waveforms
    drv.go();
    BuzzCount++;
    delay(2000); // Wait after buzz
    Serial.println("Facing North - Vibrating!\n");

    }
  }
else
  {
  BuzzCount = 0;
  }

  //Light up red on-board LED to show that hat is calibrated and working
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH); // turn the LED on (HIGH is the voltage level)
  
}
