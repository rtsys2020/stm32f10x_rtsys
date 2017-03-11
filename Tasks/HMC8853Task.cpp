/**
 * @file HMC8853Task.cpp
 *
 * @date Dec 20, 2016
 * @author jupiter
 * @description
 */
#include <HMC5883L/HMC5883L.h>
#include <Wire.h>
#include <HardWire.h>

void heading(HMC5883L *compass);
void calibration(HMC5883L *compass);
//HardWire HWire1(1, I2C_FAST_MODE); // I2c1




HardWire HWire1(1, I2C_BUS_RESET);//I2C_FAST_MODE
HardWire HWire2(2, I2C_BUS_RESET);

//#define COMPASS1
#define COMPASS2


#ifdef COMPASS1
HMC5883L compass;
#endif

#ifdef COMPASS2
HMC5883L compass2;
#endif
int previousDegree;

#define DebugPort  USBserial

void canvens(void);
float processing(HMC5883L *compass);

void HMC8885Task(void const *argument)
{
  DebugPort.begin(115200);
  DebugPort.println("HMC8885Task!");

  pinMode(PB3,OUTPUT);
  pinMode(PB5,OUTPUT);
  pinMode(PC13,INPUT_PULLUP);
  //rs485 send data enable
  //  while(hmc5883lInit()!=1){
  //      delay(1000);
  //      DebugPort.println("Try!");
  //  }

//  while(DebugPort.available()==0){
//      DebugPort.print("*");
//      delay(500);
//  }
  DebugPort.flush();
#ifdef COMPASS1
  // Initialize HMC5883L
    while (!compass.begin(&Wire1))
//  while (!compass.begin(&HWire1))
    {
      delay(500);
      DebugPort.println("compass try.!");
    }


  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
//  calibration(&compass);
#endif

#ifdef COMPASS2
  while (!compass2.begin(&Wire2))
//    while (!compass2.begin(&HWire2))
    {
      delay(500);
      DebugPort.println("compass2 try.!");
    }
  // Set measurement range
  compass2.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass2.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass2.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass2.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  compass2.setOffset(0, 0);
//  calibration(&compass2);

  //  checkSettings();
#endif
  int16_t mag[3];

  while(1)
    {

      canvens();

//      float dig1 = processing(&compass);
//      float dig2 = processing(&compass2);
//      DebugPort.print(dig1);
//      DebugPort.print(":");
//      DebugPort.print(dig2);
//      DebugPort.print(":");
//      DebugPort.print(dig2-dig1);
//      DebugPort.print("\n");
    }
}


void calibration(HMC5883L *compass)
{

  int minX = 0;
  int maxX = 0;
  int minY = 0;
  int maxY = 0;
  int offX = 0;
  int offY = 0;
  DebugPort.println("start Calibration");
  while(DebugPort.available()==0){
      // Determine Min / Max values
      Vector mag = compass->readRaw();
      if (mag.XAxis < minX) minX = mag.XAxis;
      if (mag.XAxis > maxX) maxX = mag.XAxis;
      if (mag.YAxis < minY) minY = mag.YAxis;
      if (mag.YAxis > maxY) maxY = mag.YAxis;
      DebugPort.print(".");
      delay(200);
  }
  DebugPort.flush();
  DebugPort.println("end Calibration");
  // Calculate offsets
  offX = (maxX + minX)/2;
  offY = (maxY + minY)/2;

  compass->setOffset(offX, offY);


}


void canvens(void)
{
  long x = micros();
  char buff[20];
  float v1=1;
#ifdef COMPASS1
  Vector norm = compass.readNormalize();
  // Output
  DebugPort.print("sensor1:");

  DebugPort.print(norm.XAxis);
  DebugPort.print(" : ");

  DebugPort.print(norm.YAxis);
  DebugPort.print(" : ");

  DebugPort.print(norm.ZAxis);
//  v1 = sqrt(norm.XAxis*norm.XAxis+norm.YAxis*norm.YAxis+norm.ZAxis*norm.ZAxis);
//  DebugPort.print(" : v=");
//  DebugPort.print(v1);
#endif
  //      digitalWrite(PB3,1);
  //      digitalWrite(PB5,1);
  DebugPort.println("");

  long x2 = micros();
  float v2;
#ifdef COMPASS2
  Vector norm2 = compass2.readNormalize();
  // Output
  DebugPort.print("sensor2:");

  DebugPort.print(norm2.XAxis);
  DebugPort.print(" : ");

  DebugPort.print(norm2.YAxis);
  DebugPort.print(" : ");

  DebugPort.print(norm2.ZAxis);
#endif

//#ifdef COMPASS1
//#ifdef COMPASS2
//  v2 = sqrt(norm2.XAxis*norm2.XAxis+norm2.YAxis*norm2.YAxis+norm2.ZAxis*norm2.ZAxis);
//  DebugPort.print(" : v=");
//  DebugPort.println(v2);
//  delay(200);
//  //      digitalWrite(PB3,0);
//  //      digitalWrite(PB5,0);
//  float dotProduct;
//  dotProduct = norm.XAxis*norm2.XAxis+norm.YAxis*norm2.YAxis+norm.ZAxis*norm2.ZAxis;
////      dotProduct = norm2.XAxis;
//  DebugPort.print("dotProduct:");
//  DebugPort.print(dotProduct);
//
//  float radian;
//  radian = acos(dotProduct/(v1*v2));
//  DebugPort.print("   radian:");
//  DebugPort.println(radian);
//#endif
//#endif
  DebugPort.println(" ");
//  DebugPort.println("");
  // One loop: ~5ms @ 115200 serial.
  // We need delay ~28ms for allow data rate 30Hz (~33ms)
  delay(400);
}


float processing(HMC5883L *compass)
{
  long x = micros();
  Vector norm = compass->readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = (heading * 180/M_PI);

  // Fix HMC5883L issue with angles
  float fixedHeadingDegrees;

  if (headingDegrees >= 1 && headingDegrees < 240)
  {
    fixedHeadingDegrees = map(headingDegrees, 0, 239, 0, 179);
  } else
  if (headingDegrees >= 240)
  {
    fixedHeadingDegrees = map(headingDegrees, 240, 360, 180, 360);
  }

  // Smooth angles rotation for +/- 3deg
  int smoothHeadingDegrees = round(fixedHeadingDegrees);

  if (smoothHeadingDegrees < (previousDegree + 3) && smoothHeadingDegrees > (previousDegree - 3))
  {
    smoothHeadingDegrees = previousDegree;
  }

  previousDegree = smoothHeadingDegrees;

////  // Output
//  DebugPort.print(norm.XAxis);
//  DebugPort.print(":");
//  DebugPort.print(norm.YAxis);
//  DebugPort.print(":");
//  DebugPort.print(norm.ZAxis);
//  DebugPort.print(":");
//  DebugPort.print(headingDegrees);
//  DebugPort.print(":");
//  DebugPort.print(fixedHeadingDegrees);
//  DebugPort.print(":");
//  DebugPort.print(smoothHeadingDegrees);
//  DebugPort.println();

  // One loop: ~5ms @ 115200 serial.
  // We need delay ~28ms for allow data rate 30Hz (~33ms)
  delay(30);
  return headingDegrees;
}



/**
 *
 */
void checkSettings(HMC5883L *compass)
{
  DebugPort.print("Selected range: ");

  switch (compass->getRange())
  {
    case HMC5883L_RANGE_0_88GA: DebugPort.println("0.88 Ga"); break;
    case HMC5883L_RANGE_1_3GA:  DebugPort.println("1.3 Ga"); break;
    case HMC5883L_RANGE_1_9GA:  DebugPort.println("1.9 Ga"); break;
    case HMC5883L_RANGE_2_5GA:  DebugPort.println("2.5 Ga"); break;
    case HMC5883L_RANGE_4GA:    DebugPort.println("4 Ga"); break;
    case HMC5883L_RANGE_4_7GA:  DebugPort.println("4.7 Ga"); break;
    case HMC5883L_RANGE_5_6GA:  DebugPort.println("5.6 Ga"); break;
    case HMC5883L_RANGE_8_1GA:  DebugPort.println("8.1 Ga"); break;
    default: DebugPort.println("Bad range!");
  }

  DebugPort.print("Selected Measurement Mode: ");
  switch (compass->getMeasurementMode())
  {
    case HMC5883L_IDLE: DebugPort.println("Idle mode"); break;
    case HMC5883L_SINGLE:  DebugPort.println("Single-Measurement"); break;
    case HMC5883L_CONTINOUS:  DebugPort.println("Continuous-Measurement"); break;
    default: Serial.println("Bad mode!");
  }

  DebugPort.print("Selected Data Rate: ");
  switch (compass->getDataRate())
  {
    case HMC5883L_DATARATE_0_75_HZ: DebugPort.println("0.75 Hz"); break;
    case HMC5883L_DATARATE_1_5HZ:  DebugPort.println("1.5 Hz"); break;
    case HMC5883L_DATARATE_3HZ:  DebugPort.println("3 Hz"); break;
    case HMC5883L_DATARATE_7_5HZ: DebugPort.println("7.5 Hz"); break;
    case HMC5883L_DATARATE_15HZ:  DebugPort.println("15 Hz"); break;
    case HMC5883L_DATARATE_30HZ: DebugPort.println("30 Hz"); break;
    case HMC5883L_DATARATE_75HZ:  DebugPort.println("75 Hz"); break;
    default: DebugPort.println("Bad data rate!");
  }

  DebugPort.print("Selected number of samples: ");
  switch (compass->getSamples())
  {
    case HMC5883L_SAMPLES_1: DebugPort.println("1"); break;
    case HMC5883L_SAMPLES_2: DebugPort.println("2"); break;
    case HMC5883L_SAMPLES_4: DebugPort.println("4"); break;
    case HMC5883L_SAMPLES_8: DebugPort.println("8"); break;
    default: DebugPort.println("Bad number of samples!");
  }

}

