/*
08-APR-2015

Blue LED Sled, 3 photoresistors, 3 output LEDs
will it ever get motors and fulfil its long lost destiny?
*/

#include <EEPROMex.h>

#include <IRremote.h>

#define HC_REMOTE_SPECIAL_FOR_MP3
#include <HC_RemoteCodes.h>

#define IR_REMOTE_PIN 11
IRrecv irRemote(IR_REMOTE_PIN);
decode_results irData;


#define PHOTO_PIN_LEFT A1
#define PHOTO_PIN_MID A2
#define PHOTO_PIN_RIGHT A3

#define NOSE_PIN_LEFT 9
#define NOSE_PIN_MID 10
#define NOSE_PIN_RIGHT 18
// Digital 18 is Analog 4 on UNO


#define MOTOR_ENABLE_PIN_L 6
#define MOTOR_ENABLE_PIN_R 5

#define MOTOR_DIR_PIN_L1 4
#define MOTOR_DIR_PIN_L2 8

#define MOTOR_DIR_PIN_R1 3
#define MOTOR_DIR_PIN_R2 19
// Digital 19 is Analog 5 on UNO

#define BATTERY_MONITOR_PIN A0
#define BATTERY_INDICATOR_PIN 12



#define LED_SIGNAL_PIN 13


// how frequently should I check the battery voltage?
#define BATTERY_MONITOR_INTERVAL_MILLIS 1000

// 660=6.4v, 690=6.7v (technically they are rounded up from 657 and 688)
// 20150501 Ok, so I let it get to 660 (voltage read 6.42) and now the batter won't charge back up. Bumping to 700=ERROR, 730=WARN
// after letting it sit for a little bit, reading 6.7v, but charger is still flashing red
#define BATTERY_LOW_ERROR 700
#define BATTERY_LOW_WARN 730

#define NOSE_DEFAULT_STATE LOW

#define MOTOR_SPEED_STOP 0
// real FULL=255, HALF=127
// 190 and 85 are working well
#define MOTOR_SPEED_FULL 255
#define MOTOR_SPEED_HALF 90

// add this fudge factor to the calibrated thresh, 50 was working for a bit but seemed to cause oversteer. maybe.
#define CALIBRATED_THRESHOLD_FUDGE_FACTOR 0

// how many millis to rotate in last seen line direction if no eyes see the line
#define EVASIVE_ROTATE_MILLIS 30
#define EVASIVE_RECENT_WINDOW_MILLIS 7000

#define CALIBRATION_DURATION_MILLIS 1650


#define MODE_PARK 0
#define MODE_DRIVE 1
#define MODE_ABOUT_FACE 2
#define MODE_CALIBRATE 3

#define EEPROM_ADDR_START 0

//#define BATTERY_MONITOR_DUMP_ENABLE
//#define EEPROM_BATTERY_DUMP_ENABLE
//#define SERIAL_ADC_DUMP_ENABLE
#define EEPROM_CALIBRATION_DUMP_ENABLE
#define EEPROM_DEFAULT_CALIBRATION


struct crabbyControl {
  int photoValueLeft = 0;
  int photoValueMid = 0;
  int photoValueRight = 0;

  // the calibration data will be set to defaults by setup()
  int calibratedMinLeft=0;
  int calibratedThreshLeft=0;
  int calibratedMaxLeft=0;
  int calibratedValueLeft=0;

  int calibratedMinMid=0;
  int calibratedThreshMid=0;
  int calibratedMaxMid=0;
  int calibratedValueMid=0;

  int calibratedMinRight=0;
  int calibratedThreshRight=0;
  int calibratedMaxRight=0;
  int calibratedValueRight=0;

  int pidLastError=0;
  int pidIntegral=0;
  int pidZeroPosition=0;

  boolean lineIsDark=false;

  boolean leftOverLine = false;
  boolean midOverLine = false;
  boolean rightOverLine = false;

  long lastSawLineLeft=0;
  long lastSawLineMid=0;
  long lastSawLineRight=0;

  int mode = MODE_PARK;



  long batteryMonitorCTRL=0;
  long pidSerialDumpCTRL=0;
  long interpretSerialDumpCTRL=0;

} crabby;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  pinMode(BATTERY_INDICATOR_PIN, OUTPUT);

  pinMode(MOTOR_ENABLE_PIN_L, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN_R, OUTPUT);

  pinMode(MOTOR_DIR_PIN_L1, OUTPUT);
  pinMode(MOTOR_DIR_PIN_L2, OUTPUT);
  pinMode(MOTOR_DIR_PIN_R1, OUTPUT);
  pinMode(MOTOR_DIR_PIN_R2, OUTPUT);

  analogWrite(MOTOR_ENABLE_PIN_L, LOW);
  analogWrite(MOTOR_ENABLE_PIN_R, LOW);

  digitalWrite(MOTOR_DIR_PIN_L1, LOW);
  digitalWrite(MOTOR_DIR_PIN_L2, LOW);
  digitalWrite(MOTOR_DIR_PIN_R1, LOW);
  digitalWrite(MOTOR_DIR_PIN_R2, LOW);


  pinMode(NOSE_PIN_LEFT, OUTPUT);
  pinMode(NOSE_PIN_MID, OUTPUT);
  pinMode(NOSE_PIN_RIGHT, OUTPUT);
  digitalWrite(NOSE_PIN_LEFT, NOSE_DEFAULT_STATE);
  digitalWrite(NOSE_PIN_MID, NOSE_DEFAULT_STATE);
  digitalWrite(NOSE_PIN_RIGHT, NOSE_DEFAULT_STATE);

  pinMode(LED_SIGNAL_PIN, OUTPUT);
  digitalWrite(LED_SIGNAL_PIN, LOW);

  clearCalibration();


#ifdef EEPROM_BATTERY_DUMP_ENABLE
  // dump the battery monitor ADC value from EEPROM
  Serial.print("Battery value from EEPROM:\t");
  // ADDR+22 is where battery is saved
  Serial.println(EEPROM.readInt(EEPROM_ADDR_START+22));

#endif


  // dump any existing calibration data from EEPROM to Serial, lets me use battery to run a calibration, then plug to USB and get results
#ifdef EEPROM_CALIBRATION_DUMP_ENABLE
  Serial.println("Calibration Data in EEPROM:");
  
  Serial.print("Left  (min, thresh, max): ");
  Serial.print(EEPROM.readInt(EEPROM_ADDR_START+0));
  Serial.print("\t");
  Serial.print(EEPROM.readInt(EEPROM_ADDR_START+2));
  Serial.print("\t");
  Serial.print(EEPROM.readInt(EEPROM_ADDR_START+4));

  Serial.print("\nMid   (min, thresh, max): ");
  Serial.print(EEPROM.readInt(EEPROM_ADDR_START+6));
  Serial.print("\t");
  Serial.print(EEPROM.readInt(EEPROM_ADDR_START+8));
  Serial.print("\t");
  Serial.print(EEPROM.readInt(EEPROM_ADDR_START+10));

  Serial.print("\nRight (min, thresh, max): ");
  Serial.print(EEPROM.readInt(EEPROM_ADDR_START+12));
  Serial.print("\t");
  Serial.print(EEPROM.readInt(EEPROM_ADDR_START+14));
  Serial.print("\t");
  Serial.println(EEPROM.readInt(EEPROM_ADDR_START+16));
  
  Serial.print("Line is Dark? ");
  Serial.println(EEPROM.readInt(EEPROM_ADDR_START+18));
  
  Serial.print("PID Zero Position: ");
  Serial.println(EEPROM.readInt(EEPROM_ADDR_START+20));
  
#endif

#ifdef EEPROM_DEFAULT_CALIBRATION
  crabby.calibratedMinLeft    = EEPROM.readInt(EEPROM_ADDR_START+0);
  crabby.calibratedThreshLeft = EEPROM.readInt(EEPROM_ADDR_START+2);
  crabby.calibratedMaxLeft    = EEPROM.readInt(EEPROM_ADDR_START+4);

  crabby.calibratedMinMid    = EEPROM.readInt(EEPROM_ADDR_START+6);
  crabby.calibratedThreshMid = EEPROM.readInt(EEPROM_ADDR_START+8);
  crabby.calibratedMaxMid    = EEPROM.readInt(EEPROM_ADDR_START+10);

  crabby.calibratedMinRight    = EEPROM.readInt(EEPROM_ADDR_START+12);
  crabby.calibratedThreshRight = EEPROM.readInt(EEPROM_ADDR_START+14);
  crabby.calibratedMaxRight    = EEPROM.readInt(EEPROM_ADDR_START+16);
  
  crabby.lineIsDark            = EEPROM.readInt(EEPROM_ADDR_START+18)==1?true:false;
  
  crabby.pidZeroPosition       = EEPROM.readInt(EEPROM_ADDR_START+20);
#endif 



  irRemote.enableIRIn();
}

void loop() {
  
  if ((millis() - crabby.batteryMonitorCTRL) > BATTERY_MONITOR_INTERVAL_MILLIS) {
    crabby.batteryMonitorCTRL = millis();

   delay(1);
   int batteryMonitorRaw = analogRead(BATTERY_MONITOR_PIN);
#ifdef BATTERY_MONITOR_DUMP_ENABLE
   Serial.print("Battery ADC Value:\t");
   Serial.println(batteryMonitorRaw); 
#endif


  // if we are on USB/FTDI power and no battery is plugged in, we get values around 26 off the ADC.  So, only process if the ADC reading is greater than 100
  if (batteryMonitorRaw>100) {
    if (batteryMonitorRaw<BATTERY_LOW_ERROR) {
      // too low, blink indicator and shut off subsystems
      //easiest way to block subsystems is to infinite loop right here :-)
      
      // save power by shutting down motors and nose LEDs
      setMotors(MOTOR_SPEED_STOP, HIGH, HIGH, MOTOR_SPEED_STOP, HIGH, HIGH);
      digitalWrite(NOSE_PIN_LEFT, LOW);
      digitalWrite(NOSE_PIN_MID, LOW);
      digitalWrite(NOSE_PIN_RIGHT, LOW);
            
      
      while (true) {
        // you'll need to reset the chip to get out of this loop
        delay(1500);
        digitalWrite(BATTERY_INDICATOR_PIN, HIGH);
        delay(500);
        digitalWrite(BATTERY_INDICATOR_PIN, LOW);
      }
    } else if (batteryMonitorRaw<BATTERY_LOW_WARN) {
      // battery is getting low, turn on the indicator and do nothing else
      digitalWrite(BATTERY_INDICATOR_PIN, HIGH);
    } else {
       // battery is fine, turn the indicator off
       digitalWrite(BATTERY_INDICATOR_PIN, LOW);
    }
  }
  }  


  // optionally put in a timer so it only updates the values on my schedule
  readPhotoSensors();

  // interpret the data
  interpretPhotoSensors();

  // show the adoring fans which sensors see lines
  rudolphWithYourNoseSoBright();

#ifdef SERIAL_ADC_DUMP_ENABLE
  // print the results to the serial monitor:
  Serial.print(crabby.photoValueLeft);
  Serial.print("\t");
  Serial.print(crabby.photoValueMid);
  Serial.print("\t");
  Serial.println(crabby.photoValueRight);
#endif

  if (irRemote.decode(&irData)) {
    // we have instructions from the operator
    irRemote.resume();

    Serial.print(irData.value, HEX);

    switch (irData.value) {
      case HC_RC_POWER:
        Serial.println("\tPOWER");
        // power is always STOP
        crabby.mode = MODE_PARK;
        break;
      case HC_RC_PLAYPAUSE:
        // toggle the mode
        crabby.mode = (crabby.mode == MODE_PARK) ? MODE_DRIVE : MODE_PARK;
        Serial.print("\tPLAYPAUSE, new mode is ");
        Serial.println(crabby.mode);
        break;
      case HC_RC_SQUIGGLE:
        // only if we're stopped, put us in MODE_ABOUT_FACE
        if (crabby.mode==MODE_PARK) {
          crabby.mode=MODE_ABOUT_FACE;
          Serial.println("\tSQUIGGLE while parked, begin ABOUT_FACE");
        } else {
          Serial.println("\tSQUIGGLE no park, no ABOUT_FACE");
        }
        break;
      case HC_RC_MODE:
        // only if we're stopped, begin calibration
        if (crabby.mode==MODE_PARK) {
          crabby.mode=MODE_CALIBRATE;
          Serial.println("\tMODE while parked, begin CALIBRATION");
        } else {
          Serial.println("\tMODE no park, no CALIBRATE");
        }
        break;
      case HC_RC_EQ:
        // user manually centered us on the line, calculate the position
        if (crabby.mode==MODE_PARK) {
          crabby.pidZeroPosition = -1 * (crabby.calibratedValueLeft + crabby.calibratedValueMid + crabby.calibratedValueRight);
#if defined(EEPROM_CALIBRATION_DUMP_ENABLE) || defined(EEPROM_DEFAULT_CALIBRATION)
          EEPROM.writeInt(EEPROM_ADDR_START+20, crabby.pidZeroPosition);
#endif
          Serial.print("\tEQ while parked, recording zero position:\t");
          Serial.println(crabby.pidZeroPosition);
        } else {
          Serial.println("\tEQ no park, no zero position calcs performed");
        }
        break;
      case HC_RC_USD:
#ifdef EEPROM_BATTERY_DUMP_ENABLE
        // save the raw value read to EEPROM at ADDR+18
        delay(10);
        EEPROM.writeInt(EEPROM_ADDR_START+22, analogRead(BATTERY_MONITOR_PIN));
        Serial.println("Wrote Battery ADC Value to EEPROM");

#endif
        break;
      case REPEAT:
        Serial.println("\tREPEAT, ignore");
        break;
      default:
        // ignore any other button presses
        Serial.println("\tunknown IR command");
        break;
    }
  }

  if (crabby.mode == MODE_ABOUT_FACE) {
    // spin the bot on its axis for a fixed number of seconds.
    setMotors(MOTOR_SPEED_FULL, HIGH, LOW, MOTOR_SPEED_FULL, LOW, HIGH);
//    delay(4700);
    delay((int)(CALIBRATION_DURATION_MILLIS/2));
    crabby.mode = MODE_PARK;
    
  } else if (crabby.mode == MODE_CALIBRATE) {
    doCalibrate();
    crabby.mode = MODE_PARK;
    
  } else if (crabby.mode == MODE_DRIVE) {


  // ALGORITHM SELECTOR.  Set this to "if (true)" to do manual navigating, and "if (false)" to do PID navigating
  if (true) {
    // BEGIN MANUAL SECTION (i.e. not PID)

    if (crabby.midOverLine && crabby.leftOverLine && crabby.rightOverLine) {
      // all 3 sensors see the line (wide line, eh.), full steam ahead
      setMotors(MOTOR_SPEED_FULL, HIGH, LOW, MOTOR_SPEED_FULL, HIGH, LOW);

    } else if (crabby.midOverLine) {
      // we're at least partially centered, lets see which side is off
      if (crabby.leftOverLine) {
        // too far right, need to correct back to the left by slowing the left motor, keep right full
        setMotors(MOTOR_SPEED_HALF, HIGH, LOW, MOTOR_SPEED_FULL, HIGH, LOW);

      } else {
        // too far left, need to correct back to the right by slowing the right motor, keep left full
        setMotors(MOTOR_SPEED_FULL, HIGH, LOW, MOTOR_SPEED_HALF, HIGH, LOW);
      }

    } else if (crabby.leftOverLine && !crabby.rightOverLine) {
      // we're too far to the right, extreme steering needed, power right only
      setMotors(MOTOR_SPEED_STOP, LOW, LOW, MOTOR_SPEED_FULL, HIGH, LOW);

    } else if (crabby.rightOverLine && !crabby.leftOverLine) {
      // we're too far to the left, extreme steering needed, power left only
      setMotors(MOTOR_SPEED_FULL, HIGH, LOW, MOTOR_SPEED_STOP, LOW, LOW);
      
    } else if (!crabby.leftOverLine && !crabby.midOverLine && !crabby.rightOverLine) {
      // none of them are over the line, take evasive action

      if ((millis()-crabby.lastSawLineLeft)<EVASIVE_RECENT_WINDOW_MILLIS && crabby.lastSawLineLeft>=crabby.lastSawLineRight) {
        // left saw the line (A) recently and (B) more recently than the right did
        // left gets a ">=" to tie-break in the event the two sides saw the line exactly the same time
        
        // rotate tiny bit left
        setMotors(MOTOR_SPEED_HALF, LOW, HIGH, MOTOR_SPEED_FULL, HIGH, LOW);
        delay(EVASIVE_ROTATE_MILLIS);
        
      } else if ((millis()-crabby.lastSawLineRight)<EVASIVE_RECENT_WINDOW_MILLIS && crabby.lastSawLineLeft<crabby.lastSawLineRight) {
        // right saw the line (A) recently and (B) more recently than the left did
        
        // rotate tiny bit right
        setMotors(MOTOR_SPEED_FULL, HIGH, LOW, MOTOR_SPEED_HALF, LOW, HIGH);
        delay(EVASIVE_ROTATE_MILLIS);
      }
    }
/*
    if (crabby.midOverLine && crabby.leftOverLine && crabby.rightOverLine) {
      // all 3 sensors see the line (wide line, eh.), full steam ahead
      setMotors(MOTOR_SPEED_FULL, HIGH, LOW, MOTOR_SPEED_FULL, HIGH, LOW);

    } else if (crabby.midOverLine) {
      // we're at least partially centered, lets see which side is off
      if (crabby.leftOverLine) {
        // too far right, need to correct back to the left by slowing the left motor, keep right full
        setMotors(MOTOR_SPEED_HALF, HIGH, LOW, MOTOR_SPEED_FULL, HIGH, LOW);

      } else {
        // too far left, need to correct back to the right by slowing the right motor, keep left full
        setMotors(MOTOR_SPEED_FULL, HIGH, LOW, MOTOR_SPEED_HALF, HIGH, LOW);
      }

    } else if (crabby.leftOverLine && !crabby.rightOverLine) {
      // we're too far to the right, extreme steering needed, power right only
      setMotors(MOTOR_SPEED_STOP, LOW, LOW, MOTOR_SPEED_FULL, HIGH, LOW);

    } else if (crabby.rightOverLine && !crabby.leftOverLine) {
      // we're too far to the left, extreme steering needed, power left only
      setMotors(MOTOR_SPEED_FULL, HIGH, LOW, MOTOR_SPEED_STOP, LOW, LOW);

    } else {
      // none of the eyes see the line.  Did either side recently see a line? if so we can try to reclaim the line.
      if ((millis()-crabby.lastSawLineLeft)<EVASIVE_RECENT_WINDOW_MILLIS && crabby.lastSawLineLeft>=crabby.lastSawLineRight) {
        // left saw the line (A) recently and (B) more recently than the right did
        // left gets a ">=" to tie-break in the event the two sides saw the line exactly the same time
        
        // rotate tiny bit left
        setMotors(MOTOR_SPEED_HALF, LOW, HIGH, MOTOR_SPEED_FULL, HIGH, LOW);
        delay(EVASIVE_ROTATE_MILLIS);
        
      } else if ((millis()-crabby.lastSawLineRight)<EVASIVE_RECENT_WINDOW_MILLIS && crabby.lastSawLineLeft<crabby.lastSawLineRight) {
        // right saw the line (A) recently and (B) more recently than the left did
        
        // rotate tiny bit right
        setMotors(MOTOR_SPEED_FULL, HIGH, LOW, MOTOR_SPEED_HALF, LOW, HIGH);
        delay(EVASIVE_ROTATE_MILLIS);
      } else {
        // none of them saw the line recently, stop the motors and let the next iteration pick up if we found the line.
        setMotors(MOTOR_SPEED_STOP, LOW, LOW, MOTOR_SPEED_STOP, LOW, LOW);
      }
    }
*/
    
  } else {
    // BEGIN PID SECTION
    
    // pidError is <0 if to the left, 0 if centered, >0 if to the right
    // the zeroPosition is determined at calibration, a value to zero out the system when centered
    int pidError = crabby.calibratedValueLeft+crabby.calibratedValueMid+crabby.calibratedValueRight+crabby.pidZeroPosition;
    if (crabby.calibratedValueLeft>crabby.calibratedValueRight) {
      pidError *= -1;
    }
        
    // integral has total error over time
    crabby.pidIntegral = crabby.pidIntegral + pidError;

    // derivative is rate of change in error from one loop to the next        
    int pidDerivative = pidError - crabby.pidLastError;

    int rightBaseSpeed=160;
    int rightMaxSpeed=230;

    int leftBaseSpeed=160;
    int leftMaxSpeed=230;

/*
      int rightBaseSpeed=120;
      int rightMaxSpeed=160;

      int leftBaseSpeed=120;
      int leftMaxSpeed=160;
*/
    // the magic happens here, tune those PID constants!
    //                           Kp                              Ki                    Kd
    int pidValue = (pidError * 110 / 100) + (crabby.pidIntegral * 0) + (pidDerivative * 45);     

    crabby.pidLastError = pidError;
      
    int leftMotorSpeed  = constrain(leftBaseSpeed  - pidValue, 0, leftMaxSpeed);
    int rightMotorSpeed = constrain(rightBaseSpeed + pidValue, 0, rightMaxSpeed);

    setMotors(leftMotorSpeed, HIGH, LOW, rightMotorSpeed, HIGH, LOW);    
  }  
    
    
  } else {
    // default action (including MODE_PARK) is to stop moving
    setMotors(MOTOR_SPEED_STOP, LOW, LOW, MOTOR_SPEED_STOP, LOW, LOW);
  }
}

void readPhotoSensors() {
  // we're intentionally eating 4 millis = 0.4% of processing time to allow for input multiplexing

  delay(1);  // stupid multiplexed high impedance analog inputs
  crabby.photoValueLeft = analogRead(PHOTO_PIN_LEFT);
  delay(1);  // stupid multiplexed high impedance analog inputs
  crabby.photoValueMid = analogRead(PHOTO_PIN_MID);
  delay(1);  // stupid multiplexed high impedance analog inputs
  crabby.photoValueRight = analogRead(PHOTO_PIN_RIGHT);
  delay(1);  // stupid multiplexed high impedance analog inputs
}

void clearCalibration() {
  // the ADC returns 0-1023 inclusive
  crabby.calibratedMinLeft=1024;
  crabby.calibratedThreshLeft=-1;
  crabby.calibratedMaxLeft=0;

  crabby.calibratedMinMid=1024;
  crabby.calibratedThreshMid=-1;
  crabby.calibratedMaxMid=0;

  crabby.calibratedMinRight=1024;
  crabby.calibratedThreshRight=-1;
  crabby.calibratedMaxRight=0;
}

void doCalibrate() {

  setMotors(MOTOR_SPEED_FULL, LOW, HIGH, MOTOR_SPEED_FULL, HIGH, LOW);

  
  long calibrateBeginMillis=millis();
  
  int leftVals[100];
  int midVals[100];
  int rightVals[100];
  
  int i=0;
  while ((millis()-calibrateBeginMillis)<CALIBRATION_DURATION_MILLIS) {
    // read the photo values
    readPhotoSensors();
    
    // store the photo values
    leftVals[i]=crabby.photoValueLeft;
    midVals[i]=crabby.photoValueMid;
    rightVals[i]=crabby.photoValueRight;
    
    i++;
    
    // delay such that we cannot do more than 100 of these in the interval (knowing readPhotoSensors() takes at least 4 millis)
//    Serial.print("Delaying in calibration for millis:\t");
//    Serial.print((int)((CALIBRATION_DURATION_MILLIS-4)/100));
//    Serial.println();
    
    delay((int)((CALIBRATION_DURATION_MILLIS-4)/100));
  }

  setMotors(MOTOR_SPEED_STOP, HIGH, HIGH, MOTOR_SPEED_STOP, HIGH, HIGH);

  
  /* now crunch the numbers for each sensor:
   * 1) the overall average is the threshold
   * 2) The average of everything under the first threshold is the min
   * 3) The average of everything over the first threshold is the max  
   */
  int meanOverall, meanBelow, meanAbove;

  // slight change in plans.  using the mean puts the threshold too close to the background (non-line) color, so
  // the real threshold will be the midpoint between the avg of those points below the mean and the avg of those points above the mean.
  meanOverall = arrayMean(leftVals, i);
  // assume dark line on light bg, meanBelow is likely the color of the background (as seen by this sensor)
  meanBelow = arrayMeanThreshold(leftVals, meanOverall, true, i);
  // assume dark line on light bg, meanAbove is likely the color of the line (as seen by this sensor)
  meanAbove = arrayMeanThreshold(leftVals, meanOverall, false, i);
  
  crabby.calibratedMinLeft = meanBelow;
  crabby.calibratedThreshLeft = (int)((meanBelow+meanAbove)/2);
  crabby.calibratedMaxLeft = meanAbove;




  meanOverall = arrayMean(midVals, i);
  // assume dark line on light bg, meanBelow is likely the color of the background (as seen by this sensor)
  meanBelow = arrayMeanThreshold(midVals, meanOverall, true, i);
  // assume dark line on light bg, meanAbove is likely the color of the line (as seen by this sensor)
  meanAbove = arrayMeanThreshold(midVals, meanOverall, false, i);

  crabby.calibratedMinMid = meanBelow;
  crabby.calibratedThreshMid = (int)((meanBelow+meanAbove)/2);
  crabby.calibratedMaxMid = meanAbove;





  meanOverall = arrayMean(rightVals, i);
  // assume dark line on light bg, meanBelow is likely the color of the background (as seen by this sensor)
  meanBelow = arrayMeanThreshold(rightVals, meanOverall, true, i);
  // assume dark line on light bg, meanAbove is likely the color of the line (as seen by this sensor)
  meanAbove = arrayMeanThreshold(rightVals, meanOverall, false, i);

  crabby.calibratedMinRight = meanBelow;
  crabby.calibratedThreshRight = (int)((meanBelow+meanAbove)/2);
  crabby.calibratedMaxRight = meanAbove;

  
/*
  crabby.calibratedMinLeft = arrayMin(leftVals, i);
  crabby.calibratedMinMid = arrayMin(midVals,i);
  crabby.calibratedMinRight = arrayMin(rightVals,i);
  
  crabby.calibratedMaxLeft = arrayMax(leftVals,i);
  crabby.calibratedMaxMid = arrayMax(midVals,i);
  crabby.calibratedMaxRight = arrayMax(rightVals,i);
*/


  // determine if the line is darker or lighter than the background.
  // the assumption is that you see more background than line during calibration, so 
  // the mean will be closer to the background (which is either min or max)
  // NOTE: we're simply using the values for the right eye, which remain sitting in our 3 vars used above
  
  
  if ((crabby.calibratedMaxRight-meanOverall)<(meanOverall-crabby.calibratedMinRight)) {
//  if ((arrayMax(rightVals,i)-meanOverall)<(meanOverall-arrayMin(rightVals,i))) {
  /*
    Serial.print("distance: max(");
    Serial.print(arrayMax(rightVals,i));
    Serial.print(") mean(");
    Serial.print(meanOverall);
    Serial.print(") min(");
    Serial.print(arrayMin(rightVals,i));
    Serial.print(") dist (");
    Serial.print(arrayMax(rightVals,i)-meanOverall);
    Serial.print(") (");
    Serial.print(meanOverall-arrayMin(rightVals,i));
    Serial.println(")");
*/
    // the gap between mean and max is smaller than the gap between mean and min, so bg is darker
    crabby.lineIsDark=false;
  } else {
    crabby.lineIsDark=true;
  }
  
  

  Serial.println("Calibration Data:");
  
  Serial.print("Left  (min, thresh, max): ");
  Serial.print(crabby.calibratedMinLeft);
  Serial.print("\t");
  Serial.print(crabby.calibratedThreshLeft);
  Serial.print("\t");
  Serial.print(crabby.calibratedMaxLeft);

  Serial.print("\nMid   (min, thresh, max): ");
  Serial.print(crabby.calibratedMinMid);
  Serial.print("\t");
  Serial.print(crabby.calibratedThreshMid);
  Serial.print("\t");
  Serial.print(crabby.calibratedMaxMid);

  Serial.print("\nRight (min, thresh, max): ");
  Serial.print(crabby.calibratedMinRight);
  Serial.print("\t");
  Serial.print(crabby.calibratedThreshRight);
  Serial.print("\t");
  Serial.println(crabby.calibratedMaxRight);
  
  if (crabby.lineIsDark) {
    Serial.println("Following a DARK line");
  } else {
    Serial.println("Following a LITE line");
  }
  
#if defined(EEPROM_CALIBRATION_DUMP_ENABLE) || defined(EEPROM_DEFAULT_CALIBRATION)
  EEPROM.writeInt(EEPROM_ADDR_START+0, crabby.calibratedMinLeft);
  EEPROM.writeInt(EEPROM_ADDR_START+2, crabby.calibratedThreshLeft);
  EEPROM.writeInt(EEPROM_ADDR_START+4, crabby.calibratedMaxLeft);

  EEPROM.writeInt(EEPROM_ADDR_START+6, crabby.calibratedMinMid);
  EEPROM.writeInt(EEPROM_ADDR_START+8, crabby.calibratedThreshMid);
  EEPROM.writeInt(EEPROM_ADDR_START+10, crabby.calibratedMaxMid);

  EEPROM.writeInt(EEPROM_ADDR_START+12, crabby.calibratedMinRight);
  EEPROM.writeInt(EEPROM_ADDR_START+14, crabby.calibratedThreshRight);
  EEPROM.writeInt(EEPROM_ADDR_START+16, crabby.calibratedMaxRight);
  
  EEPROM.writeInt(EEPROM_ADDR_START+18, crabby.lineIsDark?1:0);
#endif
}

int arrayMean(int* list, int numVals) {
 long total = 0;
  for (int i=0; i<numVals; i++) {
   total+= list[i];
  }
 return (int)(total/numVals); 
}
int arrayMeanThreshold(int* list, int threshold, boolean isBelow, int numVals) {
 long total = 0;
 int count=0;
  for (int i=0; i<numVals; i++) {
    if ((isBelow && list[i]<threshold)||(!isBelow && list[i]>threshold)) {
     total+= list[i];
     count++;
    }
  }
 return (int)(total/count); 
}
int arrayMax(int* list, int numVals) {
 int max=-1;
  for (int i=0; i<numVals; i++) {
   if (list[i]>max) {
     max = list[i];
   }
  }
 return max; 
}
int arrayMin(int* list, int numVals) {
 int min=1025;
  for (int i=0; i<numVals; i++) {
   if (list[i]<min) {
     min = list[i];
   }
  }
 return min; 
}

void interpretPhotoSensors() {
  // for now, white paper is considered NOT line, black toner and blue tape are considered LINE.

  /* some experimental values that seem to work:
   * blue tape on white paper: 510, 340, 480
   * red tape on office carpet: 680, 515, 675
   */

  int workingThreshLeft  = crabby.calibratedThreshLeft<0?680:crabby.calibratedThreshLeft+CALIBRATED_THRESHOLD_FUDGE_FACTOR;
  int workingThreshMid   = crabby.calibratedThreshMid<0?515:crabby.calibratedThreshMid+CALIBRATED_THRESHOLD_FUDGE_FACTOR;
  int workingThreshRight = crabby.calibratedThreshRight<0?675:crabby.calibratedThreshRight+CALIBRATED_THRESHOLD_FUDGE_FACTOR;
  
  if (( crabby.lineIsDark && crabby.photoValueLeft > workingThreshLeft) ||
      (!crabby.lineIsDark && crabby.photoValueLeft < workingThreshLeft)) {
    crabby.leftOverLine = true;
    crabby.lastSawLineLeft=millis();
  } else {
    crabby.leftOverLine = false;
  }

  if (( crabby.lineIsDark && crabby.photoValueMid > workingThreshMid) ||
      (!crabby.lineIsDark && crabby.photoValueMid < workingThreshMid)) {
    crabby.midOverLine = true;
    crabby.lastSawLineMid=millis();
  } else {
    crabby.midOverLine = false;
  }

  if (( crabby.lineIsDark && crabby.photoValueRight > workingThreshRight) ||
      (!crabby.lineIsDark && crabby.photoValueRight < workingThreshRight)) {
    crabby.rightOverLine = true;
    crabby.lastSawLineRight=millis();
  } else {
    crabby.rightOverLine = false;
  }

  if (crabby.lineIsDark) {
    // distance from Line color
    crabby.calibratedValueLeft = crabby.photoValueLeft - crabby.calibratedMaxLeft;
    crabby.calibratedValueMid = crabby.photoValueMid - crabby.calibratedMaxMid;
    crabby.calibratedValueRight = crabby.photoValueRight - crabby.calibratedMaxRight;
  } else {
    crabby.calibratedValueLeft = crabby.photoValueLeft - crabby.calibratedMinLeft;
    crabby.calibratedValueMid = crabby.photoValueMid - crabby.calibratedMinMid;
    crabby.calibratedValueRight = crabby.photoValueRight - crabby.calibratedMinRight;
  }    



  if ((millis()-crabby.interpretSerialDumpCTRL)>200) {
    crabby.interpretSerialDumpCTRL=millis();
    Serial.print("L:\t");
    Serial.print(crabby.lineIsDark?crabby.calibratedMaxLeft:crabby.calibratedMinLeft);
    Serial.print("\t");
    Serial.print(crabby.photoValueLeft);
    Serial.print("\t");
    Serial.print(crabby.calibratedValueLeft);

    Serial.print("\tM:\t");
    Serial.print(crabby.lineIsDark?crabby.calibratedMaxMid:crabby.calibratedMinMid);
    Serial.print("\t");
    Serial.print(crabby.photoValueMid);
    Serial.print("\t");
    Serial.print(crabby.calibratedValueMid);

    Serial.print("\tR:\t");
    Serial.print(crabby.lineIsDark?crabby.calibratedMaxRight:crabby.calibratedMinRight);
    Serial.print("\t");
    Serial.print(crabby.photoValueRight);
    Serial.print("\t");
    Serial.print(crabby.calibratedValueRight);
  
    Serial.print("\t");
    Serial.print(crabby.calibratedValueLeft + crabby.calibratedValueMid + crabby.calibratedValueRight + crabby.pidZeroPosition);
    Serial.println();
  }

  
  

}

void rudolphWithYourNoseSoBright() {
  digitalWrite(NOSE_PIN_LEFT, crabby.leftOverLine ? HIGH : LOW);
  digitalWrite(NOSE_PIN_MID, crabby.midOverLine ? HIGH : LOW);
  digitalWrite(NOSE_PIN_RIGHT, crabby.rightOverLine ? HIGH : LOW);
}

void setMotors(int leftEnable, boolean leftA, boolean leftB, int rightEnable, boolean rightA, boolean rightB) {
  analogWrite(MOTOR_ENABLE_PIN_L, leftEnable);
  analogWrite(MOTOR_ENABLE_PIN_R, rightEnable);

  digitalWrite(MOTOR_DIR_PIN_L1, leftA);
  digitalWrite(MOTOR_DIR_PIN_L2, leftB);
  digitalWrite(MOTOR_DIR_PIN_R1, rightA);
  digitalWrite(MOTOR_DIR_PIN_R2, rightB);
}
