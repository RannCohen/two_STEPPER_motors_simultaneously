#include <iom6450.h>                                                          // for p manipulation

// communication
String inputString = "";                                                      // a String to hold incoming data
bool stringComplete = false;                                                  // whether the string is complete
// --

// general
int16_t system_err = 1;                                                       // return values from functions
// --

typedef struct Motor
{
  int dir_mask;                                                               // direction pin
  int pulse_mask;                                                             // pulse pin
  bool rev;                                                                   // reverse
  int32_t cur_pos;                                                            // current position
  int32_t req_pos;                                                            // required position
  const int Home_mask;                                                        // Home pin for every axile
} Motor;

//          dir_mask | pulse_mask | rev | cur_pos | req_pos | Home_mask
//           76543210   76543210                              76543210
Motor M1 = {B00001000, B00010000,    0,      0,       0,     B00100000};
Motor M2 = {B00010000, B00100000,    0,      0,       0,     B01000000};
Motor M3 = {B00100000, B00100000,    0,      0,       0,     B10000000};

Motor* motors[] = {&M1, &M2, &M3};

void setup()
{
  Serial.begin(115200);
  inputString.reserve(200);                                                   // reserve 200 bytes for the string
  // motor 1 setup
  DDRH |= (M1.dir_mask);                                                      // pin 6 as output
  PORTH &= ~(M1.dir_mask);                                                    // write 6 as low
  DDRE |= (M1.pulse_mask);                                                    // pin 2 as output
  PORTE &= ~(M1.pulse_mask);                                                  // write 2 as low
  // motor 2 setup
  DDRH |= (M2.dir_mask);                                                      // pin 7 as output
  PORTH &= ~(M2.dir_mask);                                                    // write 7 as low
  DDRE |= (M2.pulse_mask);                                                    // pin 3 as output
  PORTE &= ~(M2.pulse_mask);                                                  // write 3 as low
  // motor 3 setup
  DDRH |= (M3.dir_mask);                                                      // pin 8 as output
  PORTH &= ~(M3.dir_mask);                                                    // write 8 as low
  DDRG |= (M3.pulse_mask);                                                    // pin 4 as output
  PORTG &= ~(M3.pulse_mask);                                                  // write 4 as low
  // home sensor
  DDRB &= ~(M1.Home_mask);                                                    // pin 11 as input
  DDRB &= ~(M2.Home_mask);                                                    // pin 12 as input

  Serial.println("Hello Gantry");
}


void loop()
{
  if (stringComplete)
  {
    Serial.println(inputString);
    if (inputString == "h" || inputString == "H")
    {
      system_err = goHome(M1, M2);
    }
    else
    {
      int dist = inputString.toInt();
      if (((dist == 0 && inputString == "0") || dist) && (!system_err))       // (if dist is 0 & string is 0 or just dist) and system error is false
      {
        M1.req_pos = movment(dist);
        M2.req_pos = M1.req_pos;
        if (!system_err) system_err = motorGo(M1, M2);
      }
      else Serial.println("EMERGENCY STOP");
    }
    inputString = "";                                                          // clear the string for the next loop
    stringComplete = false;                                                    // set the string flag down
  }
  delay(5);
}

// communication ISR
void serialEvent()
{
  while (Serial.available())
  {
    char inChar = (char)Serial.read();                                         // get the new char:
    if (inChar == '\n')
      stringComplete = true;                                                   // if the incoming character is a newline, set a flag so the main loop can do something about it:
    else
      inputString += inChar;                                                   // add it to the inputString:
  }
}
// --

int32_t movment(int32_t x) {
  return constrain(x, 0, 1150) * 500;                                         // calculate and constrain mm to clicks
}


int motorGo(Motor &M_A, Motor &M_B)
{
  int32_t err_A;                                                               // for the closed loop error
  int32_t err_B;                                                               // for the closed loop error
  err_A = M_A.req_pos - M_A.cur_pos;                                           // calc the error
  err_B = M_B.req_pos - M_B.cur_pos;                                           // calc the error
  int32_t clicks_done_A = 0;                                                   // for acc and desc
  int32_t clicks_done_B = 0;                                                   // for acc and desc
  int period = 15;                                                             // delay for pulses
  while (err_A || err_B)                                                       // as long that there is an error
  {
    if (err_A > 0)                                                // if the error is greater then zero
    {
      PORTH |= M_A.dir_mask;                                                   // set direction for motor A
      M_A.cur_pos++;                                                           // add step for the cur pos of motor A
      err_A--;                                                                 // subtract step from motor A
      clicks_done_A++;                                                         // add step for the clicks done (for the acc and desc)
    } else if (err_A < 0) {
      PORTH &= ~M_A.dir_mask;                                                   // set direction for motor A
      M_A.cur_pos--;                                                           // add step for the cur pos of motor A
      err_A++;                                                                 // subtract step from motor A
      clicks_done_A++;
    }

    if (err_B > 0)                                                // if the error is greater then zero
    {
      PORTH |= M_B.dir_mask;                                                   // set direction for motor A
      M_B.cur_pos++;                                                           // add step for the cur pos of motor A
      err_B--;                                                                 // subtract step from motor A
      clicks_done_B++;                                                         // add step for the clicks done (for the acc and desc)
    } else if (err_B < 0) {
      PORTH &= ~M_B.dir_mask;                                                   // set direction for motor A
      M_B.cur_pos--;                                                           // add step for the cur pos of motor A
      err_B++;                                                                 // subtract step from motor A
      clicks_done_B++;
    }
  
    PORTE |= M_A.pulse_mask;                                                   // pulse A high
    PORTE |= M_B.pulse_mask;                                                   // pulse B high
    delayMicroseconds(period);
    PORTE &= ~M_A.pulse_mask;                                                  // pulse A low
    PORTE &= ~M_B.pulse_mask;                                                  // pulse A low
    delayMicroseconds(period);
    // acceleration
    if ((clicks_done_A == 10000)) period = 12;
    if ((clicks_done_A == 30000)) period = 8;
    // deceleration
    if ((abs(err_A) == 30000)) period = 10;
    if ((abs(err_A) == 10000)) period = 12;
    if (Serial.available()) return 1;
  }
  delay(50);
  Serial.println("Done my move");
  return 0;
}


int goHome(Motor &M_A, Motor &M_B)
{
  while ((PINB >> 5 & B00100000 >> 5) || (PINB >> 6 & B01000000 >> 6))        //digital read of pins 11&12
  {
    PORTH &= ~M_A.dir_mask;                                                   //backword
    PORTH &= ~M_B.dir_mask;                                                   //backword
    PORTE |= M_A.pulse_mask;
    PORTE |= M_B.pulse_mask;
    delayMicroseconds(50);
    if (PINB >> 5 & B00100000 >> 5)PORTE &= ~M_A.pulse_mask;                  // if pin 11 is true give  pulse would ya
    if (PINB >> 6 & B01000000 >> 6)PORTE &= ~M_B.pulse_mask;                  // if pin 12 is true give  pulse would ya
    delayMicroseconds(50);
    if (Serial.available()) return 1;                                         // if anykey is pressed stop the loop tnd return true to the system error
  }
  M_A.cur_pos = 0;
  M_A.req_pos = 0;
  M_B.cur_pos = 0;
  M_B.req_pos = 0;
  Serial.println("Honey i'm Home!");                                          // if all goes well - set the cur pos and req pos to 0 AKA home
  return 0;
}



















  //    Serial.print(err_A); Serial.print(" , ");Serial.println(err_B);



    //    if (err_A > 0 || err_B > 0)                                                // if the error is greater then zero
    //    {
    //      PORTH |= M_A.dir_mask;                                                   // set direction for motor A
    //      PORTH |= M_B.dir_mask;                                                   // set direction for motor B
    //      M_A.cur_pos++;                                                           // add step for the cur pos of motor A
    //      M_B.cur_pos++;                                                           // add step for the cur pos of motor B
    //      err_A--;                                                                 // subtract step from motor A
    //      err_B--;                                                                 // subtract step from motor B
    //      clicks_done_A++;                                                         // add step for the clicks done (for the acc and desc)
    //      clicks_done_B++;                                                         // add step for the clicks done (for the acc and desc)
    //    }
    //    else if (err_A < 0 || err_B < 0)                                           // if the error is less then zero
    //    {
    //      PORTH &= (~M_A.dir_mask);                                                // set direction for motor A
    //      PORTH &= (~M_B.dir_mask);                                                // set direction for motor B
    //      M_A.cur_pos--;                                                           // subtract step for the cur pos of motor A
    //      M_B.cur_pos--;                                                           // subtract step for the cur pos of motor B
    //      err_A++;                                                                 // add step from motor A
    //      err_B++;                                                                 // add step from motor B
    //      clicks_done_A++;                                                         // add step for the clicks done (for the acc and desc)
    //      clicks_done_B++;                                                         // add step for the clicks done (for the acc and desc)
    //    }