/***************************************************
* RTHOR Energia code
*
* This is RTHOR, well his initial code anyway. I chose
* to develop in Energia using CCS for the ease and
* speed of coding, given the deadline of the semester.
* The first part of the code is setup for the serial
* servos used in the 6 DOF robotic arm. We chose to
* use the serial servos because of their torque at
* 17 kg, their higher accuracy, and for the ability
* they have to send and receive data such as position,
* temperature, voltage, etc.
*
* While refining this Energia code, I began to port
* this project over to use the TIRTOS meant for the
* MSP432. I plan on uploading this code after some
* further development.
 ****************************************************/
//#include <Wire.h>
//#include "Pixy2.h"
#include <stdio.h>

/*****************************************************
 * SERVO SETUP
 *****************************************************/
 // These are our Serial Servo Motor ID tags
#define ID1   1
#define ID2   2
#define ID3   3
#define ID4   4
#define ID5   5
#define ID6   6

#define GET_LOW_BYTE(A) (uint8_t)((A))
//Macro function  get lower 8 bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Macro function  get higher 8 bits of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//put A as higher 8 bits   B as lower 8 bits   which amalgamated into 16 bits integer
#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36
#define LOBOT_SERVO_MIN_POSITION         100
#define LOBOT_SERVO_MAX_POSITION         1000
#define LOBOT_SERVO_MIN_BASE_POS         440
#define LOBOT_SERVO_MAX_GRIP_CLOSE       765
#define LOBOT_SERVO_MAX_GRIP_OPEN        500
#define LOBOT_DEBUG 0  /*Debug - print debug value*/
 byte LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}
 void LobotSerialServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time)
{
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  Serial.write(buf, 10);
}
 void LobotSerialServoStopMove(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  Serial.write(buf, 6);
}
 void LobotSerialServoSetID(HardwareSerial &SerialX, uint8_t oldID, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  Serial.write(buf, 7);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
 }
 void LobotSerialServoSetMode(HardwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed)
{
  byte buf[10];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);
 #ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Set Mode");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
   Serial.write(buf, 10);
}
 void LobotSerialServoLoad(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);

  Serial.write(buf, 7);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
 }
 void LobotSerialServoUnload(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);

  Serial.write(buf, 7);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}
 int LobotSerialServoReceiveHandle(HardwareSerial &SerialX, byte *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;
   while (Serial.available()) {
    rxBuf = Serial.read();
    delayMicroseconds(100);
    if (!frameStarted) {
      if (rxBuf == LOBOT_SERVO_FRAME_HEADER) {
        frameCount++;
        if (frameCount == 2) {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted) {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3) {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7) {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3) {

#ifdef LOBOT_DEBUG
        Serial.print("RECEIVE DATA:");
        for (i = 0; i < dataCount; i++) {
          Serial.print(recvBuf[i], HEX);
          Serial.print(":");
        }
        Serial.println(" ");
#endif
         if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {

#ifdef LOBOT_DEBUG
          Serial.println("Check SUM OK!!");
          Serial.println("");
#endif
           frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
}
 int LobotSerialServoReadPosition(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);
 #ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Pos READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
   while (Serial.available())
    Serial.read();
   Serial.write(buf, 6);
   while (!Serial.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }
   if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;
 #ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
 int LobotSerialServoReadVin(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);
 #ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO VIN READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
   while (Serial.available())
    SerialX.read();
   Serial.write(buf, 6);
   while (!Serial.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }
   if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2049;
 #ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

 /*****************************************************
  * SERVO MOTION CONTROLS
  *
  * Here I use the servo location to generate an offset
  * position so that the motion of the servos is more
  * smooth. The servo moves toward the desired position
  * at on speed, then during the last 100 clicks of
  * rotaion the speed is slowed down to control jerky
  * movement.
  *
  * The time it takes for the individual servos to move
  * has to be taken into account otherwise the next
  * movement will overlap the previous motion if it has
  * not completed.
  *
  * Each servo has its own function to smooth out the
  * rotations, with the exception of the wrist and
  * grabber servos, as their motion does not generate
  * shaky movement to the arm.
  *****************************************************/

void setBaseServo(int position){
    int offset; // the last segment of rotation
    // check to see how far the desired psotition is from the current location
    // if it is a small position change, an offset is not needed for smooth motion
    if((LobotSerialServoReadPosition(Serial, ID1) - position) == 200 || (position - LobotSerialServoReadPosition(Serial, ID1)) == 200)
        offset = 0;
    // check what orientation the offset has to be in based on current position and set the offset accordingly
    else if(LobotSerialServoReadPosition(Serial, ID1) < position)
        offset = -100;
    else
        offset = 100;

    // move the servo almost to the desired position and wait for the servo to get there
    LobotSerialServoMove(Serial, ID1, position + offset, 1000);
    delay(1100);
    // then finish moving to the desired position
    LobotSerialServoMove(Serial, ID1, position, 700);
}

void setServo2(int position){
    int offset;
    if((LobotSerialServoReadPosition(Serial, ID2) - position) == 200 || (position - LobotSerialServoReadPosition(Serial, ID1)) == 200)
        offset = 0;
    else if(LobotSerialServoReadPosition(Serial, ID2) < position)
        offset = -100;
    else
        offset = 100;

    LobotSerialServoMove(Serial, ID2, position + offset, 1000);
    delay(1100);
    LobotSerialServoMove(Serial, ID2, position, 700);
}

void setServo3(int position){
    int offset;
    if((LobotSerialServoReadPosition(Serial, ID3) - position) == 200 || (position - LobotSerialServoReadPosition(Serial, ID1)) == 200)
        offset = 0;
    else if(LobotSerialServoReadPosition(Serial, ID3) < position)
        offset = -100;
    else
        offset = 100;

    LobotSerialServoMove(Serial, ID3, position + offset, 1000);
    delay(1100);
    LobotSerialServoMove(Serial, ID3, position, 700);
}

void setServo4(int position){
    int offset;
    if((LobotSerialServoReadPosition(Serial, ID4) - position) == 200 || (position - LobotSerialServoReadPosition(Serial, ID1)) == 200)
        offset = 0;
    else if(LobotSerialServoReadPosition(Serial, ID4) < position)
        offset = -100;
    else
        offset = 100;

    LobotSerialServoMove(Serial, ID4, position + offset, 1000);
    delay(1100);
    LobotSerialServoMove(Serial, ID4, position, 700);
}

void setServo5(int position){
    LobotSerialServoMove(Serial, ID5, position, 1000);
}

void setGripperServo(int grip){
    LobotSerialServoMove(Serial, ID6, grip, 1000);
}

void setServoStartPos(){
    // move the arm to a predefined start position with an initial wait just in case there are
    // motions that have not completed
    delay(1000);
    LobotSerialServoMove(Serial, ID1, 850, 1000);
    LobotSerialServoMove(Serial, ID2, 750, 1000);
    LobotSerialServoMove(Serial, ID3, 100, 1000);
    LobotSerialServoMove(Serial, ID4, 1000, 1000);
    LobotSerialServoMove(Serial, ID5, 520, 1000);
    LobotSerialServoMove(Serial, ID6, 500, 1000);
}

void setServoDeliverPos(){
    // move the arm to a predefined delivery position with an initial wait just in case there are
    // motions that have not completed
    delay(1000);
    LobotSerialServoMove(Serial, ID2, 900, 1000);
    LobotSerialServoMove(Serial, ID1, 700, 2000);
    LobotSerialServoMove(Serial, ID2, 725, 1000);
    LobotSerialServoMove(Serial, ID3, 550, 1000);
    LobotSerialServoMove(Serial, ID1, 450, 2000);
    LobotSerialServoMove(Serial, ID4, 650, 1000);
    LobotSerialServoMove(Serial, ID5, 115, 1000);
}

void setup() {
    // set the baud rate
    Serial.begin(115200);
    delay(1500);
    // get the arm ready
    setServoStartPos();
    delay(1000);
}
void loop() {

    int x, y; // the location of the object to retrieve

    // The following three routines move the arm to predefined locations to pick up objects
    // that are about an inch wide. This is done for testing and arm motion proof of concept.
    setBaseServo(1000);
    setServo2(600);
    setServo3(460);
    setServo4(600);
    setServo2(400);
    setServo5(510);
    delay(1000);
    setGripperServo(645);
    delay(1000);
    setServoDeliverPos();
    delay(3000);
    setGripperServo(500);
    delay(1000);
    setServoStartPos();
    delay(3000);

    setBaseServo(875);
    setServo2(600);
    setServo3(460);
    setServo4(600);
    setServo2(400);
    setServo5(510);
    delay(1000);
    setGripperServo(645);
    delay(1000);
    setServoDeliverPos();
    delay(3000);
    setGripperServo(500);
    delay(1000);
    setServoStartPos();
    delay(3000);

    setBaseServo(750);
    setServo2(600);
    setServo3(460);
    setServo4(600);
    setServo2(400);
    setServo5(510);
    delay(1000);
    setGripperServo(645);
    delay(1000);
    setServoDeliverPos();
    delay(3000);
    setGripperServo(500);
    delay(1000);
    setServoStartPos();
    delay(3000);

}
