#include <math.h>
#include <serialize.h>
#include <stdarg.h>

#include "packet.h"
#include "constants.h"
#include "buffer.h"

typedef enum
{
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
} TDirection;
volatile TDirection dir = STOP;

// buffer arrays for serialisation/deserialisation
volatile TBuffer sendBuff, recBuff;

// Use 140-byte buffers sendBuff & recBuff since TComms size = 140 bytes > TPacket size
#define BUFF_LEN                140

// Atmega328P AVR clock frequency
#define CLOCKFREQ               16000000
#define BAUDRATE                9600

// Power management constants
#define PRR_TWI_MASK            0b10000000
#define PRR_SPI_MASK            0b00000100
#define ADCSRA_ADC_MASK         0b10000000
#define PRR_ADC_MASK            0b00000001
#define PRR_TIMER2_MASK         0b01000000
#define PRR_TIMER0_MASK         0b00100000
#define PRR_TIMER1_MASK         0b00001000

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      179

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20

#define ALEX_LENGTH         4
#define ALEX_WIDTH          6
#define PI                  3.141592654

#define LF_MUL               0.997
#define LR_MUL               1

float AlexDiagonal = 0.0;
float AlexCirc = 0.0;

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  5   // Left forward pin
#define LR                  6   // Left reverse pin
#define RF                  9  // Right forward pin
#define RR                  10  // Right reverse pin
#define PIN5 (1 << 5)
#define PIN6 (1 << 6)
#define PIN9 (1 << 1)
#define PIN10 (1 << 2)

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variables to keep track of whether we have to moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

/*
 * Watchdog Timer (WDT)
 */

void WDT_off(void)
{
  /* Global interrupt should be turned OFF here if not
  already done so */
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1<<WDRF);
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional
  time-out */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;
  /* Global interrupt should be turned ON here if
  subsequent operations after calling this function do
  not require turning off global interrupt */
}

void setupPowerSaving() {
  // Turn off the Watchdog Timer
  WDT_off();
  // Set PRR to shut down TWI
  PRR |= PRR_TWI_MASK;
  // Set PRR to shut down SPI
  PRR |= PRR_SPI_MASK;
  // Set ADCSRA to disable ADC,
  // then set PRR to shut down ADC
  ADCSRA |= ADCSRA_ADC_MASK;
  PRR |= PRR_ADC_MASK;
  // Set the SMCR to choose the IDLE sleep mode
  // Do not set the Sleep Enable (SE) bit yet
  // SMCR &= SMCR_IDLE_MODE_MASK;
  // Set Port B Pin 5 as output pin, then write a logic LOW
  // to it so that the LED tied to Arduino's Pin 13 is OFF.
  DDRB |= PIN5;
  PORTB &= ~PIN5;
}

/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = (uint32_t) leftForwardTicks;
  statusPacket.params[1] = (uint32_t) rightForwardTicks;
  statusPacket.params[2] = (uint32_t) leftReverseTicks;
  statusPacket.params[3] = (uint32_t) rightReverseTicks;
  statusPacket.params[4] = (uint32_t) leftForwardTicksTurns;
  statusPacket.params[5] = (uint32_t) rightForwardTicksTurns;
  statusPacket.params[6] = (uint32_t) leftReverseTicksTurns;
  statusPacket.params[7] = (uint32_t) rightReverseTicksTurns;
  statusPacket.params[8] = (uint32_t) forwardDist;
  statusPacket.params[9] = (uint32_t) reverseDist;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011;
  PORTD |= 0b1100;
  
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  //leftTicks++;
  //Serial.print("LEFT: ");
  //Serial.println(leftTicks);
  switch(dir) {
    case FORWARD:
      leftForwardTicks++;
      forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case BACKWARD:
      leftReverseTicks++;
      reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case LEFT:
      leftReverseTicksTurns++;
      break;
    case RIGHT:
      leftForwardTicksTurns++;
      break;
  }
}

void rightISR()
{
  //rightTicks++;
  //Serial.print("RIGHT: ");
  //Serial.println(rightTicks);
  switch(dir) {
    case FORWARD:
      rightForwardTicks++;
      //forwardDist = (unsigned long) ((float) rightForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case BACKWARD:
      rightReverseTicks++;
      //reverseDist = (unsigned long) ((float) rightReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case LEFT:
      rightForwardTicksTurns++;
      break;
    case RIGHT:
      rightReverseTicksTurns++;
      break;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EIMSK = 0b11;
  EICRA = 0b1010;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect) {
  leftISR();
}

ISR(INT1_vect) {
  rightISR();
}

// Implement INT0 and INT1 ISRs above.

ISR(TIMER0_COMPA_vect) 
{

}

ISR(TIMER0_COMPB_vect) 
{

}

ISR(TIMER1_COMPA_vect) 
{

}

ISR(TIMER1_COMPB_vect) 
{

}

// ISRs for serial read and write operations
ISR(USART_RX_vect) {
    unsigned char data = UDR0;

    writeBuffer(&recBuff, data);
}

ISR(USART_UDRE_vect) {
    unsigned char data;
    TBufferResult result = readBuffer(&sendBuff, &data);
    
    if (result == BUFFER_OK) {
        UDR0 = data;
    } else if (result == BUFFER_EMPTY) {
        UCSR0B &= 0b11011111;
    }
}


/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  // Serial.begin(9600);

  // Initialise send and receive buffer to appropriate length(in BUF_LEN macro)
  initBuffer(&sendBuff, BUFF_LEN);
  initBuffer(&recBuff, BUFF_LEN);

  // Only set bits 1 & 2 for data size of 8 bits. Disable parity bits(bits 4 & 5), use Asynchronous USART mode(bits 6 & 7), and use just 1 stop bit(bit 3)
  UCSR0C = 0b00000110;
  // Zero everything in OCSR0A to disable double-speed and multiprocessor modes
  UCSR0A = 0;

  // Calculate value for UBRR register to use a 9600bps baudrate
  uint16_t baudrate = (CLOCKFREQ / 16 / BAUDRATE) - 1;
  // Right shift baudrate by 8 to get the 8 high bits for UBBR0H
  UBRR0H = (uint8_t)(baudrate >> 8);
  // Truncate the 8 high bits in baudrate variable to store the low bits in UBRR0L
  UBRR0L = (uint8_t)baudrate;
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.

  // Set bits 3 & 4 to enable the USART receiver and transmitter, set bits 5 & 7 to enable USART_RX and USART_UDRE interrupts
  // Clear bit 2 to set data size to 8 bits, clear bits 0:1 since we don't use 9-bit data size, disable USART_TX interrupt since using USART_UDRE interrupt is slightly more efficient
  UCSR0B = 0b10111000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  /*
   * Old Scratch code for serial reading
  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
  */
  
  int count;
  
  for (count = 0; dataAvailable(&recBuff); count++) {
        readBuffer(&recBuff, (unsigned char*)&buffer[count]);
  }
  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  // Old Scratch code for serial writing
  // Serial.write(buffer, len);

  TBufferResult result = BUFFER_OK;

  for (int i = 1; i < len && result == BUFFER_OK; i++) 
  {
    result = writeBuffer(&sendBuff, buffer[i]);
  }

  // Write the first byte of the buffer into UDR0 so that USART_UDRE gets triggered and the ISR takes over for writing the rest of the buffer's bytes
  UDR0 = buffer[0];

  // Enable the USART_UDRE interrupt
  UCSR0B |= 0b100000;
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B2IN - Pin 9, PB1, OC1A
   *    B1IN - Pin 10, PB2, OC1B
   */
  DDRD |= (PIN6 | PIN5);
  TCNT0 = 0;
  TIMSK0 |= 0b110; // OCIEA = 1 OCIEB = 1
  OCR0A = 0;
  OCR0B = 0;
  TCCR0B = 0b00000011; // Set prescalar to 64

  DDRB |= (PIN9 | PIN10);
  TCNT1 = 0;
  TIMSK1 |= 0b110;
  OCR1A = 0;
  OCR1B = 0;
  TCCR1B = 0b00000011; // Set prescalar to 64
   
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  TCCR0A = 0b10100001; // Set PWM mode to Phase-correct
  TCCR1A = 0b10100001; // Set PWM mode to Phase-correct 8-bit to match pwmVal()'s output size(8-bit analog)
}

void analogWriteBm(int port, int pwmVal) 
{
  switch(port)
  {
    case LF:
      OCR0B = pwmVal;
      break;
    case LR:
      OCR0A = pwmVal;
      break;
    case RF:
      OCR1A = pwmVal;
      break;
    case RR:
      OCR1B = pwmVal;
      break;
  }

}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  int val = pwmVal(speed);
  

  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.
  if(dist > 0)
    deltaDist = dist;
  else
    deltaDist=9999999;
  newDist=forwardDist + deltaDist;
  //dbprint("forwardDist: %d", forwardDist);
  //dbprint("deltaDist: %d", deltaDist);

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.

  dir = FORWARD;
  
  analogWriteBm(LF, val * LF_MUL);
  analogWriteBm(RF, val);
  analogWriteBm(LR, 0);
  analogWriteBm(RR, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{

  int val = pwmVal(speed);

  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.
  if(dist > 0)
    deltaDist = dist;
  else
    deltaDist=9999999;
  newDist=reverseDist + deltaDist;

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.

  dir = BACKWARD;
  
  analogWriteBm(LR, val * LR_MUL);
  analogWriteBm(RR, val);
  analogWriteBm(LF, 0);
  analogWriteBm(RF, 0);
}

unsigned long computeDeltaTicks(float ang) {
  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  int val = pwmVal(speed);

  // For now we will ignore ang. We will fix this in Week 9.
  if(ang == 0)
    deltaTicks=99999999;
  else
    deltaTicks=computeDeltaTicks(ang);
  targetTicks = leftReverseTicksTurns + deltaTicks;
  
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.

  dir = LEFT;
  
  analogWriteBm(LR, val);
  analogWriteBm(RF, val);
  analogWriteBm(LF, 0);
  analogWriteBm(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  int val = pwmVal(speed);

  // For now we will ignore ang. We will fix this in Week 9.
  if(ang == 0)
    deltaTicks=99999999;
  else
    deltaTicks=computeDeltaTicks(ang);
  targetTicks = rightReverseTicksTurns + deltaTicks;
  
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.

  dir = RIGHT;

  analogWriteBm(RR, val);
  analogWriteBm(LF, val);
  analogWriteBm(LR, 0);
  analogWriteBm(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  analogWriteBm(LF, 0);
  analogWriteBm(LR, 0);
  analogWriteBm(RF, 0);
  analogWriteBm(RR, 0);
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0; 
  rightForwardTicks=0;
  leftReverseTicks=0; 
  rightReverseTicks=0;
  leftForwardTicksTurns=0; 
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0; 
  rightReverseTicksTurns=0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  /*switch(which)
  {
    case 0:*/
      clearCounters();
      /*break;

    case 1:
      leftTicks=0;
      break;

    case 2:
      rightTicks=0;
      break;

    case 3:
      leftRevs=0;
      break;

    case 4:
      rightRevs=0;
      break;

    case 5:
      forwardDist=0;
      break;

    case 6:
      reverseDist=0;
      break;
  }*/
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        //dbprint("Arduino moving forward\n");
        forward((float) command->params[0], (float) command->params[1]);
      break;

    /*
     * Implement code for other commands here.
     * 
     */
    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_STOP:
        sendOK();
        forward((float) command->params[0], 0.0);
      break;
    case COMMAND_GET_STATS:
        sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
      break;
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_WIDTH *
        ALEX_WIDTH));
  AlexCirc = PI * AlexDiagonal;

  cli();
  setupPowerSaving();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

// forward(0, 100);

// Uncomment the code below for Week 9 Studio 2


 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 

  if(deltaDist > 0)
  {
    if(dir == FORWARD)
    {
        if(forwardDist > newDist)
        {
            deltaDist=0;
            newDist=0;
            stop();
        }
    }
    else {
        if(dir == BACKWARD)
        {
            if(reverseDist > newDist)
            {
              deltaDist=0;
              newDist=0;
              stop();
            }
        }
        else {
            if(dir == STOP)
            {
              deltaDist=0;
              newDist=0;
              stop();
            }
        }
    }
  }

  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else {
      if (dir == RIGHT) {
        if (rightReverseTicksTurns >= targetTicks) {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
      } else {
        if (dir == STOP) {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
      }
    }
  }
      
      
}
