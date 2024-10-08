// Code using Robust Arduino Serial Protocol: https://github.com/araffin/arduino-robust-serial

#ifndef ORDER_H
#define ORDER_H

// Define the orders that can be sent and received
enum Order {
  HELLO = 0,
  SERVO = 1,
  MOTOR = 2,
  ALREADY_CONNECTED = 3,
  ERROR = 4,
  RECEIVED = 5,
  STOP = 6,
  READENCODERr = 7,
  READENCODERl = 8,
  RESETENC = 9,
  ULTRASONIC = 10,
  STOP_ORDER = 11,
};

typedef enum Order Order;

#endif
