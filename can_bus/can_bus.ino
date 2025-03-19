#include "luxmeter.h"
#include "pid.h"
#include "init.h"
#include "can.h"
#include "ring_buffer.h"

//can bus init
#include <mcp2515.h>  // Ensure this library is included

pico_unique_board_id_t pico_board_id; // Full ID
uint8_t node_address;                 // Short ID
struct can_frame canMsgTx, canMsgRx;  // CAN bus message structures

unsigned long counter = 0;            // Message counter
MCP2515::ERROR err;                   // CAN error tracking

unsigned long time_to_write;          // Next time to send message
unsigned long write_delay = 1000;     // Delay in milliseconds

const int BUFSZ = 100;                // Buffer size
char printbuf[BUFSZ];                 // Buffer for printing

MCP2515 can0(spi0, 17);               // CAN controller object


void setup() {
  pico_get_unique_board_id(&pico_board_id);
  node_address = pico_board_id.id[7];//check
  Serial.begin();
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode(); //setLoopbackMode()debug
  unsigned long current_time = millis();
  time_to_write = current_time + write_delay;
}

void loop() {
  unsigned long current_time = millis();

  if (current_time >= time_to_write) {
    // Prepare CAN message
    canMsgTx.can_id = node_address;
    canMsgTx.can_dlc = 8;

    unsigned long div = counter;

    // Convert counter value to ASCII and store in CAN message data
    for (int i = 0; i < canMsgTx.can_dlc; i++) {
        canMsgTx.data[i] = '0' + (div % 10);  // Convert digit to ASCII
        div /= 10;
    }

    // Send CAN message
    err = can0.sendMessage(&canMsgTx);

    // Print message info
    snprintf(printbuf, BUFSZ, "Sending message %ld from node %x\n", counter++, node_address);
    Serial.print(printbuf);

    // Update next write time
    time_to_write = current_time + write_delay;
}

// Read incoming CAN messages
  while ((err = can0.readMessage(&canMsgRx)) == MCP2515::ERROR_OK) {
    unsigned long rx_msg = 0;
    unsigned long mult = 1;

    // Convert received ASCII digits to integer
    for (int i = 0; i < canMsgRx.can_dlc; i++) {
        rx_msg += mult * (canMsgRx.data[i] - '0');
        mult *= 10;
    }

    // Print received message
    snprintf(printbuf, BUFSZ, "\t\t\t\tReceived message %ld from node %x\n", rx_msg, (int)canMsgRx.can_id);
    Serial.print(printbuf);
  }

  // Handle error case
  if (err == MCP2515::ERROR_FAIL) {
    Serial.println("Error");
}


void setup1() {

}

void loop1() {


}