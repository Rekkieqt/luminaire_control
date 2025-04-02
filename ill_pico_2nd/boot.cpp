#include "boot.h"
#include <Arduino.h> 

// CAN ID Encoding Function
uint16_t encodeCanId(uint8_t sender, uint8_t receiver, uint8_t task, uint8_t flags) {
    return ((flags & 0x03) << 9) |   // 2 bits for flags
           ((task & 0x07) << 6)  |   // 3 bits for task
           ((receiver & 0x07) << 3) | // 3 bits for receiver
           (sender & 0x07);          // 3 bits for sender
}

// CAN ID Decoding Function
void decodeCanId(uint16_t canId, uint8_t &sender, uint8_t &receiver, uint8_t &task, uint8_t &flags) {
    flags = (canId >> 9) & 0x03;
    task = (canId >> 6) & 0x07;
    receiver = (canId >> 3) & 0x07;
    sender = canId & 0x07;
}

// Constructor: Initialize buffer and attributes
boot::boot(uint16_t id, int size) : nodeId(id), bufferSize(size), bufferIndex(0) {
    buffer = new node_data[bufferSize];  // Allocate memory for buffer
    boot_init();
}

// Destructor: Free allocated memory
boot::~boot() {
    delete[] buffer;
    Serial.println("Boot buffer deallocated.");
}

// Initialize buffer
void boot::boot_init() {
    if (!buffer) {
        Serial.println("Memory allocation failed!");
        return;
    }
    Serial.print("Buffer initialized with size: ");
    Serial.println(bufferSize);
}

// CAN Message Handling
void boot::NODE_BOOT(canbus_comm* hermes) {
    unsigned long startTime = millis();
    srand(micros());
    uint8_t randomValue = rand() % 256;

    uint16_t id = encodeCanId(0, 0, 0, 0);
    msg_to_can inner_frm_core0;

    // Send initial message
    hermes->send_msg(id, HEAD_FLAG, randomValue, &inner_frm_core0);

    // Listen for messages for 2 seconds
    while (millis() - startTime < 2000) {
        if(hermes->recv_msg(&inner_frm_core0)){
            hermes->process_msg_core0(&inner_frm_core0);
            uint16_t receivedId = inner_frm_core0.wrapped.can_id;

            // Store in buffer if ID is different from node ID
            if (receiv<<edId != nodeId && bufferIndex < bufferSize) {
                buffer[bufferIndex++] = inner_frm_core0;
                std::cout << "Stored CAN Message with ID: " << receivedId << std::endl;
            }

            Serial.println(inner_frm_core0.wrapped.can_msg);
            Serial.println(inner_frm_core0.wrapped.can_id);
            }
        }
}
