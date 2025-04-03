#include "boot.h"
#include <Arduino.h> 
#include "init.h"

boot::boot() {
}

boot::~boot() {
}

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


// // Constructor: Initialize buffer and attributes
// boot::boot(uint16_t id, int size) : nodeId(id), bufferSize(size), bufferIndex(0) {
//     buffer = new node_data[bufferSize];  // Allocate memory for buffer
//     boot_init();
// }

// // Destructor: Free allocated memory
// boot::~boot() {
//     delete[] buffer;
//     Serial.println("Boot buffer deallocated.");
// }

// // Initialize buffer
// void boot::boot_init() {
//     if (!buffer) {
//         Serial.println("Memory allocation failed!");
//         return;
//     }
//     Serial.print("Buffer initialized with size: ");
//     Serial.println(bufferSize);
// }

// // CAN Message Handling
// void boot::NODE_BOOT(canbus_comm* hermes) {
//     unsigned long startTime = millis();
//     srand(micros());
//     uint8_t randomValue = rand() % 256;

//     uint16_t id = encodeCanId(0, 0, 0, 0);
//     msg_to_can inner_frm_core0;

//     // Send initial message
//     hermes->send_msg(id, HEAD_FLAG, randomValue, &inner_frm_core0);

//     // Listen for messages for 2 seconds
//     while (millis() - startTime < 2000) {
//         if(hermes->recv_msg(&inner_frm_core0)){
//             hermes->process_msg_core0(&inner_frm_core0);
//             uint16_t receivedId = inner_frm_core0.wrapped.can_id;

//             // Store in buffer if ID is different from node ID
//             if (receiv<<edId != nodeId && bufferIndex < bufferSize) {
//                 buffer[bufferIndex++] = inner_frm_core0;
//                 std::cout << "Stored CAN Message with ID: " << receivedId << std::endl;
//             }

//             Serial.println(inner_frm_core0.wrapped.can_msg);
//             Serial.println(inner_frm_core0.wrapped.can_id);
//             }
//         }
// }



void boot::NODE_BOOT(canbus_comm* hermes,msg_to_can inner_frm_core0) {
    unsigned long startTime;
    srand(micros()); // Seed random generator
    uint8_t randomValue;
    bool idConflict;
    int conflits = 0;

    node_data* nodeBuffer = nullptr;  // Dynamic array for storing node messages
    int bufferSize = 0;               // Number of stored elements

    randomValue = rand() % 256;
    uint16_t can_id = encodeCanId(0,0,0,0);
    idConflict = false;
    int nodeId = randomValue;

    msg_to_can received_msg;

    // Store my own nodeId in the buffer
    nodeBuffer = new node_data[1];  
    nodeBuffer[0] = {nodeId, 0.0f}; // Initialize with 0.0 for G
    bufferSize = 1;


    // Send ID request to see if it's taken
    while (!hermes->send_msg(can_id, HEAD_FLAG, randomValue, &inner_frm_core0))
    {
        continue;
    }
    
    

    // Listen for responses for 2 seconds
    startTime = millis();
    while (millis() - startTime < 5000) {
        if (hermes->recv_msg(&received_msg)) {
            hermes->process_msg_core0(&received_msg);
            uint16_t receivedId = received_msg.wrapped.can_msg.can_id;
            Serial.println(receivedId);

            // If another node has the same ID, regenerate
            if (receivedId == nodeId) {
                idConflict = true;
                Serial.println("ID Conflict Detected! Retrying...");
                break;  // Exit early to regenerate ID
            }
            
            // // Store the received message if it's from another node
            // if (receivedId != nodeId && bufferIndex < bufferSize) {
            //     //buffer[bufferIndex++] = {receivedId, received_msg}; // Store message in buffer
            //     // std::cout << "Stored CAN Message from node ID: " << receivedId << std::endl;
            // }


            // Check if message is already in the buffer
            bool found = false;
            for (int i = 0; i < bufferSize; i++) {
                if (nodeBuffer[i].id == receivedId) {
                    found = true;
                    // Remove the duplicate by shifting elements
                    for (int j = i; j < bufferSize - 1; j++) {
                        nodeBuffer[j] = nodeBuffer[j + 1];
                    }
                    bufferSize--; // Reduce size
                    Serial.print("Removed duplicate message from node ID: ");
                    Serial.println(receivedId);
                    break;
                }
            }

            if (!found && receivedId != nodeId) {
                // Allocate memory for a new node
                node_data* temp = new node_data[bufferSize + 1];
                for (int i = 0; i < bufferSize; i++) {
                    temp[i] = nodeBuffer[i];
                }
                temp[bufferSize].id = receivedId;
                bufferSize++;

                delete[] nodeBuffer; // Free old memory
                nodeBuffer = temp;

                Serial.print("Stored CAN Message from node ID: ");
                Serial.println(receivedId);
            }
        }
    }
    do {
        //place the verification of the messages seeing the 
    } while (idConflict);  // Keep retrying until a unique ID is assigned
    while (conflits>0)
    {
        if (hermes->recv_msg(&received_msg)) {
            hermes->process_msg_core0(&received_msg);
            uint16_t receivedId = received_msg.wrapped.can_msg.can_id;
        }
    }
    Serial.print("Node initialized successfully with ID: ");
    Serial.println(nodeId);


      Serial.println("\nFinal Node Buffer:");
    for (int i = 0; i < bufferSize; i++) {
      Serial.print("Node ID: ");
      Serial.print(nodeBuffer[i].id);
      // Serial.print(" | G: ");
      // Serial.println(nodeBuffer[i].G, 6); // Print float with 6 decimal places
  }
}




// void NODE_BOOT(){
//     unsigned long startTime = millis();
//     srand(micros());
//     uint8_t randomValue = rand() % 256;
//     uint16_t id = encodeCanId(0,0,0,0);
  
//     hermes.send_msg(id,HEAD_FLAG,randomValue,&inner_frm_core0);
  
//     while(millis() - startTime < 2000){
//       hermes.recv_msg(&inner_frm_core0);
//       hermes.process_msg_core0(&inner_frm_core0);
  
//     }
//   }