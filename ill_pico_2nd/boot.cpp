#include "boot.h"
#include <Arduino.h> 
#include "init.h"

boot::boot() {
}

boot::~boot() {
}

// CAN ID Encoding Function
uint16_t encodeCanId(uint8_t sender, uint8_t receiver, uint8_t header, uint8_t header_flag) {
    return ((header_flag & 0x03) << 9) |   // 2 bits for flags
           ((header & 0x07) << 6)  |   // 3 bits for task
           ((receiver & 0x07) << 3) | // 3 bits for receiver
           (sender & 0x07);          // 3 bits for sender
}

// CAN ID Decoding Function
void decodeCanId(uint16_t canId, uint8_t &sender, uint8_t &receiver, uint8_t &header, uint8_t &header_flag) {
    header_flag = (canId >> 9) & 0x03;
    header = (canId >> 6) & 0x07;
    receiver = (canId >> 3) & 0x07;
    sender = canId & 0x07;
}

void boot::NODE_BOOT(canbus_comm* hermes, msg_to_can* inner_frame) {
    unsigned long startTime;
    srand(micros()); // Seed random generator
    uint64_t randomValue;
    bool idConflict;
    int conflits = 0;

    node_data* nodeBuffer = nullptr;  // Dynamic array for storing node messages
    int bufferSize = 0;               // Number of stored elements

    randomValue = rand() % 256;
    Serial.print("msg_data: ");
    Serial.println(randomValue);
    uint16_t can_id = encodeCanId(0,0,0,0);
    idConflict = false;
    int nodeId = randomValue;

    // Store my own nodeId in the buffer
    nodeBuffer = new node_data[1];  
    nodeBuffer[0] = {nodeId, 0.0f}; // Initialize with 0.0 for G
    bufferSize = 1;


    // Send ID request to see if it's taken
    //msg_to_can* inner_frame, uint16_t canm_id, void* data = nullptr ,size_t size_data = 0
    while (!hermes->send_msg(inner_frame, can_id, &randomValue, sizeof(randomValue)))
    {
        continue;
    }
    
    Serial.println("\nInitial Node Buffer:");
    for (int i = 0; i < bufferSize; i++) {
      Serial.print("Node ID: ");
      Serial.println(nodeBuffer[i].id);

    // Listen for responses for 2 seconds
    startTime = millis();
    while (millis() - startTime < 5000) {
        if (hermes->recv_msg(inner_frame)) {
            hermes->process_msg_core0(inner_frame);
            //memcpy is sugested... maybe change the process_msg function a bit ...
            uint64_t receivedId{0};
            memcpy(&receivedId, inner_frame->wrapped.can_msg.data, sizeof(uint64_t));
            Serial.println("MESSAGE RCV:");
            Serial.println(receivedId);

            // If another node has the same ID, regenerate
            if (receivedId == nodeId) {
                idConflict = true;
                Serial.println("ID Conflict Detected! Retrying...");
                break;  // Exit early to regenerate ID
            }
            

            Serial.println("\n Middle Node Buffer:");
            for (int i = 0; i < bufferSize; i++) {
              Serial.print("Node ID: ");
              Serial.println(nodeBuffer[i].id);
            }


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
      /*
      
      
      
      */
       //place the verification of the messages seeing the 
    } while (idConflict);  // Keep retrying until a unique ID is assigned
    while (conflits>0)
    {
        if (hermes->recv_msg(inner_frame)) {
            hermes->process_msg_core0(inner_frame);
            uint16_t receivedId = inner_frame->wrapped.can_msg.can_id;
        }
    }
    Serial.print("Node initialized successfully with ID: ");
    Serial.println(nodeId);


    // Serial.println("\nFinal Node Buffer:");
    // for (int i = 0; i < bufferSize; i++) {
    //   Serial.print("Node ID: ");
    //   Serial.print(nodeBuffer[i].id);
      // Serial.print(" | G: ");
      // Serial.println(nodeBuffer[i].G, 6); // Print float with 6 decimal places
  }
}
