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
    int conflicts = 0;

    node_data* nodeBuffer = nullptr;  // Dynamic array for storing node messages
    int bufferSize = 0;               // Number of stored elements

    count_id* uniqueBuffer = nullptr;
    int uniquevalues = 0;

    randomValue = 1;
    Serial.print("msg_data: ");
    Serial.println(randomValue);
    uint16_t can_id = encodeCanId(1,1,1,1);
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
    }

    // Listen for responses for 2 seconds
    startTime = millis();
    while (millis() - startTime < 5000) {
        if (hermes->recv_msg(inner_frame)) {
            hermes->process_msg_core0(inner_frame);


            uint64_t receivedId{0};
            memcpy(&receivedId, inner_frame->wrapped.can_msg.data, sizeof(uint64_t));
            Serial.println("MESSAGE RCV:");
            Serial.println(receivedId);


            if (receivedId == nodeId) {
                idConflict = true;
                Serial.println("ID Conflict Detected! Retrying...");
            }

                

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

    //Necessito de enviar um ack
    int num_ack=0;
    while (num_ack<(bufferSize-1))
    {
        if (hermes->recv_msg(inner_frame)) {
            hermes->process_msg_core0(inner_frame);
            //verificar se é ack
            num_ack++;

        }
    }
    

    uniqueBuffer = new count_id[bufferSize];

    for (int i = 0; i < bufferSize; i++) {
        int currentId = nodeBuffer[i].id;
        bool found = false;
    
        for (int j = 0; j < uniquevalues; j++) {
            if (uniqueBuffer[j].id == currentId) {
                uniqueBuffer[j].c++;
                found = true;
                break;
            }
        }
    
        if (!found) {
            uniqueBuffer[uniquevalues].id = currentId;
            uniqueBuffer[uniquevalues].c = 1;
            uniquevalues++;
        }
    }

    for (int i = 0; i < uniquevalues; i++) {
        Serial.print("ID ");
        Serial.print(uniqueBuffer[i].id);
        Serial.print(" apareceu ");
        Serial.print(uniqueBuffer[i].c);
        Serial.println(" vez(es).");
    }

    node_data* nodes = nullptr; 
    nodes = new node_data[bufferSize];  
    nodes[0] = {0, 0.0f}; // Initialize with 0.0 for G
    int num_nodes = 0;

    for (int i = 0; i < uniquevalues; i++)
    {
        if (uniqueBuffer[i].c>1)
        {
            conflicts+=uniqueBuffer[i].c;
            
        }else{
            nodes[num_nodes].id=uniqueBuffer[i].id;
            num_nodes++;

        }
        
    }
    


    if(idConflict) {

        do {

            //delete[] nodeBuffer; // Free old memory
            bufferSize=0;

            Serial.println("Entrei paara corrigir minha colisao");
    
    
            do {//Get random value different from node buffer
                randomValue = rand() % 255;
        
                bool exists = false;
                for (int i = 0; i < num_nodes; i++) {
                    if (nodes[i].id == randomValue) {
                        exists = true;
                        break;
                    }
                }
                if (!exists) break;
                
            } while (true);
            nodeId= randomValue;
            Serial.println("ESTE VAI OUTRA VEZ");
            Serial.println(randomValue);
    
    
            while (!hermes->send_msg(inner_frame, can_id, &randomValue, sizeof(randomValue)))
            {
                continue;
            }

            // Allocate memory for a new node
            node_data* temp = new node_data[bufferSize + 1];
            for (int i = 0; i < bufferSize; i++) {
                temp[i] = nodeBuffer[i];
            }
            temp[bufferSize].id = randomValue;
            bufferSize++;

            delete[] nodeBuffer; // Free old memory
            nodeBuffer = temp;
    
            //rcv and read the number of times of the conflicts
            int a=1;
            idConflict=false;
            while(a<conflicts){
    
                if (hermes->recv_msg(inner_frame)) {
                    hermes->process_msg_core0(inner_frame);
                    a++;
                    
                    uint64_t receivedId{0};
                    memcpy(&receivedId, inner_frame->wrapped.can_msg.data, sizeof(uint64_t));
                    Serial.println("MESSAGE RCV:");
                    Serial.println(receivedId);


                    if (receivedId == nodeId) {
                        idConflict = true;
                        Serial.println("ID Conflict Detected! Retrying...");

                    }
        
                        
        
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
            
            delete[] uniqueBuffer;
            uniqueBuffer = new count_id[bufferSize];

            for (int i = 0; i < bufferSize; i++) {
                int currentId = nodeBuffer[i].id;
                bool found = false;
            
                for (int j = 0; j < uniquevalues; j++) {
                    if (uniqueBuffer[j].id == currentId) {
                        uniqueBuffer[j].c++;
                        found = true;
                        break;
                    }
                }
            
                if (!found) {
                    uniqueBuffer[uniquevalues].id = currentId;
                    uniqueBuffer[uniquevalues].c = 1;
                    uniquevalues++;
                }
            }

            conflicts=0;
            for (int i = 0; i < uniquevalues; i++)
            {
                if (uniqueBuffer[i].c>1)
                {
                    conflicts+=uniqueBuffer[i].c;
                    
                }else{
                    nodes[num_nodes].id=uniqueBuffer[i].id;
                    num_nodes++;
        
                }
                
            }
                    
   
    
        } while (idConflict);  // Keep retrying until a unique ID is assigned



    }
    //UPDATE nodeBuffer when my id is already correct
    while (conflicts>0)
    {

        Serial.println("corrigir a dos outros");

        delete[] nodeBuffer; // Free old memory

        //rcv and read the number of times of the conflicts
        int a=1;
        idConflict=false;
        while(a<conflicts){

            if (hermes->recv_msg(inner_frame)) {
                hermes->process_msg_core0(inner_frame);
                a++;
                
                uint64_t receivedId{0};
                memcpy(&receivedId, inner_frame->wrapped.can_msg.data, sizeof(uint64_t));
                Serial.println("MESSAGE RCV:");
                Serial.println(receivedId);


                if (receivedId == nodeId) {
                    idConflict = true;
                    Serial.println("ID Conflict Detected! Retrying...");

                }
    
                    
    
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
        
        delete[] uniqueBuffer;
        uniqueBuffer = new count_id[bufferSize];

        for (int i = 0; i < bufferSize; i++) {
            int currentId = nodeBuffer[i].id;
            bool found = false;
        
            for (int j = 0; j < uniquevalues; j++) {
                if (uniqueBuffer[j].id == currentId) {
                    uniqueBuffer[j].c++;
                    found = true;
                    break;
                }
            }
        
            if (!found) {
                uniqueBuffer[uniquevalues].id = currentId;
                uniqueBuffer[uniquevalues].c = 1;
                uniquevalues++;
            }
        }

        conflicts=0;
        for (int i = 0; i < uniquevalues; i++)
        {
            if (uniqueBuffer[i].c>1)
            {
                conflicts+=uniqueBuffer[i].c;
                
            }else{
                nodes[num_nodes].id=uniqueBuffer[i].id;
                num_nodes++;
    
            }
            
        }


    }




    Serial.print("Node initialized successfully with ID: ");
    Serial.println(nodeId);


    Serial.println("\nFinal Node Buffer:");
    for (int i = 0; i < num_nodes; i++) {
      Serial.print("Node ID: ");
      Serial.print(nodes[i].id);
      Serial.print(" | G: ");
      Serial.println(nodes[i].G, 6); // Print float with 6 decimal places
  }
}
