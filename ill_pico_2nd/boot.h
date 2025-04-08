#ifndef BOOT_H
#define BOOT_H

#include "init.h"
#include "comms.h"

uint16_t encodeCanId(uint8_t sender, uint8_t receiver, uint8_t header, uint8_t header_flag);
void decodeCanId(uint16_t canId, uint8_t &sender, uint8_t &receiver, uint8_t &header, uint8_t &header_flag);

struct count_id {
    uint8_t id;
    int c;
};

class boot {
private:
    // struct node_data* buffer;  // Pointer to message buffer
    // int bufferSize;             // Max buffer size
    // int bufferIndex;            // Number of stored messages
    // uint16_t nodeId;            // Node's CAN ID

public:
    uint8_t nodeId, num_nodes;

    explicit boot();  // Constructor (uint16_t id, int size = 10)
    ~boot();  // Destructor

    //void boot_init();  // Initialize buffer
    void NODE_BOOT(canbus_comm* herms, msg_to_can* inner_frame);  // Process CAN messages
};

#endif // BOOT_H
