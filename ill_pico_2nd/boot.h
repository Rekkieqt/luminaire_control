#ifndef BOOT_H
#define BOOT_H

#include "init.h"
#include "comms.h"

class boot {
private:
    // struct node_data* buffer;  // Pointer to message buffer
    // int bufferSize;             // Max buffer size
    // int bufferIndex;            // Number of stored messages
    // uint16_t nodeId;            // Node's CAN ID

public:
    explicit boot();  // Constructor (uint16_t id, int size = 10)
    ~boot();  // Destructor

    //void boot_init();  // Initialize buffer
    void NODE_BOOT(canbus_comm* herms, msg_to_can* inner_frame);  // Process CAN messages
};

#endif // BOOT_H
