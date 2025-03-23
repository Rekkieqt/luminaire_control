#include "can.h"
#include "mcp2515.h"
#include "luxmeter.h"
#include "pid.h"
#include "comms.h"
#include "ring_buffer.h"
#include "init.h"

pico_unique_board_id_t pico_board_id; // Full ID
uint8_t node_id;                 			// Short ID
MCP2515 canbuz {spi0, CSpin, TXpin, RXpin, SCKpin, SPIclock}; // canbus init 

uint64_t counter{0}; // incr or data send
struct repeating_timer write_timer; // write repeating timer
volatile bool got_irq {false}; // receive msg flat
volatile bool time_to_write {false}; // receive msg flat

msg_to_can inner_frm_core0;
msg_to_can inner_frm_core1;
canbus_comm msger;
bool timer_seq( struct repeating_timer *t ){ 
	time_to_write = true;
	return true;
}

//the interrupt service routine for core 1
void read_interrupt(uint gpio, uint32_t events) {
	got_irq = true;
}

void setup() {
	pico_get_unique_board_id(&pico_board_id); // might not be needed
	node_id = pico_board_id.id[7]; //check
	Serial.begin();
  add_repeating_timer_ms( -50, timer_seq, NULL, &write_timer); //xx Hz	
}

void loop() {
	//loop to gen n send can frames
	if(time_to_write) {
		time_to_write = false; 
		msger.send_msg(node_id,BROADCAST,counter++,&inner_frm_core0);
  	}
  	msger.recv_msg(&inner_frm_core0);
	msger.process_msg_core0(&inner_frm_core0);
}

void setup1() {
	Serial.begin();
	canbuz.reset();
	canbuz.setBitrate(CAN_1000KBPS);
	canbuz.setNormalMode();
	gpio_set_irq_enabled_with_callback( INTpin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt );
}

void loop1() {
	msger.process_can_core1(&inner_frm_core1,&canbuz,got_irq);
	msger.recv_msg(&inner_frm_core1);
	msger.send_can(&inner_frm_core1,&canbuz); 
}

/*
void inner_frm_to_fifo(msg_to_can* inner_frame) {
	for(int i = 0; i < sizeof(inner_frame->in_msg) / sizeof(uint32_t); i++) { 
		rp2040.fifo.push_nb(inner_frame->in_msg[i]);
	}
}

void recv_msg(msg_to_can* inner_frame) {
	if (rp2040.fifo.pop_nb(&inner_frame->in_msg[0])) {
		for (int i = 1; i < sizeof(inner_frame->in_msg) / sizeof(uint32_t); i++) {
    	rp2040.fifo.pop_nb(&inner_frame->in_msg[i]);
  		}	
	}
}

void send_can(msg_to_can* inner_frame, MCP2515* can) {
	if (inner_frame->wrapped.internal_msg[0] == 0xff){
		inner_frame->wrapped.internal_msg[0] = 0x00;
		can->sendMessage(&inner_frame->wrapped.can_msg);			
		inner_frame->wrapped.internal_msg[1] = ERR_INFO; //special flag for just error frm
		inner_frame->wrapped.internal_msg[2] = can->getInterrupts();
		inner_frame->wrapped.internal_msg[3] = can->getErrorFlags();
		inner_frm_to_fifo(inner_frame);
	}
}

void send_msg(uint8_t id, uint8_t header, uint64_t data, msg_to_can* inner_frame) {
	inner_frame->wrapped.can_msg.can_id = id;
	inner_frame->wrapped.can_msg.can_dlc = sizeof(data);
	memcpy(inner_frame->wrapped.can_msg.data, &data, sizeof(data));
	inner_frame->wrapped.internal_msg[0] = 0xff; // unread flag
	inner_frame->wrapped.internal_msg[1] = header;
	inner_frame->wrapped.internal_msg[2] = sizeof(can_frame);
	//inner_frame.internal_msg[2] = 0; extra 
	inner_frm_to_fifo(inner_frame);
}

void process_can_core1(msg_to_can* inner_frame, MCP2515* can, volatile bool& _got_irq) { //receive can bus, send to core 0 through fifo, maybe do sum if necessary
	if (_got_irq) {
		_got_irq = false;
		inner_frame->wrapped.internal_msg[0] = 0xff; // unread flag
		inner_frame->wrapped.internal_msg[1] = 0x00; // inner frame header 
		inner_frame->wrapped.internal_msg[2] = can->getInterrupts(); //irq
		inner_frame->wrapped.internal_msg[3] = can->getErrorFlags(); //errors
		if(inner_frame->wrapped.internal_msg[2] & MCP2515::CANINTF_RX0IF) {
			can->readMessage( MCP2515::RXB0, &inner_frame->wrapped.can_msg );
			inner_frm_to_fifo(inner_frame);
			}
		if(inner_frame->wrapped.internal_msg[2] & MCP2515::CANINTF_RX1IF) {
			can->readMessage( MCP2515::RXB1, &inner_frame->wrapped.can_msg );
			inner_frm_to_fifo(inner_frame);
			}
		if(inner_frame->wrapped.internal_msg[3] & 0b11111000) {
			//maybe something extra
			can->clearRXnOVRFlags(); 
			can->clearInterrupts();
		}
	} 
}

void process_msg_core0(msg_to_can* inner_frame) { //receive can thru fifo, process data, do something on necessity
	if(inner_frame->wrapped.internal_msg[0] == 0xff) {
		inner_frame->wrapped.internal_msg[0] = 0x00; //mark as read
		switch (inner_frame->wrapped.internal_msg[1]) {
	      case BROADCAST:
	        // print
	        Serial.println("Msg Broadcast!");
	        break;
	      case ERR_INFO: {
	      	char canintf_str[] {"| MERRF | WAKIF | ERRIF | TX2IF | TX0IF | TX1IF | RX1IF | RX0IF | "};
			char eflg_str [] {"| RX1OV | RX0OV | TXBO | TXEP | RXEP | TXWAR | RXWAR | EWARN | "};
			//if x inner_frame->internal_msg[1];
			Serial.println("-----------------------------------------------------------------");
	 		Serial.println( canintf_str );
	 		Serial.print(" | ");
	 		for (int bit = 7; bit >= 0; bit--) {
	 			Serial.print(" "); Serial.write( bitRead(inner_frame->wrapped.internal_msg[1], bit ) ? '1' : '0' ); Serial.print(" | ");
	 		}
			//if x inner_frame->internal_msg[2];
			Serial.println("");
	 		Serial.println("-----------------------------------------------------------------");
	 		Serial.println(eflg_str);
	 		Serial.print("| ");
	 		for (int bit = 7; bit >= 0; bit--) {
	 			Serial.print(" "); Serial.write(bitRead(inner_frame->wrapped.internal_msg[2], bit) ? '1' : '0'); Serial.print(" | ");
			}
	 		Serial.println("");
	 		Serial.println("-----------------------------------------------------------------");
	        break;
	    }
	      default:
	        // do something	
        	Serial.println("Header Error!");
		}
	}
}
*/