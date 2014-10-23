
#include "MoteToMote.h"
#include <Timer.h>
 
module MoteToMoteC
{
	uses 
	{
		interface Boot;
		interface Leds;
		interface Timer<TMilli>;
	}

	uses
	{
		interface Packet;
		interface AMPacket;
		interface AMSend;
		interface SplitControl as AMControl;
		interface Receive;
	}
}

implementation
{
	uint8_t dummyVar1 = 123;
	
	event void Boot.booted(){
    	call Timer.startPeriodic(500);
    	call AMControl.start();
  }
	
	bool RadioBusy = FALSE;
	message_t packet;
	
	event void Timer.fired()
	{
  		if (!RadioBusy) 
  		{
  			
		    MoteToMoteMsg_t *msg = call Packet.getPayload(&packet, sizeof(MoteToMoteMsg_t));
		   	msg->NodeId = TOS_NODE_ID;
			msg->Data = dummyVar1;
		    
		    if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(MoteToMoteMsg_t)) == SUCCESS) 
		    {
      			RadioBusy = TRUE;
    		}
  		}
	}
	
	event void AMSend.sendDone( message_t *msg, error_t error)
	{
		if( msg == &packet)
		{
			RadioBusy = FALSE;
		}	
	}
	
	event void AMControl.startDone(error_t error)
	{
		if(error == 0)
		{
			call Leds.led1On();
		}
		else
		{
			call AMControl.start();
		}
	}
	
	event void AMControl.stopDone(error_t error)
	{
		//
	}
	
	event message_t * Receive.receive(message_t *msg, void *payload, uint8_t len)
	{
		if(len == sizeof(MoteToMoteMsg_t))
		{
			MoteToMoteMsg_t *incomingPacket = (MoteToMoteMsg_t*) payload;
			//incomingPacket -> NoteId == 2;
			//nalezy zmienic NoteId i wybrac z którego sensora chcemy otrzymywac dane
			
			uint8_t data = incomingPacket->Data;
			
			if(data = 1)
			{
				call Leds.led2On();
			}
			else
			{
				call Leds.led2Off();
			}
		}
		return msg;
	}
}