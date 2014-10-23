configuration MoteToMoteAppC
{
	//
}
implementation
{
	//Glowne komponenty
	components MoteToMoteC as App;
	components MainC;
	components LedsC;
	components new TimerMilliC();

	App.Boot -> MainC;
	App.Leds -> LedsC;
	App.Timer -> TimerMilliC;
	
	//Komunikacja radiowa
	components ActiveMessageC;
	components new AMSenderC(AM_RADIO);
	components new AMReceiverC(AM_RADIO);
	
	App.Packet -> AMSenderC;
	App.AMPacket -> AMSenderC;
	App.AMSend -> AMSenderC;
	App.AMControl -> ActiveMessageC;
	App.Receive -> AMReceiverC;
}