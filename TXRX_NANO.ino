void Read_Serial1_Nano() {

  if (Mower_PIXHAWK == 0) {

	byte recvBuff [10] = {0};

	if (SerialCom1.update ())
	{
	  byte length = SerialCom1.getLength ();
	  if (length > sizeof (recvBuff)) length = sizeof (recvBuff);
	  memcpy (&recvBuff, SerialCom1.getData (), length);

      RawValueAmp =   recvBuff[0] | recvBuff[1] << 8;
      RawValueVolt =  recvBuff[2] | recvBuff[3] << 8;
      Rain_Detected = recvBuff[4] | recvBuff[5] << 8;
      RawWheelAmp =   recvBuff[6] | recvBuff[7] << 8;
      RawDrillAmp =   recvBuff[8] | recvBuff[9] << 8;
	}  // end if something received

  //Serial.print("WAmpRaw: ");
  //Serial.print(RawWheelAmp);
  //Serial.print(" |");

  //Serial.print("DAMpRaw: ");
  //Serial.print(RawDrillAmp);
  //Serial.print(" |");

  Calculate_Volt_Amp_Charge();

  }
}
