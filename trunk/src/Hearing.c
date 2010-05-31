#pragma config(Sensor, S1,     touchSensor,               sensorTouch)
#pragma config(Sensor, S4,     soundSensor,               sensorSoundDB)

void DisplayData(short data, bool firstDisplay)
{
  //lightValue1 = SensorValue[lightSensorB];
  if(firstDisplay)
  {
    nxtDisplayCenteredTextLine(0, "Sound Reading");
    nxtDisplayCenteredBigTextLine(2, "%d", data);
  }
  else
  {
    nxtDisplayCenteredTextLine(0, "Sonar Reading");
    nxtDisplayCenteredBigTextLine(5, "%d", data);
  }
}

task main()
{
  while (true){
DisplayData(SensorValue[soundSensor],true);
wait10Msec(3);

}

}
