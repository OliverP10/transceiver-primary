#include <Arduino.h>
#include "Transceiver.h"

byte address[6] = {'R', 'x', 'A', 'A', 'A'};
TransceiverPrimary transceiver(9, 8);

unsigned long last_run_print = millis();
unsigned int run_delay_print = 1000;

unsigned long last_run_send = millis();
unsigned int run_delay_send = 50;

void setup()
{
  Serial.begin(57600);
  transceiver.setup(address);
}

void loop()
{
  if (last_run_print + run_delay_print < millis())
  {
    last_run_print = millis();
    transceiver.debug();
  }

  if (last_run_send + run_delay_send < millis())
  {
    last_run_send = millis();

    Data data1;
    data1.key = 3;
    data1.value = random(0, 9999);

    Data data2;
    data2.key = 4;
    data2.value = random(0, 9999);

    Data data3;
    data3.key = 5;
    data3.value = random(0, 9999);

    Data data4;
    data4.key = 6;
    data4.value = random(0, 9999);

    Data data5;
    data5.key = 7;
    data5.value = random(0, 9999);

    Data data6;
    data6.key = 8;
    data6.value = random(0, 9999);

    Data data7;
    data7.key = 9;
    data7.value = random(0, 9999);

    Data data8;
    data8.key = 10;
    data8.value = random(0, 9999);

    Data data9;
    data9.key = 11;
    data9.value = random(0, 9999);

    Data data10;
    data10.key = 12;
    data10.value = random(0, 9999);

    Data packet_data[10];
    packet_data[0] = data1;
    packet_data[1] = data2;
    packet_data[2] = data3;
    packet_data[3] = data4;
    packet_data[4] = data5;
    packet_data[5] = data6;
    packet_data[6] = data7;

    packet_data[7] = data8;
    packet_data[8] = data9;
    packet_data[9] = data10;

    transceiver.load_large(packet_data, 10);
  }

  transceiver.tick();
}