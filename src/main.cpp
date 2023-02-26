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
  Serial.begin(115200);
  transceiver.setup(address);
}

void loop()
{
  if (last_run_print + run_delay_print < millis())
  {
    last_run_print = millis();
    // transceiver.debug();
  }

  if (last_run_send + run_delay_send < millis())
  {
    last_run_send = millis();
    Data data;
    data.key = 2;
    data.value = random(0, 9999);

    transceiver.send(data);
  }

  transceiver.tick();
}