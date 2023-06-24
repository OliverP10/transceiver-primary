#pragma once
#include "Transceiver.h"
#include <Arduino.h>
#include <RF24.h>
#include <math.h>
#include <ArduinoJson.h>
#include <CircularBuffer.h>

// PRIMARY TRANSMITTER

int packets_sent = 0;
int packets_lost = 0;
int number_backoffs = 0;

TransceiverPrimary::TransceiverPrimary(int ce_pin, int csn_pin) : m_radio(ce_pin, csn_pin),
                                                                  m_connected(false),
                                                                  m_last_health_check(millis()),
                                                                  m_health_check_delay(1000),
                                                                  m_backoff_time(1),
                                                                  m_last_backoff(0),
                                                                  m_awaiting_acknoledge(false),
                                                                  m_buffer(new CircularBuffer<Buffered_packet, 20>()),
                                                                  m_send_packet(),
                                                                  m_received_packet(),
                                                                  m_last_packet_id(0),
                                                                  m_id_counter(0)
{
    this->m_send_packet.num_data_fields = 0;
};

TransceiverPrimary::~TransceiverPrimary()
{
    this->m_buffer->clear();
    delete this->m_buffer;
}

/*
Sets up the radio object with specified pins and address
*/
void TransceiverPrimary::setup(byte address[6], void (*callback_func)(Packet))
{
    this->m_callback_func = callback_func;
    this->m_radio.begin();
    this->m_radio.openReadingPipe(0, address);
    this->m_radio.openWritingPipe(address);
    this->m_radio.enableDynamicAck();
    this->m_radio.setPALevel(RF24_PA_MIN); // RF24_PA_MAX
    this->m_radio.setDataRate(RF24_2MBPS); // RF24_250KBPS
    this->m_radio.setAutoAck(true);
    this->m_radio.setRetries(5, 15);
    this->m_radio.startListening();
}

/*
Reads any data from the recive buffer
Checks that it has a new ID compared to last packet, and writes it to serial
Sets the radio to connected
*/
void TransceiverPrimary::receive()
{
    if (this->m_radio.available())
    {
        this->m_radio.read(&this->m_received_packet, sizeof(this->m_received_packet));
        if (this->m_last_packet_id != this->m_received_packet.id && this->m_received_packet.num_data_fields != 0)
        {
            this->m_last_packet_id = this->m_received_packet.id;
            this->write_data_to_serial();
            this->write_data_to_callback_func();
        }
        this->m_awaiting_acknoledge = false;
        this->set_connected();
    }
}

/*
Sets the radio disconnected and attempts radio resynce if last recived message was longer than m_health_check_delay
*/
void TransceiverPrimary::monitor_connection_health()
{
    if (this->m_last_health_check + this->m_health_check_delay < millis())
    {
        this->set_disconnected();
        this->syncronise_radios();
    }
}

/*
Sends a reserved key 1 to the secondary radio to tell it to sets its m_awaiting_acknoledge attribute to the value 1
Essietialy telling the secondary radio, whatever state your in set your self to awaiting acknoledge (listining)
*/
void TransceiverPrimary::syncronise_radios()
{
    Data data;
    data.key = 1;
    data.value = 1;

    Packet packet;
    packet.id = this->increment_id();
    packet.num_data_fields = 1;
    packet.data[0] = data;

    this->m_radio.stopListening();
    if (this->m_radio.write(&packet, sizeof(packet)))
    {
        this->m_awaiting_acknoledge = false;
    }
    this->m_radio.startListening();

    StaticJsonDocument<200> doc;
    doc["1"] = 1;
    serializeJson(doc, Serial);
    Serial.println();
}

/*
Used to print out usefull debug data every second
*/
void TransceiverPrimary::debug()
{
    // Serial.println();
    // Serial.print("Packets sent: ");
    // Serial.println(packets_sent);
    // packets_sent = 0;
    // Serial.print("Packets lost: ");
    // Serial.println(packets_lost);
    // packets_lost = 0;
    // Serial.print("Number of backoffs: ");
    // Serial.println(number_backoffs);
    // number_backoffs = 0;
    // Serial.print("Buffer available: ");
    // Serial.println(this->m_buffer->available());
    // Serial.print("Connected: ");
    // Serial.println(this->m_connected);
    // Serial.print("Awaiting acknoledge: ");
    // Serial.println(this->m_awaiting_acknoledge);
    // Serial.print("Backoff time: ");
    // Serial.println(this->m_backoff_time);
    // Serial.print("Last backoff: ");
    // Serial.println(this->m_last_backoff);
    // Serial.print("Free memory: ");
    // Serial.println(freeMemory());
    // Serial.print("Last health check: ");
    // Serial.println(this->m_last_health_check);
    // Serial.print("Health check delay: ");
    // Serial.println(this->m_health_check_delay);
    // Serial.print("Last packet id: ");
    // Serial.println(this->m_last_packet_id);
    // Serial.print("id counter: ");
    // Serial.println(this->m_id_counter);
}

/*
Sets the state of the radio to connected
If m_connected state changes then it will write to serial
Ressetting any accumlated backoff in the process
*/
void TransceiverPrimary::set_connected()
{
    if (!this->m_connected)
    {
        // add led
        this->m_connected = true;
        this->write_connection_status_to_serial(true);
        this->write_connection_status_to_callback_func(true);
    }
    this->m_last_health_check = millis();
    this->reset_backoff();
}

/*
Sets the state of the radio to disconnected
If m_connected state changes then it will write to serial
*/
void TransceiverPrimary::set_disconnected()
{
    if (this->m_connected)
    {
        this->m_connected = false;
        this->write_connection_status_to_serial(false);
        this->write_connection_status_to_callback_func(false);
    }
    this->m_last_health_check = millis();
}

/*
Used in the main loop of the program to handle time dependant logic and functions that need to be called frequently
*/
void TransceiverPrimary::tick()
{
    this->load_data_from_serial();
    this->receive();
    this->monitor_connection_health();
    this->send_data();
    this->clear_buffer();
}

/*
Writes data from m_received_packet to serial
*/
void TransceiverPrimary::write_data_to_serial()
{
    StaticJsonDocument<150> doc;

    for (int i = 0; i < this->m_received_packet.num_data_fields; i++)
    {
        doc[String(this->m_received_packet.data[i].key)] = this->m_received_packet.data[i].value;
    }
    serializeJson(doc, Serial);
    Serial.println();
}

/*
Passes recived data to callback function
*/
void TransceiverPrimary::write_data_to_callback_func()
{
    if (this->m_callback_func == NULL)
    {
        return;
    }
    Packet packet = this->m_received_packet;
    this->m_callback_func(packet);
}

/*
Writes the connections status to serial
*/
void TransceiverPrimary::write_connection_status_to_serial(bool connected)
{
    StaticJsonDocument<20> doc;
    doc["0"] = (connected) ? 1 : 0;
    serializeJson(doc, Serial);
    Serial.println();
}

/*
Writes the connections status to callback function
*/
void TransceiverPrimary::write_connection_status_to_callback_func(bool connected)
{
    if (this->m_callback_func == NULL)
    {
        return;
    }
    Packet packet;
    packet.id = 0;
    packet.num_data_fields = 1;
    packet.data[0].key = 0;
    packet.data[0].value = connected;
    this->m_callback_func(packet);
}

/*
    Loads an a single struct of data ready to be sent
*/
void TransceiverPrimary::load(Data data)
{
    Data dataArray[5];
    dataArray[0] = data;
    return this->load(dataArray, 1);
}

/*
    Loads an array of data with max size of 5, ready to be sent
*/
void TransceiverPrimary::load(Data data[5], unsigned char size)
{
    Packet packet;
    packet.id = this->increment_id();
    packet.num_data_fields = size;
    for (unsigned char i = 0; i < size; i++)
    {
        packet.data[i] = data[i];
    }

    if (this->m_send_packet.num_data_fields == 0)
    {
        this->m_send_packet = packet;
    }
    else
    {
        this->add_to_buffer(packet);
    }
}

/*
    Loads an array of data with specified size. The data will be split up into max packet size of 5.
*/
void TransceiverPrimary::load_data(Data *data, unsigned char size)
{
    const int max_size = 5;
    int num_packets = ceil((double)size / (double)max_size);
    int index = 0;
    for (int i = 0; i < num_packets; i++)
    {
        Data packet_data[5];
        unsigned char packet_size = 0;
        for (int j = 0; j < max_size && index < size; j++)
        {
            packet_data[j] = data[index++];
            packet_size = j;
        }
        this->load(packet_data, packet_size + 1);
    }
}

/*
    Converts string from serial into json and loads it to be sent.
    Should be around max 10 items at a time, increase StaticJsonDocument size to send more.
*/
void TransceiverPrimary::load_data_from_serial()
{
    if (Serial.available())
    {
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, Serial);
        if (error)
        {
            // Serial.println(error.f_str());
            return;
        }

        // serializeJson(doc, Serial);
        // Serial.println();

        Data packet_data[doc.size()];
        unsigned char index = 0;
        for (JsonPair kvp : doc.as<JsonObject>())
        {
            const char *key = kvp.key().c_str();
            float value = kvp.value().as<float>();

            Data data;
            data.key = atoi(key);
            data.value = value;

            packet_data[index] = data;
            index++;
        }

        this->load_data(packet_data, doc.size());
    }
}

/*
Sends the data from the send packet if specific conditions are met
If acknolged recived loads the next packet out of the buffer ready to be sent
If no acknolge then increase the backoff
*/
void TransceiverPrimary::send_data()
{
    if (this->m_awaiting_acknoledge || this->m_last_backoff + this->m_backoff_time > millis())
    {
        return;
    }

    this->m_radio.stopListening();
    bool acknoledged = this->m_radio.write(&this->m_send_packet, sizeof(this->m_send_packet));
    this->m_radio.startListening();

    if (acknoledged)
    {
        this->set_connected();
        packets_sent++;

        this->m_send_packet.num_data_fields = 0;
        this->m_awaiting_acknoledge = true;

        if (!this->m_buffer->isEmpty())
        {
            this->m_send_packet = this->m_buffer->pop().packet;
        }
    }
    else
    {
        this->increase_backoff();
    }
}

/*
Adds packet to buffer if there is already an item waiting to be sent
*/
void TransceiverPrimary::add_to_buffer(Packet packet)
{
    Buffered_packet buffered_packet;
    buffered_packet.packet = packet;
    buffered_packet.created_time = millis();
    this->m_buffer->unshift(buffered_packet);
}

/*
Clears packets out of the buffer if they are older than max_packet_lifetime attribute
*/
void TransceiverPrimary::clear_buffer()
{
    if (this->m_buffer->isEmpty())
    {
        return;
    }

    const unsigned int max_packet_lifetime = 1000;
    while (!this->m_buffer->isEmpty() && this->m_buffer->last().created_time + max_packet_lifetime < millis())
    {
        this->m_buffer->pop();
        packets_lost++;
    }
}

/*
Resets the backoff to default value of 2
*/
void TransceiverPrimary::reset_backoff()
{
    this->m_backoff_time = 2;
}

/*
Increases the backoff by a random amount exponentially but is capped by a random nunber between 800 to 1100
*/
void TransceiverPrimary::increase_backoff()
{
    const int max_backoff = random(800, 1100);
    this->m_backoff_time = min(this->m_backoff_time * (1 + (random(0, 99) / 100.0)), max_backoff);
    number_backoffs++;
    // Serial.println("Increasing backoff to: ");
    // Serial.println(this->m_backoff_time);
}

/*
Increments the packet ID
if packet ID is greater than 254 rolls back around to 1
*/
unsigned char TransceiverPrimary::increment_id()
{
    this->m_id_counter++;
    if (m_id_counter > 254)
    {
        m_id_counter = 1;
    }
    return m_id_counter;
}