#pragma once
#include "Transceiver.h"
#include <Arduino.h>
#include <RF24.h>
#include <math.h>
#include <ArduinoJson.h>
#include <CircularBuffer.h>

// PRIMARY TRANSMITTER

TransceiverPrimary::TransceiverPrimary(int ce_pin, int csn_pin) : m_radio(ce_pin, csn_pin),
                                                                  m_connected(false),
                                                                  m_last_health_check(millis()),
                                                                  m_health_check_delay(1000),
                                                                  m_radio_listening(false),
                                                                  m_backoff_time(1),
                                                                  m_last_backoff(0),
                                                                  m_buffer(new CircularBuffer<Buffered_packet, 10>()),
                                                                  m_received_packet(),
                                                                  m_last_packet_id(0),
                                                                  m_id_counter(0){};

TransceiverPrimary::~TransceiverPrimary()
{
    this->m_buffer->clear();
    delete this->m_buffer;
}

void TransceiverPrimary::setup(byte address[6])
{
    this->m_radio.begin();
    this->m_radio.openReadingPipe(0, address);
    this->m_radio.openWritingPipe(address);
    this->m_radio.enableDynamicAck();
    this->m_radio.setPALevel(RF24_PA_MIN); // RF24_PA_MAX
    this->m_radio.setDataRate(RF24_250KBPS);
    this->m_radio.setAutoAck(true);
    this->m_radio.setRetries(5, 15);
    this->m_radio.stopListening();

    this->connect();
}

void TransceiverPrimary::monitor_connection_health()
{
    if (this->m_last_health_check + this->m_health_check_delay < millis())
    {
        this->send_telemetry_error();
    }
}

void TransceiverPrimary::connect()
{
    Data data;
    data.key = 0;
    data.value = 1;
    this->send(data);
}

// Sends telemetry error warning that the it has recivied no telemetry to send from serial but its still connected (if the message gets through)
bool TransceiverPrimary::send_telemetry_error()
{
    Data data;
    data.key = 100;
    data.value = 1;

    bool acknowledged = this->send(data, false);
    if (acknowledged == true)
    {
        this->set_connected();
    }
    else
    {
        this->set_disconnected();
    }
    return acknowledged;
}

void TransceiverPrimary::debug()
{
    Serial.println();
    Serial.print("Buffer: ");
    Serial.println(this->m_buffer->size());
    // Serial.print("Buffer available: ");
    // Serial.println(this->m_buffer->available());
    // Serial.print("Backoff time: ");
    // Serial.println(this->m_backoff_time);
    // Serial.print("Free memory: ");
    // Serial.println(freeMemory());
    // Serial.print("Last health check: ");
    // Serial.println(this->m_last_health_check);
    // Serial.print("Health check delay: ");
    // Serial.println(this->m_health_check_delay);
    // Serial.print("Last backoff: ");
    // Serial.println(this->m_last_backoff);
    // Serial.print("Last packet id: ");
    // Serial.println(this->m_last_packet_id);
    // Serial.print("id counter: ");
    // Serial.println(this->m_id_counter);
}

void TransceiverPrimary::set_connected()
{
    if (!this->m_connected)
    {
        this->m_connected = true;
        this->write_connection_status_to_serial(true);
    }
    this->m_last_health_check = millis();
    this->reset_backoff();
}

void TransceiverPrimary::set_disconnected()
{
    if (this->m_connected)
    {
        this->m_connected = false;
        this->write_connection_status_to_serial(false);
    }
    this->m_last_health_check = millis();
}

void TransceiverPrimary::start_radio_listening()
{
    if (this->m_radio_listening == false)
    {
        this->m_radio.startListening();
        this->m_radio_listening = true;
    }
}

void TransceiverPrimary::stop_radio_listening()
{
    if (this->m_radio_listening == true)
    {
        this->m_radio.stopListening();
        this->m_radio_listening = false;
    }
}

void TransceiverPrimary::tick()
{
    this->receive();
    this->clear_buffer();
    this->monitor_connection_health();
}

void TransceiverPrimary::receive()
{
    this->start_radio_listening();
    if (this->m_radio.available())
    {
        this->m_radio.read(&this->m_received_packet, sizeof(this->m_received_packet));
        if (this->m_last_packet_id != this->m_received_packet.id)
        {
            this->write_data_to_serial();
            this->m_last_packet_id = this->m_received_packet.id;
        }
        this->set_connected();
    }
}

void TransceiverPrimary::write_data_to_serial()
{
    DynamicJsonDocument doc(sizeof(this->m_received_packet.data));
    for (int i = 0; i < this->m_received_packet.num_data_fields; i++)
    {
        doc[String(this->m_received_packet.data[i].key)] = this->m_received_packet.data[i].value;
    }
    serializeJson(doc, Serial);
}

void TransceiverPrimary::write_connection_status_to_serial(bool connected)
{
    StaticJsonDocument<200> doc;
    doc["0"] = (connected) ? 1 : 0;
    serializeJson(doc, Serial);
}

bool TransceiverPrimary::send(Data data, bool buffer)
{
    Data dataArray[7];
    dataArray[0] = data;
    return this->send(dataArray, 1, buffer);
}

bool TransceiverPrimary::send(Data data[7], int size, bool buffer)
{
    this->stop_radio_listening();

    Packet packet;
    for (int i = 0; i < size; i++)
    {
        packet.data[i] = data[i];
    }
    packet.id = this->incrementId();
    packet.num_data_fields = size;

    if (!this->m_buffer->isEmpty() && buffer)
    {
        this->add_to_buffer(packet);
        return false;
    }

    bool acknowledged = this->m_radio.write(&packet, sizeof(packet));

    if (acknowledged)
    {
        this->set_connected();
    }
    else
    {
        if (buffer)
        {
            this->add_to_buffer(packet);
        }
    };
    return acknowledged;
}

bool TransceiverPrimary::sendLarge(Data *data, int size, bool buffer)
{
    bool all_packets_sent = true;
    const int max_size = 7;
    int num_packets = ceil(size / max_size);

    int index = 0;
    for (int i = 0; i < num_packets; i++)
    {
        Data packet_data[7];
        unsigned char packet_size = 0;
        for (int j = 0; j < max_size && index < size; j++)
        {
            packet_data[j] = data[index++];
            packet_size = j;
        }

        if (!this->send(packet_data, packet_size, buffer))
        {
            all_packets_sent = false;
        };
    }
    return all_packets_sent;
}

bool TransceiverPrimary::send_buffered_packet(Packet packet)
{
    this->stop_radio_listening();
    bool acknowledged = this->m_radio.write(&packet, sizeof(packet));

    if (acknowledged)
    {
        this->set_connected();
    }
    return acknowledged;
}

void TransceiverPrimary::add_to_buffer(Packet packet)
{
    Buffered_packet buffered_packet;
    buffered_packet.packet = packet;
    buffered_packet.created_time = millis();
    this->m_buffer->unshift(buffered_packet);
    this->m_last_backoff = millis();
    this->increase_backoff();
}

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
    }

    if (this->m_last_backoff + this->m_backoff_time > millis())
    {
        return;
    }

    while (!this->m_buffer->isEmpty())
    {
        if (this->send_buffered_packet(this->m_buffer->last().packet))
        {
            this->m_buffer->pop();
        }
        else
        {
            this->m_last_backoff = millis();
            this->increase_backoff();
            return;
        }
    }
}

void TransceiverPrimary::reset_backoff()
{
    this->m_backoff_time = 1;
}

void TransceiverPrimary::increase_backoff()
{
    this->m_backoff_time = this->m_backoff_time * (1 + (random(0, 99) / 100.0));
}

unsigned char TransceiverPrimary::incrementId()
{
    this->m_id_counter++;
    if (m_id_counter > 254)
    {
        m_id_counter = 1;
    }
    return m_id_counter;
}