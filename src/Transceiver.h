#pragma once
#include <RF24.h>
#include <CircularBuffer.h>

struct Data
{
    unsigned int key;
    float value;
};

struct Packet
{
    unsigned char id;
    unsigned char num_data_fields;
    Data data[5];
} __attribute__((packed));

struct Buffered_packet
{
    Packet packet;
    unsigned long created_time;
};

class TransceiverPrimary
{
private:
    RF24 m_radio;
    bool m_connected;
    unsigned long m_last_health_check;
    unsigned int m_health_check_delay;
    float m_backoff_time;
    unsigned long m_last_backoff;
    bool m_awaiting_acknoledge;
    CircularBuffer<Buffered_packet, 20> *m_buffer;
    Packet m_send_packet;
    Packet m_received_packet;
    unsigned char m_last_packet_id;
    unsigned char m_id_counter;

public:
    TransceiverPrimary(int ce_pin, int csn_pin);
    ~TransceiverPrimary();
    TransceiverPrimary(const TransceiverPrimary &other) = delete;
    void tick();
    void load_data(Data *data, unsigned char size);
    void setup(byte address[6]);
    void debug();

private:
    void load(Data data);
    void load(Data data[5], unsigned char size);
    void set_connected();
    void set_disconnected();
    void monitor_connection_health();
    void receive();
    void send_data();
    void load_data_from_serial();
    void write_data_to_serial();
    void write_connection_status_to_serial(bool connected);
    void add_to_buffer(Packet Packet);
    void clear_buffer();
    void reset_backoff();
    void increase_backoff();
    unsigned char increment_id();
    void syncronise_radios();
};