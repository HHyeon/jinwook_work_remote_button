

#ifndef ____STM_NRF24_LIBRARY_ARDUINO_FORKED
#define ____STM_NRF24_LIBRARY_ARDUINO_FORKED


#include "nRF24L01.h"

#define _BV(n) (1<<n)

uint8_t write_reg(uint8_t reg, uint8_t data);
uint8_t read_reg(uint8_t reg);

uint8_t NRF24L01_Init(void);

void openWritingPipe(const uint8_t *radio_address, uint8_t addr_len, uint8_t payload_len);
void openReadingPipe(uint8_t child, const uint8_t *radio_address, uint8_t addr_len, uint8_t payload_len);

void startListening(uint8_t *radio_address, uint8_t addr_len);
void stopListening(void);

uint8_t nrf24_rxfifo_available();
void nrf24_read_payload( uint8_t* buf, uint8_t rlen);
uint8_t get_incoming_pipeno();

uint8_t nrf24_write_polling( uint8_t* buf, uint8_t payload_len);
void nrf24_write_IT(uint8_t* buf, uint8_t payload_len);
void nrf24_ack_write_IT(uint8_t* buf, uint8_t payload_len, uint8_t pipeno);
uint8_t nrf24_ack_write_polling(uint8_t* buf, uint8_t payload_len, uint8_t pipeno);

void setPALevel(uint8_t level);
uint8_t setDataRate(rf24_datarate_e speed);

uint8_t get_status(void);
uint8_t nrf24_clear_flags();


void nrf24_flush_tx(void);
void nrf24_flush_rx(void);

void nrf24_enableAckPayload(void);
void nrf24_setAutoAck(uint8_t enable);

#endif