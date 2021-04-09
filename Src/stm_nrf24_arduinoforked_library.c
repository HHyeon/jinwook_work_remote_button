

#include "main.h"
#include "stm_nrf24_arduinoforked_library.h"
#include "string.h"


extern SPI_HandleTypeDef hspi1;





#define NRF24_XFER_TIMEOUT 0x1000
#define NRF24_CS_BEGIN() HAL_GPIO_WritePin(SPI1_NRF_CSN_GPIO_Port, SPI1_NRF_CSN_Pin, GPIO_PIN_RESET);
#define NRF24_CS_END() HAL_GPIO_WritePin(SPI1_NRF_CSN_GPIO_Port, SPI1_NRF_CSN_Pin, GPIO_PIN_SET);
#define NRF24_CE_LOW() HAL_GPIO_WritePin(SPI1_NRF_CE_GPIO_Port, SPI1_NRF_CE_Pin, GPIO_PIN_RESET);
#define NRF24_CE_HIGH() HAL_GPIO_WritePin(SPI1_NRF_CE_GPIO_Port, SPI1_NRF_CE_Pin, GPIO_PIN_SET);

static uint32_t txDelay;
static uint8_t dummy[32];
static uint8_t pdata[2];

static const uint8_t child_pipe[] =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const uint8_t child_pipe_enable[] =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};


uint8_t write_reg(uint8_t reg, uint8_t data)
{
  uint8_t res = 0;
  pdata[0] = ( W_REGISTER | ( REGISTER_MASK & reg ) );
  pdata[1] = data;
  
  NRF24_CS_BEGIN();
  HAL_SPI_TransmitReceive(&hspi1, &pdata[0], &res, 1, NRF24_XFER_TIMEOUT);
  HAL_SPI_Transmit(&hspi1, &pdata[1], 1 ,NRF24_XFER_TIMEOUT);
  NRF24_CS_END();
  
  return res;
}

void write_registers(uint8_t reg, const uint8_t* buf, uint8_t len)
{
  pdata[0] = ( W_REGISTER | ( REGISTER_MASK & reg ) );
  
  NRF24_CS_BEGIN();
  HAL_SPI_Transmit(&hspi1, pdata, 1 ,NRF24_XFER_TIMEOUT);
  HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, len ,NRF24_XFER_TIMEOUT*len);
  NRF24_CS_END();
}

uint8_t read_reg(uint8_t reg)
{
  uint8_t data;
  
  NRF24_CS_BEGIN();
  
  data = R_REGISTER | ( REGISTER_MASK & reg );
  HAL_SPI_Transmit(&hspi1, &data, 1, NRF24_XFER_TIMEOUT);
  HAL_SPI_Receive(&hspi1, &data, 1, NRF24_XFER_TIMEOUT);
  
  NRF24_CS_END();
  
  return data;
}


uint8_t setDataRate(rf24_datarate_e speed)
{
  uint8_t result = 0;
  uint8_t setup = read_reg(RF_SETUP) ;
  
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;

  txDelay=250;
  
  if( speed == RF24_250KBPS )
  {
    setup |= _BV( RF_DR_LOW ) ;
    txDelay=450;
  }
  else
  {
    if ( speed == RF24_2MBPS )
    {
      setup |= _BV(RF_DR_HIGH);
      txDelay=190;
    }
  }
  write_reg(RF_SETUP,setup);
  
  if ( read_reg(RF_SETUP) == setup )
  {
    result = 1;
  }
  return result;
}

static void toggle_features(void)
{
  pdata[0] = ACTIVATE;
  pdata[1] = 0x73;
  
  NRF24_CS_BEGIN();
  HAL_SPI_Transmit(&hspi1, pdata, 2 ,NRF24_XFER_TIMEOUT);
  NRF24_CS_END();
}

static void setChannel(uint8_t channel)
{
  const uint8_t max_channel = 125;
  write_reg(RF_CH,rf24_min(channel,max_channel));
}

void nrf24_flush_rx(void)
{
  pdata[0] = FLUSH_RX;
  NRF24_CS_BEGIN();
  HAL_SPI_Transmit(&hspi1, pdata, 1 ,NRF24_XFER_TIMEOUT);
  NRF24_CS_END();  
}

void nrf24_flush_tx(void)
{
  pdata[0] = FLUSH_TX;
  NRF24_CS_BEGIN();
  HAL_SPI_Transmit(&hspi1, pdata, 1 ,NRF24_XFER_TIMEOUT);
  NRF24_CS_END();  
}

static void powerUp(void)
{
   uint8_t cfg = read_reg(NRF_CONFIG);

   // if not powered up then power up and wait for the radio to initialize
   if (!(cfg & _BV(PWR_UP))){
      write_reg(NRF_CONFIG, cfg | _BV(PWR_UP));

      // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
	  // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
	  // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
      HAL_Delay(5);
   }
}


void openWritingPipe(const uint8_t *radio_address, uint8_t addr_len, uint8_t payload_len)
{
  write_registers(child_pipe[0],radio_address, addr_len);
  write_reg(child_payload_size[0],payload_len);
  write_registers(TX_ADDR, radio_address, addr_len);
}

void openReadingPipe(uint8_t child, const uint8_t *radio_address, uint8_t addr_len, uint8_t payload_len)
{
  write_registers(child_pipe[child], radio_address, addr_len);
  write_reg(child_payload_size[child],payload_len);
  write_reg(EN_RXADDR,read_reg(EN_RXADDR) | _BV(child_pipe_enable[child]));
}

void closeReadingPipe( uint8_t pipe )
{
  write_reg(EN_RXADDR,read_reg(EN_RXADDR) & ~_BV(child_pipe_enable[pipe]));
}

void startListening(uint8_t *radio_address, uint8_t addr_len)
{
  HAL_Delay(txDelay);
  
  write_reg(NRF_CONFIG, read_reg(NRF_CONFIG) | _BV(PRIM_RX));
  nrf24_clear_flags();

  write_registers(child_pipe[0], radio_address, addr_len);
  
  if(read_reg(FEATURE) & _BV(EN_ACK_PAY)){
	nrf24_flush_tx();
  }

}

void stopListening(void)
{
  HAL_Delay(txDelay);
  
  if(read_reg(FEATURE) & _BV(EN_ACK_PAY)){
    HAL_Delay(txDelay); 
    nrf24_flush_tx();
  }
  write_reg(NRF_CONFIG, ( read_reg(NRF_CONFIG) ) & ~_BV(PRIM_RX) );
 
  write_reg(EN_RXADDR,read_reg(EN_RXADDR) | _BV(child_pipe_enable[0])); // Enable RX on pipe0
}

uint8_t get_status(void)
{
  pdata[0] = RF24_NOP;
  
  NRF24_CS_BEGIN();
  HAL_SPI_TransmitReceive(&hspi1, &pdata[0], &pdata[1], 1,NRF24_XFER_TIMEOUT);
  NRF24_CS_END();
  
  return pdata[1];
}

static void write_payload(uint8_t* buf, uint8_t payload_len) // send 32 byte
{
  pdata[0] = W_TX_PAYLOAD;
  if(payload_len > 32) payload_len = 32;
  uint8_t dummylen = 32 - payload_len;
  
  NRF24_CS_BEGIN();
  
  HAL_SPI_Transmit(&hspi1, pdata, 1, NRF24_XFER_TIMEOUT);

  HAL_SPI_Transmit(&hspi1, buf, payload_len, NRF24_XFER_TIMEOUT);
  
  if(dummylen)
  {
    memset(dummy, 0, sizeof(dummy));
    HAL_SPI_Transmit(&hspi1, dummy, dummylen, NRF24_XFER_TIMEOUT);    
  }

  NRF24_CS_END();
}

static void write_ack_payload(uint8_t* buf, uint8_t payload_len,uint8_t pipeno) // send 32 byte
{
  pdata[0] = W_ACK_PAYLOAD | (pipeno&0x7);
  if(payload_len > 32) payload_len = 32;
  uint8_t dummylen = 32 - payload_len;
  
  NRF24_CS_BEGIN();
  
  HAL_SPI_Transmit(&hspi1, pdata, 1, NRF24_XFER_TIMEOUT);

  HAL_SPI_Transmit(&hspi1, buf, payload_len, NRF24_XFER_TIMEOUT);
  
  if(dummylen)
  {
    memset(dummy, 0, sizeof(dummy));
    HAL_SPI_Transmit(&hspi1, dummy, dummylen, NRF24_XFER_TIMEOUT);    
  }

  NRF24_CS_END();
}

uint8_t nrf24_write_polling( uint8_t* buf, uint8_t payload_len)
{
  write_payload(buf, payload_len);
  
  while( ! ( (get_status())  & ( _BV(TX_DS) | _BV(MAX_RT) ))) ;

  uint8_t status = nrf24_clear_flags();
  
  if(status & (_BV(MAX_RT)))
  {
    nrf24_flush_tx();
    return 0;
  }
  
   return 1;
}

void nrf24_write_IT(uint8_t* buf, uint8_t payload_len)
{
  write_payload(buf, payload_len);
}

void nrf24_ack_write_IT(uint8_t* buf, uint8_t payload_len, uint8_t pipeno)
{
  write_ack_payload(buf, payload_len, pipeno);
}

uint8_t nrf24_ack_write_polling(uint8_t* buf, uint8_t payload_len, uint8_t pipeno)
{
  write_ack_payload(buf, payload_len, pipeno);
  
  while( ! ( (get_status())  & ( _BV(TX_DS) | _BV(MAX_RT) ))) ;
  
  uint8_t status = nrf24_clear_flags();
  
  if(status & (_BV(MAX_RT)))
  {
    nrf24_flush_tx();
    return 0;
  }
  
  return 1;
}


static void read_payload(uint8_t* buf, uint8_t rlen)
{
  if(rlen > 32) rlen = 32;
  uint8_t dummyrlen = 32 - rlen;

  pdata[0] = R_RX_PAYLOAD;
  
  NRF24_CS_BEGIN();
  
  HAL_SPI_Transmit(&hspi1, pdata, 1, NRF24_XFER_TIMEOUT);
  HAL_SPI_Receive(&hspi1, buf, rlen, NRF24_XFER_TIMEOUT);

  if(dummyrlen)
    HAL_SPI_Receive(&hspi1, dummy, dummyrlen, NRF24_XFER_TIMEOUT);

  NRF24_CS_END();
}

void nrf24_read_payload( uint8_t* buf, uint8_t rlen)
{

  // Fetch the payload
  read_payload( buf , rlen);

  //Clear the two possible interrupt flags with one command
  nrf24_clear_flags();

}

static void setRetries(uint8_t delay, uint8_t count)
{
  write_reg(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}

uint8_t NRF24L01_Init(void)
{
  uint8_t setup=0;
  
  NRF24_CS_END();
  
  HAL_Delay(1);
  
  write_reg(NRF_CONFIG, 0x0C);
  
  setRetries(15,15);
  
  setDataRate( RF24_250KBPS );
    
  setup = read_reg(RF_SETUP);
  
  setDataRate( RF24_250KBPS ) ;
  
  toggle_features();
  write_reg(FEATURE,0);
  write_reg(DYNPD,0);
  
  nrf24_clear_flags();
  
  setChannel(76);
  
  nrf24_flush_rx();
  nrf24_flush_tx();
  
  powerUp();  
  
  write_reg(NRF_CONFIG, ( read_reg(NRF_CONFIG) ) & ~_BV(PRIM_RX) );  
  
  NRF24_CE_HIGH();

  return ( setup != 0 && setup != 0xff );
}


uint8_t nrf24_rxfifo_available()
{
  if (!( read_reg(FIFO_STATUS) & _BV(RX_EMPTY) ) )
  {
    return 1;
  }
  return 0;
}

uint8_t get_incoming_pipeno()
{
  pdata[0] = get_status();
  pdata[0]>>=RX_P_NO;
  pdata[0]&=0x07;
  return pdata[0];
}


void setPALevel(uint8_t level)
{
  uint8_t setup = read_reg(RF_SETUP) & 0xF8;

  if(level > 3) level = 3;
  
  level = (level << 1) | 1;
  
  write_reg( RF_SETUP, setup |= level ) ;
}

uint8_t nrf24_clear_flags()
{
  return write_reg(NRF_STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
}


void nrf24_setAutoAck(uint8_t enable)
{
  if ( enable )
    write_reg(EN_AA, 0x3F);
  else
    write_reg(EN_AA, 0);
}

void nrf24_enableAckPayload(void)
{
    write_reg(FEATURE,read_reg(FEATURE) | _BV(EN_ACK_PAY));
}
