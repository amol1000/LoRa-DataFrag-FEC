#include "mbed.h"
#include "sx1276-hal.h"
#include "debug.h"

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   0


/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                    868000000 // Hz
#define TX_OUTPUT_POWER                                 14        // 14 dBm

#if USE_MODEM_LORA == 1

    #define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                                  //  1: 250 kHz,
                                                                  //  2: 500 kHz,
                                                                  //  3: Reserved]
    #define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
    #define LORA_CODINGRATE                             1         // [1: 4/5,
                                                                  //  2: 4/6,
                                                                  //  3: 4/7,
                                                                  //  4: 4/8]
    #define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
    #define LORA_SYMBOL_TIMEOUT                         5         // Symbols
    #define LORA_FIX_LENGTH_PAYLOAD_ON                  false
    #define LORA_IQ_INVERSION_ON                        false

#elif USE_MODEM_FSK == 1

    #define FSK_FDEV                                    25e3      // Hz
    #define FSK_DATARATE                                9600      // bps
    #define FSK_BANDWIDTH                               50e3      // Hz
    #define FSK_AFC_BANDWIDTH                           83.333e3  // Hz
    #define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
    #define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
    #error "Please define a modem in the compiler options."
#endif

#define RX_TIMEOUT_VALUE                                3000000   // in us
#define BUFFER_SIZE                                     32        // Define the payload size here

/*
 * Callback functions prototypes
 */
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * @brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int8_t rssi, int8_t snr );

/*!
 * @brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * @brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * @brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/*
 *  Global variables declarations
 */
typedef RadioState States_t;

/*
 *  Global variables declarations
 */
SX1276MB1xAS Radio( OnTxDone, OnTxTimeout, OnRxDone, OnRxTimeout, OnRxError );

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

volatile States_t State = LOWPOWER;

double RssiValue = 0.0;
double SnrValue = 0.0;

int main() 
{
    uint8_t i;
    bool isMaster = true;
    
    debug( "\n\r\n\r     SX1276 Ping Pong Demo Application \n\r" );
        
#if defined TARGET_NUCLEO_L152RE
    debug( DEBUG_MESSAGE, "         > Nucleo-L152RE Platform <\r\n" );
#elif defined TARGET_KL25Z
    debug( DEBUG_MESSAGE, "         > KL25Z Platform <\r\n" );
#elif defined TARGET_LPC11U6X
    debug( DEBUG_MESSAGE, "         > LPC11U6X Platform <\r\n" );
#else
    debug( DEBUG_MESSAGE, "         > Untested Platform <\r\n" );
#endif
    
    debug( DEBUG_MESSAGE, "SX1276 Chipset Version = 0x%x \n\r", Radio.Read( REG_VERSION ) );
    
    Radio.SetChannel( RF_FREQUENCY ); 

#if USE_MODEM_LORA == 1

    debug("\n\r\n\r              > LORA Mode < \n\r\n\r");
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                         LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                         LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                         true, LORA_IQ_INVERSION_ON, 3000000 );
    
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                         LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                         LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                         true, LORA_IQ_INVERSION_ON, true );
                         
#elif USE_MODEM_FSK == 1

    debug("\n\r\n\r              > FSK Mode < \n\r\n\r");
    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                         FSK_DATARATE, 0,
                         FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                         true, 0, 3000000 );
    
    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                         0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                         0, FSK_FIX_LENGTH_PAYLOAD_ON, true,
                         false, true );
                         
#else

#error "Please define a modem in the compiler options."

#endif
     
    debug( DEBUG_MESSAGE, "Starting Ping-Pong loop\r\n" ); 
        
    Radio.Rx( RX_TIMEOUT_VALUE );
    
    while( 1 )
    {
        switch( State )
        {
        case RX:
            if( isMaster == true )
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
                    {
                        debug( "Pong...\r\n" );
                        // Send the next PING frame            
                        Buffer[0] = 'P';
                        Buffer[1] = 'I';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload 
                        for( i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        wait_ms( 10 ); 
                        Radio.Send( Buffer, BufferSize );
                    }
                    else if( strncmp( ( const char* )Buffer,
                             ( const char* )PingMsg, 4 ) == 0 )
                    { // A master already exists then become a slave
                        isMaster = false;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            else
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    {
                        debug( "Ping...\r\n" );
                        // Send the reply to the PONG string
                        Buffer[0] = 'P';
                        Buffer[1] = 'O';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload 
                        for( i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        wait_ms( 10 );  
                        Radio.Send( Buffer, BufferSize );
                    }
                }
            }
            State = LOWPOWER;
            break;
        case TX:
            if ( isMaster )
            {
                debug("...Ping\r\n" );
            }
            else
            {
                debug("...Pong\r\n" );
            }
            
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
            if( isMaster == true )
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for( i = 4; i < BufferSize; i++ )
                {
                    Buffer[i] = i - 4;
                }
                wait_ms( 10 ); 
                Radio.Send( Buffer, BufferSize );
            }
            else
            {
                Radio.Rx( RX_TIMEOUT_VALUE );  
            }             
            State = LOWPOWER;
            break;
        case RX_ERROR:
            if( isMaster == true )
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for( i = 4; i < BufferSize; i++ )
                {
                    Buffer[i] = i - 4;
                }
                wait_ms( 10 );  
                Radio.Send( Buffer, BufferSize );
            }
            else
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
            }
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case LOWPOWER:
            break;
        default:
            State = LOWPOWER;
            break;
        }    
    }
}

void OnTxDone( void )
{
    debug( DEBUG_MESSAGE, ":OnTxDone\n\r" );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int8_t rssi, int8_t snr)
{
    debug( DEBUG_MESSAGE, ":OnRxDone\n\r" );
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
}

void OnTxTimeout( void )
{
    debug( DEBUG_MESSAGE, ":OnTxTimeout\n\r" );
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    debug( DEBUG_MESSAGE, ":OnRxTimeout\n\r" );
    Radio.Sleep( );
    Buffer[ BufferSize ] = 0;
    State = RX_TIMEOUT;
}

void OnRxError( void )
{
    debug( DEBUG_MESSAGE, ":OnRxError\n\r" );
    Radio.Sleep( );
    State = RX_ERROR;
}
