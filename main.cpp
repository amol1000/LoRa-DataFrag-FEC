#include "mbed.h"
#include "main.h"
#include "sx1276-hal.h"

extern "C"{
    #include "frag.h"
    #include "packets.h"
}

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   1

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
    #define LORA_FHSS_ENABLED                           false
    #define LORA_NB_SYMB_HOP                            4
    #define LORA_IQ_INVERSION_ON                        false
    #define LORA_CRC_ENABLED                            true

#elif USE_MODEM_FSK == 1

    #define FSK_FDEV                                    25000     // Hz
    #define FSK_DATARATE                                19200     // bps
    #define FSK_BANDWIDTH                               50000     // Hz
    #define FSK_AFC_BANDWIDTH                           83333     // Hz
    #define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
    #define FSK_FIX_LENGTH_PAYLOAD_ON                   false
    #define FSK_CRC_ENABLED                             true

#else
    #error "Please define a modem in the compiler options."
#endif

#define RX_TIMEOUT_VALUE                                3500      // in ms
#define BUFFER_SIZE                                     32        // Define the payload size here

#define SEC_TO_MSEC  (1000)


/* brief::
M == The  initial  data  block  that  needs  to  be  transported  must
     first  be  fragmented  into  M  data 417fragments  of arbitrary  but  equal  length
N == bytes to be sent
*/
#define FRAG_NB                 (10) // data block will be divided into 10 fragments
#define FRAG_SIZE               (19) // each fragment size will be 10 bytes
// thus data block size is 10 * 10 == 100
#define FRAG_CR                 (FRAG_NB - 5) // basically M/N
#define FRAG_PER                (0.3)// changes the lost packet count
#define FRAG_TOLERENCE          (10 + FRAG_NB * (FRAG_PER + 0.05))
#define LOOP_TIMES              (1)
#define DEBUG
#define IS_MASTER               (0)

#if IS_MASTER
frag_enc_t encobj;
//uint8_t enc_dt[FRAG_NB * FRAG_SIZE]; // 100 bytes
uint8_t enc_buf[FRAG_NB * FRAG_SIZE + FRAG_CR * FRAG_SIZE]; //+ FRAG_NB * FRAG_CR]; // //100 + 20 * 10 + 20 * 10 == 500 bytes

#else
frag_dec_t decobj;
uint8_t dec_buf[(FRAG_NB + FRAG_CR) * FRAG_SIZE ];
uint8_t dec_flash_buf[(FRAG_NB + FRAG_CR) * FRAG_SIZE];
#endif

/*
 *  Global variables declarations
 */
typedef enum
{
    LOWPOWER = 0,
    IDLE,

    RX,
    RX_TIMEOUT,
    RX_ERROR,

    TX,
    TX_TIMEOUT,

    CAD,
    CAD_DONE
}AppStates_t;

volatile AppStates_t State = LOWPOWER;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

RawSerial pc(USBTX, USBRX);
/*
 *  Global variables declarations
 */
SX1276MB1xAS Radio( NULL );

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE]; // cast into packet, each packet can be of size 21 bytes

int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;

#if !IS_MASTER
int flash_write(uint32_t addr, uint8_t *buf, uint32_t len)
{
    memcpy(dec_flash_buf + addr, buf, len);
    return 0;
}

int flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    memcpy(buf, dec_flash_buf + addr, len);
    return 0;
}
#endif
void putbuf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        debug("%02X ", buf[i]);
    }
    debug("\r\n");
}

void put_bool_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        debug("%d ", buf[i]);
    }
    debug("\r\n");
}

void frag_encobj_log(frag_enc_t *encobj)
{

    int i;

    printf("uncoded blocks:\r\n");
    for (i = 0; i < encobj->num * encobj->unit; i += encobj->unit) {
        putbuf(encobj->line + i, encobj->unit);
    }

    printf("\ncoded blocks:\r\n");
    for (i = 0; i < encobj->cr * encobj->unit; i += encobj->unit) {
        putbuf(encobj->rline + i, encobj->unit);
    }

    printf("\nmatrix line:\r\n");
    for (i = 0; i < encobj->num * encobj->cr; i += encobj->num) {
        put_bool_buf(encobj->mline + i, encobj->num);
    }
}

void radioEvents(){

    bool isMaster = IS_MASTER;
    uint16_t frag_tx = 0;

    while( 1 )
    {
        switch( State )
        {
        case RX:
            //rx_count++;
#if isMaster == 1
            if( isMaster == true )
            {
                debug("Master is receiving data\r\n");
            }
#elif IS_MASTER == 0
            if(!isMaster) //slave
            {
                if( BufferSize > 0 )
                {
                    debug("Data from master\r\n");
                    dataFrag *packet = (dataFrag*) Buffer;
                    //putbuf(Buffer, BUFFER_SIZE);

                    debug("seq_num %d\r\n", packet->seqNum);
                    if(packet->seqNum != frag_tx){
                        debug("received giberrish(seqNum is %d, should have been %d), dropping corrupt packet\r\n",
                                packet->seqNum, frag_tx);
                        frag_tx++;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                        State = LOWPOWER;
                        break;
                    }
                    putbuf(packet->data, FRAG_SIZE);

                    frag_tx++;
                    if(packet->seqNum == 8 || packet->seqNum == 5 /*|| packet->seqNum == 42 || packet->seqNum == 30*/
                        ){
                        debug("data dropped\r\n");
                        Radio.Rx( RX_TIMEOUT_VALUE );
                        State = LOWPOWER;
                        break;
                    }

                    int ret = frag_dec(&decobj, packet->seqNum+1, packet->data, decobj.cfg.size);
                    if (ret == FRAG_DEC_ONGOING) {
                        //printf("\n");
                        debug(" decoding ongoing\r\n");
                    } else if (ret >= 0) {
                        printf("dec complete (reconstruct %d packets)\r\n", ret);
                        frag_dec_log(&decobj);
                        Radio.Rx( RX_TIMEOUT_VALUE );
                        State = LOWPOWER;
                        break;
                    } else {
                        printf("dec error %d\r\n", ret);
                        //frag_dec_log(&decobj);
                        Radio.Rx( RX_TIMEOUT_VALUE );
                        State = LOWPOWER;
                        break;

                    }
                }
            }
#endif
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case TX:
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
#if IS_MASTER == 1
            if( isMaster == true )
            {
                if(frag_tx >= encobj.num + encobj.cr){
                    break;
                }
                debug("RX_Timeout... sending data set fragments\r\n");
                debug("sending fragment:%d: \t", frag_tx);

                dataFrag Frag = {0, {0}};
                dataFrag *packet = &Frag;
                packet->seqNum = frag_tx;

                memcpy(packet->data, encobj.line + frag_tx * FRAG_SIZE, FRAG_SIZE);
                putbuf(encobj.line + frag_tx*FRAG_SIZE, FRAG_SIZE);

                debug("sending packet with seq: %d & data : \t", packet->seqNum);
                putbuf(packet->data, FRAG_SIZE);
                wait_ms( 10 );
                Radio.Send( (uint8_t*)packet, BUFFER_SIZE);
                frag_tx++;
            }
#endif
            if(!isMaster)
            {
                debug("Master(%d): waiting for data \r\n", isMaster);
                Radio.Rx( RX_TIMEOUT_VALUE );
            }
            State = LOWPOWER;
            break;
        case RX_ERROR:
            debug("RX_ERROR\r\n");
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case LOWPOWER:
            wait_ms(1000);
            break;
        default:
            State = LOWPOWER;
            break;
        }
    }
}

int main( void )
{
    bool isMaster = IS_MASTER;

    while(1){
        debug("Press 1 to start, isMaster(%d)\r\n", isMaster);
        char c = pc.getc();
        if(c == '1'){
             break;
        }
    }

    int enc_size = (FRAG_NB * FRAG_SIZE + FRAG_CR * FRAG_SIZE + FRAG_NB * FRAG_CR);
    debug("enc size is %d\r\n", enc_size);
    //enc_buf = (uint8_t*)malloc(enc_size*sizeof(uint8_t));

#if IS_MASTER == 1
    uint16_t i;
    if(isMaster){
        for (i = 0; i < FRAG_NB * FRAG_SIZE; i++) {
            enc_buf[i] = i;
        }


        encobj.dt = enc_buf;
        encobj.maxlen = (FRAG_NB * FRAG_SIZE + FRAG_CR * FRAG_SIZE + FRAG_NB * FRAG_CR);
        clock_t start, end;
        //encobj.unit = FRAG_SIZE;
        //encobj.num = (FRAG_NB*FRAG_SIZE)/FRAG_SIZE;
       // encobj.cr = FRAG_CR
        //encobj.line = obj->dt;
        //encobj.rline = obj->dt + len;
        //encobj.mline = obj->dt + len + cr * unit;
        start = clock();
        int ret = frag_enc(&encobj, enc_buf, FRAG_NB * FRAG_SIZE, FRAG_SIZE, FRAG_CR);
        end = clock();
        printf("enc ret %d, maxlen %d, duration %f\r\n", ret, encobj.maxlen, (end - start)/(double)CLOCKS_PER_SEC);
        frag_encobj_log(&encobj);
    }
#elif IS_MASTER == 0
    if(!isMaster) {
        printf("\n\n-------------------\n");
        decobj.cfg.dt = dec_buf;
        decobj.cfg.maxlen = sizeof(dec_buf);
        decobj.cfg.nb = FRAG_NB;
        decobj.cfg.size = FRAG_SIZE;
        decobj.cfg.tolerence = FRAG_TOLERENCE;
        decobj.cfg.frd_func = flash_read;
        decobj.cfg.fwr_func = flash_write;
        int len = frag_dec_init(&decobj);
        debug("memory cost: %d, nb %d, size %d, tol %d\n",
           len,
           decobj.cfg.nb,
           decobj.cfg.size,
           decobj.cfg.tolerence);
    }
#endif
    debug( "\n\n\r     SX1276 Ping Pong Demo Application \n\n\r" );

    // Initialize Radio driver
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxError = OnRxError;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init( &RadioEvents );

    // verify the connection with      the board
    while( Radio.Read( REG_VERSION ) == 0x00  )
    {
        debug( "Radio could not be detected!\n\r", NULL );
        wait( 1 );
    }

    debug_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1276MB1LAS ) ), "\n\r > Board Type: SX1276MB1LAS < \n\r" );
    debug_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1276MB1MAS ) ), "\n\r > Board Type: SX1276MB1MAS < \n\r" );

    Radio.SetChannel( RF_FREQUENCY );

#if USE_MODEM_LORA == 1

    debug_if( LORA_FHSS_ENABLED, "\n\n\r             > LORA FHSS Mode < \n\n\r" );
    debug_if( !LORA_FHSS_ENABLED, "\n\n\r             > LORA Mode < \n\n\r" );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                         LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                         LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                         LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                         LORA_IQ_INVERSION_ON, 2000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                         LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                         LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                         LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                         LORA_IQ_INVERSION_ON, true );

#elif USE_MODEM_FSK == 1

    debug("\n\n\r              > FSK Mode < \n\n\r" );
    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                         FSK_DATARATE, 0,
                         FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                         FSK_CRC_ENABLED, 0, 0, 0, 2000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                         0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                         0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, FSK_CRC_ENABLED,
                         0, 0, false, true );

#else

#error "Please define a modem in the compiler options."

#endif

    debug_if( DEBUG_MESSAGE, "Starting Ping-Pong loop\r\n" );

    Radio.Rx( RX_TIMEOUT_VALUE );

    radioEvents();
}

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
    //debug_if( DEBUG_MESSAGE, "> OnTxDone\n\r" );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
    //debug_if( DEBUG_MESSAGE, "> OnRxDone\n\r" );
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
    //debug_if( DEBUG_MESSAGE, "> OnTxTimeout\n\r" );
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    Buffer[BufferSize] = 0;
    State = RX_TIMEOUT;
    //debug_if( DEBUG_MESSAGE, "> OnRxTimeout\n\r" );
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    //debug_if( DEBUG_MESSAGE, "> OnRxError\n\r" );
}
