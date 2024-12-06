/*______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
(C)2024 Semtech

Description:
    LoRaHub Hardware Abstraction Layer

License: Revised BSD License, see LICENSE.TXT file include in the project
*/

/* -------------------------------------------------------------------------- */
/* --- DEPENDENCIES --------------------------------------------------------- */

#include <string.h>

#include <esp_timer.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "lorahub_aux.h"
#include "lorahub_hal.h"

#include "radio_context.h"
#include "ral_defs.h"

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)
#define ATLEN   256

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define CHECK_NULL( a )       \
    if( a == NULL )           \
    {                         \
        return LGW_HAL_ERROR; \
    }

#define SET_PPM_ON( bw, dr )                                                             \
    ( ( ( bw == BW_125KHZ ) && ( ( dr == DR_LORA_SF11 ) || ( dr == DR_LORA_SF12 ) ) ) || \
      ( ( bw == BW_250KHZ ) && ( dr == DR_LORA_SF12 ) ) )

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */


static const char* TAG_HAL = "LORAHUB_HAL";

#define LORA_SYNC_WORD_PRIVATE 0x12  // 0x12 Private Network
#define LORA_SYNC_WORD_PUBLIC 0x34   // 0x34 Public Network

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

static bool    is_started = false;
static uint8_t rx_status  = RX_STATUS_UNKNOWN;
static uint8_t tx_status  = TX_STATUS_UNKNOWN;

static struct lgw_conf_rxrf_s rxrf_conf = { .freq_hz = 868100000, .rssi_offset = 0.0, .tx_enable = true, .power = 20 };

static struct lgw_conf_rxif_s rxif_conf = {
    .bandwidth = BW_125KHZ, .coderate = CR_LORA_4_5, .datarate = DR_LORA_SF7, .modulation = MOD_LORA
};


/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */
static int uart_get_data(uint8_t* data);
static int uart_send_data(const char* data);
static void bin_to_hex (char* in, char* out, int len);
static int lgw_radio_set_cfg( uint8_t radio, uint32_t freq_hz, uint32_t datarate, uint8_t bandwidth, uint8_t power );

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

int lgw_connect( void )
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    /* GPIO configuration for radio */
    /*
    gpio_reset_pin( radio_context.gpio_busy );
    gpio_set_direction( radio_context.gpio_busy, GPIO_MODE_INPUT );

    gpio_reset_pin( radio_context.gpio_rst );
    gpio_set_direction( radio_context.gpio_rst, GPIO_MODE_OUTPUT );

    gpio_reset_pin( radio_context.gpio_dio1 );
    gpio_set_direction( radio_context.gpio_dio1, GPIO_MODE_INPUT );
    gpio_set_intr_type( radio_context.gpio_dio1, GPIO_INTR_POSEDGE );
    */

    return LGW_HAL_SUCCESS;
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_rxrf_setconf( struct lgw_conf_rxrf_s* conf )
{
    CHECK_NULL( conf );

    /* check if the concentrator is running */
    if( is_started == true )
    {
        ESP_LOGI( TAG_HAL, "ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE CHANGING CONFIGURATION\n" );
        return LGW_HAL_ERROR;
    }

    memcpy( &rxrf_conf, conf, sizeof( struct lgw_conf_rxrf_s ) );

    return LGW_HAL_SUCCESS;
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxif_setconf( struct lgw_conf_rxif_s* conf )
{
    CHECK_NULL( conf );

    /* check if the concentrator is running */
    if( is_started == true )
    {
        ESP_LOGI( TAG_HAL, "ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE CHANGING CONFIGURATION\n" );
        return LGW_HAL_ERROR;
    }

    memcpy( &rxif_conf, conf, sizeof( struct lgw_conf_rxif_s ) );

    return LGW_HAL_SUCCESS;
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_start( void )
{
    int err;

    if( is_started == true )
    {
        ESP_LOGW( TAG_HAL, "Note: LoRa concentrator already started, restarting it now\n" );
    }

    /* Check configuration */
    if( rxrf_conf.freq_hz == 0 )
    {
        ESP_LOGE( TAG_HAL, "ERROR: radio frequency not configured\n" );
        return LGW_HAL_ERROR;
    }
    if( rxif_conf.modulation == MOD_UNDEFINED )
    {
        ESP_LOGE( TAG_HAL, "ERROR: modulation type not configured\n" );
        return LGW_HAL_ERROR;
    }
    if( rxif_conf.bandwidth == BW_UNDEFINED )
    {
        ESP_LOGE( TAG_HAL, "ERROR: modulation bandwidth not configured\n" );
        return LGW_HAL_ERROR;
    }
    if( rxif_conf.coderate == CR_UNDEFINED )
    {
        ESP_LOGE( TAG_HAL, "ERROR: modulation coderate not configured\n" );
        return LGW_HAL_ERROR;
    }
    if( rxif_conf.datarate == DR_UNDEFINED )
    {
        ESP_LOGE( TAG_HAL, "ERROR: modulation datarate not configured\n" );
        return LGW_HAL_ERROR;
    }

    /* Configure SPI and GPIOs */
    err = lgw_connect( );
    if( err == LGW_HAL_ERROR )
    {
        ESP_LOGE( TAG_HAL, "ERROR: FAILED TO CONNECT BOARD\n" );
        return LGW_HAL_ERROR;
    }

    /* Update RX status */
    rx_status = RX_OFF;

    /* Set RX */
    lgw_radio_set_cfg( LORARX, rxrf_conf.freq_hz, rxif_conf.datarate, rxif_conf.bandwidth, rxrf_conf.power );

    /* Update RX status */
    rx_status = RX_ON;

    /* Update TX status */
    if( rxrf_conf.tx_enable == false )
    {
        tx_status = TX_OFF;
    }
    else
    {
        tx_status = TX_FREE;
    }

    /* set hal state */
    is_started = true;

    return LGW_HAL_SUCCESS;
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_stop( void )
{
    if( is_started == false )
    {
        ESP_LOGI( TAG_HAL, "Note: LoRa concentrator was not started...\n" );
        return LGW_HAL_SUCCESS;
    }

    /* set hal state */
    is_started = false;

    return LGW_HAL_SUCCESS;
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_receive( uint8_t max_pkt, struct lgw_pkt_rx_s* pkt_data )
{
    struct lgw_pkt_rx_s* p = &pkt_data[0];

    //bool                 irq_received;
    int                  nb_packet_received = 0;

    int i, j;
    int uart_data_len = 0;

    uint8_t uart_data[MAX_DATA_LEN] = {'\0'};
    uint8_t* rx_data;

    char tmp_char[16] = {'\0'};

    /* max_pkt is ignored, only 1 packet can be received at a time */

    /* check if the concentrator is running */
    if ( is_started == false ) {
        ESP_LOGE( TAG_HAL, "ERROR: CONCENTRATOR IS NOT RUNNING, START IT BEFORE RECEIVING\n" );
        return LGW_HAL_ERROR;
    }

    memset( p, 0, sizeof( struct lgw_pkt_rx_s ) );

    //nb_packet_received = lgw_radio_get_pkt( &irq_received, &count_us, &rssi, &snr, &status, &size, p->payload );

    uart_data_len = uart_get_data(uart_data);

    if (uart_data_len > 8) { /* +RXD=snr,rssi,len,data */
        if (strncmp((char*)uart_data, "+RXD=", 5))
            return 0;
        else
            rx_data = uart_data + 5;
    } else {
        return 0;
    }

    //ESP_LOGE( TAG_HAL, "(RXDATA=%s)(len=%d)\n", (char*)rx_data, uart_data_len);
    nb_packet_received = 1;

    /* SNR */
    for (i = 0, j = 0; i < (uart_data_len - 5); i++) {
        if (rx_data[i] == ',') {
            i++;
            break;
        }
        if (rx_data[i] == ' ') {
            continue;
        }
        tmp_char[j++] = rx_data[i];
    }
    tmp_char[j] = '\0';
    p->snr = (float)strtol(tmp_char, NULL, 10);

    //ESP_LOGI( TAG_HAL, "i=%d, SNR=%d\n", i, *snr );

    /* RSSI */
    for (j = 0; i < (uart_data_len - 5); i++) {
        if (rx_data[i] == ',') {
            i++;
            break;
        }
        if (rx_data[i] == ' ') {
            continue;
        }
        tmp_char[j++] = rx_data[i];
    }

    tmp_char[j] = '\0';
    p->rssic = (float)strtol(tmp_char, NULL, 10);

    //ESP_LOGI( TAG_HAL, "i=%d, RSSI=%d\n", i, *rssi );

    /* SIZE */
    for (j = 0; i < (uart_data_len - 5); i++) {
        if (rx_data[i] == ',') {
            i++;
            break;
        }
        if (rx_data[i] == ' ') {
            continue;
        }
        tmp_char[j++] = rx_data[i];
    }

    tmp_char[j] = '\0';
    p->size = strtoul(tmp_char, NULL, 10);
    //ESP_LOGI( TAG_HAL, "char=%s, SIZE=%u\n", tmp_char, *size );

    for (j = 0; i < (uart_data_len - 5); i++) {
        if (rx_data[i] == '\r' || rx_data[i] == '\n') {
            break;
        }
        if (rx_data[i] == ' ') {
            continue;
        }
        p->payload[j++] = rx_data[i];
    }

    p->payload[j] = '\0';

    /* Initialize return values */
    lgw_get_instcnt(&(p->count_us));
    p->freq_hz    = rxrf_conf.freq_hz;
    p->status     = STAT_CRC_OK;
    p->if_chain   = 0;
    p->rf_chain   = 0;
    p->modulation = rxif_conf.modulation;
    p->datarate   = rxif_conf.datarate;
    p->bandwidth  = rxif_conf.bandwidth;
    p->coderate   = rxif_conf.coderate;

    /* Compensate timestamp with for radio processing delay */
    // uint32_t count_us_correction = lgw_radio_timestamp_correction( rxif_conf.datarate, rxif_conf.bandwidth );
    //ESP_LOGI( TAG_HAL, "count_us correction: %lu us", count_us_correction );
    //p->count_us -= count_us_correction;
    ESP_LOGI( TAG_HAL, "count_us correction: %lu us", p->count_us );

    /* Reconfigure RX */
    /*
    if ( irq_received == true ) {
        lgw_radio_set_cfg( LORARX, rxrf_conf.freq_hz, rxif_conf.datarate, rxif_conf.bandwidth, rxrf_conf.power );
    }
    */

    return nb_packet_received;
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_send( struct lgw_pkt_tx_s* pkt_data )
{
    char ral_data[MAX_DATA_LEN] = {'\0'};

    /* check if the concentrator is running */
    if( is_started == false )
    {
        ESP_LOGE( TAG_HAL, "ERROR: CONCENTRATOR IS NOT RUNNING, START IT BEFORE SENDING\n" );
        return LGW_HAL_ERROR;
    }

    /* Update RX status */
    rx_status = RX_SUSPENDED;

    /* Configure for TX */
    lgw_radio_set_cfg( LORATX, pkt_data->datarate, pkt_data->coderate, pkt_data->bandwidth, pkt_data->rf_power );

    /* Update TX status */
    tx_status = TX_SCHEDULED;

    /* Get TCXO startup time, if any */
    uint32_t tcxo_startup_time_in_tick = 0;
    uint32_t tcxo_startup_time_us = tcxo_startup_time_in_tick * 15625 / 1000;

    /* Wait for time to send packet */
    uint32_t count_us_now;
    do
    {
        lgw_get_instcnt( &count_us_now );
        WAIT_US( 100 );
    } while( ( int32_t )( pkt_data->count_us - count_us_now ) > ( int32_t ) tcxo_startup_time_us );

    /* Update TX status */
    tx_status = TX_EMITTING;

    char payload[MAX_DATA_LEN - 12] = {'\0'};
    bin_to_hex((char*)pkt_data->payload, payload, pkt_data->size);
    snprintf(ral_data, sizeof(ral_data), "AT+TX=%d,%s\r\n", strlen(payload), payload);

    uart_send_data(ral_data);

    /* Update TX status */
    tx_status = TX_FREE;

    /* Back to RX config */
    lgw_radio_set_cfg( LORARX, rxrf_conf.freq_hz, rxif_conf.datarate, rxif_conf.bandwidth, rxrf_conf.power );

    /* Update RX status */
    rx_status = RX_ON;

    return LGW_HAL_SUCCESS ;
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_status( uint8_t rf_chain, uint8_t select, uint8_t* code )
{
    // ESP_LOGI(TAG_HAL, "lgw_status()");

    /* check input variables */
    CHECK_NULL( code );
    if( rf_chain >= LGW_RF_CHAIN_NB )
    {
        ESP_LOGE( TAG_HAL, "ERROR: NOT A VALID RF_CHAIN NUMBER\n" );
        return LGW_HAL_ERROR;
    }

    /* Get status */
    if( select == TX_STATUS )
    {
        if( is_started == false )
        {
            *code = TX_OFF;
        }
        else
        {
            *code = tx_status;
        }
    }
    else if( select == RX_STATUS )
    {
        if( is_started == false )
        {
            *code = RX_OFF;
        }
        else
        {
            *code = rx_status;
        }
    }
    else
    {
        ESP_LOGE( TAG_HAL, "ERROR: SELECTION INVALID, NO STATUS TO RETURN\n" );
        return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_instcnt( uint32_t* inst_cnt_us )
{
    int64_t count_us_64 = esp_timer_get_time( );

    *inst_cnt_us = ( uint32_t ) count_us_64;

    return LGW_HAL_SUCCESS;
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint32_t lgw_time_on_air( const struct lgw_pkt_tx_s* packet )
{
    uint32_t toa_ms, toa_us;

    if( packet == NULL )
    {
        ESP_LOGE( TAG_HAL, "ERROR: Failed to compute time on air, wrong parameter\n" );
        return 0;
    }

    if( packet->modulation == MOD_LORA )
    {
        toa_us = lora_packet_time_on_air( packet->bandwidth, packet->datarate, packet->coderate, packet->preamble,
                                          packet->no_header, packet->no_crc, packet->size, NULL, NULL, NULL );
        toa_ms = ( uint32_t )( ( double ) toa_us / 1000.0 + 0.5 );
    }
    else
    {
        toa_ms = 0;
        ESP_LOGE( TAG_HAL, "ERROR: Cannot compute time on air for this packet, unsupported modulation (0x%02X)\n",
                  packet->modulation );
    }

    return toa_ms;
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void lgw_get_min_max_freq_hz( uint32_t* min_freq_hz, uint32_t* max_freq_hz )
{
    *min_freq_hz = 860000000;
    *max_freq_hz = 960000000;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void lgw_get_min_max_power_dbm( int8_t* min_power_dbm, int8_t* max_power_dbm )
{
    *min_power_dbm = 10;

    *max_power_dbm = 26;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int lgw_radio_set_cfg( uint8_t radio, uint32_t freq_hz, uint32_t datarate, uint8_t bandwidth, uint8_t power )
{
    char atcmd[ATLEN] = {'\0'};
    uint8_t res[ATLEN] = {'\0'};
    uint8_t ral_bw = 0;
    switch (bandwidth) {
        case BW_125KHZ:
            ral_bw = 0;
            break;
        case BW_250KHZ:
            ral_bw = 1;
            break;
        case BW_500KHZ:
            ral_bw = 0;
            break;
        default:
            ral_bw = 0;
            break;
    }
    switch (radio) {
        case LORARX:
            snprintf(atcmd, sizeof(atcmd), "AT+RXCFG=%lu,%lu,%u,%u\r\n", freq_hz, datarate, ral_bw, power);
            break;
        case LORATX:
            snprintf(atcmd, sizeof(atcmd), "AT+TXCFG=%lu,%lu,%u,%u\r\n", freq_hz, datarate, ral_bw, power);
            break;
        default:
            return LGW_HAL_ERROR;
    }

    uart_send_data(atcmd);
    uart_get_data(res);
    if (strncmp((char*)res, "+RES=CFG:OK", 13)) {
        return LGW_HAL_SUCCESS;
    } else {
        return LGW_HAL_ERROR;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int uart_send_data(const char* data)
{
    const int len = strlen(data);
    const int tx_bytes = uart_write_bytes(UART_NUM_1, data, len);
    return tx_bytes;
}

static int uart_get_data(uint8_t* data)
{
    int rx_bytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    data[rx_bytes] = 0;
    return rx_bytes;
}

static void bin_to_hex (char* in, char* out, int len) {
    char* ptr = out;
    while (len--) {
        sprintf(ptr, "%02x", *in++);
        ptr += 2;
    }
}

/* --- EOF ------------------------------------------------------------------ */
