#include "uart_imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#define FLOAT_TO_D16QN(a, n) ((int16_t)((a) * (1 << (n))))

#define UART_NUM UART_NUM_1
#define BUF_SIZE 128
#define PIN_TXD 32
#define PIN_RXD 35

//position of PROC data
#define GYRX_POS 5
#define GYRY_POS 9
#define GYRZ_POS 13
#define ACCX_POS 5
#define ACCY_POS 9
#define ACCZ_POS 13

//position of EF data (Quaternion)
#define QUATA_POS 5
#define QUATB_POS 7
#define QUATC_POS 9
#define QUATD_POS 11
#define FLOAT_FROM_BYTE_ARRAY(buff, n) ((buff[n] << 8) | (buff[n + 1]));
#define FLOAT_FROM_DOUBLE_BYTE_ARRAY(buff, n) ((buff[n] << 24) | (buff[n + 1] << 16) | (buff[n + 2] << 8) | (buff[n + 3]));


/* Qvalues for each fields */
#define IMU_QN_ACC 11
#define IMU_QN_GYR 11
#define IMU_QN_EF 0

union float_int
{
    float f;
    unsigned long ul;
};

struct strcut_imu_data
{
    union float_int gyr_x;
    union float_int gyr_y;
    union float_int gyr_z;
    union float_int acc_x;
    union float_int acc_y;
    union float_int acc_z;
    union float_int quat_a;
    union float_int quat_b;
    union float_int quat_c;
    union float_int quat_d;
//    union float_int roll;
//    union float_int pitch;
//    union float_int yaw;
};
struct strcut_imu_data imu = {0};

static intr_handle_t handle_console;

uint8_t test = 0;

// Receive buffer to collect incoming data
uint8_t rxbuf[128] = {0};     			//default buffer
uint8_t rxbuf_procGyro[128] = {0};     	//procGyro buffer
uint8_t rxbuf_procAcc[128] = {0};     	//procAcc buffer
uint8_t rxbuf_Quat[128] = {0};     		//Quat buffer
//uint8_t rxbuf_procEuler[128] = {0};     //procEuler buffer

//uint8_t rxbuf_imu[128] = {0}; //buffer for  IMU packets
//uint8_t rxbuf_ef[128] = {0};  //buffer for estimation filter packets

/* Define mailbox for thread safe exchange between interupt and main loop*/
QueueHandle_t procGyro_mailbox;
QueueHandle_t procAcc_mailbox;
QueueHandle_t Quat_mailbox;
//QueueHandle_t procEuler_mailbox;

//QueueHandle_t imu_mailbox;
//QueueHandle_t ef_mailbox;

int intr_cpt = 0;
uint8_t read_index_procGyro = 0; //where to read the latest updated data
uint8_t read_index_procAcc = 0; //where to read the latest updated data
uint8_t read_index_Quat = 0; //where to read the latest updated data
//uint8_t read_index_procEuler = 0; //where to read the latest updated data

//uint8_t read_index_imu = 0; //where to read the latest updated imu data
//uint8_t read_index_ef = 0;  //where to read the latest updated ef data

/*
 * Define UART interrupt subroutine to ackowledge interrupt
 */
static void IRAM_ATTR uart_intr_handle(void *arg)
{
    uint16_t rx_fifo_len, status;
    uint16_t i = 0;
    status = UART1.int_st.val;             // read UART interrupt Status
    rx_fifo_len = UART1.status.rxfifo_cnt; // read number of bytes in UART buffer
    intr_cpt++;
    //read all bytes from rx fifo
    for (i = 0; i < rx_fifo_len; i++)
    {
        rxbuf[i] = UART1.fifo.rw_byte; // read all bytes
    }
    // Fix of esp32 hardware bug as in https://github.com/espressif/arduino-esp32/pull/1849
    while (UART1.status.rxfifo_cnt || (UART1.mem_rx_status.wr_addr != UART1.mem_rx_status.rd_addr))
    {
        UART1.fifo.rw_byte;
    }
    i = 0;
    //read frame
    while ((i + 5) < rx_fifo_len) //while at least a full header (5bytes) is in the buffer: 's' 'n' 'p' + Packet type + Address
    {
        int size = 0;
        //Gesamtgröße des DAtenpackets: X (Payloadlänge steht #TODO + 2 (Checksum) + 5 (header structure: [0x73 - 0x6E - 0x70 - PAcket type - Addresse])
        if ((rxbuf[i + 3] & 0b10000000) == 0b10000000)
        {
			if ((rxbuf[i + 3] & 0b01000000) == 0b01000000)
			{
				uint8_t a = rxbuf[i + 3] >> 2;	// Die Batchlänge isolieren (HasData, Is Batch, Batchlänge3, Batchlänge2, Batchlänge1, Batchlänge0, Hidden, CFailed) Hiden nud CFailed werden weggeschoben
				a = a & 0b00001111;				// Die oberen Bits sind dann Has Data und IsBatch, die müssen auch weg
				size = 4 * a + 7;
			}
			else
			{	
				size = 11;	
			}
        }
        else
        {	
			size = 7;
		}
		test = size;
        if (rxbuf[i] != 0x73 || rxbuf[i + 1] != 0x6E || rxbuf[i + 2] != 0x70)
        {
            break; //The data doesn't look like the expected header
        }
        if (size > rx_fifo_len)
        {
            break;
        }
        switch (rxbuf[i + 4]) //Adresse im 5ten PAketbyte checken
        {
        case (0x61): //erstes Register für procGyro -> is BAtch, also hängen die DAten aneinander in einem PAcket => X, Y, Z, 
            xQueueOverwriteFromISR(procGyro_mailbox, &rxbuf[i], NULL);
            break;
        case (0x65): //erstes Register für procAcc
            xQueueOverwriteFromISR(procAcc_mailbox, &rxbuf[i], NULL);
            break;
        case (0x6D): //erstes Register für Quat
            xQueueOverwriteFromISR(Quat_mailbox, &rxbuf[i], NULL);
            break;
//        case (0x70): //erstes Register für ProcEuler
//            xQueueOverwriteFromISR(procEuler_mailbox, &rxbuf[i], NULL);
//            break;
        default:
            break; // We don't deal with this descriptor
        }
        i += size;
    }
    // clear UART interrupt status
    uart_clear_intr_status(UART_NUM, status);
}

inline bool check_IMU_CRC(unsigned char *data, int len)
{
    if (len < 2)
        return false;
    unsigned char checksum_byte1 = 0;
    unsigned char checksum_byte2 = 0;
    for (int i = 0; i < (len - 2); i++)
    {
        checksum_byte1 += data[i];
        checksum_byte2 += checksum_byte1;
    }
    return (data[len - 2] == checksum_byte1 && data[len - 1] == checksum_byte2);
}

inline int parse_IMU_data()
{

    xQueuePeek(procGyro_mailbox, &rxbuf_procGyro, 0);
    xQueuePeek(procAcc_mailbox, &rxbuf_procAcc, 0);
    xQueuePeek(Quat_mailbox, &rxbuf_Quat, 0);
//    xQueuePeek(procEuler_mailbox, &rxbuf_procEuler, 0);

    /***rawAcc****/
    //(check_IMU_CRC(rxbuf_imu, 34))
    if (1)
    {
        imu.gyr_x.ul = FLOAT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procGyro, GYRX_POS);
        imu.gyr_y.ul = FLOAT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procGyro, GYRY_POS);
        imu.gyr_z.ul = FLOAT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procGyro, GYRZ_POS);
    }
    /***rawGyro****/
	if (1)
    {
        imu.acc_x.ul = FLOAT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procAcc, ACCX_POS);	// z.B. (HEADER:) 73 6e 70 cc (Adresse:) 65 (AccelX 2er Komplement:) 00 42 (AccelY 2er Komplement:) ff df (AccelZ 2er Komplement:) ef 74 (Reserviert:) 00 00 (AccelTime:) 45 d7 47 64 (Checksum:) 07 c0
        imu.acc_y.ul = FLOAT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procAcc, ACCY_POS);
        imu.acc_z.ul = FLOAT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procAcc, ACCZ_POS);

    }
    /***procAcc****/
    //(check_IMU_CRC(rxbuf_ef, 38))
    if (1)
    {
		imu.quat_a.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_Quat, QUATA_POS);
        imu.quat_b.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_Quat, QUATB_POS);
        imu.quat_c.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_Quat, QUATC_POS);
        imu.quat_d.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_Quat, QUATD_POS);

    }
//    /***procEuler****/
//    if (1)
//    {
//        imu.roll.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_procEuler, R_POS);
//        imu.pitch.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_procEuler, P_POS);
//        imu.yaw.ul = FLOAT_FROM_BYTE_ARRAY(rxbuf_procEuler, Y_POS);
//    }
    return 0;
}

uint16_t get_gyr_x_in_D16QN() { return FLOAT_TO_D16QN(imu.gyr_x.f, IMU_QN_GYR); }
uint16_t get_gyr_y_in_D16QN() { return FLOAT_TO_D16QN(imu.gyr_y.f, IMU_QN_GYR); }
uint16_t get_gyr_z_in_D16QN() { return FLOAT_TO_D16QN(imu.gyr_z.f, IMU_QN_GYR); }

uint16_t get_acc_x_in_D16QN() { return FLOAT_TO_D16QN(imu.acc_x.f, IMU_QN_ACC); }
uint16_t get_acc_y_in_D16QN() { return FLOAT_TO_D16QN(imu.acc_y.f, IMU_QN_ACC); }
uint16_t get_acc_z_in_D16QN() { return FLOAT_TO_D16QN(imu.acc_z.f, IMU_QN_ACC); }

uint16_t get_linacc_x_in_D16QN() { return FLOAT_TO_D16QN(imu.acc_x.f, IMU_QN_ACC); }
uint16_t get_linacc_y_in_D16QN() { return FLOAT_TO_D16QN(imu.acc_y.f, IMU_QN_ACC); }
uint16_t get_linacc_z_in_D16QN() { return FLOAT_TO_D16QN(imu.acc_z.f, IMU_QN_ACC); }

uint16_t get_roll_in_D16QN() { return 1; }
uint16_t get_pitch_in_D16QN() { return 1; }
uint16_t get_yaw_in_D16QN() { return 1; }

void print_imu()
{
    printf("\n%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f",
           imu.acc_x.f,
           imu.acc_y.f,
           imu.acc_z.f,
           imu.gyr_x.f,
           imu.gyr_y.f,
           imu.gyr_z.f,
           imu.quat_a.f,
           imu.quat_b.f,
           imu.quat_c.f,
           imu.quat_d.f);
//           imu.roll.f,
//           imu.pitch.f,
//           imu.yaw.f,
}

void print_table(uint8_t *ptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        printf("%02x ", ptr[i]);
    }
    printf("\n");
}

void custom_write_uart(unsigned char *buff, int size)
{
    int i = 0;
    for (i = 0; i < size; i++)
    {
        UART1.fifo.rw_byte = buff[i];
    }
}

int imu_init()
{
    /* Init uart */
    printf("Initialising uart for IMUF...\n");

    procGyro_mailbox = xQueueCreate(1, 128);
    procAcc_mailbox = xQueueCreate(1, 128);
    Quat_mailbox = xQueueCreate(1, 128);
//  procEuler_mailbox = xQueueCreate(1, 128);
//	Configure UART 115200 bauds
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    uart_param_config(UART_NUM, &uart_config);
    uart_set_rx_timeout(UART_NUM, 3); //timeout in symbols, this will generate an interrupt per RX data frame
    uart_set_pin(UART_NUM, PIN_TXD, PIN_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    const char cmd0[11] = {0x73, 0x6E, 0x70, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0xD2};                               // COMRates1 PACKET: 's', 'n', 'p', 10000000, Adresse 01, Daten: 00 00 00 00 Raw Accel 0Hz, Raw Gyro 3Hz, Raw Mag 0Hz, Checksum 1 & 0: 01 D2
    const char cmd1[11] = {0x73, 0x6E, 0x70, 0x80, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0xD3};                               // COMRates2 PACKET: 's', 'n', 'p', 10000000, Adresse 02, Daten: 00 00 00 00
    const char cmd2[11] = {0x73, 0x6E, 0x70, 0x80, 0x03, 0x03, 0x03, 0x00, 0x00, 0x01, 0xDA};	    	 		            // COMRates3 PACKET: 's', 'n', 'p', 10000000, Adresse 03, Daten: 03 03 00 00 Proc Accel 3Hz, Proc Gyro 3Hz, Proc Mag 0Hz, Checksum 1 & 0: 01 DA
    const char cmd3[11] = {0x73, 0x6E, 0x70, 0x80, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01, 0xD5};                               // COMRates4 PACKET: 's', 'n', 'p', 10000000, Adresse 04, Daten: 00 00 00 00
    const char cmd4[11] = {0x73, 0x6E, 0x70, 0x80, 0x05, 0x03, 0x00, 0x00, 0x00, 0x01, 0xD9};                               // COMRates5 PACKET: 's', 'n', 'p', 10000000, Adresse 05, Daten: 03 00 00 00 Quat 3Hz, Euler 0Hz, Pos 0Hz, Vel 0Hz, Checksum 1 & 0: 01 D9
    const char cmd5[11] = {0x73, 0x6E, 0x70, 0x80, 0x06, 0x00, 0x00, 0x00, 0x00, 0x01, 0xD7};                               // COMRates6 PACKET: 's', 'n', 'p', 10000000, Adresse 06, Daten: 00 00 00 00
    const char cmd6[11] = {0x73, 0x6E, 0x70, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x01, 0xD8};                               // COMRates7 PACKET: 's', 'n', 'p', 10000000, Adresse 07, Daten: 00 00 00 00
    const char cmd7[11] = {0x73, 0x6E, 0x70, 0x80, 0x08, 0x00, 0x00, 0x00, 0x03, 0x01, 0xDC};                               // COMMisc PACKET: 's', 'n', 'p', 10000000, Adresse 08, Daten: 00 00 00 07 Bit2: Gyro beim Starten nullen, Bit1: Quaternionen nutzen, statt Euler, Bit0: Magnetometer für den Zustand nutzen
    
    // Flash Kommando:
    //const char cmd8[7] = {0x73, 0x6E, 0x70, 0x00, 0xAB, 0x01, 0xFC};                               							// Command Flash PACKET: 's', 'n', 'p', 00000000, Adresse AB, Daten: 0x00, 0xAB, 0x01, 0xFC

    vTaskDelay(100 / portTICK_PERIOD_MS); //Let the IMU some time to boot    (TODO: read uart and wait for IMU acknoledgment on cmd0 to optimize boot time and/or detect the absence of IMU)
    custom_write_uart(cmd0, sizeof(cmd0));
    vTaskDelay(3);
    custom_write_uart(cmd1, sizeof(cmd1));
    vTaskDelay(3);
    custom_write_uart(cmd2, sizeof(cmd2));
    vTaskDelay(3);
    custom_write_uart(cmd3, sizeof(cmd3));
    vTaskDelay(3);
    custom_write_uart(cmd4, sizeof(cmd4));
    vTaskDelay(3);
    custom_write_uart(cmd5, sizeof(cmd5));
    vTaskDelay(3);
    custom_write_uart(cmd6, sizeof(cmd6));
    vTaskDelay(3);
    custom_write_uart(cmd7, sizeof(cmd7));
    vTaskDelay(3);
    //custom_write_uart(cmd8, sizeof(cmd8));
    //vTaskDelay(3);
    
    uart_set_baudrate(UART_NUM, 115200); //statt 921600
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_disable_tx_intr(UART_NUM);
    uart_disable_rx_intr(UART_NUM);
    uart_isr_free(UART_NUM);
    uart_isr_register(UART_NUM, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console);
    uart_enable_rx_intr(UART_NUM);
    
    while (1) //for debug
    {
        parse_IMU_data();
        printf(" intr_cpt:%d\n", intr_cpt);
        printf("rxbuf: %d      ", test);
        print_table(rxbuf, 80);
        printf("rxbuf_procGyro: ");
        print_table(rxbuf_procGyro, 80);
        printf("rxbuf_procAcc: ");
        print_table(rxbuf_procAcc, 80);
        printf("rxbuf_Quaternion: ");
        print_table(rxbuf_Quat, 80);
//        printf("rxbuf_procEuler: ");
//        print_table(rxbuf_procEuler, 80);
        print_imu();
        vTaskDelay(300/portTICK_PERIOD_MS);
    }
    return 0;
}
