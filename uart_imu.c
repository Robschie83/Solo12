#include "uart_imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <math.h>
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
#define GYR_TIME 17
#define ACCX_POS 5
#define ACCY_POS 9
#define ACCZ_POS 13
#define ACC_TIME 17

//position of EF data (Quaternion)
#define QUATA_POS 5
#define QUATB_POS 7
#define QUATC_POS 9
#define QUATD_POS 11
#define QUAT_TIME 13
#define INT_FROM_BYTE_ARRAY(buff, n) ((buff[n] << 8) | (buff[n + 1]));
#define INT_FROM_DOUBLE_BYTE_ARRAY(buff, n) ((buff[n] << 24) | (buff[n + 1] << 16) | (buff[n + 2] << 8) | (buff[n + 3]));

/* Qvalues for each fields */
#define IMU_QN_ACC 11
#define IMU_QN_GYR 11
#define IMU_QN_EUL 13

union float_int
{
    float f;
    uint32_t ul;
    int16_t si;
};

struct strcut_imu_data
{
    union float_int gyr_x;
    union float_int gyr_y;
    union float_int gyr_z;
    union float_int gyr_time;
    union float_int acc_x;
    union float_int acc_y;
    union float_int acc_z;
    union float_int acc_time;
    union float_int linacc_x;
    union float_int linacc_y;
    union float_int linacc_z;
    union float_int euler_roll;
    union float_int euler_pitch;
    union float_int euler_yaw;
    union float_int quat_a;
    union float_int quat_b;
    union float_int quat_c;
    union float_int quat_d;
    union float_int quat_time;
};

struct strcut_imu_data imu = {0};

static intr_handle_t handle_console;

// const float RAD_TO_DEG = 180.0f / M_PI; 

uint32_t test = 0;

// Receive buffer to collect incoming data
uint8_t rxbuf[128] = {0};     			//default buffer
uint8_t rxbuf_procGyro[128] = {0};     	//procGyro buffer
uint8_t rxbuf_procAcc[128] = {0};     	//procAcc buffer
uint8_t rxbuf_Quat[128] = {0};     		//Quat buffer

/* Define mailbox for thread safe exchange between interupt and main loop*/
QueueHandle_t procGyro_mailbox;
QueueHandle_t procAcc_mailbox;
QueueHandle_t Quat_mailbox;

int intr_cpt = 0;
uint8_t read_index_procGyro = 0; //where to read the latest updated data
uint8_t read_index_procAcc = 0; //where to read the latest updated data
uint8_t read_index_Quat = 0; //where to read the latest updated data

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
    uint32_t summe = 0;
    uint8_t checksum_byte1 = 0;
    uint16_t checksum_byte2 = 0;
    for (int i = 0; i < (len - 2); i++)
    {
        summe += data[i];
    }
    checksum_byte2 = summe & 0x00FF;									// bitweises UND um das untere Byte zu extrahieren, oberes Byte wird genullt
    checksum_byte1 = (summe & 0xFF00) >> 8;								// bitweises UND mit dem oberen Byte schmeisst das untere weg   
    return (data[len - 2] == checksum_byte1 && data[len - 1] == checksum_byte2);	// die letzten beiden Bytes im Paket enthalten die Summe über alle Daten ab 's' bis vor den Checksum Bytes sieh UM7 Datenblatt
}

inline int parse_IMU_data()
{
    xQueuePeek(procGyro_mailbox, &rxbuf_procGyro, 0);
    xQueuePeek(procAcc_mailbox, &rxbuf_procAcc, 0);
    xQueuePeek(Quat_mailbox, &rxbuf_Quat, 0);

    imu.gyr_time.ul = INT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procGyro, GYR_TIME);			// Zeiten aus den PAketen holen
    imu.acc_time.ul = INT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procAcc, ACC_TIME);
    imu.quat_time.ul = INT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_Quat, QUAT_TIME); 

	if (imu.gyr_time.ul == imu.acc_time.ul && imu.acc_time.ul == imu.quat_time.ul)	// sind alle Zeiten identisch?
	{
		/***rawGyro****/
		if (check_IMU_CRC(rxbuf_procGyro, 23)) 										// Summencheck siehe DAtenblatt
		{
			imu.gyr_x.ul = INT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procGyro, GYRX_POS);		// [°/sec]
			imu.gyr_x.f = imu.gyr_x.f * M_PI * 0.00555555556f;							// [rad/sec]
			imu.gyr_y.ul = INT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procGyro, GYRY_POS);
			imu.gyr_y.f = imu.gyr_y.f * M_PI * 0.00555555556f;
			imu.gyr_z.ul = INT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procGyro, GYRZ_POS);
			imu.gyr_z.f = imu.gyr_z.f * M_PI * 0.00555555556f;
		}
		/***rawAcc****/
		if (check_IMU_CRC(rxbuf_procAcc, 23))
		{
			imu.acc_x.ul = INT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procAcc, ACCX_POS);		// z.B. (HEADER:) 73 6e 70 cc (Adresse:) 65 (AccelX IEEE:) XX XX XX XX (AccelY IEEE:) XX XX XX XX (AccelZ IEEE:) XX XX XX XX (Accel Time:) XX XX XX XX (Checksum:) 07 c0 [m/s²]
			imu.acc_y.ul = INT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procAcc, ACCY_POS);
			imu.acc_z.ul = INT_FROM_DOUBLE_BYTE_ARRAY(rxbuf_procAcc, ACCZ_POS);
		}
		/***procAcc****/
		if (check_IMU_CRC(rxbuf_Quat, 19))
		{
			imu.quat_a.si = INT_FROM_BYTE_ARRAY(rxbuf_Quat, QUATA_POS);
			imu.quat_a.f = (float)imu.quat_a.si * 0.00003356933f;					// entspricht / 29789.09091 ausm DAtenblatt für die Quaternionumrechnung aus dem integer   
			float qw = imu.quat_a.f;
			imu.quat_b.si = INT_FROM_BYTE_ARRAY(rxbuf_Quat, QUATB_POS);
			imu.quat_b.f = (float)imu.quat_b.si * 0.00003356933f;
			float qx = imu.quat_b.f;
			imu.quat_c.si = INT_FROM_BYTE_ARRAY(rxbuf_Quat, QUATC_POS);
			imu.quat_c.f = (float)imu.quat_c.si * 0.00003356933f;
			float qy = imu.quat_c.f;
			imu.quat_d.si = INT_FROM_BYTE_ARRAY(rxbuf_Quat, QUATD_POS);
			imu.quat_d.f = (float)imu.quat_d.si * 0.00003356933f;
			float qz = imu.quat_d.f;
			
			// 1. Roll (X-Achse)
			float sinr_cosp = 2.0f * (qw * qx + qy * qz);
			float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
			imu.euler_roll.f = atan2f(sinr_cosp, cosr_cosp);						// * RAD_TO_DEG;

			// 2. Pitch (Y-Achse) mit Schutz gegen mathematischen Überlauf (Gimbal Lock)
			float sinp = 2.0f * (qw * qy - qz * qx);
			if (fabsf(sinp) >= 1.0f) {
				imu.euler_pitch.f = copysignf(M_PI * 0.5f, sinp);					// * RAD_TO_DEG; // Nutze 90 Grad, nein in rad!
			} else {
				imu.euler_pitch.f = asinf(sinp);									// * RAD_TO_DEG;
			}
			// 3. Yaw (Z-Achse / Himmelsrichtung)
			float siny_cosp = 2.0f * (qw * qz + qx * qy);
			float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
			imu.euler_yaw.f = atan2f(siny_cosp, cosy_cosp);							// * RAD_TO_DEG;
			
			// 4. Schwerkraftanteile der Quaternionen - LinAcc - Berechnung
			float gx = 2.0f * (qx * qz - qw * qy);
			float gy = 2.0f * (qw * qx + qy * qz);
			float gz = (qw * qw) - (qx * qx) - (qy * qy) + (qz * qz);
			imu.linacc_x.f = imu.acc_x.f + gx;
			imu.linacc_y.f = imu.acc_y.f + gy;
			imu.linacc_z.f = imu.acc_z.f + gz;
		};
	}
    return 0;
}

uint16_t get_gyr_x_in_D16QN() { return FLOAT_TO_D16QN(imu.gyr_x.f, IMU_QN_GYR); }
uint16_t get_gyr_y_in_D16QN() { return FLOAT_TO_D16QN(imu.gyr_y.f, IMU_QN_GYR); }
uint16_t get_gyr_z_in_D16QN() { return FLOAT_TO_D16QN(imu.gyr_z.f, IMU_QN_GYR); }

uint16_t get_acc_x_in_D16QN() { return FLOAT_TO_D16QN(imu.acc_x.f, IMU_QN_ACC); }
uint16_t get_acc_y_in_D16QN() { return FLOAT_TO_D16QN(imu.acc_y.f, IMU_QN_ACC); }
uint16_t get_acc_z_in_D16QN() { return FLOAT_TO_D16QN(imu.acc_z.f, IMU_QN_ACC); }

uint16_t get_linacc_x_in_D16QN() { return FLOAT_TO_D16QN(imu.linacc_x.f, IMU_QN_ACC); }
uint16_t get_linacc_y_in_D16QN() { return FLOAT_TO_D16QN(imu.linacc_y.f, IMU_QN_ACC); }
uint16_t get_linacc_z_in_D16QN() { return FLOAT_TO_D16QN(imu.linacc_z.f, IMU_QN_ACC); }

uint16_t get_roll_in_D16QN() { return FLOAT_TO_D16QN(imu.euler_roll.f, IMU_QN_EUL); }
uint16_t get_pitch_in_D16QN() { return FLOAT_TO_D16QN(imu.euler_pitch.f, IMU_QN_EUL); }
uint16_t get_yaw_in_D16QN() { return FLOAT_TO_D16QN(imu.euler_yaw.f, IMU_QN_EUL); }

void print_imu()
{
    printf("\n%.4f %.4f %.4f %.4f \n%.4f %.4f %.4f %.4f \n%.4f %.4f %.4f \n%.4f %.4f %.4f %.4f",
			imu.acc_x.f,
			imu.acc_y.f,
			imu.acc_z.f,
			imu.acc_time.f,
			imu.gyr_x.f,
			imu.gyr_y.f,
			imu.gyr_z.f,
			imu.gyr_time.f,
			imu.linacc_x.f,
			imu.linacc_y.f,
			imu.linacc_z.f,
			imu.euler_roll.f,
			imu.euler_pitch.f,
			imu.euler_yaw.f,
			imu.quat_time.f);
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
    
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};							

	uart_param_config(UART_NUM, &uart_config);
    uart_set_rx_timeout(UART_NUM, 3); 									// timeout in symbols, this will generate an interrupt per RX data frame
    uart_set_pin(UART_NUM, PIN_TXD, PIN_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    const char cmd0[11] = {0x73, 0x6E, 0x70, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0xD2};                               // COMRates1 PACKET: 's', 'n', 'p', 10000000, Adresse 01, Daten: 00 00 00 00 Raw Accel 0Hz, Raw Gyro 0Hz, Raw Mag 0Hz, Checksum 1 & 0: 01 D2
    const char cmd1[11] = {0x73, 0x6E, 0x70, 0x80, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0xD3};                               // COMRates2 PACKET: 's', 'n', 'p', 10000000, Adresse 02, Daten: 00 00 00 00
    const char cmd2[11] = {0x73, 0x6E, 0x70, 0x80, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0xD2};	    	 		            // COMRates3 PACKET: 's', 'n', 'p', 10000000, Adresse 03, Daten: 03 03 00 00 Proc Accel 3Hz, Proc Gyro 3Hz, Proc Mag 0Hz, Checksum 1 & 0: 01 DA für DEBUG
    const char cmd3[11] = {0x73, 0x6E, 0x70, 0x80, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01, 0xD5};                               // COMRates4 PACKET: 's', 'n', 'p', 10000000, Adresse 04, Daten: 00 00 00 00
    const char cmd4[11] = {0x73, 0x6E, 0x70, 0x80, 0x05, 0xFF, 0x00, 0x00, 0x00, 0x02, 0xD5};                               // COMRates5 PACKET: 's', 'n', 'p', 10000000, Adresse 05, Daten: 03 00 00 00 Quat 3Hz, Euler 0Hz, Pos 0Hz, Vel 0Hz, Checksum 1 & 0: 01 D9 für DEBUG
    const char cmd5[11] = {0x73, 0x6E, 0x70, 0x80, 0x06, 0x00, 0x00, 0x00, 0x00, 0x01, 0xD7};                               // COMRates6 PACKET: 's', 'n', 'p', 10000000, Adresse 06, Daten: 00 00 00 00
    const char cmd6[11] = {0x73, 0x6E, 0x70, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x01, 0xD8};                               // COMRates7 PACKET: 's', 'n', 'p', 10000000, Adresse 07, Daten: 00 00 00 00
    const char cmd7[11] = {0x73, 0x6E, 0x70, 0x80, 0x08, 0x00, 0x00, 0x00, 0x03, 0x01, 0xDC};                               // COMMisc PACKET: 's', 'n', 'p', 10000000, Adresse 08, Daten: 00 00 00 07 Bit2: Gyro beim Starten nullen, Bit1: Quaternionen nutzen, statt Euler, Bit0: Magnetometer für den Zustand nutzen
    
    // Flash Kommando:
    // const char cmd8[7] = {0x73, 0x6E, 0x70, 0x00, 0xAB, 0x01, 0xFC};                               						// Command Flash PACKET: 's', 'n', 'p', 00000000, Adresse AB, Daten: 0x00, 0xAB, 0x01, 0xFC
	// Zero Gyro Kommando:
    const char cmd9[7] = {0x73, 0x6E, 0x70, 0x00, 0xAD, 0x01, 0xFE};                               							// Command Flash PACKET: 's', 'n', 'p', 00000000, Adresse AD, Daten: 0x00, 0xAD, 0x01, 0xFE
	const char cmd10[11] = {0x73, 0x6E, 0x70, 0x80, 0x00, 0xB0, 0x00, 0x00, 0x00, 0x02, 0x81};								// Setting Register 0x00 Baudrate: 921600

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
    custom_write_uart(cmd9, sizeof(cmd9));
    vTaskDelay(3);
    custom_write_uart(cmd10, sizeof(cmd10));
    vTaskDelay(3);
    
    uart_set_baudrate(UART_NUM, 921600); 								// 921600 zum DEBUG: 115200
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_disable_tx_intr(UART_NUM);
    uart_disable_rx_intr(UART_NUM);
    uart_isr_free(UART_NUM);
    uart_isr_register(UART_NUM, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console);
    uart_enable_rx_intr(UART_NUM);
    
    while (0) //for debug
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
        print_imu();
        vTaskDelay(300/portTICK_PERIOD_MS);
    }
    return 0;
}
