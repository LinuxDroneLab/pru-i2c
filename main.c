#include <stdint.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <pru_rpmsg.h>
#include <pru_i2c_driver.h>
#include <pru_hmc5883l_driver.h>
#include <MPU6050.h>
#include <pru_mpu6050_driver.h>
#include "resource_table.h"

volatile register unsigned __R31;

#define VIRTIO_CONFIG_S_DRIVER_OK       4
#define PAYLOAD_CONTENT_OFFSET          2

unsigned char payload[RPMSG_BUF_SIZE];

uint8_t active = 0;

void initBuffers()
{
    {
        int i = 0;
        for (i = 0; i < RPMSG_BUF_SIZE; i++)
        {
            payload[i] = '\0';
        }
    }
}

struct pru_rpmsg_transport transport;
unsigned short src, dst, len;

uint8_t sendData(uint8_t deviceNumber, uint8_t dataBytes, unsigned char* data) {
    payload[0] = 'H';
    payload[1] = 'D';
    pru_rpmsg_send(&transport, dst, src, data - PAYLOAD_CONTENT_OFFSET, dataBytes + PAYLOAD_CONTENT_OFFSET);
    return 1; // FIXME: da adeguare alla risposta di pru_rpmsg_send
}


uint8_t sendTestData(uint8_t deviceNumber, uint8_t dataBytes, unsigned char* data) {
    payload[0] = 'H';
    payload[1] = 'S';
    pru_rpmsg_send(&transport, dst, src, data - PAYLOAD_CONTENT_OFFSET, dataBytes + PAYLOAD_CONTENT_OFFSET);
    return 1;
}

HMC5883LConf hmc5883lConf2 = {
       payload + PAYLOAD_CONTENT_OFFSET,
       payload + PAYLOAD_CONTENT_OFFSET,
       sendData,
       sendTestData
};

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} MPU6450Motion6;

uint8_t pulseHMC5883L() {
        if (pru_hmc5883l_driver_Detect(2))
        {
            if (pru_hmc5883l_driver_Enable(2))
            {
                if(!pru_hmc5883l_driver_Pulse(2)) {
                    payload[0] = 'H';
                    payload[1] = 'F';
                    payload[1] = 'D';
                    pru_rpmsg_send(&transport, dst, src, payload, 3);
                }
            }
            else
            {
                pru_rpmsg_send(&transport, dst, src, "HTK", 4);
            }
        }
        else
        {
            payload[0] = 'T';
            payload[1] = 'F';
            pru_rpmsg_send(&transport, dst, src, payload,
            PAYLOAD_CONTENT_OFFSET);
        }
    return 1;

}

uint8_t executeTasks() {
    return pulseHMC5883L();
}

uint8_t checkMessages() {
    return 1;
}

/**
 * main.c
 */
int main(void)
{
    volatile unsigned char *status;

    /* Allow OCP master port access by the PRU so the PRU can read external memories */
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    /* Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us */
    CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

    /* Make sure the Linux drivers are ready for RPMsg communication */
    /* this is another place where a hang could occur */
    status = &resourceTable.rpmsg_vdev.status;
    while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK))
        ;

    /* Initialize the RPMsg transport structure */
    /* this function is defined in rpmsg_pru.c.  It's sole purpose is to call pru_virtqueue_init twice (once for
     vring0 and once for vring1).  pru_virtqueue_init is defined in pru_virtqueue.c.  It's sole purpose is to
     call vring_init.  Not sure yet where that's defined, but it appears to be part of the pru_rpmsg iface.*/
    /* should probably test for RPMSG_SUCCESS.  If not, then the interface is not working and code should halt */
    pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0,
                   &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

    /* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
    // In a real-time environment, rather than waiting forever, this can probably be run loop-after-loop
    // until success is achieved.  At that point, set a flag and then enable the send/receive functionality
    while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_DESC,
    CHAN_PORT) != PRU_RPMSG_SUCCESS)
        ;
    uint32_t counter = 0;
    MPU6450Motion6* motion6 = (MPU6450Motion6*)(payload + PAYLOAD_CONTENT_OFFSET);
    while (1)
    {
        if (active)
        {
            payload[0] = 'M';
            payload[1] = '6';
//            getMotion6(&motion6->ax, &motion6->ay, &motion6->az, &motion6->gx, &motion6->gy, &motion6->gz);
            motion6->ax = pru_mpu6050_driver_GetClockSource();
            motion6->ay = pru_mpu6050_driver_GetAccelerationX();
            motion6->az = 2;
            motion6->gx = 3;
            motion6->gy = 4;
            motion6->gz = 5;
            pru_rpmsg_send(&transport, dst, src, payload, sizeof(MPU6450Motion6)+ PAYLOAD_CONTENT_OFFSET);
            for(counter = 0; counter < 100000; counter++) {

            }
               // executeTasks();

        }

        if (__R31 & HOST_INT)
        {
            if (pru_rpmsg_receive(&transport, &src, &dst, payload,
                                  &len) == PRU_RPMSG_SUCCESS)
            {
                if (payload[0] == 'S' && payload[1] == 'T')
                {
                    active = 1;
                    pru_rpmsg_send(&transport, dst, src, "ST", 3);
                    pru_hmc5883l_driver_Conf(2, &hmc5883lConf2);
                    // pru_mpu6050_driver_Initialize();
                    if(pru_mpu6050_driver_TestConnection()) {
                        pru_rpmsg_send(&transport, dst, src, "MP", 3);
                        pru_mpu6050_driver_Initialize();
//                        setClockSource(MPU6050_CLOCK_PLL_XGYRO);
//                        pru_mpu6050_driver_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
//                        setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
//                        setSleepEnabled(0); // thanks to Jack Elston for pointing this one out!
                        pru_rpmsg_send(&transport, dst, src, "MP", 3);
                    } else {
                        pru_rpmsg_send(&transport, dst, src, "MK", 3);
                    }
                }
            }
            else
            {
                CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
            }
        }
    }
}

/* SELF TESTS
//                        if (pru_hmc5883l_driver_SelfTestsHMC5883L(payload+PAYLOAD_CONTENT_OFFSET))
//                        {
//                        }
//                        else
//                        {
//                            payload[0] = 'H';
//                            payload[1] = 'F';
//                            payload[1] = 'S';
//                            pru_rpmsg_send(&transport, dst, src, payload, 3);
//                        }
 *
 */

/*
 * Come dovrebbe essere:
 * DeviceList contiene una lista di deviceId con associato il canale di connessione (i2c1/i2c2)
 *   Il canale di connessione serve per poter identificare di quale device si tratta
 *   ad esempio: imu telecamera, imu drone.
 *   per ora faccio due soli canali (i2c1/i2c2)
 *
 * uint8_t detectDevices(DeviceList* deviceList) {
 *   detect all devices connected and supported
 *     detectHMC5833L
 *     detectMPU6050
 *     detectBMP085
 *     i detect sopra devono poter essere invocati anche singolarmente
 * }
 *
 * Nella device list si riporta l'id del device ed un puntatore di struttura del contesto di configurazione ed esecuzione fornito dal driver.
 * Il main modifica il contesto di esecuzione per fornire:
 * - buffer in cui inserire i dati letti (per i sensori non è necessario prevedere la scrittura)
 * - canale di trasmissione dati (rpmsg1/2)
 * - callback da invocare quando i dati sono stati letti (puntatore di funzione da invocare per eseguire la send_rpmsg)
 * - invece della callback è possibile usare un flag 'dati pronti' e lasciare la gestione al main (in polling)
 * - ogni quanto tempo eseguire la calibrazione mediante selftests
 * - buffer dati e callback (o flag 'dati pronti') per fornire i risultati dei selftests
 *
 * il main potrebbe avere ua struttura di questo tipo:
 * detectDevices(devlist);
 * configureDevices(devlist);
 * while(1) {
 *   device->hmc5833l[0]->pulse(); <-- nota: i sensori sono programmati per fornire dati con una data frequenza. Nient'altro
 *   device->mpu6050[0]->pulse();
 *   device->mpu6050[1]->pulse();
 *   device->bmp085[0]->pulse();
 *   device->tempSensor[0]->pulse();
 *   Ogni pulse deve durare il minor tempo possibile.
 *   Evitare cicli di wait ed usare piùttosto stati di avanzamento (vedi selftests. Richiedono 250 millis per l'esecuzione)
 * }
 *
 * uint8_t sendHMC5833LData(hmc5833l* device) {
 *   ... send data to ARM with rmpsg
 * }
 * uint8_t sendHMC5833LDataError(hmc5833l* device) {
 *   ... manage error ...
 * }
 */
