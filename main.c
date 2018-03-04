#include <stdint.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <pru_rpmsg.h>
#include <pru_i2c_driver.h>
#include <pru_hmc5883l_driver.h>
#include "resource_table.h"
#include "MPU6050.h"

volatile register unsigned __R31;

#define VIRTIO_CONFIG_S_DRIVER_OK       4
#define CLKACTIVITY_I2C_FCLK            24
#define CLKACTIVITY_L4LS_GCLK           8
#define PAYLOAD_CONTENT_OFFSET          2

unsigned char payload[RPMSG_BUF_SIZE];

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
/**
 * main.c
 */
int main(void)
{
    struct pru_rpmsg_transport transport;
    unsigned short src, dst, len;
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
    uint32_t cycles = 2000000;

    uint8_t active = 0;


    while (1)
    {
        if (active)
        {
            counter %= cycles;
            counter++;
            if (counter == cycles)
            {
                if (pru_hmc5883l_driver_TestConnection())
                {
                    payload[0] = 'T';
                    payload[1] = 'T';
                    pru_rpmsg_send(&transport, dst, src, payload,PAYLOAD_CONTENT_OFFSET);
                    if (pru_hmc5883l_driver_InitHMC5883L())
                    {
                        pru_rpmsg_send(&transport, dst, src, "HTO", 4);
                        if (pru_hmc5883l_driver_ReadHMC5883LData(payload+PAYLOAD_CONTENT_OFFSET))
                        {
                            payload[0] = 'H';
                            payload[1] = 'D';
                            pru_rpmsg_send(&transport, dst, src, payload, 8);
                        }
                        else
                        {
                            payload[0] = 'H';
                            payload[1] = 'F';
                            payload[1] = 'D';
                            pru_rpmsg_send(&transport, dst, src, payload, 3);
                        }
                        if (pru_hmc5883l_driver_SelfTestsHMC5883L(payload+PAYLOAD_CONTENT_OFFSET))
                        {
                            payload[0] = 'H';
                            payload[1] = 'S';
                            pru_rpmsg_send(&transport, dst, src, payload, 8);
                        }
                        else
                        {
                            payload[0] = 'H';
                            payload[1] = 'F';
                            payload[1] = 'S';
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
            }
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
                }
            }
            else
            {
                CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
            }
        }
    }
}

/* Procedura di transmitting I2C che dicono funzionare (a me non sembra ...)
 * FROM: https://e2e.ti.com/support/arm/sitara_arm/f/791/p/520841/1893086?pi316653=1#pi316653=1
 *
 * Finally I got it working. The whole procedure for I2C polling mode is a bit different than described in TRM, so may be this information is useful for somebody:

 Initialisation:

 - set up correct PIN-muxing
 - set up I2C1 module clock
 - disable I2C by clearing EN-bit in register I2C_CON
 - disable auto-idle mode by clearing bit AUTOIDLE in register I2C_SYSC
 - set the I2C clock to 400 kHz
 - set slave-address 0x0F (so left-shift the real address of 0x1E by one!) by writing it into register I2C_SA <---- cosa fa? sembra errato!
 - enable I2C by setting EN-bit in register I2C_CON

 Then transmission is done this way:

 - set the number of bytes (3) to be transferred by writing them into register I2C_CNT
 - initiate for transfer by writing bits STP, TRX and MST into register I2C_CON
 - add bit STT in register I2C_CON
 - now in a loop until all three bytes have been sent:
 - wait until bit XRDY in register I2C_IRQSTATUS_RAW is set
 - write next data byte to be transmitted into register I2C_DATA
 - write a 1 into bit XRDY in register I2C_IRQSTATUS_RAW to clear the XRDY state
 */
