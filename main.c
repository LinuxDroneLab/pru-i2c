#include <stdint.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <pru_rpmsg.h>
#include <string.h>
#include "pru_i2c.h"
#include "resource_table.h"
#include "HMC5883L.h"

volatile register unsigned __R31;

#define VIRTIO_CONFIG_S_DRIVER_OK       4
#define CLKACTIVITY_I2C_FCLK            24
#define CLKACTIVITY_L4LS_GCLK           8


struct EcapData
{
    char cmd[8];
    uint32_t reg[8];
};

unsigned char payload[RPMSG_BUF_SIZE];
struct EcapData *result = (struct EcapData *) payload;

uint8_t buffer[6] = {'A','B','C','D','E', 0 };

int readBytes(uint8_t address, uint8_t reg, uint8_t bytes, uint8_t* buffer)
{

    {
        int i = 0;
        for(i = 0; i < 8; i++) {
            result->reg[i] = 0;
        }
    }
    uint32_t ticks = 0;
    uint32_t maxTicks = 20000000;

    // TODO poll low for BB in I2C_IRQSTATUS_RAW
    while(CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_BB) {
        ticks++;
        if(ticks > maxTicks) {
            return 0;
        }
    }

    /*
     * send reg
     * set  I2C_CON: STT / I2C_CON: STP
     * Note: DCOUNT is data count value in I2C_CNT register.
     * STT = 1, STP = 0, Conditions = Start, Bus Activities = S-A-D.
     * STT = 0, STP = 1, Conditions = Stop, Bus Activities = P.
     * STT = 1, STP = 1, Conditions = Start-Stop (DCOUNT=n), Bus Activities = S-A-D..(n)..D-P.
     * STT = 1, STP = 0, Conditions = Start (DCOUNT=n), Bus Activities = S-A-D..(n)..D.
     * 0h = No action or start condition detected
     * 1h = Start condition queried
     *
     * FROM: http://processors.wiki.ti.com/index.php/StarterWare_HSI2C
     * EXAMPLES: http://processors.wiki.ti.com/index.php/StarterWare_02.00.01.01_User_Guide#HSI2C
     *
     * Before Configuring the I2C configuration and DataCount register make sure that I2C registers are ready for access by polling the Access ready bit of IRQ RAW status register
     * Finally the data transfer is started by commanding a START on the bus using I2CMasterStart()
     *
     */

    CT_I2C2.I2C_BUF_bit.I2C_BUF_TXFIFO_CLR = 0b1; // clear TX FIFO
    CT_I2C2.I2C_BUF_bit.I2C_BUF_RXFIFO_CLR = 0b1; // clear RX FIFO

    CT_I2C2.I2C_SA_bit.I2C_SA_SA = address; // 7 bit address
    CT_I2C2.I2C_CNT_bit.I2C_CNT_DCOUNT = 1; // devo inviare reg (1 byte)
//    CT_I2C2.I2C_CON_bit.I2C_CON_MST = 0b1; // master mode
//    CT_I2C2.I2C_CON_bit.I2C_CON_TRX = 0b1; // transmitter mode
//    CT_I2C2.I2C_CON_bit.I2C_CON_STP = 0b0; // Stop condition not required
//    CT_I2C2.I2C_CON_bit.I2C_CON_STT = 0b1; // Start condition <-- qui parte la comunicazione
    CT_I2C2.I2C_CON = 0x8601;

    // debug
    result->reg[0] = CT_I2C2.I2C_CNT;
    result->reg[1] = CT_I2C2.I2C_CON;
    result->reg[2] = CT_I2C2.I2C_IRQSTATUS_RAW;
    result->reg[3] = CT_I2C2.I2C_BUFSTAT;
    result->reg[4] = 0x6F;

    // poll for XRDY = 1 ? clear XRDY?
    ticks = 0;
    while(!CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_XRDY) {
        ticks++;
        if(ticks > maxTicks) {
            return 0;
        }
    }
    // qui XRDY = 1

    // write register to read
    CT_I2C2.I2C_DATA = reg;
    CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_XRDY = 0x1;

    // wait for access ready
    ticks = 0;
    while(!CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_ARDY) {
        ticks++;
        if(ticks > maxTicks) {
            return 0;
        }
    }
    // debug
    result->reg[0] = CT_I2C2.I2C_CNT;
    result->reg[1] = CT_I2C2.I2C_CON;
    result->reg[2] = CT_I2C2.I2C_IRQSTATUS_RAW;
    result->reg[3] = CT_I2C2.I2C_BUFSTAT;
    result->reg[4] = 0x70;

    /*
     * Da analizzare: 26/02/2018
     * Sizeof struct: 40
     * Message received from PRU:DATAOK, reg1-0x3, reg2-0x8403, reg3-0x1014, reg4-0x8003, reg5-0x73, reg6-0x3, reg7-0x11c, reg8-0x37
     * Message received from PRU:DATAKO, reg1-0x1, reg2-0x8600, reg3-0x11c, reg4-0x8000, reg5-0x70, reg6-0x0, reg7-0x0, reg8-0x0
     * NOTA: reg2-0x8600 (dovrebbe essere 0x8601)
     * Qualcosa non va nella inizializzazione del ciclo. Riporta anche valori precedenti degli interrupt (vedi reg3 = reg7)
     */


    if(CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_NACK) {
        CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_NACK = 0x1;
    }
    if(CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_BF) {
        CT_I2C2.I2C_SYSC_bit.I2C_SYSC_SRST = 0b1; // SoftReset Normal Mode
        return 0;
    }

    // 16793856 - uint32_t m1 = (*CM_PER_L4LS_CLKSTCTRL);
    // debug
    result->reg[0] = CT_I2C2.I2C_CNT;
    result->reg[1] = CT_I2C2.I2C_CON;
    result->reg[2] = CT_I2C2.I2C_IRQSTATUS_RAW;
    result->reg[3] = CT_I2C2.I2C_BUFSTAT;
    result->reg[4] = 0x71;


    // FIXME: qui mi arriva un nack?

    // read data
    CT_I2C2.I2C_CNT_bit.I2C_CNT_DCOUNT = bytes; // bytes
//    CT_I2C2.I2C_SA_bit.I2C_SA_SA = address;
//    CT_I2C2.I2C_CON_bit.I2C_CON_STT = 0b0; //
//    CT_I2C2.I2C_CON_bit.I2C_CON_MST = 0b1; // master mode
//    CT_I2C2.I2C_CON_bit.I2C_CON_TRX = 0b0; // receiver mode
//    CT_I2C2.I2C_CON_bit.I2C_CON_STP = 0b1; // Stop condition required
//    CT_I2C2.I2C_CON_bit.I2C_CON_STT = 0b1; // Start condition (this is a repeated start)
      CT_I2C2.I2C_CON = 0x8403;

      // debug
      result->reg[0] = CT_I2C2.I2C_CNT;
      result->reg[1] = CT_I2C2.I2C_CON;
      result->reg[2] = CT_I2C2.I2C_IRQSTATUS_RAW;
      result->reg[3] = CT_I2C2.I2C_BUFSTAT;
      result->reg[4] = 0x72;

    uint8_t count;
    for(count = 0; count < bytes; count++ ) {
        // wait data
        ticks = 0;
        while(!CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_RRDY) {
            ticks++;
            if(ticks > maxTicks) {
                return 0;
            }
        }
        buffer[count] = CT_I2C2.I2C_DATA;
        // require next data
        CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_RRDY = 0b1;
        if(CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_AERR) {
            break;
        }
        uint32_t i = 0;
        for(i = 0; i < 20000; i++); // wait 1us
    }

    // debug
    result->reg[4] = 0x73;
    result->reg[5] = CT_I2C2.I2C_CNT;
    result->reg[6] = CT_I2C2.I2C_IRQSTATUS_RAW;
    result->reg[7] = CT_I2C2.I2C_SCLH;

    return count;
}

int testConnection()
{
    int nbytes = readBytes(HMC5883L_ADDRESS, HMC5883L_RA_ID_A, 3, buffer);
    if (nbytes == 3)
    {
        return (buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3');
    }
    return 0;
}

void set400KHz() {
    // prescaler
    CT_I2C2.I2C_PSC = 0x02; // 24MHz
    /*
     * tLow = (SCLL +7)*42ns
     * 42ns is the time period at 24MHz,
     * tLow = (1000000000ns/400000Hz)/2) is the time period (in ns) at low signal on SCL
     * SCLL = tLow/42ns -7
     * SCLL = 1250/42 -7
     * SCLL is like 23 (rounded)
     */
    CT_I2C2.I2C_SCLL = 0x17;
    /*
     * tHigh = (SCLH +5)*42ns
     * 42ns is the time period at 24MHz,
     * tHigh = (1000000000ns/400000Hz)/2) is the time period (in ns) at high signal on SCL
     * SCLH = tHigh/42ns -5
     * SCLH = 1250/42 -5
     * SCLH is like 25 (rounded)
     */
    CT_I2C2.I2C_SCLH = 0x19;
}

void set100KHz() {
    // prescaler
    CT_I2C2.I2C_PSC = 0x04; // 12MHz
    /*
     * tLow = (SCLL +7)*83ns
     * 83ns is the time period at 12MHz,
     * tLow = (1000000000ns/100000Hz)/2) is the time period (in ns) at low signal on SCL
     * SCLL = tLow/83ns -7 = 53,241
     * SCLL = 1250/83 -7
     * SCLL is like 53 (rounded)
     */
    CT_I2C2.I2C_SCLL = 0x35;
    /*
     * tHigh = (SCLH +5)*83ns
     * 83ns is the time period at 12MHz,
     * tHigh = (1000000000ns/100000Hz)/2) is the time period (in ns) at high signal on SCL
     * SCLH = tHigh/83ns - 5 = 55,25
     * SCLH = 1250/83 - 5
     * SCLH is like 55 (rounded)
     */
    CT_I2C2.I2C_SCLH = 0x37;
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

    // for timeout
    uint32_t ticks = 0;
    uint32_t maxTicks = 20000000;

    strcpy(result->cmd, "DATA");

    uint8_t active = 0;

    /**************************************************************
     * C O N F I G U R A Z I O N E   I 2 C 2   A N D   C L O C K S
     **************************************************************/
    uint32_t * CM_PER_L4LS_CLKSTCTRL = (uint32_t *)0x44E00000;
    (*CM_PER_L4LS_CLKSTCTRL) = ((*CM_PER_L4LS_CLKSTCTRL) | (1 << CLKACTIVITY_I2C_FCLK) | (1 << CLKACTIVITY_L4LS_GCLK)) & 0xFFFFFFFC; // CLKTRCTRL = 0x00

    /*
     * FROM: https://e2e.ti.com/support/arm/sitara_arm/f/791/p/458311/1659097
     * deve essere abilitato CM_PER_I2C2_CLKCTRL;
     */
    uint32_t * CM_PER_I2C2_CLKCTRL = (uint32_t *)0x44E00044;
    (*CM_PER_I2C2_CLKCTRL) = 2;

//    CT_I2C2.I2C_CON_bit.I2C_CON_I2C_EN = 0b0; // i2c reset
    CT_I2C2.I2C_SYSC_bit.I2C_SYSC_SRST = 0b1; // SoftReset Normal Mode
    // wait for reset completed
    ticks = 0;
    while(!CT_I2C2.I2C_SYSS_bit.I2C_SYSS_RDONE) {
        ticks++;
        if(ticks > maxTicks) {
            break; // stica ...
        }
    }

    // SYSC system control register
    CT_I2C2.I2C_SYSC_bit.I2C_SYSC_AUTOIDLE = 0b0; // AutoIdle disabled
    CT_I2C2.I2C_SYSC_bit.I2C_SYSC_ENAWAKEUP = 0b0; // wakeup disabled
    CT_I2C2.I2C_SYSC_bit.I2C_SYSC_IDLEMODE = 0b01; // no idleMode
    CT_I2C2.I2C_SYSC_bit.I2C_SYSC_CLKACTIVITY = 0b11; // active clocks: interface ocp clock and functional system clock

//    set400KHz();
    set100KHz();


    // I2C_BUF as default: DMA disabled and buffer tx/rx lenght = 1

    // I2C_OA Own Address register
    // must be set?
    // CT_I2C2.I2C_OA_bit.I2C_OA_OA = 0b0000100010; // address 0x22


    // i2c2 master mode
    CT_I2C2.I2C_CON_bit.I2C_CON_I2C_EN = 0b1; // i2c enabled

    // TODO: configurare i pins per i2c2
    while (1)
    {
        if (active)
        {
            counter %= cycles;
            counter++;
            if (counter == cycles)
            {
                if (testConnection())
                {
                    strcpy(result->cmd, "DATAOK");
                    pru_rpmsg_send(&transport, dst, src, payload,
                                   sizeof(struct EcapData));
                }
                else
                {
                    strcpy(result->cmd, "DATAKO");
                    pru_rpmsg_send(&transport, dst, src, payload,
                                   sizeof(struct EcapData));
                }
            }
        }
        if (__R31 & HOST_INT)
        {
            if (pru_rpmsg_receive(&transport, &src, &dst, payload,
                                  &len) == PRU_RPMSG_SUCCESS)
            {
                int eq = strncmp("START", (const char *) payload, 5);
                if (eq == 0)
                {
                    active = 1;
                    pru_rpmsg_send(&transport, dst, src, "STARTED", 8);
                }
                else if (eq < 0)
                {
                    active = 0;
                    pru_rpmsg_send(&transport, dst, src, "MINOR", 6);
                }
                else if (eq > 0)
                {
                    active = 0;
                    pru_rpmsg_send(&transport, dst, src, "MAJOR", 6);
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
