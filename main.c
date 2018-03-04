#include <stdint.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <pru_rpmsg.h>
#include "pru_i2c.h"
#include "resource_table.h"
#include "HMC5883L.h"
#include "MPU6050.h"

volatile register unsigned __R31;

#define VIRTIO_CONFIG_S_DRIVER_OK       4
#define CLKACTIVITY_I2C_FCLK            24
#define CLKACTIVITY_L4LS_GCLK           8
#define PAYLOAD_CONTENT_OFFSET          2

unsigned char payload[RPMSG_BUF_SIZE];

inline void delayMicros(uint8_t micros)
{
    uint16_t cycles = micros * 100;
    uint16_t i = 0;
    for (i = 0; i < cycles; i++)
    {
    };
}

uint32_t ticks = 0;
uint32_t maxTicks = 100000000;
inline uint8_t waitBB()
{
    uint32_t ticks = 0;
    while (CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_BB)
    {
        ticks++;
        if (ticks > MAX_CYCLES_WAITING)
        {
            return 0;
        }
    }
    return 1;
}
inline uint8_t waitBF()
{
    uint32_t ticks = 0;
    while (!CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_BF)
    {
        ticks++;
        if (ticks > MAX_CYCLES_WAITING)
        {
            return 0;
        }
    }
    return 1;
}
uint8_t waitXRDY()
{
    uint32_t ticks = 0;
    while (!CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_XRDY)
    {
        ticks++;
        if (ticks > maxTicks)
        {
            return 0;
        }
    }
    return 1;
}
uint8_t waitRRDY()
{
    uint32_t ticks = 0;
    while (!CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_RRDY)
    {
        ticks++;
        if (ticks > maxTicks)
        {
            return 0;
        }
    }
    return 1;
}
uint8_t waitARDY()
{
    uint32_t ticks = 0;
    while (!CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_ARDY)
    {
        ticks++;
        if (ticks > maxTicks)
        {
            return 0;
        }
    }
    return 1;
}

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

uint8_t readBytes(uint8_t address, uint8_t reg, uint8_t bytes, uint8_t* buffer)
{

    if (!waitBB())
    {
        return 0;
    }

    CT_I2C2.I2C_SA_bit.I2C_SA_SA = address; // 7 bit address
    CT_I2C2.I2C_CNT_bit.I2C_CNT_DCOUNT = 1; // 1 byte to transmit
    CT_I2C2.I2C_CON = 0x8601; // MST/TRX/STT
    delayMicros(7);

    if (!waitXRDY())
    {
        return 0;
    }
    // write register to read
    CT_I2C2.I2C_DATA = reg;
    CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_XRDY = 0b1;

    // wait access to registers
    if (!waitARDY())
    {
        return 0;
    }
    delayMicros(6);

    if (CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_AERR
            | CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_NACK)
    {
        return 0;
    }

    // read data
    CT_I2C2.I2C_CNT_bit.I2C_CNT_DCOUNT = bytes; // bytes to reveive
    CT_I2C2.I2C_CON = 0x8403; // MST/STP/STT
    delayMicros(24);
    CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_ARDY = 0b1;

    // wait data
    if (!waitRRDY())
    {
        return 0;
    }

    uint8_t count;
    for (count = 0; count < bytes; count++)
    {
        // read byte
        buffer[count] = CT_I2C2.I2C_DATA;

        // require next data
        CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_RRDY = 0b1;

        // wait data
        if (!waitRRDY())
        {
            return 0;
        }
        delayMicros(1);

        if (CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_AERR
                | CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_NACK)
        {
            return 0;
        }
    }

    // wait for access ready
    if (!waitARDY())
    {
        return 0;
    }
    delayMicros(6);

    // wait for bus free
    // wait data
    if (!waitBF())
    {
        return 0;
    }

    // serve?
    CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_ARDY = 0b1;
    CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_XRDY = 1;
    CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_RRDY = 1;
    return count;
}

uint8_t writeBytes(uint8_t address, uint8_t reg, uint8_t bytes, uint8_t* buffer)
{

    if (!waitBB())
    {
        return 0;
    }

    CT_I2C2.I2C_SA_bit.I2C_SA_SA = address; // 7 bit address
    CT_I2C2.I2C_CNT_bit.I2C_CNT_DCOUNT = bytes + 1; // 1 byte to transmit
    CT_I2C2.I2C_CON = 0x8603; // MST/TRX/STT/STP
    delayMicros(7);

    if (!waitXRDY())
    {
        return 0;
    }
    // write register to read
    CT_I2C2.I2C_DATA = reg;
    CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_XRDY = 0b1;
    delayMicros(1);
    if (CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_AERR
            | CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_NACK)
    {
        return 0;
    }

    uint8_t count;
    for (count = 0; count < bytes; count++)
    {
        waitXRDY();
        CT_I2C2.I2C_DATA = buffer[count];
        CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_XRDY = 0b1;
        delayMicros(1);
        if (CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_AERR
                | CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_NACK)
        {
            return 0;
        }
    }

    // wait for access ready
    if (!waitARDY())
    {
        return 0;
    }
    delayMicros(6);

    // wait for bus free
    // wait data
    if (!waitBF())
    {
        return 0;
    }

    // serve?
    CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_ARDY = 0b1;
    CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_XRDY = 1;
    CT_I2C2.I2C_IRQSTATUS_RAW_bit.I2C_IRQSTATUS_RAW_RRDY = 1;
    return count;
}

uint8_t readReg(uint8_t address, uint8_t reg, uint8_t* buffer)
{
    return readBytes(address, reg, 1, buffer);
}
uint8_t writeReg(uint8_t address, uint8_t reg, uint8_t value)
{
    return writeBytes(address, reg, 1, &value);
}

int testConnection()
{
    initBuffers();
    int nbytes = readBytes(HMC5883L_ADDRESS, HMC5883L_RA_ID_A, 3,
                           payload + PAYLOAD_CONTENT_OFFSET);
    if (nbytes == 3
            && (((payload + PAYLOAD_CONTENT_OFFSET)[0] == 'H'
                    && (payload + PAYLOAD_CONTENT_OFFSET)[1] == '4'
                    && (payload + PAYLOAD_CONTENT_OFFSET)[2] == '3')))
    {
        return nbytes;
    }
    return 0;
}

void set400KHz()
{
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

void set100KHz()
{
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

/**************************************************************************
 *
 * C O M P A S S
 *
 **************************************************************************/
/*
 * bool MyGY86::initCompass() {
 // TODO: gestire multimaster. Verificare libreria i2c utilizzata per il controllo del bus
 // questo eviterebbe di switchare continuamente il mastermode
 setMasterModeEnabled(false);
 setI2CBypass(true);
 setGainCompass();
 setRateAndAverageCompass();
 setModeCompass();
 setI2CBypass(false);
 setMasterModeEnabled(true);
 return true;
 }

 setMasterModeEnabled:
 1) master mode as parameter
 2) other bits as is
 */
//uint8_t setMasterModeEnabled(uint8_t enabled) {
//  uint8_t reg;
//  uint8_t bytes = readReg(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, &reg);
//  if (enabled) {
//    reg |= (1 << MPU6050_I2C_MST_EN_BIT);
//  } else {
//    reg &= ~(1 << MPU6050_I2C_MST_EN_BIT);
//  }
//  writeReg(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, reg);
//  return 1;
//}
uint8_t initHMC5883L()
{
    // Single Write
    uint8_t result = writeReg(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_A, 0x70);
    if (result > 0)
    {
        result = writeReg(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_B, 0x20);
        if (result > 0)
        {
            result = writeReg(HMC5883L_ADDRESS, HMC5883L_RA_MODE, 0x00);
        }
    }
    return result;
}

uint8_t readHMC5883LData(unsigned char* data)
{
    uint8_t result = readBytes(HMC5883L_ADDRESS, HMC5883L_RA_DATAX_H, 2,
                               data + PAYLOAD_CONTENT_OFFSET);
    if (result == 2)
    {
        result += readBytes(HMC5883L_ADDRESS, HMC5883L_RA_DATAY_H, 2,
                            data + PAYLOAD_CONTENT_OFFSET + 2);
        if (result == 4)
        {
            result += readBytes(HMC5883L_ADDRESS, HMC5883L_RA_DATAZ_H, 2,
                                data + PAYLOAD_CONTENT_OFFSET + 4);
        }
    }
    if (result == 6)
    {
        return result;
    }
    else
    {
        return 0;
    }
}

uint8_t selfTestsHMC5883L(unsigned char* data)
{
    uint8_t RegModeCurrent;
    uint8_t RegACurrent;
    if (readReg(HMC5883L_ADDRESS, HMC5883L_RA_MODE, &RegModeCurrent))
    {
        if (readReg(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_A, &RegACurrent))
        {
            if (writeReg(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_A, 0x12)) // positive test
            {
                if (writeReg(HMC5883L_ADDRESS, HMC5883L_RA_MODE, 0x01)) // single mode
                {
                    {
                        uint8_t status;
                        while(1) {
                            if(readReg(HMC5883L_ADDRESS, HMC5883L_RA_STATUS, &status)) {
                                if(!(status & 0x01)) {
                                    break;
                                }
                            }
                        }
                        while(1) {
                            if(readReg(HMC5883L_ADDRESS, HMC5883L_RA_STATUS, &status)) {
                                if(status & 0x01) {
                                    if (readHMC5883LData(data))
                                    {
                                        if (writeReg(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_A,
                                                     RegACurrent))
                                        {
                                            if (writeReg(HMC5883L_ADDRESS, HMC5883L_RA_MODE,
                                                         RegModeCurrent))
                                            {
                                                return 1;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                }
            }
        }
    }
    return 0;
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

    uint8_t active = 0;

    /**************************************************************
     * C O N F I G U R A Z I O N E   I 2 C 2   A N D   C L O C K S
     **************************************************************/
    uint32_t * CM_PER_L4LS_CLKSTCTRL = (uint32_t *) 0x44E00000;
    (*CM_PER_L4LS_CLKSTCTRL) = ((*CM_PER_L4LS_CLKSTCTRL)
            | (1 << CLKACTIVITY_I2C_FCLK) | (1 << CLKACTIVITY_L4LS_GCLK))
            & 0xFFFFFFFC; // CLKTRCTRL = 0x00

    /*
     * FROM: https://e2e.ti.com/support/arm/sitara_arm/f/791/p/458311/1659097
     * deve essere abilitato CM_PER_I2C2_CLKCTRL;
     */
    uint32_t * CM_PER_I2C2_CLKCTRL = (uint32_t *) 0x44E00044;
    (*CM_PER_I2C2_CLKCTRL) = 2;

//    CT_I2C2.I2C_CON_bit.I2C_CON_I2C_EN = 0b0; // i2c reset
    CT_I2C2.I2C_SYSC_bit.I2C_SYSC_SRST = 0b1; // SoftReset Normal Mode
    // wait for reset completed
    ticks = 0;
    while (!CT_I2C2.I2C_SYSS_bit.I2C_SYSS_RDONE)
    {
        ticks++;
        if (ticks > maxTicks)
        {
            break; // stica ...
        }
    }

    // SYSC system control register
    CT_I2C2.I2C_SYSC_bit.I2C_SYSC_AUTOIDLE = 0b0; // AutoIdle disabled
    CT_I2C2.I2C_SYSC_bit.I2C_SYSC_ENAWAKEUP = 0b0; // wakeup disabled
    CT_I2C2.I2C_SYSC_bit.I2C_SYSC_IDLEMODE = 0b01; // no idleMode
    CT_I2C2.I2C_SYSC_bit.I2C_SYSC_CLKACTIVITY = 0b00; // active clocks: interface ocp clock and functional system clock

//    set400KHz();
    set400KHz();

    // I2C_BUF as default: DMA disabled and buffer tx/rx lenght = 1

    // I2C_OA Own Address register
    // must be set?
    // CT_I2C2.I2C_OA_bit.I2C_OA_OA = 0b0000100010; // address 0x22

    // i2c2 master mode
    CT_I2C2.I2C_CON_bit.I2C_CON_I2C_EN = 0b1; // i2c enabled
    ticks = 0;
    while (!CT_I2C2.I2C_SYSS_bit.I2C_SYSS_RDONE)
    {
        ticks++;
        if (ticks > maxTicks)
        {
            break; // stica ...
        }
    }

    while (1)
    {
        if (active)
        {
            counter %= cycles;
            counter++;
            if (counter == cycles)
            {
                uint8_t bytes = 0;
                if (bytes = testConnection())
                {
                    payload[0] = 'T';
                    payload[1] = 'T';
                    pru_rpmsg_send(&transport, dst, src, payload,
                                   bytes + PAYLOAD_CONTENT_OFFSET);
                    if (initHMC5883L())
                    {
                        pru_rpmsg_send(&transport, dst, src, "HTO", 4);
                        if (readHMC5883LData(payload))
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
                        if (selfTestsHMC5883L(payload))
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
