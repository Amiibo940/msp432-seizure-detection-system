#include "msp.h"

// wiring for detection module
// wire accelerometer X output to P5.5 / A0
// wire accelerometer Y output to P5.4 / A1
// wire accelerometer VCC to 3.3V
// wire accelerometer GND to GND
// wire buzzer signal to P2.4
// wire buzzer GND to GND
// onboard LEDs are used for local feedback
// wire detection P3.3 TX to response P3.2 RX
// wire detection P3.2 RX to response P3.3 TX
// wire detection GND to response GND

// packet constants
#define START_BYTE      0xAA
#define END_BYTE        0x55
#define PKT_SENSOR      0x01
#define PKT_EVENT       0x02
#define PKT_COMMAND     0x03
#define PKT_THRESHOLD   0x04
#define PKT_HEARTBEAT   0x05
#define PKT_ACK         0x06
#define PKT_STATUS      0x07

// command values
#define CMD_ARM         'A'
#define CMD_DISARM      'D'
#define CMD_MUTE        'M'
#define CMD_RESET       'R'
#define CMD_TEST        'T'

// globals
volatile unsigned int x = 0;
volatile unsigned int y = 0;
volatile unsigned int magnitude = 0;
volatile unsigned int peakMag = 0;

volatile unsigned char lvl = 0;
volatile unsigned char oldLvl = 9;
volatile unsigned char armed = 1;

volatile unsigned int sampleCount = 0;
volatile unsigned int tonicCount = 0;
volatile unsigned int clonicCount = 0;
volatile unsigned int lowMoveCount = 0;
volatile unsigned int sendCount = 0;
volatile unsigned int episodeCount = 0;
volatile unsigned int bootSeconds = 0;

// heartbeat
volatile unsigned int heartbeatCount = 0;
volatile unsigned int heartbeatTimeout = 0;
volatile unsigned int sampleRate = 50;
volatile unsigned char commFault = 0;

// buzzer mute
volatile unsigned char muted = 0;
volatile unsigned int muteSeconds = 0;

// test mode
volatile unsigned char testMode = 0;
volatile unsigned char testLevel = 0;
volatile unsigned int testSeconds = 0;

// thresholds
volatile unsigned int spikeThreshold = 1200;
volatile unsigned int tonicThreshold = 2000;
volatile unsigned int clonicThreshold = 1500;

#define CENTER_VALUE       8192
#define NORMAL_THRESHOLD   600

// packet parser globals
volatile unsigned char pktState = 0;
volatile unsigned char pktType = 0;
volatile unsigned char pktPayload[8];
volatile unsigned char pktIndex = 0;
volatile unsigned char pktLen = 0;
volatile unsigned char pktChecksum = 0;
volatile unsigned char pktRxChecksum = 0;
volatile unsigned char pktReady = 0;
volatile unsigned int packetErrors = 0;

// prototypes
void adc(void);
void uart(void);
void ports(void);
void pwm(void);
void systick50Hz(void);
void systick100Hz(void);

void sendByte(unsigned char data);
void sendPacket(unsigned char type, unsigned char payload[], unsigned char len);
unsigned char packetLength(unsigned char type);
void parsePacketByte(unsigned char data);
void processPacket(void);

void sendSensorPacket(void);
void sendEventPacket(unsigned char eventLevel);
void sendHeartbeat(void);
void sendAck(unsigned char ackType);
void sendStatus(void);

void detectLevel(void);
void resetDetect(void);
void updateThresholds(unsigned int pot);
void localFeedback(void);
void buzzerOff(void);

void setTestLevel(unsigned char newLevel);
void startLevelTest(void);
void levelTestStep(void);

// adc setup for accelerometer
void adc(void){
    // P5.5 = A0, P5.4 = A1
    P5->SEL1 |= BIT5 | BIT4;
    P5->SEL0 |= BIT5 | BIT4;

    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);

    ADC14->CTL0 = ADC14_CTL0_ON |
                  ADC14_CTL0_MSC |
                  ADC14_CTL0_SHT0__192 |
                  ADC14_CTL0_SHP |
                  ADC14_CTL0_CONSEQ_1;

    ADC14->MCTL[0] = ADC14_MCTLN_INCH_0;
    ADC14->MCTL[1] = ADC14_MCTLN_INCH_1 | ADC14_MCTLN_EOS;

    ADC14->IER0 = ADC14_IER0_IE1;
    ADC14->CTL0 |= ADC14_CTL0_ENC;
}

// UART setup
void uart(void){
    // P3.2 = RX and P3.3 = TX
    P3->SEL0 |= BIT2 | BIT3;
    P3->SEL1 &= ~(BIT2 | BIT3);

    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST;

    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST |
                      EUSCI_A_CTLW0_SSEL__SMCLK;

    // 3 MHz / 9600 baud
    EUSCI_A2->BRW = 3000000 / (9600 * 16);
    EUSCI_A2->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS) |
                      EUSCI_A_MCTLW_OS16;

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;

    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;

    NVIC->ISER[0] = 1 << ((EUSCIA2_IRQn) & 31);
}

// LED setup
void ports(void){
    // onboard LEDs
    P1->DIR |= BIT0;
    P2->DIR |= BIT0 | BIT1 | BIT2;

    P1->OUT &= ~BIT0;
    P2->OUT &= ~(BIT0 | BIT1 | BIT2);
}

// PWM setup for buzzer
void pwm(void){
    // P2.4 = buzzer PWM output
    P2->DIR |= BIT4;
    P2->SEL0 |= BIT4;
    P2->SEL1 &= ~BIT4;

    TIMER_A0->CCR[0] = 3000 - 1;
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[1] = 0;

    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK |
                    TIMER_A_CTL_MC__UP |
                    TIMER_A_CTL_CLR;
}

// normal sampling rate
void systick50Hz(void){
    // 3 MHz / 50 Hz = 60000
    SysTick->CTRL = 0;
    SysTick->LOAD = 60000 - 1;
    SysTick->VAL = 0;

    sampleRate = 50;

    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk |
                     SysTick_CTRL_ENABLE_Msk;
}

// faster sampling rate
void systick100Hz(void){
    // 3 MHz / 100 Hz = 30000
    SysTick->CTRL = 0;
    SysTick->LOAD = 30000 - 1;
    SysTick->VAL = 0;

    sampleRate = 100;

    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk |
                     SysTick_CTRL_ENABLE_Msk;
}

// send one byte
void sendByte(unsigned char data){
    while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A2->TXBUF = data;
}

// get payload size for packet type
unsigned char packetLength(unsigned char type){
    if(type == PKT_SENSOR){
        return 6;
    }
    else if(type == PKT_EVENT){
        return 5;
    }
    else if(type == PKT_COMMAND){
        return 1;
    }
    else if(type == PKT_THRESHOLD){
        return 2;
    }
    else if(type == PKT_HEARTBEAT){
        return 0;
    }
    else if(type == PKT_ACK){
        return 1;
    }
    else if(type == PKT_STATUS){
        return 3;
    }

    return 0;
}

// send packet with checksum
void sendPacket(unsigned char type, unsigned char payload[], unsigned char len){
    unsigned char i;
    unsigned char checksum = type;

    sendByte(START_BYTE);
    sendByte(type);

    for(i = 0; i < len; i++){
        checksum ^= payload[i];
        sendByte(payload[i]);
    }

    sendByte(checksum);
    sendByte(END_BYTE);
}

// parse one received byte
void parsePacketByte(unsigned char data){
    if(pktState == 0){
        if(data == START_BYTE){
            pktState = 1;
            pktIndex = 0;
            pktChecksum = 0;
        }
    }
    else if(pktState == 1){
        pktType = data;
        pktLen = packetLength(pktType);
        pktChecksum = pktType;
        pktIndex = 0;

        if(pktLen > 8){
            pktState = 0;
            packetErrors++;
        }
        else if(pktLen == 0){
            pktState = 3;
        }
        else{
            pktState = 2;
        }
    }
    else if(pktState == 2){
        pktPayload[pktIndex] = data;
        pktChecksum ^= data;
        pktIndex++;

        if(pktIndex >= pktLen){
            pktState = 3;
        }
    }
    else if(pktState == 3){
        pktRxChecksum = data;
        pktState = 4;
    }
    else if(pktState == 4){
        if(data == END_BYTE && pktRxChecksum == pktChecksum){
            pktReady = 1;
        }
        else{
            packetErrors++;
        }

        pktState = 0;
    }
}

// send sensor data to response module
void sendSensorPacket(void){
    unsigned char payload[6];

    payload[0] = lvl;
    payload[1] = (magnitude >> 8) & 0xFF;
    payload[2] = magnitude & 0xFF;
    payload[3] = (peakMag >> 8) & 0xFF;
    payload[4] = peakMag & 0xFF;
    payload[5] = 0;

    sendPacket(PKT_SENSOR, payload, 6);
}

// send event when level changes
void sendEventPacket(unsigned char eventLevel){
    unsigned char payload[5];

    payload[0] = eventLevel;
    payload[1] = (bootSeconds >> 8) & 0xFF;
    payload[2] = bootSeconds & 0xFF;
    payload[3] = (peakMag >> 8) & 0xFF;
    payload[4] = peakMag & 0xFF;

    sendPacket(PKT_EVENT, payload, 5);
}

// send heartbeat packet
void sendHeartbeat(void){
    unsigned char payload[1];

    sendPacket(PKT_HEARTBEAT, payload, 0);
}

// send ack packet
void sendAck(unsigned char ackType){
    unsigned char payload[1];

    payload[0] = ackType;

    sendPacket(PKT_ACK, payload, 1);
}

// send status packet
void sendStatus(void){
    unsigned char payload[3];

    payload[0] = armed;
    payload[1] = sampleRate;
    payload[2] = commFault;

    sendPacket(PKT_STATUS, payload, 3);
}

// update sensitivity from pot
void updateThresholds(unsigned int pot){
    // larger pot value = less sensitive
    spikeThreshold = 800 + (pot / 4);
    tonicThreshold = 1200 + (pot / 4);
    clonicThreshold = 1000 + (pot / 4);
}

// sets test level values
void setTestLevel(unsigned char newLevel){
    lvl = newLevel;
    testLevel = newLevel;

    if(lvl == 0){
        magnitude = 0;
    }
    else if(lvl == 1){
        magnitude = 1300;
    }
    else if(lvl == 2){
        magnitude = 2200;
    }
    else if(lvl == 3){
        magnitude = 2800;
    }
    else if(lvl == 4){
        magnitude = 3500;
    }
    else if(lvl == 5){
        magnitude = 200;
    }

    if(magnitude > peakMag){
        peakMag = magnitude;
    }

    oldLvl = lvl;

    sendEventPacket(lvl);
    sendSensorPacket();
}

// starts level test mode
void startLevelTest(void){
    testMode = 1;
    testSeconds = 0;

    sampleCount = 0;
    tonicCount = 0;
    clonicCount = 0;
    lowMoveCount = 0;
    episodeCount = 0;
    sendCount = 0;

    setTestLevel(1);
}

// moves up one level every 3 seconds
void levelTestStep(void){
    testSeconds++;

    sendSensorPacket();

    if(testSeconds >= 3){
        testSeconds = 0;

        if(testLevel < 5){
            setTestLevel(testLevel + 1);
        }
        else{
            testMode = 0;
            setTestLevel(5);
        }
    }
}

// reset detection
void resetDetect(void){
    lvl = 0;
    oldLvl = 9;
    magnitude = 0;
    peakMag = 0;

    sampleCount = 0;
    tonicCount = 0;
    clonicCount = 0;
    lowMoveCount = 0;
    sendCount = 0;
    episodeCount = 0;

    muted = 0;
    muteSeconds = 0;

    testMode = 0;
    testLevel = 0;
    testSeconds = 0;

    buzzerOff();

    systick50Hz();
}

// detection logic
void detectLevel(void){
    unsigned int dx;
    unsigned int dy;

    if(!armed){
        lvl = 0;
        magnitude = 0;
        lowMoveCount = 0;
        return;
    }

    if(x > CENTER_VALUE){
        dx = x - CENTER_VALUE;
    }
    else{
        dx = CENTER_VALUE - x;
    }

    if(y > CENTER_VALUE){
        dy = y - CENTER_VALUE;
    }
    else{
        dy = CENTER_VALUE - y;
    }

    magnitude = dx + dy;

    if(magnitude > peakMag){
        peakMag = magnitude;
    }

    sampleCount++;

    if(lvl >= 2 && lvl <= 4){
        episodeCount++;
    }

    if(magnitude < NORMAL_THRESHOLD){
        lowMoveCount++;
    }
    else{
        lowMoveCount = 0;
    }

    if(lvl > 0 && lvl < 4 && lowMoveCount > 100){
        resetDetect();
        return;
    }

    if(lvl == 0){
        if(magnitude > spikeThreshold){
            lvl = 1;
            sampleCount = 0;
            tonicCount = 0;
            clonicCount = 0;
            lowMoveCount = 0;
            episodeCount = 0;
            peakMag = magnitude;

            systick100Hz();
        }
    }
    else if(lvl == 1){
        if(magnitude > tonicThreshold){
            tonicCount++;
        }

        if(tonicCount > 300){
            lvl = 2;
            sampleCount = 0;
            clonicCount = 0;
            lowMoveCount = 0;
            episodeCount = 0;
        }

        if(sampleCount > 1000 && tonicCount < 50){
            resetDetect();
            return;
        }
    }
    else if(lvl == 2){
        if(magnitude > clonicThreshold){
            clonicCount++;
        }

        if(clonicCount > 500){
            lvl = 3;
            sampleCount = 0;
            lowMoveCount = 0;
        }
    }
    else if(lvl == 3){
        if(episodeCount > 2000){
            lvl = 4;
            sampleCount = 0;
            lowMoveCount = 0;
        }
    }
    else if(lvl == 4){
        if(lowMoveCount > 200){
            lvl = 5;
            sampleCount = 0;
        }
    }
    else if(lvl == 5){
        lvl = 5;
    }
}

// local LEDs and buzzer
void localFeedback(void){
    P1->OUT &= ~BIT0;
    P2->OUT &= ~(BIT0 | BIT1 | BIT2);

    // if disarmed, everything stays off
    if(!armed){
        buzzerOff();
        return;
    }

    // communication fault alarm only works when armed
    if(commFault){
        P1->OUT |= BIT0;

        if(!muted){
            TIMER_A0->CCR[1] = 2000;
        }
        else{
            buzzerOff();
        }

        return;
    }

    if(lvl == 0){
        P2->OUT |= BIT1;       // green
        buzzerOff();
    }
    else if(lvl == 1){
        P2->OUT |= BIT0;       // red
        buzzerOff();
    }
    else if(lvl == 2){
        P1->OUT |= BIT0;

        if(!muted){
            TIMER_A0->CCR[1] = 500;
        }
        else{
            buzzerOff();
        }
    }
    else if(lvl == 3){
        P1->OUT |= BIT0;
        P2->OUT |= BIT0;

        if(!muted){
            TIMER_A0->CCR[1] = 1000;
        }
        else{
            buzzerOff();
        }
    }
    else if(lvl == 4){
        P1->OUT |= BIT0;
        P2->OUT |= BIT0 | BIT1 | BIT2;

        if(!muted){
            TIMER_A0->CCR[1] = 1500;
        }
        else{
            buzzerOff();
        }
    }
    else if(lvl == 5){
        P2->OUT |= BIT0 | BIT1;
        buzzerOff();
    }
}

// buzzer off
void buzzerOff(void){
    TIMER_A0->CCR[1] = 0;
}

// process received packet
void processPacket(void){
    unsigned char cmd;
    unsigned int pot;

    heartbeatTimeout = 0;
    commFault = 0;

    if(pktType == PKT_HEARTBEAT){
        // no payload
    }
    else if(pktType == PKT_COMMAND){
        cmd = pktPayload[0];

        if(cmd == CMD_ARM){
            armed = 1;
            oldLvl = 9;
            sendAck(PKT_COMMAND);
        }
        else if(cmd == CMD_DISARM){
            armed = 0;

            resetDetect();

            // force system to stay disarmed after resetDetect
            armed = 0;

            lvl = 0;
            magnitude = 0;
            peakMag = 0;
            muted = 0;
            muteSeconds = 0;
            testMode = 0;
            testLevel = 0;
            testSeconds = 0;

            buzzerOff();
            P1->OUT &= ~BIT0;
            P2->OUT &= ~(BIT0 | BIT1 | BIT2);

            sendSensorPacket();
            sendAck(PKT_COMMAND);
        }
        else if(cmd == CMD_MUTE){
            muted = 1;
            muteSeconds = 0;
            buzzerOff();
            sendAck(PKT_COMMAND);
        }
        else if(cmd == CMD_RESET){
            resetDetect();
            sendSensorPacket();
            sendAck(PKT_COMMAND);
        }
        else if(cmd == CMD_TEST){
            if(armed && lvl == 0){
                startLevelTest();
            }

            sendAck(PKT_COMMAND);
        }
    }
    else if(pktType == PKT_THRESHOLD){
        pot = ((unsigned int)pktPayload[0] << 8) | pktPayload[1];

        updateThresholds(pot);
        sendAck(PKT_THRESHOLD);
    }
}

// SysTick starts ADC and heartbeat
void SysTick_Handler(void){
    if(armed && !testMode){
        ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
    }

    heartbeatCount++;

    if(heartbeatCount >= sampleRate){
        heartbeatCount = 0;
        bootSeconds++;

        sendHeartbeat();

        heartbeatTimeout++;

        if(heartbeatTimeout >= 5){
            commFault = 1;
        }

        if(testMode && armed){
            levelTestStep();
        }

        if(muted && armed){
            muteSeconds++;

            if(muteSeconds >= 60){
                muted = 0;
                muteSeconds = 0;
            }
        }

        sendStatus();
    }

    localFeedback();
}

// ADC interrupt
void ADC14_IRQHandler(void){
    unsigned char previousLvl;

    if(ADC14->IFGR0 & ADC14_IFGR0_IFG1){
        if(testMode){
            x = ADC14->MEM[0];
            y = ADC14->MEM[1];
            return;
        }

        if(!armed){
            x = ADC14->MEM[0];
            y = ADC14->MEM[1];
            return;
        }

        x = ADC14->MEM[0];
        y = ADC14->MEM[1];

        previousLvl = lvl;

        detectLevel();

        if(lvl != previousLvl){
            sendEventPacket(lvl);
        }

        sendCount++;

        if((lvl != oldLvl) || (sendCount >= 10)){
            sendSensorPacket();
            oldLvl = lvl;
            sendCount = 0;
        }
    }
}

// UART interrupt
void EUSCIA2_IRQHandler(void){
    unsigned char data;

    if(EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG){
        data = EUSCI_A2->RXBUF;

        parsePacketByte(data);

        if(pktReady){
            pktReady = 0;
            processPacket();
        }
    }
}

// main
int main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    ports();
    pwm();
    adc();
    uart();
    systick50Hz();

    __enable_irq();

    while(1){
        __sleep();
        __no_operation();
    }
}
