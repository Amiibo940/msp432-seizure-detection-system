#include "msp.h"
#include "lcdLib_432.h"
// wiring for response module
// wire response P3.2 RX to detection P3.3 TX
// wire response P3.3 TX to detection P3.2 RX
// wire response GND to detection GND
// wire potentiometer middle pin to P4.7 / A6
// wire buzzer signal to P2.4
// buttons use P1.1 and P1.4 to GND
// LCD uses lcdLib_432 wiring
// 7-segment display uses pins listed below

// packet constants
#define START_BYTE      0xAA
#define END_BYTE        0x55

#define PKT_SENSOR      0x01
#define PKT_EVENT       0x02
#define PKT_COMMAND     0x03
#define PKT_THRESHOLD   0x04
#define PKT_HEARTBEAT   0x05
#define PKT_ACK         0x06

// command values
#define CMD_ARM         'A'
#define CMD_DISARM      'D'
#define CMD_MUTE        'M'
#define CMD_RESET       'R'
#define CMD_TEST        'T'

// globals
volatile unsigned char lvl = 0;
volatile unsigned int magnitude = 0;
volatile unsigned char armed = 1;

volatile unsigned int potVal = 0;
volatile unsigned int lastSentPot = 0;

volatile unsigned int msCount = 0;
volatile unsigned int secCount = 0;
volatile unsigned int seizureSeconds = 0;
volatile unsigned char timerRunning = 0;
volatile unsigned char timerFrozen = 0;

volatile unsigned int heartbeatCount = 0;
volatile unsigned int noUartSeconds = 0;
volatile unsigned char heartbeatTimeout = 0;
volatile unsigned char commFault = 0;
volatile unsigned char noUartData = 1;

volatile unsigned char displayDigit = 0;
volatile unsigned char flashState = 0;
volatile unsigned int flashCount = 0;

volatile unsigned char muted = 0;

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
void uart(void);
void adc(void);
void systick(void);
void ports(void);
void ports7seg(void);
void pwm(void);

void allDigitsOff(void);
void enableDigit(unsigned char digit);
void setSegments(unsigned char pattern);
unsigned char getDisplayPattern(unsigned char num);
void formatSeconds(unsigned int value);
void display(void);
void resetResponse(void);
void buzzerOff(void);
void alarm(void);

void sendByte(unsigned char data);
unsigned char packetLength(unsigned char type);
void sendPacket(unsigned char type, unsigned char payload[], unsigned char len);
void parsePacketByte(unsigned char data);

void sendCommand(unsigned char cmd);
void sendThreshold(unsigned int value);
void sendHeartbeat(void);
void sendAck(unsigned char ackType);
void processPacket(void);

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

// ADC setup for potentiometer
void adc(void){
    // P4.7 = A6
    P4->SEL1 |= BIT7;
    P4->SEL0 |= BIT7;

    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);

    ADC14->CTL0 = ADC14_CTL0_ON |
                  ADC14_CTL0_SHT0__192 |
                  ADC14_CTL0_SHP;

    ADC14->MCTL[0] = ADC14_MCTLN_INCH_6;
    ADC14->IER0 = ADC14_IER0_IE0;
    ADC14->CTL0 |= ADC14_CTL0_ENC;
}

// SysTick setup
void systick(void){
    // 3 MHz / 1000 = 1 ms
    SysTick->CTRL = 0;
    SysTick->LOAD = 3000 - 1;
    SysTick->VAL = 0;

    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk |
                     SysTick_CTRL_ENABLE_Msk;
}

// GPIO setup
void ports(void){
    // onboard LEDs
    P1->DIR |= BIT0;
    P2->DIR |= BIT0 | BIT1 | BIT2;

    P1->OUT &= ~BIT0;
    P2->OUT &= ~(BIT0 | BIT1 | BIT2);

    // P1.1 and P1.4 buttons
    P1->DIR &= ~(BIT1 | BIT4);
    P1->REN |= BIT1 | BIT4;
    P1->OUT |= BIT1 | BIT4;

    P1->IES |= BIT1 | BIT4;
    P1->IFG &= ~(BIT1 | BIT4);
    P1->IE |= BIT1 | BIT4;

    NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31);
}

// seven segment pin setup
void ports7seg(void){
    // segments: A P5.1, F P5.0, D P5.7, E P5.6
    P5->DIR |= BIT0 | BIT1 | BIT6 | BIT7;

    // segments: B P6.6, digit 3 P6.7
    P6->DIR |= BIT6 | BIT7;

    // segments: C P2.6, G P2.5, digits P2.7 and P2.3
    P2->DIR |= BIT3 | BIT5 | BIT6 | BIT7;

    allDigitsOff();

    P5->OUT &= ~(BIT0 | BIT1 | BIT6 | BIT7);
    P6->OUT &= ~BIT6;
    P2->OUT &= ~(BIT5 | BIT6);
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

// turn off all 7-seg digits
void allDigitsOff(void){
    P2->OUT &= ~(BIT7 | BIT3);
    P6->OUT &= ~BIT7;
}

// enable one digit
void enableDigit(unsigned char digit){
    allDigitsOff();

    if(digit == 0){
        P2->OUT |= BIT7;
    }
    else if(digit == 1){
        P2->OUT |= BIT3;
    }
    else{
        P6->OUT |= BIT7;
    }
}

// set seven segment pattern
void setSegments(unsigned char pattern){
    P5->OUT &= ~(BIT1 | BIT0 | BIT7 | BIT6);
    P6->OUT &= ~BIT6;
    P2->OUT &= ~(BIT6 | BIT5);

    if(pattern & BIT0){
        P5->OUT |= BIT1;       // A
    }
    if(pattern & BIT1){
        P6->OUT |= BIT6;       // B
    }
    if(pattern & BIT2){
        P2->OUT |= BIT6;       // C
    }
    if(pattern & BIT3){
        P5->OUT |= BIT7;       // D
    }
    if(pattern & BIT4){
        P5->OUT |= BIT6;       // E
    }
    if(pattern & BIT5){
        P5->OUT |= BIT0;       // F
    }
    if(pattern & BIT6){
        P2->OUT |= BIT5;       // G
    }
}

// get display pattern for number
unsigned char getDisplayPattern(unsigned char num){
    if(num == 0){
        return BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
    }
    else if(num == 1){
        return BIT1 | BIT2;
    }
    else if(num == 2){
        return BIT0 | BIT1 | BIT3 | BIT4 | BIT6;
    }
    else if(num == 3){
        return BIT0 | BIT1 | BIT2 | BIT3 | BIT6;
    }
    else if(num == 4){
        return BIT1 | BIT2 | BIT5 | BIT6;
    }
    else if(num == 5){
        return BIT0 | BIT2 | BIT3 | BIT5 | BIT6;
    }
    else if(num == 6){
        return BIT0 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6;
    }
    else if(num == 7){
        return BIT0 | BIT1 | BIT2;
    }
    else if(num == 8){
        return BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6;
    }
    else if(num == 9){
        return BIT0 | BIT1 | BIT2 | BIT3 | BIT5 | BIT6;
    }

    return 0;
}

// format seconds on 7-seg
void formatSeconds(unsigned int value){
    unsigned char ones;
    unsigned char tens;
    unsigned char hundreds;

    if(value > 999){
        value = 999;
    }

    ones = value % 10;
    tens = (value / 10) % 10;
    hundreds = (value / 100) % 10;

    if(displayDigit == 0){
        setSegments(getDisplayPattern(ones));
        enableDigit(0);
    }
    else if(displayDigit == 1){
        setSegments(getDisplayPattern(tens));
        enableDigit(1);
    }
    else{
        setSegments(getDisplayPattern(hundreds));
        enableDigit(2);
    }

    displayDigit++;

    if(displayDigit >= 3){
        displayDigit = 0;
    }
}

// LCD display
void display(void){
    lcdClear();

    if(commFault){
        lcdSetText("UART ERROR", 0, 0);
        lcdSetText("CHECK WIRES", 0, 1);
    }
    else if(!armed){
        lcdSetText("DISARMED", 0, 0);
        lcdSetText("Lvl:", 0, 1);
        lcdSetInt(lvl, 4, 1);
    }
    else{
        lcdSetText("ARMED Lvl:", 0, 0);
        lcdSetInt(lvl, 10, 0);

        lcdSetText("M:", 0, 1);
        lcdSetInt(magnitude, 2, 1);

        lcdSetText("T:", 8, 1);
        lcdSetInt(seizureSeconds, 10, 1);
    }
}

// reset response module timer
void resetResponse(void){
    lvl = 0;
    magnitude = 0;
    seizureSeconds = 0;
    timerRunning = 0;
    timerFrozen = 0;
    muted = 0;
    buzzerOff();
    display();
}

// buzzer off
void buzzerOff(void){
    TIMER_A0->CCR[1] = 0;
}

// UART fault alarm
void alarm(void){
    if(commFault){
        if(flashState){
            P1->OUT |= BIT0;
            P2->OUT |= BIT0;
        }
        else{
            P1->OUT &= ~BIT0;
            P2->OUT &= ~BIT0;
        }

        if(!muted){
            TIMER_A0->CCR[1] = 1500;
        }
        else{
            buzzerOff();
        }
    }
    else{
        P1->OUT &= ~BIT0;
        P2->OUT &= ~(BIT0 | BIT1 | BIT2);
        buzzerOff();
    }
}

// send one byte
void sendByte(unsigned char data){
    while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A2->TXBUF = data;
}

// get packet payload length
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

    return 0;
}

// send packet
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

// parse incoming UART byte
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

// send command packet
void sendCommand(unsigned char cmd){
    unsigned char payload[1];

    payload[0] = cmd;

    sendPacket(PKT_COMMAND, payload, 1);
}

// send threshold packet
void sendThreshold(unsigned int value){
    unsigned char payload[2];

    payload[0] = (value >> 8) & 0xFF;
    payload[1] = value & 0xFF;

    sendPacket(PKT_THRESHOLD, payload, 2);
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

// process received packet
void processPacket(void){
    unsigned char eventLvl;

    noUartData = 0;
    noUartSeconds = 0;
    heartbeatTimeout = 0;
    commFault = 0;

    if(pktType == PKT_HEARTBEAT){
        // no payload
    }
    else if(pktType == PKT_SENSOR){
        if(armed){
            lvl = pktPayload[0];
            magnitude = ((unsigned int)pktPayload[1] << 8) | pktPayload[2];

            if(lvl == 0){
                seizureSeconds = 0;
                timerRunning = 0;
                timerFrozen = 0;
            }

            display();
        }
    }
    else if(pktType == PKT_EVENT){
        eventLvl = pktPayload[0];

        if(armed){
            lvl = eventLvl;

            if(lvl == 0){
                seizureSeconds = 0;
                timerRunning = 0;
                timerFrozen = 0;
            }

            display();
        }
    }
    else if(pktType == PKT_ACK){
        // ACK received
    }
}

// SysTick interrupt
void SysTick_Handler(void){
    msCount++;
    heartbeatCount++;
    flashCount++;

    if(msCount >= 5){
        msCount = 0;

        if(commFault){
            if(displayDigit == 0){
                setSegments(BIT0 | BIT3 | BIT4 | BIT5 | BIT6);    // E
                enableDigit(2);
            }
            else if(displayDigit == 1){
                setSegments(BIT4 | BIT6);                          // r
                enableDigit(1);
            }
            else{
                setSegments(BIT4 | BIT6);                          // r
                enableDigit(0);
            }

            displayDigit++;

            if(displayDigit >= 3){
                displayDigit = 0;
            }
        }
        else{
            formatSeconds(seizureSeconds);
        }
    }

    if(flashCount >= 500){
        flashCount = 0;
        flashState = !flashState;
    }

    if(heartbeatCount >= 1000){
        heartbeatCount = 0;
        secCount++;

        sendHeartbeat();

        if(noUartData){
            noUartSeconds++;
        }

        if(noUartSeconds >= 5){
            heartbeatTimeout = 1;
            commFault = 1;
        }

        if(armed){
            if(lvl >= 2 && lvl <= 4){
                timerRunning = 1;
                timerFrozen = 0;
            }
            else if(lvl == 5){
                timerRunning = 0;
                timerFrozen = 1;
            }
            else if(lvl == 0){
                timerRunning = 0;
                timerFrozen = 0;
                seizureSeconds = 0;
            }

            if(timerRunning && !timerFrozen){
                seizureSeconds++;
            }
        }

        display();
    }

    if((secCount % 1) == 0){
        ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
    }

    alarm();
}

// ADC interrupt
void ADC14_IRQHandler(void){
    if(ADC14->IFGR0 & ADC14_IFGR0_IFG0){
        potVal = ADC14->MEM[0];
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

// button interrupt
void PORT1_IRQHandler(void){
    if(P1->IFG & BIT1){
        P1->IFG &= ~BIT1;

        armed = !armed;

        if(armed){
            sendCommand(CMD_ARM);
        }
        else{
            sendCommand(CMD_DISARM);
            resetResponse();
        }

        display();
    }

    if(P1->IFG & BIT4){
        P1->IFG &= ~BIT4;

        if(lvl == 0){
            sendCommand(CMD_TEST);
        }
        else if(lvl == 4){
            muted = 1;
            sendCommand(CMD_MUTE);
            buzzerOff();
        }
        else if(lvl == 5){
            sendCommand(CMD_RESET);
            resetResponse();
        }

        display();
    }
}

// main
int main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    lcdInit();
    lcdClear();

    ports();
    ports7seg();
    pwm();
    adc();
    uart();
    systick();

    __enable_irq();

    display();

    while(1){
        if(potVal > lastSentPot + 200 || lastSentPot > potVal + 200){
            sendThreshold(potVal);
            lastSentPot = potVal;
        }

        __sleep();
        __no_operation();
    }
}
