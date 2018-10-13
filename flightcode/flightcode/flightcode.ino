//#define KICKSAT_DEBUG


#include <RH_RF22.h>
#include <ax25.h>
#include <SdFat.h>

#define LOOP_PERIOD_MS 30000
bool awake = true;

//Radio Stuff
RHHardwareSPI spi;
RH_RF22 radio(SPI_CS_RFM, RF_NIRQ, spi);
uint8_t rxBuffer[32];
uint8_t rxLen = sizeof(rxBuffer);

RH_RF22::ModemConfig FSK1k2 = {
  0x2B, //reg_1c
  0x03, //reg_1f
  0x41, //reg_20
  0x60, //reg_21
  0x27, //reg_22
  0x52, //reg_23
  0x00, //reg_24
  0x9F, //reg_25
  0x2C, //reg_2c - Only matters for OOK mode
  0x11, //reg_2d - Only matters for OOK mode
  0x2A, //reg_2e - Only matters for OOK mode
  0x80, //reg_58
  0x60, //reg_69
  0x09, //reg_6e
  0xD5, //reg_6f
  0x24, //reg_70
  0x22, //reg_71
  0x01  //reg_72
};

void setup() {
  
  //Watchdog + Loop Timers
  pinMode(WDT_WDI, OUTPUT);
  digitalWrite(WDT_WDI, LOW);
  timer_setup();

  //LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //Radio (don't put in shutdown mode - consumes lots of current for some weird reason)
  pinMode(RF_SDN, OUTPUT);
  digitalWrite(RF_SDN, LOW);
  for(int k = 0; k < rxLen; ++k) {
    rxBuffer[k] = 0; //Fill receive buffer with 0s
  }

  //Sensors (Chip Select off)
  pinMode(SPI_CS_XTB1, OUTPUT);
  digitalWrite(SPI_CS_XTB1, HIGH);
  pinMode(SPI_CS_XTB2, OUTPUT);
  digitalWrite(SPI_CS_XTB2, HIGH);
  pinMode(SPI_CS_XTB3, OUTPUT);
  digitalWrite(SPI_CS_XTB3, HIGH);
  pinMode(SPI_CS_XTB4, OUTPUT);
  digitalWrite(SPI_CS_XTB4, HIGH);

  //SD Card (Chip Select off)
  pinMode(SPI_CS_SD, OUTPUT);
  digitalWrite(SPI_CS_SD, HIGH);

  //MRAM (Chip Select off)
  pinMode(SPI_CS_MRAM, OUTPUT);
  digitalWrite(SPI_CS_MRAM, HIGH);

  //Burn Wires (everything pulled low)
  pinMode(BURN_RELAY_A, OUTPUT);
  digitalWrite(BURN_RELAY_A, LOW);
  pinMode(BURN_RELAY_B, OUTPUT);
  digitalWrite(BURN_RELAY_B, LOW);
  pinMode(PIN_ENAB_BURN1, OUTPUT);
  digitalWrite(PIN_ENAB_BURN1, LOW);
  pinMode(PIN_ENAB_BURN2, OUTPUT);
  digitalWrite(PIN_ENAB_BURN2, LOW);
  pinMode(PIN_ENAB_BURN3, OUTPUT);
  digitalWrite(PIN_ENAB_BURN3, LOW);
  pinMode(PIN_ENAB_BURN4, OUTPUT);
  digitalWrite(PIN_ENAB_BURN4, LOW);
  pinMode(PIN_ENAB_BURN5, OUTPUT);
  digitalWrite(PIN_ENAB_BURN5, LOW);

  //GPS (turned off)
  pinMode(PIN_ENAB_GPS, OUTPUT);
  digitalWrite(PIN_ENAB_GPS, LOW);
  
  SPI.begin();

  #ifdef KICKSAT_DEBUG
  SerialUSB.begin(115200);
  #endif

  go_to_sleep();
}


void loop() {
  
}

void go_to_sleep() {
  SYSCTRL->VREG.bit.RUNSTDBY = 1; //Make sure the voltage regulator stays on while asleep
  SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk; //Enable sleep on exit from interrupts
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;   //Enable deep sleep mode
  //PM->SLEEP.reg |= 2;  // Enable Idle1 mode - sleep CPU clock only
  __DSB();
  __WFI();

}

void TC4_Handler() {     
  // Check for overflow (OVF) interrupt
  if (TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF)             
  {
    //Toggle the watchdog
    digitalWrite(WDT_WDI, HIGH);
    delayMicroseconds(2);
    digitalWrite(WDT_WDI, LOW);
    TC4->COUNT16.INTFLAG.bit.OVF = 1;              // Clear the OVF interrupt flag
  }
}

void TC5_Handler() {     
  // Check for overflow (OVF) interrupt
  if (TC5->COUNT16.INTFLAG.bit.OVF && TC5->COUNT16.INTENSET.bit.OVF)             
  {
    //Do other stuff...
    digitalWrite(LED_BUILTIN, LOW);
    delay(3000);
    digitalWrite(LED_BUILTIN, HIGH);
    TC5->COUNT16.INTFLAG.bit.OVF = 1;              // Clear the OVF interrupt flag
  }
}

void timer_setup() {
  
  // Set up the generic clock (GCLK4) used to clock timers
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(2) |          // Divide the 32.768KHz clock source by 2
                     GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                      GCLK_GENCTRL_GENEN |         // Enable GCLK4
                      GCLK_GENCTRL_RUNSTDBY |      // Run in standby mode
                      GCLK_GENCTRL_SRC_OSCULP32K | // Set the 32.768KHz low-power internal clock source
                      GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  // Feed GCLK4 to TC4 and TC5
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                      GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                      GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization


  //Setup watchdog timer
  TC4->COUNT16.CC[0].reg = 0x0400;                 // Set the TC4 CC0 register to give a 1 second count-down
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization

  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  TC4->COUNT16.INTFLAG.bit.OVF = 1;                        // Clear the interrupt flags
  TC4->COUNT16.INTENSET.bit.OVF = 1;                       // Enable TC4 interrupts
 
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16 |     // Set prescaler to 16 to give 1.024kHz
                            TC_CTRLA_WAVEGEN_MFRQ |        // Put the timer TC4 into match frequency (MFRQ) mode
                            TC_CTRLA_RUNSTDBY |            // Run in standby mode
                            TC_CTRLA_ENABLE;               // Enable TC4
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization


  //Setup loop timer 
  TC5->COUNT16.CC[0].reg = 0x2800; // 30 = 0x7800, 45 = 0xB400, 60 = 0xF000; // Set the TC5 CC0 register to give a 10 second count-down
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization

  NVIC_SetPriority(TC5_IRQn, 3);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC5 to 3 (lowest so it is preempted by watchdog)
  NVIC_EnableIRQ(TC5_IRQn);         // Connect TC5 to Nested Vector Interrupt Controller (NVIC)

  TC5->COUNT16.INTFLAG.bit.OVF = 1;                        // Clear the interrupt flags
  TC5->COUNT16.INTENSET.bit.OVF = 1;                       // Enable TC5 interrupts
 
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16 |     // Set prescaler to 16 to give 1.024kHz
                            TC_CTRLA_WAVEGEN_MFRQ |        // Put the timer TC5 into match frequency (MFRQ) mode
                            TC_CTRLA_RUNSTDBY |            // Run in standby mode
                            TC_CTRLA_ENABLE;               // Enable TC5
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization
}



