void InitRTCInt(void)
{
  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |            //50-50 Duty(Though,it will not reflect on any output pin)
                      GCLK_GENCTRL_GENEN |          //Enable generic clock generator
                      GCLK_GENCTRL_SRC_OSCULP32K |  //Internal 32kHz low power clock as source
                      GCLK_GENCTRL_ID(4) |          //Select GCLK 4
                      GCLK_GENCTRL_DIVSEL |         //Set GLCK divisor as 2 to the power of (divisor) value
                      GCLK_GENCTRL_RUNSTDBY;        //Run on standby
  while (GCLK->STATUS.bit.SYNCBUSY);

  //Set Clock divider for GCLK4
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(4) |         //Select clock divisor to divide by 32 (2 ^ (4 + 1))
                     GCLK_GENDIV_ID(4);           //GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              //Wait for synchronization

 //Connect GCLK4 output to RTC
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK4 |  // Select GCLK4
                      GCLK_CLKCTRL_ID_RTC |     // Connect to the RTC
                      GCLK_CLKCTRL_CLKEN;       // Enable GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);            // Wait for synchronization

  // RTC configuration (rtc.h)--------------------------------------------------
  RTC->MODE1.CTRL.bit.ENABLE = 0;                       // Disable the RTC
  while (RTC->MODE0.STATUS.bit.SYNCBUSY);               // Wait for synchronization

  RTC->MODE1.CTRL.bit.SWRST = 1;                       // Software reset the RTC
  while (RTC->MODE0.STATUS.bit.SYNCBUSY);              // Wait for synchronization

  RTC->MODE1.CTRL.reg |= RTC_MODE1_CTRL_PRESCALER_DIV1024 |     // Set prescaler to 1024
                         RTC_MODE1_CTRL_MODE_COUNT16;           // Set RTC to mode 0, 32-bit timer

  // SET TIME IN SEC HERE AFTER RTC_MODE1_PER_PER
  RTC->MODE1.PER.reg = RTC_MODE1_PER_PER(14);                   // Interrupt time 15s: 1Hz/(14 + 1)
  while (RTC->MODE0.STATUS.bit.SYNCBUSY);                       // Wait for synchronization

  // Configure RTC interrupts ------------------------------------------
  RTC->MODE1.INTENSET.reg = RTC_MODE0_INTENSET_CMP0;             // Enable RTC overflow interrupts

  NVIC_SetPriority(RTC_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for RTC
  NVIC_EnableIRQ(RTC_IRQn);         // Connect RTC to Nested Vector Interrupt Controller (NVIC)

  // Enable Deep Sleep Mode--------------------------------------------------------------
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;// | SCB_SCR_SLEEPONEXIT_Msk;  // Put the SAMD21 in deep sleep upon executing the __WFI() function
  NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_SLEEPPRM_DISABLED;        // Disable auto power reduction during sleep - SAMD21 Errata 1.14.2
  

  // Enable RTC--------------------------------------------------------------
  RTC->MODE1.CTRL.bit.ENABLE = 1;                       // Enable the RTC
  while (RTC->MODE1.STATUS.bit.SYNCBUSY);               // Wait for synchronization
}

void RTC_Handler(void)
{
  RTC->MODE1.INTFLAG.bit.OVF = 1;                                  // Reset the overflow interrupt flag
}


void setup() {
  InitRTCInt();
  // Serial.begin(9600);

}

void loop() {
  //PUT CODE HERE

  
  //END CODE
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;           // Disable SysTick interrupts
  __DSB();                                              // Complete outstanding memory operations - not required for SAMD21 ARM Cortex M0+
  __WFI();                                              // Put the SAMD21 into deep sleep, Zzzzzzzz...
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;            // Enable SysTick interrupts
}
