// Desired PWM Frequency in Hertz //////////////////////////////////////////////

const uint16_t PWM_FREQ =               1000                                    ;

// Don't change! ///////////////////////////////////////////////////////////////

uint16_t TIM_ARR        = (uint16_t)    (24000000/PWM_FREQ)-1                   ;

/*******************************************************************************
 * Function Name  : setPWM
 * Description    : Sets the PIN's desired autonomous PWM duty cycle
 * Input          : Pin, PWM Value (0-254)
 * Output         : None.
 * Return         : None
 *******************************************************************************/

void                    setPWM          (uint8_t pin, uint8_t value)            ;
