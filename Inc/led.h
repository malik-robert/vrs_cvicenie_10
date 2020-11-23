/*
  ******************************************************************************
  * File Name          : led.h
  * Description        : Súbor definuje pomocné prvky ovládania LED.
  ******************************************************************************
*/

#ifndef __LED_H
#define __LED_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
typedef enum {MAN, AUTO} mode;		// Možné módy ovládania LED.
typedef enum {DOWN, UP} state;		// Možné stavy ovládania LED. (DOWN => LED zhasína, UP => LED rozsvecuje)

#define PWM_INCREMENT		1		// Inkrement intenzity svietenia LED [%].
#define PWM_MIN				0		// Minimálna intenzita svietenia LED [%].
#define PWM_MAX				99		// Maximálna intenzita svietenia LED [%].
/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
void setDutyCycle(uint8_t D);
state setLedState(mode led_mode, state led_state, uint8_t intensity, int8_t intensity_set_point);
uint8_t setIntensity(mode led_mode, state led_state, uint8_t intensity, int8_t intensity_set_point);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ led_H */
