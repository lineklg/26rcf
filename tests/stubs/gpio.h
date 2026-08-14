#ifndef TEST_GPIO_H
#define TEST_GPIO_H

#include "main.h"

extern GPIO_TypeDef *PUMP_GPIO_Port;
extern uint16_t PUMP_Pin;
extern GPIO_TypeDef *BM_A_EA_GPIO_Port;
extern uint16_t BM_A_EA_Pin;
extern GPIO_TypeDef *BM_A_EB_GPIO_Port;
extern uint16_t BM_A_EB_Pin;
extern GPIO_TypeDef *BM_B_EA_GPIO_Port;
extern uint16_t BM_B_EA_Pin;
extern GPIO_TypeDef *BM_B_EB_GPIO_Port;
extern uint16_t BM_B_EB_Pin;
extern GPIO_TypeDef *BM_C_EA_GPIO_Port;
extern uint16_t BM_C_EA_Pin;
extern GPIO_TypeDef *BM_C_EB_GPIO_Port;
extern uint16_t BM_C_EB_Pin;
extern GPIO_TypeDef *BM_D_EA_GPIO_Port;
extern uint16_t BM_D_EA_Pin;
extern GPIO_TypeDef *BM_D_EB_GPIO_Port;
extern uint16_t BM_D_EB_Pin;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);

#endif
