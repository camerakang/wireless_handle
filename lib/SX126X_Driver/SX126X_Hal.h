#ifndef __SX126X_HAL_H__
#define __SX126X_HAL_H__
#ifdef __cplusplus
extern "C"
{
#endif
#include "stdbool.h"

void SX126X_DIO1_INPUT(void);
void SX126X_DIO1_INTENABLE(void);
void SX126X_DIO1_INTDISABLE(void);
bool SX126X_DIO1_GetState(void);
void SX126X_DIO2_INPUT(void);
void SX126X_DIO2_INTENABLE(void);
void SX126X_DIO2_INTDISABLE(void);
bool SX126X_DIO2_GetState(void);
void SX126X_Busy_INPUT(void);
void SX126X_Busy_INTENABLE(void);
void SX126X_Busy_INTDISABLE(void);
bool SX126X_Busy_GetState(void);
void SX126xWaitOnBusy( void );
void SX126X_SWCTL2_OUTPUT(bool PinState);
void SX126X_SWCTL1_OUTPUT(bool PinState);
void IO_SET(bool PinState);
void SX126X_NSS_OUTPUT(bool PinState);
void SX126X_RESET_OUTPUT(bool PinState);
void SX126X_SPIGPIO_Init(void);
void SX126X_SPI_Init(void);
unsigned char SX126X_ReadWriteByte(unsigned char data);

#ifdef __cplusplus
}
#endif
#endif //__SX126X_HAL_H__
