#ifndef BLUETOOTH_LINK_H
#define BLUETOOTH_LINK_H

#include "main.h"

/*
 * Transport UART per il modulo Bluetooth seriale/BLE.
 * L'algoritmo stradale potrà inviare in futuro frame STATUS ed EVENT tramite
 * queste funzioni senza dipendere dall'handle UART concreto.
 */
void BluetoothLink_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef BluetoothLink_Send(const uint8_t *data, uint16_t length);
HAL_StatusTypeDef BluetoothLink_SendText(const char *text);

#endif /* BLUETOOTH_LINK_H */
