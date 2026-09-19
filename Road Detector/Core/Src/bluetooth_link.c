#include "bluetooth_link.h"

#include <string.h>

static UART_HandleTypeDef *bluetooth_uart = NULL;

void BluetoothLink_Init(UART_HandleTypeDef *uart)
{
  bluetooth_uart = uart;
}

HAL_StatusTypeDef BluetoothLink_Send(const uint8_t *data, uint16_t length)
{
  if ((bluetooth_uart == NULL) || (data == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  /* I frame STATUS possono superare 80 byte a 9600 baud. */
  return HAL_UART_Transmit(bluetooth_uart, (uint8_t *)data, length, 250U);
}

HAL_StatusTypeDef BluetoothLink_SendText(const char *text)
{
  size_t length;

  if (text == NULL)
  {
    return HAL_ERROR;
  }

  length = strlen(text);
  if (length > UINT16_MAX)
  {
    return HAL_ERROR;
  }

  return BluetoothLink_Send((const uint8_t *)text, (uint16_t)length);
}
