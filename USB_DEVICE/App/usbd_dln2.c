/**
 * @file    usbd_dln2.c
 * @brief   Vendor-specific bulk USB class emulating the Diolan DLN-2:
 *          CTRL handshake + GPIO commands on handle=0x0002.
 *          Unknown commands reply result=0x81 to fail fast on the host.
 */

#include <string.h>

#include "usbd_dln2.h"
#include "usbd_ctlreq.h"
#include "main.h"

#define DLN2_CONFIG_DESC_SIZE           32U

static uint8_t USBD_DLN2_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_DLN2_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_DLN2_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_DLN2_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_DLN2_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t *USBD_DLN2_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_DLN2_GetDeviceQualifierDesc(uint16_t *length);

USBD_ClassTypeDef USBD_DLN2 = {
  USBD_DLN2_Init,
  USBD_DLN2_DeInit,
  USBD_DLN2_Setup,
  NULL,                            /* EP0_TxSent */
  NULL,                            /* EP0_RxReady */
  USBD_DLN2_DataIn,
  USBD_DLN2_DataOut,
  NULL,                            /* SOF */
  NULL,                            /* IsoINIncomplete */
  NULL,                            /* IsoOUTIncomplete */
  USBD_DLN2_GetFSCfgDesc,          /* HS */
  USBD_DLN2_GetFSCfgDesc,          /* FS */
  USBD_DLN2_GetFSCfgDesc,          /* Other Speed */
  USBD_DLN2_GetDeviceQualifierDesc,
};

__ALIGN_BEGIN static uint8_t USBD_DLN2_CfgDesc[DLN2_CONFIG_DESC_SIZE] __ALIGN_END = {
  /* Configuration Descriptor */
  0x09,                            /* bLength */
  USB_DESC_TYPE_CONFIGURATION,     /* bDescriptorType */
  LOBYTE(DLN2_CONFIG_DESC_SIZE),   /* wTotalLength */
  HIBYTE(DLN2_CONFIG_DESC_SIZE),
  0x01,                            /* bNumInterfaces */
  0x01,                            /* bConfigurationValue */
  0x00,                            /* iConfiguration */
  0xC0,                            /* bmAttributes: self-powered */
  0x32,                            /* bMaxPower: 100 mA */

  /* Interface Descriptor */
  0x09,                            /* bLength */
  USB_DESC_TYPE_INTERFACE,         /* bDescriptorType */
  0x00,                            /* bInterfaceNumber */
  0x00,                            /* bAlternateSetting */
  0x02,                            /* bNumEndpoints */
  0xFF,                            /* bInterfaceClass: Vendor Specific */
  0x00,                            /* bInterfaceSubClass */
  0x00,                            /* bInterfaceProtocol */
  0x00,                            /* iInterface */

  /* Endpoint OUT Descriptor */
  0x07,                            /* bLength */
  USB_DESC_TYPE_ENDPOINT,          /* bDescriptorType */
  DLN2_OUT_EP,                     /* bEndpointAddress */
  0x02,                            /* bmAttributes: Bulk */
  LOBYTE(DLN2_MAX_PACKET_SIZE),
  HIBYTE(DLN2_MAX_PACKET_SIZE),
  0x00,                            /* bInterval (ignored for bulk) */

  /* Endpoint IN Descriptor */
  0x07,
  USB_DESC_TYPE_ENDPOINT,
  DLN2_IN_EP,
  0x02,                            /* Bulk */
  LOBYTE(DLN2_MAX_PACKET_SIZE),
  HIBYTE(DLN2_MAX_PACKET_SIZE),
  0x00,
};

__ALIGN_BEGIN static uint8_t USBD_DLN2_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00, 0x02,
  0x00, 0x00, 0x00,
  0x40,
  0x01,
  0x00,
};

static uint8_t USBD_DLN2_RxBuffer[DLN2_MAX_PACKET_SIZE];
static uint8_t USBD_DLN2_TxBuffer[16];

#define DLN2_HANDLE_CTRL        0x0001U
#define DLN2_HANDLE_GPIO        0x0002U

#define DLN2_CMD_GET_DEVICE_VER 0x0030U
#define DLN2_CMD_GET_DEVICE_SN  0x0031U
#define DLN2_HW_ID              0x00000200U

/* GPIO commands: wire id = (cmd | (module << 8)), module = 0x01 for GPIO. */
#define DLN2_GPIO_GET_PIN_COUNT   0x0101U
#define DLN2_GPIO_SET_DEBOUNCE    0x0104U
#define DLN2_GPIO_PIN_GET_VAL     0x010BU
#define DLN2_GPIO_PIN_SET_OUT_VAL 0x010CU
#define DLN2_GPIO_PIN_GET_OUT_VAL 0x010DU
#define DLN2_GPIO_PIN_ENABLE      0x0110U
#define DLN2_GPIO_PIN_DISABLE     0x0111U
#define DLN2_GPIO_PIN_SET_DIR     0x0113U
#define DLN2_GPIO_PIN_GET_DIR     0x0114U
#define DLN2_GPIO_PIN_SET_EVENT   0x011EU

#define DLN2_RESULT_OK            0x0000U
#define DLN2_RESULT_UNSUPPORTED   0x0081U   /* any value > 0x80 fails on host */

#define DLN2_N_PINS               17U

static const struct {
  GPIO_TypeDef *port;
  uint8_t       bit;
} dln2_pin_map[DLN2_N_PINS] = {
  {GPIOC, 13},  /* pin  0: onboard LED (active-low) */
  {GPIOB,  0},  /* pin  1 */
  {GPIOB,  1},  /* pin  2 */
  {GPIOB,  2},  /* pin  3 */
  {GPIOB,  3},  /* pin  4 */
  {GPIOB,  4},  /* pin  5 */
  {GPIOB,  5},  /* pin  6 */
  {GPIOB,  6},  /* pin  7 */
  {GPIOB,  7},  /* pin  8 */
  {GPIOB,  8},  /* pin  9 */
  {GPIOB,  9},  /* pin 10 */
  {GPIOB, 10},  /* pin 11 */
  {GPIOB, 11},  /* pin 12 */
  {GPIOB, 12},  /* pin 13 */
  {GPIOB, 13},  /* pin 14 */
  {GPIOB, 14},  /* pin 15 */
  {GPIOB, 15},  /* pin 16 */
};

static void dln2_reply(USBD_HandleTypeDef *pdev, uint16_t total_size, uint16_t result)
{
  USBD_DLN2_TxBuffer[0] = (uint8_t)total_size;
  USBD_DLN2_TxBuffer[1] = (uint8_t)(total_size >> 8);
  /* Copy id, echo, handle from the request. */
  memcpy(&USBD_DLN2_TxBuffer[2], &USBD_DLN2_RxBuffer[2], 6);
  USBD_DLN2_TxBuffer[8] = (uint8_t)result;
  USBD_DLN2_TxBuffer[9] = (uint8_t)(result >> 8);
  USBD_LL_Transmit(pdev, DLN2_IN_EP, USBD_DLN2_TxBuffer, total_size);
}

static void gpio_init_pin(uint16_t pin_idx, uint32_t mode, uint32_t pull)
{
  GPIO_InitTypeDef init = {
    .Pin = (uint16_t)(1U << dln2_pin_map[pin_idx].bit),
    .Mode = mode,
    .Pull = pull,
    .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(dln2_pin_map[pin_idx].port, &init);
}

static uint8_t gpio_is_output(uint16_t pin_idx)
{
  /* MODER stores 2 bits per pin; 0b01 = general-purpose output. */
  uint32_t mode = (dln2_pin_map[pin_idx].port->MODER >>
                   (2U * dln2_pin_map[pin_idx].bit)) & 0x3U;
  return (mode == 0x1U) ? 1U : 0U;
}

static void handle_ctrl(USBD_HandleTypeDef *pdev, uint16_t id)
{
  uint32_t payload;
  if (id == DLN2_CMD_GET_DEVICE_VER) {
    payload = DLN2_HW_ID;
  } else if (id == DLN2_CMD_GET_DEVICE_SN) {
    payload = *(uint32_t *)UID_BASE;
  } else {
    dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED);
    return;
  }
  USBD_DLN2_TxBuffer[10] = (uint8_t)payload;
  USBD_DLN2_TxBuffer[11] = (uint8_t)(payload >> 8);
  USBD_DLN2_TxBuffer[12] = (uint8_t)(payload >> 16);
  USBD_DLN2_TxBuffer[13] = (uint8_t)(payload >> 24);
  dln2_reply(pdev, 14U, DLN2_RESULT_OK);
}

static void handle_gpio(USBD_HandleTypeDef *pdev, uint16_t id, uint32_t len)
{
  /* Requests that take a pin argument have it at offset 8 (after header). */
  uint16_t pin = 0U;
  if (len >= 10U) {
    pin = (uint16_t)USBD_DLN2_RxBuffer[8] |
          ((uint16_t)USBD_DLN2_RxBuffer[9] << 8);
  }

  switch (id) {
  case DLN2_GPIO_GET_PIN_COUNT:
    USBD_DLN2_TxBuffer[10] = (uint8_t)DLN2_N_PINS;
    USBD_DLN2_TxBuffer[11] = 0U;
    dln2_reply(pdev, 12U, DLN2_RESULT_OK);
    return;

  case DLN2_GPIO_SET_DEBOUNCE:    /* no-op; HW debounce not implemented */
  case DLN2_GPIO_PIN_SET_EVENT:   /* no-op; IRQ events not implemented */
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_GPIO_PIN_ENABLE:
    if (pin >= DLN2_N_PINS) { dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED); return; }
    gpio_init_pin(pin, GPIO_MODE_INPUT, GPIO_PULLUP);
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_GPIO_PIN_DISABLE:
    /* No-op so that pin state persists after `gpioset` exits and the
     * kernel releases the line; otherwise the LED would blink once. */
    if (pin >= DLN2_N_PINS) { dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED); return; }
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_GPIO_PIN_SET_DIR:
    if (pin >= DLN2_N_PINS) { dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED); return; }
    if (len < 11U) { dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED); return; }
    if (USBD_DLN2_RxBuffer[10] == 1U) {
      gpio_init_pin(pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    } else {
      gpio_init_pin(pin, GPIO_MODE_INPUT, GPIO_PULLUP);
    }
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_GPIO_PIN_GET_DIR:
    if (pin >= DLN2_N_PINS) { dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED); return; }
    USBD_DLN2_TxBuffer[10] = (uint8_t)pin;
    USBD_DLN2_TxBuffer[11] = (uint8_t)(pin >> 8);
    USBD_DLN2_TxBuffer[12] = gpio_is_output(pin);
    dln2_reply(pdev, 13U, DLN2_RESULT_OK);
    return;

  case DLN2_GPIO_PIN_GET_VAL:
  case DLN2_GPIO_PIN_GET_OUT_VAL: {
    if (pin >= DLN2_N_PINS) { dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED); return; }
    uint32_t reg = (id == DLN2_GPIO_PIN_GET_VAL)
                   ? dln2_pin_map[pin].port->IDR
                   : dln2_pin_map[pin].port->ODR;
    USBD_DLN2_TxBuffer[10] = (uint8_t)pin;
    USBD_DLN2_TxBuffer[11] = (uint8_t)(pin >> 8);
    USBD_DLN2_TxBuffer[12] = (uint8_t)((reg >> dln2_pin_map[pin].bit) & 1U);
    dln2_reply(pdev, 13U, DLN2_RESULT_OK);
    return;
  }

  case DLN2_GPIO_PIN_SET_OUT_VAL:
    if (pin >= DLN2_N_PINS) { dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED); return; }
    if (len < 11U) { dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED); return; }
    HAL_GPIO_WritePin(dln2_pin_map[pin].port,
                      (uint16_t)(1U << dln2_pin_map[pin].bit),
                      USBD_DLN2_RxBuffer[10] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  default:
    dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED);
    return;
  }
}

static uint8_t USBD_DLN2_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  USBD_LL_OpenEP(pdev, DLN2_IN_EP, USBD_EP_TYPE_BULK, DLN2_MAX_PACKET_SIZE);
  pdev->ep_in[DLN2_IN_EP & 0x0FU].is_used = 1U;

  USBD_LL_OpenEP(pdev, DLN2_OUT_EP, USBD_EP_TYPE_BULK, DLN2_MAX_PACKET_SIZE);
  pdev->ep_out[DLN2_OUT_EP & 0x0FU].is_used = 1U;

  USBD_LL_PrepareReceive(pdev, DLN2_OUT_EP, USBD_DLN2_RxBuffer, DLN2_MAX_PACKET_SIZE);

  return (uint8_t)USBD_OK;
}

static uint8_t USBD_DLN2_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_LL_CloseEP(pdev, DLN2_IN_EP);
  pdev->ep_in[DLN2_IN_EP & 0x0FU].is_used = 0U;

  USBD_LL_CloseEP(pdev, DLN2_OUT_EP);
  pdev->ep_out[DLN2_OUT_EP & 0x0FU].is_used = 0U;

  return (uint8_t)USBD_OK;
}

static uint8_t USBD_DLN2_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  switch (req->bmRequest & USB_REQ_TYPE_MASK) {
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest) {
    case USB_REQ_GET_STATUS:
      if (pdev->dev_state == USBD_STATE_CONFIGURED) {
        uint16_t status = 0U;
        USBD_CtlSendData(pdev, (uint8_t *)&status, 2U);
      } else {
        USBD_CtlError(pdev, req);
        return (uint8_t)USBD_FAIL;
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      return (uint8_t)USBD_FAIL;
    }
    break;

  case USB_REQ_TYPE_CLASS:
  case USB_REQ_TYPE_VENDOR:
    /* No class/vendor requests defined yet: stall. */
    USBD_CtlError(pdev, req);
    return (uint8_t)USBD_FAIL;

  default:
    USBD_CtlError(pdev, req);
    return (uint8_t)USBD_FAIL;
  }
  return (uint8_t)USBD_OK;
}

static uint8_t USBD_DLN2_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(pdev);
  UNUSED(epnum);
  return (uint8_t)USBD_OK;
}

static uint8_t USBD_DLN2_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint32_t len = USBD_LL_GetRxDataSize(pdev, epnum);

  if (len >= 8U) {
    uint16_t id     = (uint16_t)USBD_DLN2_RxBuffer[2] |
                      ((uint16_t)USBD_DLN2_RxBuffer[3] << 8);
    uint16_t handle = (uint16_t)USBD_DLN2_RxBuffer[6] |
                      ((uint16_t)USBD_DLN2_RxBuffer[7] << 8);

    if (handle == DLN2_HANDLE_CTRL) {
      handle_ctrl(pdev, id);
    } else if (handle == DLN2_HANDLE_GPIO) {
      handle_gpio(pdev, id, len);
    } else {
      dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED);
    }
  }

  USBD_LL_PrepareReceive(pdev, DLN2_OUT_EP, USBD_DLN2_RxBuffer, DLN2_MAX_PACKET_SIZE);
  return (uint8_t)USBD_OK;
}

static uint8_t *USBD_DLN2_GetFSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_DLN2_CfgDesc);
  return USBD_DLN2_CfgDesc;
}

static uint8_t *USBD_DLN2_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_DLN2_DeviceQualifierDesc);
  return USBD_DLN2_DeviceQualifierDesc;
}
