/**
 * @file    usbd_dln2.c
 * @brief   Vendor-specific bulk USB class emulating the Diolan DLN-2:
 *          CTRL handshake, GPIO commands on handle=0x0002 and SPI master
 *          commands on handle=0x0004.
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

/* Largest message either way: 8-byte header + SPI port/size/attr (4) +
 * 256 data bytes on requests, 10-byte header + size (2) + 256 on replies. */
#define DLN2_MSG_MAX            272U

/* A request can span several 64-byte packets and the host sends no ZLP,
 * so packets are appended here until the header's size field is reached.
 * One extra packet of headroom because each receive is armed for 64 bytes. */
static uint8_t  USBD_DLN2_RxBuffer[DLN2_MSG_MAX + DLN2_MAX_PACKET_SIZE];
static uint16_t USBD_DLN2_RxLen;
static uint8_t  USBD_DLN2_TxBuffer[DLN2_MSG_MAX];
static uint16_t USBD_DLN2_TxLen;

/* A complete request waits here for USBD_DLN2_Poll(). The OUT endpoint is
 * not re-armed until its reply has been sent, so there is only ever one. */
static USBD_HandleTypeDef *USBD_DLN2_Dev;
static volatile uint8_t USBD_DLN2_RxReady;

#define DLN2_HANDLE_CTRL        0x0001U
#define DLN2_HANDLE_GPIO        0x0002U
#define DLN2_HANDLE_SPI         0x0004U

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

/* SPI commands, module = 0x02. Only the ones drivers/spi/spi-dln2.c sends. */
#define DLN2_SPI_ENABLE                    0x0211U
#define DLN2_SPI_DISABLE                   0x0212U
#define DLN2_SPI_SET_MODE                  0x0214U
#define DLN2_SPI_SET_FRAME_SIZE            0x0216U
#define DLN2_SPI_SET_FREQUENCY             0x0218U
#define DLN2_SPI_READ_WRITE                0x021AU
#define DLN2_SPI_READ                      0x021BU
#define DLN2_SPI_WRITE                     0x021CU
#define DLN2_SPI_SET_SS                    0x0226U
#define DLN2_SPI_SS_MULTI_ENABLE           0x0238U
#define DLN2_SPI_GET_SUPPORTED_FRAME_SIZES 0x0243U
#define DLN2_SPI_GET_SS_COUNT              0x0244U
#define DLN2_SPI_GET_MIN_FREQUENCY         0x0245U
#define DLN2_SPI_GET_MAX_FREQUENCY         0x0246U

#define DLN2_SPI_MAX_XFER_SIZE    256U
#define DLN2_SPI_ATTR_LEAVE_SS_LOW 0x01U

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

static uint16_t get_le16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void put_le16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static void put_le32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

/* Payload (if any) must already be at TxBuffer[10]. */
static void dln2_reply(USBD_HandleTypeDef *pdev, uint16_t total_size, uint16_t result)
{
  put_le16(&USBD_DLN2_TxBuffer[0], total_size);
  /* Copy id, echo, handle from the request. */
  memcpy(&USBD_DLN2_TxBuffer[2], &USBD_DLN2_RxBuffer[2], 6);
  put_le16(&USBD_DLN2_TxBuffer[8], result);
  USBD_DLN2_TxLen = total_size;
  /* Called from the main loop: keep the USB IRQ out while the transfer is
   * being set up, the HAL does not guard against that. */
  HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  USBD_LL_Transmit(pdev, DLN2_IN_EP, USBD_DLN2_TxBuffer, total_size);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
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

/* All lines start as inputs with pull-up. */
static void gpio_init_all(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  for (uint16_t i = 0U; i < DLN2_N_PINS; i++) {
    gpio_init_pin(i, GPIO_MODE_INPUT, GPIO_PULLUP);
  }
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
  put_le32(&USBD_DLN2_TxBuffer[10], payload);
  dln2_reply(pdev, 14U, DLN2_RESULT_OK);
}

static void handle_gpio(USBD_HandleTypeDef *pdev, uint16_t id, uint32_t len)
{
  /* Requests that take a pin argument have it at offset 8 (after header). */
  uint16_t pin = 0U;
  if (len >= 10U) {
    pin = get_le16(&USBD_DLN2_RxBuffer[8]);
  }

  switch (id) {
  case DLN2_GPIO_GET_PIN_COUNT:
    put_le16(&USBD_DLN2_TxBuffer[10], DLN2_N_PINS);
    dln2_reply(pdev, 12U, DLN2_RESULT_OK);
    return;

  case DLN2_GPIO_SET_DEBOUNCE:    /* no-op; HW debounce not implemented */
  case DLN2_GPIO_PIN_SET_EVENT:   /* no-op; IRQ events not implemented */
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_GPIO_PIN_ENABLE:
  case DLN2_GPIO_PIN_DISABLE:
    /* No-ops: the pin keeps its direction and level while no one holds the
     * line, and the host reads the direction back after enabling it.
     * Resetting it here made every new `gpioset` glitch the output. */
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
    put_le16(&USBD_DLN2_TxBuffer[10], pin);
    USBD_DLN2_TxBuffer[12] = gpio_is_output(pin);
    dln2_reply(pdev, 13U, DLN2_RESULT_OK);
    return;

  case DLN2_GPIO_PIN_GET_VAL:
  case DLN2_GPIO_PIN_GET_OUT_VAL: {
    if (pin >= DLN2_N_PINS) { dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED); return; }
    uint32_t reg = (id == DLN2_GPIO_PIN_GET_VAL)
                   ? dln2_pin_map[pin].port->IDR
                   : dln2_pin_map[pin].port->ODR;
    put_le16(&USBD_DLN2_TxBuffer[10], pin);
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

/* SPI1 master: SCK=PA5, MISO=PA6, MOSI=PA7, single chip select CS0=PA4
 * driven by hand. Polled, 8-bit frames only. */
#define SPI_CS_PORT   GPIOA
#define SPI_CS_PIN    GPIO_PIN_4

static uint32_t spi_cr1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                          (7U << SPI_CR1_BR_Pos);   /* mode 0, slowest */

static void spi_init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

  GPIO_InitTypeDef init = {
    .Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    .Alternate = GPIO_AF5_SPI1,
  };
  HAL_GPIO_Init(GPIOA, &init);

  HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
  init.Pin = SPI_CS_PIN;
  init.Mode = GPIO_MODE_OUTPUT_PP;
  init.Alternate = 0U;
  HAL_GPIO_Init(SPI_CS_PORT, &init);

  SPI1->CR1 = spi_cr1;
}

/* CR1 settings may only change while SPE is clear; the host disables the
 * module before any SET_* command, so just keep SPE as it is. */
static void spi_set_cr1(uint32_t mask, uint32_t value)
{
  spi_cr1 = (spi_cr1 & ~mask) | value;
  SPI1->CR1 = (SPI1->CR1 & SPI_CR1_SPE) | spi_cr1;
}

/* Clocks out len bytes from tx (0x00 if NULL), storing MISO in rx if set. */
static void spi_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len, uint8_t attr)
{
  HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);
  for (uint16_t i = 0U; i < len; i++) {
    while ((SPI1->SR & SPI_SR_TXE) == 0U) {}
    SPI1->DR = tx ? tx[i] : 0x00U;
    while ((SPI1->SR & SPI_SR_RXNE) == 0U) {}
    uint8_t b = (uint8_t)SPI1->DR;
    if (rx) {
      rx[i] = b;
    }
  }
  while ((SPI1->SR & SPI_SR_BSY) != 0U) {}
  if ((attr & DLN2_SPI_ATTR_LEAVE_SS_LOW) == 0U) {
    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
  }
}

static void handle_spi(USBD_HandleTypeDef *pdev, uint16_t id, uint16_t len)
{
  /* Every request starts with a port byte at offset 8; there is only port 0. */
  const uint8_t *req = &USBD_DLN2_RxBuffer[9];
  uint32_t pclk = HAL_RCC_GetPCLK2Freq();

  switch (id) {
  case DLN2_SPI_ENABLE:
    SPI1->CR1 = spi_cr1 | SPI_CR1_SPE;
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_SPI_DISABLE:
    while ((SPI1->SR & SPI_SR_BSY) != 0U) {}
    SPI1->CR1 = spi_cr1;
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_SPI_SET_SS:            /* one CS line, asserted by every transfer */
  case DLN2_SPI_SS_MULTI_ENABLE:
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_SPI_SET_MODE:          /* bit0 = CPHA, bit1 = CPOL, same as CR1 */
    spi_set_cr1(SPI_CR1_CPHA | SPI_CR1_CPOL, req[0] & 0x3U);
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_SPI_SET_FRAME_SIZE:
    dln2_reply(pdev, 10U, (req[0] == 8U) ? DLN2_RESULT_OK : DLN2_RESULT_UNSUPPORTED);
    return;

  case DLN2_SPI_SET_FREQUENCY: {
    /* Round down: pick the smallest divider whose rate does not exceed the
     * request, or the largest divider (/256) if even that is too fast. */
    uint32_t want = (uint32_t)req[0] | ((uint32_t)req[1] << 8) |
                    ((uint32_t)req[2] << 16) | ((uint32_t)req[3] << 24);
    uint32_t br = 0U;
    while (br < 7U && (pclk >> (br + 1U)) > want) {
      br++;
    }
    spi_set_cr1(SPI_CR1_BR, br << SPI_CR1_BR_Pos);
    put_le32(&USBD_DLN2_TxBuffer[10], pclk >> (br + 1U));
    dln2_reply(pdev, 14U, DLN2_RESULT_OK);
    return;
  }

  case DLN2_SPI_GET_SS_COUNT:
    put_le16(&USBD_DLN2_TxBuffer[10], 1U);
    dln2_reply(pdev, 12U, DLN2_RESULT_OK);
    return;

  case DLN2_SPI_GET_MIN_FREQUENCY:
  case DLN2_SPI_GET_MAX_FREQUENCY:
    put_le32(&USBD_DLN2_TxBuffer[10],
             (id == DLN2_SPI_GET_MIN_FREQUENCY) ? (pclk / 256U) : (pclk / 2U));
    dln2_reply(pdev, 14U, DLN2_RESULT_OK);
    return;

  case DLN2_SPI_GET_SUPPORTED_FRAME_SIZES:
    /* The host requires the full 1 + 36 byte table. */
    memset(&USBD_DLN2_TxBuffer[10], 0, 37U);
    USBD_DLN2_TxBuffer[10] = 1U;
    USBD_DLN2_TxBuffer[11] = 8U;
    dln2_reply(pdev, 47U, DLN2_RESULT_OK);
    return;

  case DLN2_SPI_READ_WRITE:
  case DLN2_SPI_READ:
  case DLN2_SPI_WRITE: {
    /* Request: port, size (le16), attr, then data for READ_WRITE/WRITE. */
    uint16_t size = get_le16(&req[0]);
    uint8_t attr = req[2];
    const uint8_t *data = (id == DLN2_SPI_READ) ? NULL : &req[3];
    /* With SPE clear the busy-waits in spi_xfer() would never finish. */
    if ((SPI1->CR1 & SPI_CR1_SPE) == 0U ||
        size > DLN2_SPI_MAX_XFER_SIZE || (data && len < 12U + size)) {
      dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED);
      return;
    }
    if (id == DLN2_SPI_WRITE) {
      spi_xfer(data, NULL, size, attr);
      dln2_reply(pdev, 10U, DLN2_RESULT_OK);
      return;
    }
    /* Reply: size (le16), then the bytes read. */
    spi_xfer(data, &USBD_DLN2_TxBuffer[12], size, attr);
    put_le16(&USBD_DLN2_TxBuffer[10], size);
    dln2_reply(pdev, 12U + size, DLN2_RESULT_OK);
    return;
  }

  default:
    dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED);
    return;
  }
}

static uint8_t USBD_DLN2_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  /* Only once: Init also runs from the USB IRQ when the host reconnects,
   * possibly in the middle of a request in the main loop. Re-initialising
   * then would clear SPE under a running spi_xfer() and hang it, and would
   * reset GPIO outputs. The host re-applies its SPI settings anyway. */
  static uint8_t hw_ready;
  if (!hw_ready) {
    gpio_init_all();
    spi_init();
    hw_ready = 1U;
  }

  USBD_LL_OpenEP(pdev, DLN2_IN_EP, USBD_EP_TYPE_BULK, DLN2_MAX_PACKET_SIZE);
  pdev->ep_in[DLN2_IN_EP & 0x0FU].is_used = 1U;

  USBD_LL_OpenEP(pdev, DLN2_OUT_EP, USBD_EP_TYPE_BULK, DLN2_MAX_PACKET_SIZE);
  pdev->ep_out[DLN2_OUT_EP & 0x0FU].is_used = 1U;

  /* Also runs after a bus reset: drop anything half-received or half-sent. */
  USBD_DLN2_Dev = pdev;
  USBD_DLN2_RxReady = 0U;
  USBD_DLN2_RxLen = 0U;
  USBD_DLN2_TxLen = 0U;
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

/* Reply finished. Replies that end on a packet boundary need a ZLP so the
 * host sees the end of the transfer. Only then accept the next request, so
 * a new reply never starts while the previous one is still going out. */
static uint8_t USBD_DLN2_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(epnum);

  if (USBD_DLN2_TxLen != 0U && (USBD_DLN2_TxLen % DLN2_MAX_PACKET_SIZE) == 0U) {
    USBD_DLN2_TxLen = 0U;
    USBD_LL_Transmit(pdev, DLN2_IN_EP, NULL, 0U);
    return (uint8_t)USBD_OK;
  }

  USBD_DLN2_TxLen = 0U;
  USBD_DLN2_RxLen = 0U;
  USBD_LL_PrepareReceive(pdev, DLN2_OUT_EP, USBD_DLN2_RxBuffer, DLN2_MAX_PACKET_SIZE);
  return (uint8_t)USBD_OK;
}

static uint8_t USBD_DLN2_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_DLN2_RxLen += (uint16_t)USBD_LL_GetRxDataSize(pdev, epnum);

  uint16_t size = (USBD_DLN2_RxLen >= 8U) ? get_le16(USBD_DLN2_RxBuffer) : 0U;

  if (USBD_DLN2_RxLen < 8U || (USBD_DLN2_RxLen < size && size <= DLN2_MSG_MAX)) {
    /* Message incomplete: keep appending. */
    USBD_LL_PrepareReceive(pdev, DLN2_OUT_EP, &USBD_DLN2_RxBuffer[USBD_DLN2_RxLen],
                           DLN2_MAX_PACKET_SIZE);
    return (uint8_t)USBD_OK;
  }

  USBD_DLN2_RxReady = 1U;
  return (uint8_t)USBD_OK;
}

void USBD_DLN2_Poll(void)
{
  if (!USBD_DLN2_RxReady) {
    return;
  }
  USBD_DLN2_RxReady = 0U;

  USBD_HandleTypeDef *pdev = USBD_DLN2_Dev;
  uint16_t size   = get_le16(&USBD_DLN2_RxBuffer[0]);
  uint16_t id     = get_le16(&USBD_DLN2_RxBuffer[2]);
  uint16_t handle = get_le16(&USBD_DLN2_RxBuffer[6]);

  if (size > DLN2_MSG_MAX) {
    /* Too big to hold; answer anyway so the host doesn't wait. */
    dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED);
  } else if (handle == DLN2_HANDLE_CTRL) {
    handle_ctrl(pdev, id);
  } else if (handle == DLN2_HANDLE_GPIO) {
    handle_gpio(pdev, id, size);
  } else if (handle == DLN2_HANDLE_SPI) {
    handle_spi(pdev, id, size);
  } else {
    dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED);
  }
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
