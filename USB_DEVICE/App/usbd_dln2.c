/**
 * @file    usbd_dln2.c
 * @brief   Vendor-specific bulk USB class emulating the Diolan DLN-2:
 *          CTRL handshake, GPIO commands and events on handle=0x0002/0x0000
 *          and SPI master commands on handle=0x0004.
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

/* What the IN endpoint is sending. Replies and GPIO events share it, and
 * only the end of a reply may re-arm OUT for the next request. */
#define TX_IDLE                 0U
#define TX_REPLY                1U
#define TX_EVENT                2U
static volatile uint8_t USBD_DLN2_TxKind;

/* Set by Init (USB IRQ), handled by the main loop, which owns the event
 * state below. */
static volatile uint8_t USBD_DLN2_Reconfigured;

#define DLN2_HANDLE_EVENT       0x0000U
#define DLN2_HANDLE_CTRL        0x0001U
#define DLN2_HANDLE_GPIO        0x0002U
#define DLN2_HANDLE_SPI         0x0004U
#define DLN2_HANDLE_ADC         0x0005U

/* ADC commands, module = 0x06 (handle 5). Only the ones
 * drivers/iio/adc/dln2-adc.c sends. */
#define DLN2_ADC_GET_CHANNEL_COUNT   0x0601U
#define DLN2_ADC_ENABLE              0x0602U
#define DLN2_ADC_DISABLE             0x0603U
#define DLN2_ADC_CHANNEL_ENABLE      0x0605U
#define DLN2_ADC_CHANNEL_DISABLE     0x0606U
#define DLN2_ADC_SET_RESOLUTION      0x0608U
#define DLN2_ADC_CHANNEL_GET_VAL     0x060AU
#define DLN2_ADC_CHANNEL_GET_ALL_VAL 0x060BU
#define DLN2_ADC_CHANNEL_SET_CFG     0x060CU
#define DLN2_ADC_CONDITION_MET_EV    0x0610U
#define DLN2_ADC_EVENT_NONE          0U
#define DLN2_ADC_EVENT_ALWAYS        5U

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
#define DLN2_GPIO_CONDITION_MET_EV 0x010FU
#define DLN2_GPIO_PIN_SET_EVENT   0x011EU

/* Event types; the host maps rising/falling edges to CHANGE and filters
 * them itself using the reported value. */
#define DLN2_GPIO_EVENT_NONE      0U
#define DLN2_GPIO_EVENT_CHANGE    1U
#define DLN2_GPIO_EVENT_LVL_HIGH  2U
#define DLN2_GPIO_EVENT_LVL_LOW   3U

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
#define DLN2_SPI_SS_MULTI_DISABLE          0x0239U
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

static void dln2_transmit(USBD_HandleTypeDef *pdev, uint8_t kind, uint8_t *buf, uint16_t len)
{
  /* Called from the main loop: keep the USB IRQ out while the transfer is
   * being set up, the HAL does not guard against that. */
  HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  USBD_DLN2_TxKind = kind;
  USBD_DLN2_TxLen = len;
  USBD_LL_Transmit(pdev, DLN2_IN_EP, buf, len);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

/* Payload (if any) must already be at TxBuffer[10]. */
static void dln2_reply(USBD_HandleTypeDef *pdev, uint16_t total_size, uint16_t result)
{
  put_le16(&USBD_DLN2_TxBuffer[0], total_size);
  /* Copy id, echo, handle from the request. */
  memcpy(&USBD_DLN2_TxBuffer[2], &USBD_DLN2_RxBuffer[2], 6);
  put_le16(&USBD_DLN2_TxBuffer[8], result);
  dln2_transmit(pdev, TX_REPLY, USBD_DLN2_TxBuffer, total_size);
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
  /* Cycle counter for event debounce timing. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint8_t gpio_is_output(uint16_t pin_idx)
{
  /* MODER stores 2 bits per pin; 0b01 = general-purpose output. */
  uint32_t mode = (dln2_pin_map[pin_idx].port->MODER >>
                   (2U * dln2_pin_map[pin_idx].bit)) & 0x3U;
  return (mode == 0x1U) ? 1U : 0U;
}

/* Level of every line as a bitmask, bit n = pin n (PC13, then PB0..PB15). */
static uint32_t gpio_levels(void)
{
  return ((GPIOC->IDR >> 13) & 1U) | ((GPIOB->IDR & 0xFFFFU) << 1);
}

/* GPIO events. Lines are sampled on every main loop pass rather than with
 * EXTI, because PC13 and PB13 would both need EXTI line 13. A pulse shorter
 * than one pass (up to a few ms during slow SPI transfers) can be missed.
 * All of this is only touched from the main loop. */
#define EVT_QUEUE_LEN 32U

static uint8_t  evt_type[DLN2_N_PINS];
static uint32_t evt_enabled;     /* bitmask of pins with an event type set */
static uint32_t evt_raw;         /* levels at the previous pass */
static uint32_t evt_stable;      /* debounced levels, what events report */
static uint32_t evt_since[DLN2_N_PINS];   /* DWT CYCCNT of last raw change */
static uint32_t evt_debounce;    /* in CPU cycles; 0 = off */
static uint16_t evt_count;       /* running counter reported to the host */
static struct { uint8_t pin, value; } evt_queue[EVT_QUEUE_LEN];
static uint8_t  evt_head, evt_tail;
static uint8_t  USBD_DLN2_EvtBuffer[14];

/* Full queue drops the event: the host reads the line's value itself when
 * it handles an edge, so a lost one only merges two edges. */
static void evt_push(uint16_t pin, uint32_t levels)
{
  uint8_t next = (uint8_t)((evt_head + 1U) % EVT_QUEUE_LEN);
  if (next != evt_tail) {
    evt_queue[evt_head].pin = (uint8_t)pin;
    evt_queue[evt_head].value = (uint8_t)((levels >> pin) & 1U);
    evt_head = next;
  }
}

static uint8_t evt_condition_met(uint16_t pin, uint32_t levels)
{
  uint32_t level = (levels >> pin) & 1U;
  return (evt_type[pin] == DLN2_GPIO_EVENT_CHANGE) ||
         (evt_type[pin] == DLN2_GPIO_EVENT_LVL_HIGH && level) ||
         (evt_type[pin] == DLN2_GPIO_EVENT_LVL_LOW && !level);
}

static void evt_set(uint16_t pin, uint8_t type)
{
  uint32_t levels = gpio_levels();
  evt_type[pin] = type;
  if (type == DLN2_GPIO_EVENT_NONE) {
    evt_enabled &= ~(1UL << pin);
    /* The kernel never sends debounce 0 when a debounced line is released,
     * and sends SET_DEBOUNCE before enabling events, so forget it here. */
    if (evt_enabled == 0U) {
      evt_debounce = 0U;
    }
    return;
  }
  uint32_t bit = 1UL << pin;
  evt_enabled |= bit;
  evt_raw = (evt_raw & ~bit) | (levels & bit);
  evt_stable = (evt_stable & ~bit) | (levels & bit);
  /* A level condition that already holds fires right away. */
  if (type != DLN2_GPIO_EVENT_CHANGE && evt_condition_met(pin, levels)) {
    evt_push(pin, levels);
  }
}

/* SET_DEBOUNCE carries microseconds (the kernel passes the pinconf value
 * as-is) and applies to all lines. Capped so the cycle count fits in 32 bits
 * with CYCCNT wrapping every ~44 s at 96 MHz. */
static void evt_set_debounce(uint32_t us)
{
  if (us > 40000000UL) {
    us = 40000000UL;
  }
  evt_debounce = us * (SystemCoreClock / 1000000U);
}

/* A new level counts once it has held for the debounce time (immediately
 * when debounce is off). */
static void evt_scan(void)
{
  uint32_t levels = gpio_levels();
  uint32_t now = DWT->CYCCNT;

  uint32_t moved = (levels ^ evt_raw) & evt_enabled;
  evt_raw = levels;
  while (moved != 0U) {
    uint16_t pin = (uint16_t)__builtin_ctz(moved);
    moved &= moved - 1U;
    evt_since[pin] = now;
  }

  uint32_t pending = (levels ^ evt_stable) & evt_enabled;
  while (pending != 0U) {
    uint16_t pin = (uint16_t)__builtin_ctz(pending);
    pending &= pending - 1U;
    if (now - evt_since[pin] >= evt_debounce) {
      evt_stable ^= 1UL << pin;
      if (evt_condition_met(pin, evt_stable)) {
        evt_push(pin, evt_stable);
      }
    }
  }
}

static void evt_send(USBD_HandleTypeDef *pdev)
{
  uint8_t *b = USBD_DLN2_EvtBuffer;
  /* Header without a result field, then count, type, pin, value. */
  put_le16(&b[0], sizeof(USBD_DLN2_EvtBuffer));
  put_le16(&b[2], DLN2_GPIO_CONDITION_MET_EV);
  put_le16(&b[4], 0U);
  put_le16(&b[6], DLN2_HANDLE_EVENT);
  put_le16(&b[8], ++evt_count);
  b[10] = evt_type[evt_queue[evt_tail].pin];
  put_le16(&b[11], evt_queue[evt_tail].pin);
  b[13] = evt_queue[evt_tail].value;
  evt_tail = (uint8_t)((evt_tail + 1U) % EVT_QUEUE_LEN);
  dln2_transmit(pdev, TX_EVENT, b, sizeof(USBD_DLN2_EvtBuffer));
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

  case DLN2_GPIO_SET_DEBOUNCE:    /* request: duration (le32, µs), no pin */
    if (len < 12U) { dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED); return; }
    evt_set_debounce((uint32_t)USBD_DLN2_RxBuffer[8] |
                     ((uint32_t)USBD_DLN2_RxBuffer[9] << 8) |
                     ((uint32_t)USBD_DLN2_RxBuffer[10] << 16) |
                     ((uint32_t)USBD_DLN2_RxBuffer[11] << 24));
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_GPIO_PIN_SET_EVENT: {
    /* Request: pin (le16), type, period (le16). A non-zero period asks for
     * repeated level events; the kernel always sends 0, so it's ignored. */
    uint8_t type = (len >= 11U) ? USBD_DLN2_RxBuffer[10] : 0xFFU;
    if (pin >= DLN2_N_PINS || type > DLN2_GPIO_EVENT_LVL_LOW) {
      dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED);
      return;
    }
    evt_set(pin, type);
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;
  }

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

/* SPI1 master: SCK=PA5, MISO=PA6, MOSI=PA7. Chip selects CS0..CS3 =
 * PA4, PA8, PA9, PA10, driven by hand. Polled, 8- or 16-bit frames. */
#define SPI_N_CS      4U
static const uint8_t spi_cs_bit[SPI_N_CS] = {4, 8, 9, 10};   /* on GPIOA */
static uint16_t spi_cs_all;       /* GPIOA pin mask of all CS lines */
static uint8_t  spi_cs_selected = 0x01U;   /* CS indices, from SET_SS */
static uint8_t  spi_cs_enabled;            /* CS indices, from SS_MULTI_* */

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

  for (uint8_t i = 0U; i < SPI_N_CS; i++) {
    spi_cs_all |= (uint16_t)(1U << spi_cs_bit[i]);
  }
  GPIOA->BSRR = spi_cs_all;       /* deselected (high) before driving */
  init.Pin = spi_cs_all;
  init.Mode = GPIO_MODE_OUTPUT_PP;
  init.Alternate = 0U;
  HAL_GPIO_Init(GPIOA, &init);

  SPI1->CR1 = spi_cr1;
}

/* GPIOA pin mask of the lines a transfer should pull low. */
static uint16_t spi_cs_active(void)
{
  uint16_t pins = 0U;
  uint8_t cs = spi_cs_selected & spi_cs_enabled;
  for (uint8_t i = 0U; i < SPI_N_CS; i++) {
    if (cs & (1U << i)) {
      pins |= (uint16_t)(1U << spi_cs_bit[i]);
    }
  }
  return pins;
}

/* CR1 settings may only change while SPE is clear; the host disables the
 * module before any SET_* command, so just keep SPE as it is. */
static void spi_set_cr1(uint32_t mask, uint32_t value)
{
  spi_cr1 = (spi_cr1 & ~mask) | value;
  SPI1->CR1 = (SPI1->CR1 & SPI_CR1_SPE) | spi_cr1;
}

/* Clocks out len bytes from tx (0x00 if NULL), storing MISO in rx if set.
 * In 16-bit mode each frame is two little-endian bytes; len is even. */
static void spi_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len, uint8_t attr)
{
  uint16_t step = (spi_cr1 & SPI_CR1_DFF) ? 2U : 1U;

  GPIOA->BSRR = (uint32_t)spi_cs_active() << 16;   /* active low */
  for (uint16_t i = 0U; i < len; i += step) {
    uint16_t w = 0U;
    if (tx) {
      w = (step == 2U) ? get_le16(&tx[i]) : tx[i];
    }
    while ((SPI1->SR & SPI_SR_TXE) == 0U) {}
    SPI1->DR = w;
    while ((SPI1->SR & SPI_SR_RXNE) == 0U) {}
    w = (uint16_t)SPI1->DR;
    if (rx) {
      rx[i] = (uint8_t)w;
      if (step == 2U) {
        rx[i + 1U] = (uint8_t)(w >> 8);
      }
    }
  }
  while ((SPI1->SR & SPI_SR_BSY) != 0U) {}
  if ((attr & DLN2_SPI_ATTR_LEAVE_SS_LOW) == 0U) {
    GPIOA->BSRR = spi_cs_all;
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

  case DLN2_SPI_SET_SS:            /* a 0 bit selects that CS line */
    spi_cs_selected = (uint8_t)~req[0] & ((1U << SPI_N_CS) - 1U);
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_SPI_SS_MULTI_ENABLE:
  case DLN2_SPI_SS_MULTI_DISABLE:
    if (id == DLN2_SPI_SS_MULTI_ENABLE) {
      spi_cs_enabled |= req[0] & ((1U << SPI_N_CS) - 1U);
    } else {
      spi_cs_enabled &= (uint8_t)~req[0];
    }
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_SPI_SET_MODE:          /* bit0 = CPHA, bit1 = CPOL, same as CR1 */
    spi_set_cr1(SPI_CR1_CPHA | SPI_CR1_CPOL, req[0] & 0x3U);
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_SPI_SET_FRAME_SIZE:
    if (req[0] != 8U && req[0] != 16U) {
      dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED);
      return;
    }
    spi_set_cr1(SPI_CR1_DFF, (req[0] == 16U) ? SPI_CR1_DFF : 0U);
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
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
    put_le16(&USBD_DLN2_TxBuffer[10], SPI_N_CS);
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
    USBD_DLN2_TxBuffer[10] = 2U;
    USBD_DLN2_TxBuffer[11] = 8U;
    USBD_DLN2_TxBuffer[12] = 16U;
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
        size > DLN2_SPI_MAX_XFER_SIZE || (data && len < 12U + size) ||
        ((spi_cr1 & SPI_CR1_DFF) && (size & 1U))) {
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

/* ADC1, 10-bit. Channels 0-3 = PA0-PA3, 4 = VREFINT, 5 = temperature
 * sensor. The ADC stays powered: the host enables/disables it around every
 * single read, and waking it each time would cost its startup delay. */
#define ADC_N_CH      6U
static const uint8_t adc_hw_ch[ADC_N_CH] = {0, 1, 2, 3, 17, 18};
static uint8_t  adc_ch_enabled;             /* bitmask, from CHANNEL_ENABLE */
static uint16_t adc_period[ADC_N_CH];       /* ms; 0 = no periodic event */
static uint32_t adc_next[ADC_N_CH];         /* HAL_GetTick() deadline */
static uint8_t  adc_due;                    /* bitmask of events to send */
static uint16_t adc_evt_count;
static uint8_t  USBD_DLN2_AdcEvtBuffer[15];

static void adc_init(void)
{
  __HAL_RCC_ADC1_CLK_ENABLE();

  GPIO_InitTypeDef init = {
    .Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
    .Mode = GPIO_MODE_ANALOG,
    .Pull = GPIO_NOPULL,
  };
  HAL_GPIO_Init(GPIOA, &init);

  /* 96 MHz / 6 = 16 MHz ADC clock (max 18). VREFINT + temp sensor on. */
  ADC->CCR = ADC_CCR_ADCPRE_1 | ADC_CCR_TSVREFE;
  ADC1->CR1 = ADC_CR1_RES_0;                 /* 10-bit */
  /* 480-cycle sampling (~30 µs per conversion) on every channel: the
   * internal ones need >= 10 µs, and it suits high-impedance sources. */
  ADC1->SMPR1 = 0x07FFFFFFU;
  ADC1->SMPR2 = 0x3FFFFFFFU;
  ADC1->CR2 = ADC_CR2_ADON;
}

static uint16_t adc_convert(uint8_t ch)
{
  ADC1->SQR3 = adc_hw_ch[ch];
  ADC1->SR = 0U;
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while ((ADC1->SR & ADC_SR_EOC) == 0U) {}
  return (uint16_t)ADC1->DR;
}

/* Marks channels whose period has elapsed. Deadlines advance by whole
 * periods; a host that falls far behind gets one event, not a burst. */
static void adc_scan(void)
{
  uint32_t now = HAL_GetTick();
  for (uint8_t ch = 0U; ch < ADC_N_CH; ch++) {
    if (adc_period[ch] != 0U && now - adc_next[ch] < 0x80000000UL) {
      adc_due |= (uint8_t)(1U << ch);
      adc_next[ch] += adc_period[ch];
      if (now - adc_next[ch] < 0x80000000UL) {
        adc_next[ch] = now + adc_period[ch];
      }
    }
  }
}

static void adc_evt_send(USBD_HandleTypeDef *pdev)
{
  uint8_t ch = (uint8_t)__builtin_ctz(adc_due);
  uint8_t *b = USBD_DLN2_AdcEvtBuffer;
  adc_due &= (uint8_t)~(1U << ch);
  /* Header without a result field, then count, port, channel, value,
   * type. The kernel only uses the event as a trigger. */
  put_le16(&b[0], sizeof(USBD_DLN2_AdcEvtBuffer));
  put_le16(&b[2], DLN2_ADC_CONDITION_MET_EV);
  put_le16(&b[4], 0U);
  put_le16(&b[6], DLN2_HANDLE_EVENT);
  put_le16(&b[8], ++adc_evt_count);
  b[10] = 0U;
  b[11] = ch;
  put_le16(&b[12], adc_convert(ch));
  b[14] = DLN2_ADC_EVENT_ALWAYS;
  dln2_transmit(pdev, TX_EVENT, b, sizeof(USBD_DLN2_AdcEvtBuffer));
}

static void handle_adc(USBD_HandleTypeDef *pdev, uint16_t id, uint16_t len)
{
  /* Requests: port (only 0), then channel for per-channel commands. */
  const uint8_t *req = &USBD_DLN2_RxBuffer[8];
  uint8_t ch = (len >= 10U) ? req[1] : 0xFFU;

  switch (id) {
  case DLN2_ADC_SET_RESOLUTION:            /* port, bits; the kernel sends 10 */
    dln2_reply(pdev, 10U, (ch == 10U) ? DLN2_RESULT_OK : DLN2_RESULT_UNSUPPORTED);
    return;

  case DLN2_ADC_GET_CHANNEL_COUNT:
    USBD_DLN2_TxBuffer[10] = ADC_N_CH;
    dln2_reply(pdev, 11U, DLN2_RESULT_OK);
    return;

  case DLN2_ADC_ENABLE:                    /* reply: pin conflict mask, none */
  case DLN2_ADC_DISABLE:
    put_le16(&USBD_DLN2_TxBuffer[10], 0U);
    dln2_reply(pdev, 12U, DLN2_RESULT_OK);
    return;

  case DLN2_ADC_CHANNEL_GET_ALL_VAL: {
    /* Reply: enabled mask, then 8 values (unused/disabled ones 0). */
    put_le16(&USBD_DLN2_TxBuffer[10], adc_ch_enabled);
    for (uint8_t i = 0U; i < 8U; i++) {
      uint16_t v = (i < ADC_N_CH && (adc_ch_enabled & (1U << i))) ? adc_convert(i) : 0U;
      put_le16(&USBD_DLN2_TxBuffer[12U + 2U * i], v);
    }
    dln2_reply(pdev, 28U, DLN2_RESULT_OK);
    return;
  }

  default:
    break;
  }

  if (ch >= ADC_N_CH) {
    dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED);
    return;
  }

  switch (id) {
  case DLN2_ADC_CHANNEL_ENABLE:
    adc_ch_enabled |= (uint8_t)(1U << ch);
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_ADC_CHANNEL_DISABLE:
    adc_ch_enabled &= (uint8_t)~(1U << ch);
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
    return;

  case DLN2_ADC_CHANNEL_GET_VAL:
    put_le16(&USBD_DLN2_TxBuffer[10], adc_convert(ch));
    dln2_reply(pdev, 12U, DLN2_RESULT_OK);
    return;

  case DLN2_ADC_CHANNEL_SET_CFG: {
    /* port, channel, type, period (le16, ms), low, high. The kernel only
     * uses "always" (periodic) and "none"; threshold types aren't done. */
    uint8_t type = (len >= 13U) ? req[2] : 0xFFU;
    uint16_t period = (len >= 13U) ? get_le16(&req[3]) : 0U;
    if (type == DLN2_ADC_EVENT_NONE || (type == DLN2_ADC_EVENT_ALWAYS && period == 0U)) {
      adc_period[ch] = 0U;
      adc_due &= (uint8_t)~(1U << ch);
    } else if (type == DLN2_ADC_EVENT_ALWAYS) {
      adc_period[ch] = period;
      adc_next[ch] = HAL_GetTick() + period;
    } else {
      dln2_reply(pdev, 10U, DLN2_RESULT_UNSUPPORTED);
      return;
    }
    dln2_reply(pdev, 10U, DLN2_RESULT_OK);
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
    adc_init();
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
  USBD_DLN2_TxKind = TX_IDLE;
  USBD_DLN2_Reconfigured = 1U;
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

  USBD_DLN2_Reconfigured = 1U;   /* stop and drop GPIO events */

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

/* Reply or event finished. Messages that end on a packet boundary need a
 * ZLP so the host sees the end of the transfer. Only after a reply accept
 * the next request, so a new reply never starts while the previous one is
 * still going out. */
static uint8_t USBD_DLN2_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(epnum);

  if (USBD_DLN2_TxLen != 0U && (USBD_DLN2_TxLen % DLN2_MAX_PACKET_SIZE) == 0U) {
    USBD_DLN2_TxLen = 0U;
    USBD_LL_Transmit(pdev, DLN2_IN_EP, NULL, 0U);
    return (uint8_t)USBD_OK;
  }

  if (USBD_DLN2_TxKind == TX_REPLY) {
    USBD_DLN2_RxLen = 0U;
    USBD_LL_PrepareReceive(pdev, DLN2_OUT_EP, USBD_DLN2_RxBuffer, DLN2_MAX_PACKET_SIZE);
  }
  USBD_DLN2_TxLen = 0U;
  USBD_DLN2_TxKind = TX_IDLE;
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
  if (USBD_DLN2_Reconfigured) {
    /* New host session: it re-enables the events it wants. */
    USBD_DLN2_Reconfigured = 0U;
    evt_enabled = 0U;
    evt_debounce = 0U;
    evt_head = evt_tail = 0U;
    adc_ch_enabled = 0U;
    adc_due = 0U;
    memset(adc_period, 0, sizeof(adc_period));
  }

  evt_scan();
  adc_scan();

  if (USBD_DLN2_TxKind != TX_IDLE) {
    return;
  }
  /* Requests first: the host often answers an event with a request. */
  if (!USBD_DLN2_RxReady) {
    if (USBD_DLN2_Dev->dev_state != USBD_STATE_CONFIGURED) {
      return;
    }
    if (evt_head != evt_tail) {
      evt_send(USBD_DLN2_Dev);
    } else if (adc_due != 0U) {
      adc_evt_send(USBD_DLN2_Dev);
    }
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
  } else if (handle == DLN2_HANDLE_ADC) {
    handle_adc(pdev, id, size);
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
