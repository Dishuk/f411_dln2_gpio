/**
 * @file    usbd_dln2_desc.c
 * @brief   Device + string descriptors advertising VID:PID a257:2013.
 *          Kept separate from CubeMX-generated usbd_desc.c so regen
 *          never clobbers VID/PID/strings; swapped in at boot by the
 *          USER CODE hook in usb_device.c.
 */

#include "usbd_dln2_desc.h"
#include "usbd_core.h"
#include "usbd_conf.h"

#define DLN2_VID                        0xA257U
#define DLN2_PID                        0x2013U
#define DLN2_LANGID                     0x0409U   /* English (US) */
#define DLN2_MFG_STRING                 "Diolan"
#define DLN2_PRODUCT_STRING             "DLN-2"
#define DLN2_CONFIG_STRING              "DLN-2 Config"
#define DLN2_INTERFACE_STRING           "DLN-2 Vendor Interface"

#define DEVICE_ID1                      (UID_BASE)
#define DEVICE_ID2                      (UID_BASE + 0x4U)
#define DEVICE_ID3                      (UID_BASE + 0x8U)

/* 2-byte string descriptor header + 24 bytes for 12 UTF-16 hex chars. */
#define DLN2_SERIAL_STRING_SIZE         0x1AU

static uint8_t *DLN2_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *DLN2_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *DLN2_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *DLN2_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *DLN2_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *DLN2_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *DLN2_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef DLN2_Desc_FS = {
  DLN2_DeviceDescriptor,
  DLN2_LangIDStrDescriptor,
  DLN2_ManufacturerStrDescriptor,
  DLN2_ProductStrDescriptor,
  DLN2_SerialStrDescriptor,
  DLN2_ConfigStrDescriptor,
  DLN2_InterfaceStrDescriptor,
};

__ALIGN_BEGIN static uint8_t DLN2_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
  0x12,
  USB_DESC_TYPE_DEVICE,
  0x00, 0x02,                             /* bcdUSB 2.00 */
  0x00,                                   /* bDeviceClass (defined at interface) */
  0x00,                                   /* bDeviceSubClass */
  0x00,                                   /* bDeviceProtocol */
  USB_MAX_EP0_SIZE,                       /* bMaxPacketSize0 */
  LOBYTE(DLN2_VID), HIBYTE(DLN2_VID),
  LOBYTE(DLN2_PID), HIBYTE(DLN2_PID),
  0x00, 0x01,                             /* bcdDevice 1.00 */
  USBD_IDX_MFC_STR,
  USBD_IDX_PRODUCT_STR,
  USBD_IDX_SERIAL_STR,
  USBD_MAX_NUM_CONFIGURATION,
};

__ALIGN_BEGIN static uint8_t DLN2_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
  USB_LEN_LANGID_STR_DESC,
  USB_DESC_TYPE_STRING,
  LOBYTE(DLN2_LANGID),
  HIBYTE(DLN2_LANGID),
};

__ALIGN_BEGIN static uint8_t DLN2_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

__ALIGN_BEGIN static uint8_t DLN2_StringSerial[DLN2_SERIAL_STRING_SIZE] __ALIGN_END = {
  DLN2_SERIAL_STRING_SIZE,
  USB_DESC_TYPE_STRING,
};

static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
  for (uint8_t i = 0U; i < len; i++) {
    uint8_t nibble = (uint8_t)((value >> 28) & 0x0FU);
    pbuf[2U * i] = (nibble < 0x0AU) ? (uint8_t)(nibble + '0')
                                    : (uint8_t)(nibble + 'A' - 10);
    pbuf[2U * i + 1U] = 0U;
    value <<= 4;
  }
}

static void FillSerialNumber(void)
{
  uint32_t s0 = *(uint32_t *)DEVICE_ID1;
  uint32_t s1 = *(uint32_t *)DEVICE_ID2;
  uint32_t s2 = *(uint32_t *)DEVICE_ID3;

  s0 += s2;
  if (s0 != 0U) {
    IntToUnicode(s0, &DLN2_StringSerial[2], 8U);
    IntToUnicode(s1, &DLN2_StringSerial[18], 4U);
  }
}

static uint8_t *DLN2_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(DLN2_DeviceDesc);
  return DLN2_DeviceDesc;
}

static uint8_t *DLN2_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(DLN2_LangIDDesc);
  return DLN2_LangIDDesc;
}

static uint8_t *DLN2_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)DLN2_MFG_STRING, DLN2_StrDesc, length);
  return DLN2_StrDesc;
}

static uint8_t *DLN2_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)DLN2_PRODUCT_STRING, DLN2_StrDesc, length);
  return DLN2_StrDesc;
}

static uint8_t *DLN2_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = DLN2_SERIAL_STRING_SIZE;
  FillSerialNumber();
  return DLN2_StringSerial;
}

static uint8_t *DLN2_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)DLN2_CONFIG_STRING, DLN2_StrDesc, length);
  return DLN2_StrDesc;
}

static uint8_t *DLN2_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)DLN2_INTERFACE_STRING, DLN2_StrDesc, length);
  return DLN2_StrDesc;
}
