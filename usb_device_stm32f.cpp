#include "board.h"
#include <usb_device_stm32f.h>
#include <cstring>
#include "delay_systick.h"
#include <nvic.h>
#include <cstdlib>

#define USBx_DEVICE(instance)     ((USB_OTG_DeviceTypeDef *)((unsigned int)instance + USB_OTG_DEVICE_BASE))
#define USBx_INEP(instance, i)    ((USB_OTG_INEndpointTypeDef *)((unsigned int)instance + USB_OTG_IN_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))
#define USBx_OUTEP(instance, i)   ((USB_OTG_OUTEndpointTypeDef *)((unsigned int)instance + USB_OTG_OUT_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))
#define USBx_PCGCCTL(instance)    *(__IO unsigned int*)((unsigned int)instance + USB_OTG_PCGCCTL_BASE)
#define USBx_DFIFO(instance, i)   *(__IO unsigned int*)((unsigned int)instance + USB_OTG_FIFO_BASE + (i) * USB_OTG_FIFO_SIZE)

#define DCFG_FRAME_INTERVAL_80                 0U
#define DCFG_FRAME_INTERVAL_85                 1U
#define DCFG_FRAME_INTERVAL_90                 2U
#define DCFG_FRAME_INTERVAL_95                 3U

#define USB_OTG_SPEED_HIGH                     0U
#define USB_OTG_SPEED_HIGH_IN_FULL             1U
#define USB_OTG_SPEED_LOW                      2U
#define USB_OTG_SPEED_FULL                     3U

#define DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ     (0 << 1)
#define DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ     (1 << 1)
#define DSTS_ENUMSPD_LS_PHY_6MHZ               (2 << 1)
#define DSTS_ENUMSPD_FS_PHY_48MHZ              (3 << 1)

#define USB_MASK_INTERRUPT(instance, __INTERRUPT__)     (instance->GINTMSK &= ~(__INTERRUPT__))
#define USB_UNMASK_INTERRUPT(instance, __INTERRUPT__)   (instance->GINTMSK |= (__INTERRUPT__))

#define STS_DATA_UPDT                          (2U << 17)
#define STS_SETUP_UPDT                         (6U << 17)

#define CLEAR_IN_EP_INTR(instance, __EPNUM__, __INTERRUPT__)          (USBx_INEP(instance, __EPNUM__)->DIEPINT = (__INTERRUPT__))
#define CLEAR_OUT_EP_INTR(instance, __EPNUM__, __INTERRUPT__)         (USBx_OUTEP(instance, __EPNUM__)->DOEPINT = (__INTERRUPT__))

/**
  * @brief  USB_SetDevSpeed :Initializes the DevSpd field of DCFG register
  *         depending the PHY type and the enumeration speed of the device.
  * @param  speed : device speed
  *          This parameter can be one of these values:
  *            @arg USB_OTG_SPEED_HIGH: High speed mode
  *            @arg USB_OTG_SPEED_HIGH_IN_FULL: High speed core in Full Speed mode
  *            @arg USB_OTG_SPEED_FULL: Full speed mode
  *            @arg USB_OTG_SPEED_LOW: Low speed mode
  */
static void USB_SetDevSpeed(USB_OTG_GlobalTypeDef *instance, unsigned int speed)
{
  USBx_DEVICE(instance)->DCFG |= speed;
}
/**
  * @brief  Reset the USB Core (needed after USB clock settings change)
  */
static void USB_CoreReset(USB_OTG_GlobalTypeDef *instance)
{
  unsigned int count = 0;

  /* Wait for AHB master IDLE state. */
  do
  {
    if (++count > 200000)
    {
      while (1);//timeout
    }
  }
  while ((instance->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0);

  /* Core Soft Reset */
  count = 0;
  instance->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;

  do
  {
    if (++count > 200000)
    {
      while (1);//timeout
    }
  }
  while ((instance->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);
}

void USB_Device_STM32F::USB_CoreInit()
{
  if (instance == USB_OTG_FS)
  {
    /* Select FS Embedded PHY */
    instance->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

    /* Reset after a PHY select */
    USB_CoreReset(instance);

    /* Deactivate the power down*/
    instance->GCCFG = USB_OTG_GCCFG_PWRDWN;
  }
  else
  {
    /* Power down USB FS PHY*/
    instance->GCCFG &= ~USB_OTG_GCCFG_PWRDWN;

    /* Init The ULPI Interface */
    instance->GUSBCFG &= ~(USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);

    /* Select vbus source */
    instance->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);
    if(cfg->use_external_vbus)
      instance->GUSBCFG |= USB_OTG_GUSBCFG_ULPIEVBUSD;
    /* Reset after a PHY select  */
    USB_CoreReset(instance);

    // enable dma
    instance->GAHBCFG |= USB_OTG_GAHBCFG_HBSTLEN_1 | USB_OTG_GAHBCFG_HBSTLEN_2;
    instance->GAHBCFG |= USB_OTG_GAHBCFG_DMAEN;
  }
}

static void USB_FlushTxFifo(USB_OTG_GlobalTypeDef *instance, unsigned int num)
{
  unsigned int count = 0;

  instance->GRSTCTL = ( USB_OTG_GRSTCTL_TXFFLSH | ( num << 6));

  do
  {
    if (++count > 200000)
    {
      while (1);
    }
  }
  while ((instance->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);
}

static void USB_FlushRxFifo(USB_OTG_GlobalTypeDef *instance)
{
  unsigned int count = 0;

  instance->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;

  do
  {
    if (++count > 200000)
    {
      while(1);
    }
  }
  while ((instance->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);
}

/**
  * @brief  USB_DevInit : Initializes the USB_OTG controller registers
  *         for device mode
  */
void USB_Device_STM32F::USB_DevInit()
{
  unsigned int i;

#ifdef STM32F7
  /*Activate VBUS Sensing B */
  instance->GCCFG |= USB_OTG_GCCFG_VBDEN;

  if (cfg->vbus_sensing_enable == 0)
  {
    /* Deactivate VBUS Sensing B */
    instance->GCCFG &= ~ USB_OTG_GCCFG_VBDEN;

    /* B-peripheral session valid override enable*/
    instance->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
    instance->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
  }
#else
#ifdef STM32F2
  if (cfg->vbus_sensing_enable == 0)
  {
    /*
     * disable HW VBUS sensing. VBUS is internally considered to be always
     * at VBUS-Valid level (5V).
     */
    instance->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    instance->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
    instance->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
  }
  else
  {
    /* Enable HW VBUS sensing */
    instance->GCCFG &= ~USB_OTG_GCCFG_NOVBUSSENS;
    instance->GCCFG |= USB_OTG_GCCFG_VBUSBSEN;
  }
#else
#ifdef STM32F4
  if (cfg->vbus_sensing_enable == 0)
  {
    /*
     * disable HW VBUS sensing. VBUS is internally considered to be always
     * at VBUS-Valid level (5V).
     */
    USBx_DEVICE(instance)->DCTL |= USB_OTG_DCTL_SDIS;
    instance->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    instance->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
    instance->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
  }
  else
  {
    /* Enable HW VBUS sensing */
    instance->GCCFG &= ~USB_OTG_GCCFG_NOVBUSSENS;
    instance->GCCFG |= USB_OTG_GCCFG_VBUSBSEN;
  }
#else
  #error PLEASE DEFINE CPU FAMILY
#endif
#endif
#endif

  /* Restart the Phy Clock */
  USBx_PCGCCTL(instance) = 0;

  /* Device mode configuration */
  USBx_DEVICE(instance)->DCFG |= DCFG_FRAME_INTERVAL_80;

  if (instance == USB_OTG_FS)
  {
    /* Set Full speed phy */
    USB_SetDevSpeed(instance, USB_OTG_SPEED_FULL);
  }
  else
  {
    /* Set High speed phy */
    USB_SetDevSpeed(instance, USB_OTG_SPEED_HIGH);
  }

  /* Flush the FIFOs */
  USB_FlushTxFifo(instance, 0x10); /* all Tx FIFOs */
  USB_FlushRxFifo(instance);

  /* Clear all pending Device Interrupts */
  USBx_DEVICE(instance)->DIEPMSK = 0;
  USBx_DEVICE(instance)->DOEPMSK = 0;
  USBx_DEVICE(instance)->DAINT = 0xFFFFFFFF;
  USBx_DEVICE(instance)->DAINTMSK = 0;

  for (i = 0; i < USB_MAX_ENDPOINTS; i++)
  {
    if ((USBx_INEP(instance, i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)
      USBx_INEP(instance, i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
    else
      USBx_INEP(instance, i)->DIEPCTL = 0;

    USBx_INEP(instance, i)->DIEPTSIZ = 0;
    USBx_INEP(instance, i)->DIEPINT  = 0xFF;
  }

  for (i = 0; i < USB_MAX_ENDPOINTS; i++)
  {
    if ((USBx_OUTEP(instance, i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)
    {
      USBx_OUTEP(instance, i)->DOEPCTL = (USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK);
    }
    else
    {
      USBx_OUTEP(instance, i)->DOEPCTL = 0;
    }

    USBx_OUTEP(instance, i)->DOEPTSIZ = 0;
    USBx_OUTEP(instance, i)->DOEPINT  = 0xFF;
  }

  USBx_DEVICE(instance)->DIEPMSK &= ~USB_OTG_DIEPMSK_TXFURM;

  if (instance != USB_OTG_FS) // dma setup
  {
    /*Set threshold parameters */
    USBx_DEVICE(instance)->DTHRCTL = (USB_OTG_DTHRCTL_TXTHRLEN_6 | USB_OTG_DTHRCTL_RXTHRLEN_6);
    USBx_DEVICE(instance)->DTHRCTL |= (USB_OTG_DTHRCTL_RXTHREN | USB_OTG_DTHRCTL_ISOTHREN | USB_OTG_DTHRCTL_NONISOTHREN);

    i= USBx_DEVICE(instance)->DTHRCTL;
  }

  /* Disable all interrupts. */
  instance->GINTMSK = 0;

  /* Clear any pending interrupts */
  instance->GINTSTS = 0xBFFFFFFF;

  if (instance == USB_OTG_FS)
    instance->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;

  /* Enable interrupts matching to the Device mode ONLY */
  instance->GINTMSK |= (USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |\
                    USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |\
                    USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_IISOIXFRM|\
                    USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_WUIM);

  if (manager->SofShouldBeEnabled())
    instance->GINTMSK |= USB_OTG_GINTMSK_SOFM;

  if (cfg->vbus_sensing_enable)
    instance->GINTMSK |= USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT;
}

void USB_SetRxFiFo(USB_OTG_GlobalTypeDef *instance, unsigned int size)
{
  instance->GRXFSIZ = size;
}

void USB_SetTxFiFo(USB_OTG_GlobalTypeDef *instance, unsigned int fifo, unsigned int offset, unsigned int size)
{
  if (!fifo)
    instance->DIEPTXF0_HNPTXFSIZ = (size << 16) | offset;
  else
    instance->DIEPTXF[fifo - 1] = (size << 16) | offset;
}

void USB_Device_STM32F::USB_FIFO_Init() const
{
  unsigned int i, max_size = 0;

  for (i = 0; i < USB_MAX_ENDPOINTS; i++)
  {
    unsigned int size = manager->GetEndpointMaxTransferSize(i);
    if (size > max_size)
      max_size = size;
  }
  USB_SetRxFiFo(instance, max_size);
  unsigned int offset = max_size;
  for (i = 0; i < USB_MAX_ENDPOINTS; i++)
  {
    unsigned int size = manager->GetEndpointMaxTransferSize(i);
    USB_SetTxFiFo(instance, i, offset, size);
    offset += size;
  }
}

void USB_Device_STM32F::USB_EPInit() const
{
  int i;

  /* Init endpoints structures */
  for (i = 0; i < 15; i++)
    instance->DIEPTXF[i] = 0;
}

void USB_Device_STM32F::AssignEndpointsBuffers()
{
  for (unsigned int i = 0; i <= USB_MAX_ENDPOINTS; i++)
  {
    unsigned int max_transfer_size = manager->GetEndpointMaxTransferSize(i);
    if (max_transfer_size)
      endpoint_buffers_rx[i] = (unsigned char *)malloc(max_transfer_size);
  }
}

void USB_Device_STM32F::Init(USB_DeviceManager *m)
{
  manager = m;

  // disable global interrupt
  instance->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;

  USB_CoreInit();

  // Set device mode
  instance->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
  delayms(50);

  USB_EPInit();
  USB_DevInit();

  //AssignEndpointsBuffers();
  USB_FIFO_Init();
}

void USB_Device_STM32F::InitXferBuffers()
{
  for (unsigned int i = 0; i < USB_MAX_ENDPOINTS; i++)
  {
    xfer_data[i].pointer = endpoint_buffers_rx[i];
    xfer_data[i].length = 0;
  }
}

void USB_Device_STM32F::Reset()
{
  SetAddress(0);
  InitXferBuffers();
}

void USB_Device_STM32F::Connect()
{
  if (instance == USB_OTG_FS)
    NVIC_Init(OTG_FS_IRQn, 1, 3, ENABLE);
#ifdef USB_OTG_HS
  else
    NVIC_Init(OTG_HS_IRQn, 1, 3, ENABLE);
#endif
  //Enables the controller's Global Int in the AHB Config reg
  instance->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

  USBx_DEVICE(instance)->DCTL &= ~USB_OTG_DCTL_SDIS;
  //delayms(3);
}

void USB_Device_STM32F::SetEndpointTransferType(unsigned int endpoint, USBEndpointTransferType transfer_type)
{
  unsigned int value_in = USBx_INEP(instance, endpoint)->DIEPCTL & ~USB_OTG_DIEPCTL_EPTYP;
  unsigned int value_out = USBx_OUTEP(instance, endpoint)->DOEPCTL & ~USB_OTG_DOEPCTL_EPTYP;
  unsigned int ttype = transfer_type << 18;
  USBx_INEP(instance, endpoint)->DIEPCTL = value_in | ttype;
  USBx_OUTEP(instance, endpoint)->DOEPCTL = value_out | ttype;
}

void USB_Device_STM32F::ConfigureEndpoint(unsigned int endpoint_no, USBEndpointConfiguration rx_config,
                                        USBEndpointConfiguration tx_config)
{
  ConfigureEndpointRX(endpoint_no, rx_config);
  ConfigureEndpointTX(endpoint_no, tx_config);
}

void USB_Device_STM32F::ConfigureEndpointRX(unsigned int endpoint_no, USBEndpointConfiguration config)
{
  unsigned int value = USBx_INEP(instance, endpoint_no)->DIEPCTL & ~USB_OTG_DIEPCTL_STALL;
  switch (config)
  {
    case usb_endpoint_configuration_enabled: value |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA; break;
    case usb_endpoint_configuration_nak: value |= USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_EPENA; break;
    case usb_endpoint_configuration_stall: value |= USB_OTG_DIEPCTL_STALL | USB_OTG_DIEPCTL_EPENA; break;
    default: value |= USB_OTG_DIEPCTL_EPDIS; break;
  }
  USBx_INEP(instance, endpoint_no)->DIEPCTL = value;
}

void USB_Device_STM32F::ConfigureEndpointTX(unsigned int endpoint_no, USBEndpointConfiguration config)
{
  unsigned int value = USBx_OUTEP(instance, endpoint_no)->DOEPCTL & ~USB_OTG_DOEPCTL_STALL;
  switch (config)
  {
    case usb_endpoint_configuration_enabled: value |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA; break;
    case usb_endpoint_configuration_nak: value |= USB_OTG_DOEPCTL_SNAK | USB_OTG_DOEPCTL_EPENA; break;
    case usb_endpoint_configuration_stall: value |= USB_OTG_DOEPCTL_STALL | USB_OTG_DOEPCTL_EPENA; break;
    default: value |= USB_OTG_DOEPCTL_EPDIS; break;
  }
  USBx_OUTEP(instance, endpoint_no)->DOEPCTL = value;
}

/**
  * @brief  USB_WritePacket : Writes a packet into the Tx FIFO associated
  *         with the EP/channel
  * @param  src :  pointer to source buffer
  * @param  ch_ep_num : endpoint or host channel number
  * @param  len : Number of bytes to write
  * @retval none
  */
static void USB_WritePacket(USB_OTG_GlobalTypeDef *instance, const unsigned char *src, unsigned int ch_ep_num, unsigned int len)
{
  unsigned int count32b, i;

  if (instance == USB_OTG_FS)
  {
    count32b = (len + 3) / 4;
    for (i = 0; i < count32b; i++, src += 4)
      USBx_DFIFO(instance, ch_ep_num) = *((unsigned int*)src);
  }
}

void USB_Device_STM32F::SetEndpointData(unsigned int endpoint_no, const void *data, unsigned int length)
{
  unsigned int temp = USBx_INEP(instance, endpoint_no)->DIEPTSIZ & ~(USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_XFRSIZ);
  USBx_INEP(instance, endpoint_no)->DIEPTSIZ = temp | (1 << 19) | length;
  USB_WritePacket(instance, (const unsigned char *)data, endpoint_no, length);
  ConfigureEndpointTX(endpoint_no, usb_endpoint_configuration_enabled);
}

void USB_Device_STM32F::ZeroTransfer(unsigned int endpoint_no)
{
  unsigned int temp = USBx_INEP(instance, endpoint_no)->DIEPTSIZ & ~(USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_XFRSIZ);
  USBx_INEP(instance, endpoint_no)->DIEPTSIZ = temp | (1 << 19);
  ConfigureEndpointTX(endpoint_no, usb_endpoint_configuration_enabled);
}

unsigned int GetEndpointRxLength(unsigned int endpoint)
{
  //todo
  return 0;
}

/**
  * @brief  USB_ReadPacket : read a packet from the Tx FIFO associated
  *         with the EP/channel
  * @param  dest : destination pointer
  * @param  len : Number of bytes to read
  * @retval pointer to destination buffer
  */
static void USB_ReadPacket(USB_OTG_GlobalTypeDef *instance, unsigned char *dest, unsigned int len)
{
  unsigned int i;
  unsigned int count32b = (len + 3) / 4;

  for ( i = 0; i < count32b; i++, dest += 4 )
    *((unsigned int*)dest) = USBx_DFIFO(instance, 0);
}

/**
  * @brief  USB_ReadDevAllOutEpInterrupt: return the USB device OUT endpoints interrupt status
  * @retval status
  */
static unsigned int USB_ReadDevAllOutEpInterrupt(USB_OTG_GlobalTypeDef *instance)
{
  unsigned int v = USBx_DEVICE(instance)->DAINT;
  v &= USBx_DEVICE(instance)->DAINTMSK;
  return ((v & 0xffff0000) >> 16);
}

/**
  * @brief  Returns Device OUT EP Interrupt register
  * @param  epnum : endpoint number
  *          This parameter can be a value from 0 to 15
  * @retval Device OUT EP Interrupt register
  */
static unsigned int USB_ReadDevOutEPInterrupt(USB_OTG_GlobalTypeDef *instance, unsigned int epnum)
{
  unsigned int v = USBx_OUTEP(instance, epnum)->DOEPINT;
  v &= USBx_DEVICE(instance)->DOEPMSK;
  return v;
}

void USB_Device_STM32F::USBRSTHandler()
{
  unsigned int i;

  USBx_DEVICE(instance)->DCTL &= ~USB_OTG_DCTL_RWUSIG;
  USB_FlushTxFifo(instance, 0);

  for (i = 0; i < USB_MAX_ENDPOINTS; i++)
  {
    USBx_INEP(instance, i)->DIEPINT = 0xFF;
    USBx_OUTEP(instance, i)->DOEPINT = 0xFF;
  }
  USBx_DEVICE(instance)->DAINT = 0xFFFFFFFF;
  USBx_DEVICE(instance)->DAINTMSK |= 0x10001;

  USBx_DEVICE(instance)->DOEPMSK |= USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM | USB_OTG_DOEPMSK_OTEPSPRM;
  USBx_DEVICE(instance)->DIEPMSK |= USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM;

  manager->Reset();

  instance->GINTSTS = USB_OTG_GINTSTS_USBRST;
}

void USB_Device_STM32F::OEPINTHandler()
{
  unsigned int epnum, ep_intr, epint;

  epnum = 0;

  /* Read in the device interrupt bits */
  ep_intr = USB_ReadDevAllOutEpInterrupt(instance);

  while (ep_intr)
  {
    if (ep_intr & 1)
    {
      epint = USB_ReadDevOutEPInterrupt(instance, epnum);

      if((epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC)
      {
        CLEAR_OUT_EP_INTR(instance, epnum, USB_OTG_DOEPINT_XFRC);

        manager->DataPacketReceived(epnum, endpoint_buffers_rx[epnum], xfer_data[epnum].length);
        xfer_data[epnum].length = 0;
        xfer_data[epnum].pointer = endpoint_buffers_rx[epnum];
      }

      if((epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP)
      {
        // Inform the upper layer that a setup packet is available
        manager->SetupPacketReceived(endpoint_buffers_rx[epnum]);
        CLEAR_OUT_EP_INTR(instance, epnum, USB_OTG_DOEPINT_STUP);
      }

      if((epint & USB_OTG_DOEPINT_OTEPDIS) == USB_OTG_DOEPINT_OTEPDIS)
      {
        CLEAR_OUT_EP_INTR(instance, epnum, USB_OTG_DOEPINT_OTEPDIS);
      }
      // Clear Status Phase Received interrupt
      if((epint & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR)
      {
        CLEAR_OUT_EP_INTR(instance, epnum, USB_OTG_DOEPINT_OTEPSPR);
      }
    }
    epnum++;
    ep_intr >>= 1;
  }
}

/**
  * @brief  USB_ReadDevAllInEpInterrupt: return the USB device IN endpoints interrupt status
  * @retval status
  */
static unsigned int USB_ReadDevAllInEpInterrupt (USB_OTG_GlobalTypeDef *instance)
{
  unsigned int v;
  v  = USBx_DEVICE(instance)->DAINT;
  v &= USBx_DEVICE(instance)->DAINTMSK;
  return v & 0xFFFF;
}

/**
  * @brief  Returns Device IN EP Interrupt register
  * @param  epnum : endpoint number
  *          This parameter can be a value from 0 to 15
  * @retval Device IN EP Interrupt register
  */
static unsigned int USB_ReadDevInEPInterrupt(USB_OTG_GlobalTypeDef *instance, unsigned int epnum)
{
  unsigned int v, msk, emp;

  msk = USBx_DEVICE(instance)->DIEPMSK;
  emp = USBx_DEVICE(instance)->DIEPEMPMSK;
  msk |= ((emp >> epnum) & 0x1) << 7;
  v = USBx_INEP(instance, epnum)->DIEPINT & msk;
  return v;
}

void USB_Device_STM32F::IEPINTHandler()
{
  unsigned int epnum, ep_intr, epint, fifoemptymsk;

  epnum = 0;

  /* Read in the device interrupt bits */
  ep_intr = USB_ReadDevAllInEpInterrupt(instance);

  while ( ep_intr )
  {
    if (ep_intr & 0x1) /* In ITR */
    {
      epint = USB_ReadDevInEPInterrupt(instance, epnum);

      if(( epint & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC)
      {
        fifoemptymsk = 0x1 << epnum;
        USBx_DEVICE(instance)->DIEPEMPMSK &= ~fifoemptymsk;

        CLEAR_IN_EP_INTR(instance, epnum, USB_OTG_DIEPINT_XFRC);

        manager->ContinueTransfer(epnum);
      }
      if(( epint & USB_OTG_DIEPINT_TOC) == USB_OTG_DIEPINT_TOC)
      {
        CLEAR_IN_EP_INTR(instance, epnum, USB_OTG_DIEPINT_TOC);
      }
      if(( epint & USB_OTG_DIEPINT_ITTXFE) == USB_OTG_DIEPINT_ITTXFE)
      {
        CLEAR_IN_EP_INTR(instance, epnum, USB_OTG_DIEPINT_ITTXFE);
      }
      if(( epint & USB_OTG_DIEPINT_INEPNE) == USB_OTG_DIEPINT_INEPNE)
      {
        CLEAR_IN_EP_INTR(instance, epnum, USB_OTG_DIEPINT_INEPNE);
      }
      if(( epint & USB_OTG_DIEPINT_EPDISD) == USB_OTG_DIEPINT_EPDISD)
      {
        CLEAR_IN_EP_INTR(instance, epnum, USB_OTG_DIEPINT_EPDISD);
      }
      /*if(( epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE)
      {
        USB_WriteEmptyTxFifo(instance, epnum);
      }*/
    }
    epnum++;
    ep_intr >>= 1;
  }
}

void USB_Device_STM32F::InterruptHandler()
{
  unsigned int status = instance->GINTSTS;
  status &= instance->GINTMSK;

  if (status & USB_OTG_GINTSTS_RXFLVL)
  {
    USB_MASK_INTERRUPT(instance, USB_OTG_GINTSTS_RXFLVL);

    unsigned int temp = instance->GRXSTSP;
    unsigned int length = (temp & USB_OTG_GRXSTSP_BCNT) >> 4;

    if (length)
    {
      unsigned int ep_no = temp & USB_OTG_GRXSTSP_EPNUM;
      USB_ReadPacket(instance, xfer_data[ep_no].pointer, length);
      xfer_data[ep_no].pointer += length;
      xfer_data[ep_no].length += length;
    }

    USB_UNMASK_INTERRUPT(instance, USB_OTG_GINTSTS_RXFLVL);
    return;
  }

  /* Handle SOF Interrupt */
  if(status & USB_OTG_GINTSTS_SOF)
  {
    manager->Sof();
    instance->GINTSTS = USB_OTG_GINTSTS_SOF;
    return;
  }

  if (status & USB_OTG_GINTSTS_OEPINT)
  {
    OEPINTHandler();
  }

  if (status & USB_OTG_GINTSTS_IEPINT)
  {
    IEPINTHandler();
  }

  if (status & USB_OTG_GINTSTS_MMIS)
  {
    /* incorrect mode, acknowledge the interrupt */
    instance->GINTSTS = USB_OTG_GINTSTS_MMIS;
  }

  /* Handle Resume Interrupt */
  if (status & USB_OTG_GINTSTS_WKUINT)
  {
    /* Clear the Remote Wake-up Signaling */
    USBx_DEVICE(instance)->DCTL &= ~USB_OTG_DCTL_RWUSIG;

    manager->Resume();

    instance->GINTSTS = USB_OTG_GINTSTS_WKUINT;
  }

  /* Handle Suspend Interrupt */
  if (status & USB_OTG_GINTSTS_USBSUSP)
  {
    if((USBx_DEVICE(instance)->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS)
      manager->Suspend();
    instance->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
  }

  /* Handle Reset Interrupt */
  if (status & USB_OTG_GINTSTS_USBRST)
  {
    USBRSTHandler();
  }

  /* Handle Enumeration done Interrupt */
  if (status & USB_OTG_GINTSTS_ENUMDNE)
  {
    unsigned int temp = instance->GUSBCFG & ~USB_OTG_GUSBCFG_TRDT;
    instance->GUSBCFG = temp | (instance == USB_OTG_FS ? 6 << 10 : 9 << 10);

    manager->Reset();

    instance->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
  }

  /* Handle Incomplete ISO IN Interrupt */
  if(status & USB_OTG_GINTSTS_IISOIXFR)
  {
    //todo
    instance->GINTSTS = USB_OTG_GINTSTS_IISOIXFR;
  }

  /* Handle Incomplete ISO OUT Interrupt */
  if(status & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)
  {
    //todo
    instance->GINTSTS = USB_OTG_GINTSTS_PXFR_INCOMPISOOUT;
  }

  /* Handle Connection event Interrupt */
  if(status & USB_OTG_GINTSTS_SRQINT)
  {
    //todo
    instance->GINTSTS = USB_OTG_GINTSTS_SRQINT;
  }

  /* Handle Disconnection event Interrupt */
  if(status & USB_OTG_GINTSTS_OTGINT)
  {
    unsigned int temp = instance->GOTGINT;

    //todo

    instance->GOTGINT |= temp;
  }
}

void USB_Device_STM32F::SetAddress(unsigned short address)
{
  unsigned int temp = USBx_DEVICE(instance)->DCFG & ~USB_OTG_DCFG_DAD;
  USBx_DEVICE(instance)->DCFG = temp | ((address << 4) & USB_OTG_DCFG_DAD);
}

const unsigned int USB_Device_STM32F::GetEnabledEndpoints()
{
#ifdef STM32F7
  if (instance == USB_OTG_FS)
    return 0x03F; // 6 endpoints
  else
    return 0x1FF; // 9 endpoints
#else
  return 0x0F; // 4 endpoints
#endif
}
