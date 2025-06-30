#include "board.h"
#include <usb_device_stm32f.h>
#include <cstring>
#include "delay_systick.h"

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

  USBx_DEVICE(instance)->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);

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

void USB_Device_STM32F::Init(USB_DeviceManager *m)
{
  manager = m;
  USB_CoreInit();
  // Set device mode
  instance->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
  delayms(50);
  USB_DevInit();
  //AssignEndpointsBuffers();
  USB_FIFO_Init();

  //Enables the controller's Global Int in the AHB Config reg
  instance->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
}

void USB_Device_STM32F::Reset()
{
  SetAddress(0);
}

void USB_Device_STM32F::Connect()
{
  USBx_DEVICE(instance)->DCTL &= ~USB_OTG_DCTL_SDIS;
  //delayms(3);
}

void USB_Device_STM32F::SetEndpointTransferType(unsigned int endpoint, USBEndpointTransferType transfer_type)
{
  //todo
}

void USB_Device_STM32F::ConfigureEndpoint(unsigned int endpoint_no, USBEndpointConfiguration rx_config,
                                        USBEndpointConfiguration tx_config)
{
  //todo
}

void USB_Device_STM32F::ConfigureEndpointRX(unsigned int endpoint_no, USBEndpointConfiguration config)
{
  //todo
}

void USB_Device_STM32F::ConfigureEndpointTX(unsigned int endpoint_no, USBEndpointConfiguration config)
{
  //todo
}

void USB_Device_STM32F::SetEndpointData(unsigned endpoint_no, const void *data, unsigned int length)
{
  //todo
  ConfigureEndpointTX(endpoint_no, usb_endpoint_configuration_enabled);
}

void USB_Device_STM32F::ZeroTransfer(unsigned int endpoint_no)
{
  //todo
  ConfigureEndpointTX(endpoint_no, usb_endpoint_configuration_enabled);
}

unsigned int GetEndpointRxLength(unsigned int endpoint)
{
  //todo
  return 0;
}

void USB_Device_STM32F::InterruptHandler()
{
  //todo
}

void USB_Device_STM32F::SetAddress(unsigned short address)
{
  unsigned int temp = USBx_DEVICE(instance)->DCFG & ~USB_OTG_DCFG_DAD;
  USBx_DEVICE(instance)->DCFG = temp | ((address << 4) & USB_OTG_DCFG_DAD);
}

const unsigned int USB_Device_STM32F::GetEnabledEndpoints()
{
  return 0xFFFF;
}
