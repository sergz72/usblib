#include <cstdlib>

#include "board.h"
#include <usb_device_at32f4.h>
#include <cstring>

#define PMAADDR_OFFSET 0x80

typedef struct
{
  unsigned int tx_address;
  unsigned int tx_count;
  unsigned int rx_address;
  unsigned int rx_count;
} PMA_BUFFER_ENTRY;

static void AssignEndpointBuffers(unsigned int endpoint, unsigned int rxaddress, unsigned int txaddress,
                                  unsigned int max_transfer_size)
{
  unsigned int num_blocks = max_transfer_size <= 64 ? 1 : max_transfer_size / 32 - 1;
  PMA_BUFFER_ENTRY *buf = (PMA_BUFFER_ENTRY*)(g_usb_packet_address + endpoint * 16);
  buf->tx_address = txaddress / 2;
  buf->tx_count = 0;
  buf->rx_address = rxaddress / 2;
  buf->rx_count = (num_blocks << 10) | (1 << 15); // 32 byte blocks
}

void USB_Device_AT32F4::AssignEndpointsBuffers()
{
  unsigned int address = PMAADDR_OFFSET;
  unsigned char *buf = (unsigned char*)g_usb_packet_address;
  for (unsigned int i = 0; i <= 7; i++)
  {
    unsigned int max_transfer_size = manager->GetEndpointMaxTransferSize(i);
    if (max_transfer_size)
    {
      unsigned int offset_rx = address;
      unsigned int offset_tx = address + max_transfer_size;
      AssignEndpointBuffers(i, offset_rx, offset_tx, max_transfer_size);
      address = offset_tx + max_transfer_size;
      endpoint_buffers_rx[i].pma_buffer = buf + offset_rx;
      endpoint_buffers_rx[i].buffer = (unsigned char*)malloc(max_transfer_size);
      endpoint_buffers_tx[i] = buf + offset_tx;
    }
  }
}

USB_Device_AT32F4::USB_Device_AT32F4(bool _extended_buffers)
{
  extended_buffers = _extended_buffers;
  pma16_is_32 = true;
}

void USB_Device_AT32F4::Init(USB_DeviceManager *m)
{
  manager = m;

  /* usb buffer size extend 768-1280 byte */
  usb_usbbufs_enable(USB, extended_buffers ? TRUE : FALSE);

  AssignEndpointsBuffers();

  /* clear usb core reset */
  USB->ctrl_bit.csrst = 0;

  /* clear usb interrupt status */
  USB->intsts = 0;

  /* set usb packet buffer descirption table address */
  USB->buftbl = USB_BUFFER_TABLE_ADDRESS;

  /* enable usb core and set device address to 0 */
  //USB->devaddr = 0x80;

  uint16_t interrupts_to_enable = USB_RST_INT | USB_SP_INT | USB_WK_INT | USB_TC_INT;
  if (manager->SofShouldBeEnabled())
    interrupts_to_enable |= USB_SOF_INT;
  usb_interrupt_enable(USB, interrupts_to_enable, TRUE);
}

void USB_Device_AT32F4::Reset()
{
  /* enable usb core and set device address to 0 */
  usb_set_address(USB, 0);
}

void USB_Device_AT32F4::Connect()
{
  usb_connect(USB);
}

#define CHEP_RESET ~(USB_RXSTS | USB_TXSTS | USB_RXDTS | USB_TXDTS)
#define CHEP_Read(reg) *reg & CHEP_RESET

void USB_Device_AT32F4::SetEndpointTransferType(unsigned int endpoint, USBEndpointTransferType transfer_type)
{
  endpoint &= 0x0F;
  unsigned long epkind;
  switch (transfer_type)
  {
    case usb_endpoint_transfer_type_control: epkind = 1 << 9; break;
    case usb_endpoint_transfer_type_isochronous: epkind = 2 << 9; break;
    case usb_endpoint_transfer_type_interrupt: epkind = 3 << 9; break;
    default: epkind = 0; break;
  }
  volatile unsigned long *reg = &USB->ept[endpoint];
  unsigned long value = CHEP_Read(reg);
  value |= endpoint | epkind;
  *reg = value;
}

void USB_Device_AT32F4::ConfigureEndpoint(unsigned int endpoint_no, USBEndpointConfiguration rx_config,
                                        USBEndpointConfiguration tx_config)
{
  volatile unsigned long *reg = &USB->ept[endpoint_no];
  unsigned long value = *reg;
  unsigned int state = value & (USB_RXSTS | USB_TXSTS);
  state ^= (rx_config << 12) | (tx_config << 4);
  value &= CHEP_RESET;
  value |= state;
  *reg = value;
}

void USB_Device_AT32F4::ConfigureEndpointRX(unsigned int endpoint_no, USBEndpointConfiguration config)
{
  volatile unsigned long *reg = &USB->ept[endpoint_no];
  unsigned long value = *reg;
  unsigned int state = value & USB_RXSTS;
  state ^= config << 12;
  value &= CHEP_RESET;
  value |= state;
  *reg = value;
}

void USB_Device_AT32F4::ConfigureEndpointTX(unsigned int endpoint_no, USBEndpointConfiguration config)
{
  volatile unsigned long *reg = &USB->ept[endpoint_no];
  unsigned long value = *reg;
  unsigned int state = value & USB_TXSTS;
  state ^= config << 4;
  value &= CHEP_RESET;
  value |= state;
  *reg = value;
}

void USB_Device_AT32F4::SetEndpointData(unsigned endpoint_no, const void *data, unsigned int length)
{
  CopyToPMA16(endpoint_no, data, length);
  USB_SET_TXLEN(endpoint_no, length);
  ConfigureEndpointTX(endpoint_no, usb_endpoint_configuration_enabled);
}

void USB_Device_AT32F4::ZeroTransfer(unsigned int endpoint_no)
{
  USB_SET_TXLEN(endpoint_no, 0);
  ConfigureEndpointTX(endpoint_no, usb_endpoint_configuration_enabled);
}

void USB_Device_AT32F4::EP_TCHandler(unsigned int ept_num)
{
  unsigned int ept_val = USB->ept[ept_num];

  /* in interrupt request  */
  if(ept_val & USB_TXTC)
  {
    /* clear endpoint tc flag */
    USB_CLEAR_TXTC(ept_num);
    manager->ContinueTransfer(ept_num);
  }
  else
  {
    /* get endpoint received data length */
    unsigned int length = USB_GET_RX_LEN(ept_num);
    unsigned char *buffer = endpoint_buffers_rx[ept_num].buffer;
    /* read endpoint received data */
    CopyFromPMA16(ept_num, buffer, length);

    /* clear endpoint rx tc flag */
    USB_CLEAR_RXTC(ept_num);

    /* setup and out interrupt request */
    if((ept_val & USB_SETUPTC) != 0)
    {
      /* endpoint setup interrupt request */
      manager->SetupPacketReceived(buffer);
    }
    else if(ept_val & USB_RXTC)
    {
      /* endpoint out interrupt request */
      manager->DataPacketReceived(ept_num, buffer, length);
    }
  }
}

void USB_Device_AT32F4::TCHandler()
{
  unsigned int sts_val;

  while ((sts_val = USB->intsts) & USB_TC_FLAG)
  {
    /* get the interrupt endpoint number */
    unsigned int ept_num = sts_val & USB_EPT_NUM_FLAG;

    EP_TCHandler(ept_num);
  }
}

void USB_Device_AT32F4::InterruptHandler()
{
  uint32_t sts_val = USB->intsts;
  uint32_t sts_ien = USB->ctrl;

  if(sts_val & USB_TC_FLAG)
  {
    /* endpoint tc interrupt handler */
    TCHandler();
  }

  if(sts_val & USB_RST_FLAG)
  {
    /* clear reset flag */
    usb_flag_clear(USB, USB_RST_FLAG);

    /* reset interrupt handler */
    manager->Reset();
  }

  if((sts_val & USB_SOF_FLAG) && (sts_ien & USB_SOF_INT))
  {
    /* clear sof flag */
    usb_flag_clear(USB, USB_SOF_FLAG);

    /* sof interrupt handler */
    manager->Sof();
  }

  if((sts_val & USB_LSOF_FLAG) && (sts_ien & USB_LSOF_INT))
  {
    /* clear lsof flag */
    usb_flag_clear(USB, USB_LSOF_FLAG);
  }

  if((sts_val & USB_SP_FLAG) && (sts_ien & USB_SP_INT))
  {
    /* clear suspend flag */
    usb_flag_clear(USB, USB_SP_FLAG);

    /* usb suspend interrupt handler */
    usb_enter_suspend(USB);
    manager->Suspend();
  }

  if((sts_val & USB_WK_FLAG) && (sts_ien & USB_WK_INT))
  {
    /* usb wakeup interrupt handler */
    usb_exit_suspend(USB);
    manager->Resume();

    /* clear wakeup flag */
    usb_flag_clear(USB, USB_WK_FLAG);
  }
}

void USB_Device_AT32F4::SetAddress(unsigned short address)
{
  usb_set_address(USB, address);
}

const unsigned int USB_Device_AT32F4::GetEnabledEndpoints()
{
  return 0xFF;
}
