#include "board.h"
#include <usb_device_stm32g4.h>
#include <cstring>

#define PMAADDR_OFFSET 0x40

typedef struct
{
  unsigned short tx_address;
  unsigned short tx_count;
  unsigned short rx_address;
  unsigned short rx_count;
} PMA_BUFFER_ENTRY;

static void AssignEndpointBuffers(unsigned short endpoint, unsigned short rxaddress, unsigned short txaddress,
                                  unsigned short max_transfer_size)
{
  unsigned short num_blocks = max_transfer_size <= 64 ? 1 : max_transfer_size / 32 - 1;
  PMA_BUFFER_ENTRY *buf = (PMA_BUFFER_ENTRY*)(USB_PMAADDR + endpoint * 8);
  buf->tx_address = txaddress;
  buf->tx_count = 0;
  buf->rx_address = rxaddress;
  buf->rx_count = (num_blocks << 10) | (1 << 15); // 32 byte blocks
}

void USB_Device_STM32G4::AssignEndpointsBuffers()
{
  unsigned int address = PMAADDR_OFFSET;
  unsigned char *buf = (unsigned char*)USB_PMAADDR;
  for (unsigned int i = 0; i <= 7; i++)
  {
    unsigned int max_transfer_size = manager->GetEndpointMaxTransferSize(i);
    if (max_transfer_size)
    {
      unsigned int offset_rx = address;
      unsigned int offset_tx = address + max_transfer_size;
      AssignEndpointBuffers(i, offset_rx, offset_tx, max_transfer_size);
      address = offset_tx + max_transfer_size;
      endpoint_buffers_rx[i] = buf + offset_rx;
      endpoint_buffers_tx[i] = buf + offset_tx;
    }
  }
}

USB_Device_STM32G4::USB_Device_STM32G4()
{
}

void USB_Device_STM32G4::Init(USB_DeviceManager *m)
{
  manager = m;
  AssignEndpointsBuffers();
  USB->CNTR = USB_CNTR_FRES;
  USB->CNTR = 0;
  USB->ISTR = 0;
  USB->CNTR = USB_CNTR_ERRM | USB_CNTR_PMAOVRM | USB_CNTR_CTRM
    | USB_CNTR_SUSPM | USB_CNTR_WKUPM | USB_CNTR_SOFM | USB_CNTR_RESETM;
}

void USB_Device_STM32G4::Reset()
{
  USB->DADDR = USB_DADDR_EF;
}

void USB_Device_STM32G4::Connect()
{
  USB->BCDR = 1 << 15; // enable the embedded pull-up on DP line
}

#define CHEP_RESET ~(USB_EPRX_STAT | USB_EPTX_STAT | USB_EP_DTOG_RX | USB_EP_DTOG_TX)
#define CHEP_Read(reg) *reg & CHEP_RESET

void USB_Device_STM32G4::SetEndpointTransferType(unsigned int endpoint, USBEndpointTransferType transfer_type)
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
  volatile unsigned short *reg = &USB->EP0R + endpoint * 2;
  unsigned short value = CHEP_Read(reg);
  value |= endpoint | epkind;
  *reg = value;
}

void USB_Device_STM32G4::ConfigureEndpoint(unsigned int endpoint_no, USBEndpointConfiguration rx_config,
                                        USBEndpointConfiguration tx_config)
{
  volatile unsigned short *reg = &USB->EP0R + endpoint_no * 2;
  unsigned short value = *reg;
  unsigned short state = value & (USB_EPTX_STAT | USB_EPRX_STAT);
  state ^= (rx_config << 12) | (tx_config << 4);
  value &= CHEP_RESET;
  value |= state | USB_EP_CTR_RX | USB_EP_CTR_TX;
  *reg = value;
}

void USB_Device_STM32G4::ConfigureEndpointRX(unsigned int endpoint_no, USBEndpointConfiguration config)
{
  volatile unsigned short *reg = &USB->EP0R + endpoint_no * 2;
  unsigned short value = *reg;
  unsigned short state = value & USB_EPRX_STAT;
  state ^= config << 12;
  value &= CHEP_RESET;
  value |= state | USB_EP_CTR_RX | USB_EP_CTR_TX;
  *reg = value;
}

void USB_Device_STM32G4::ConfigureEndpointTX(unsigned int endpoint_no, USBEndpointConfiguration config)
{
  volatile unsigned short *reg = &USB->EP0R + endpoint_no * 2;
  unsigned short value = *reg;
  unsigned short state = value & USB_EPTX_STAT;
  state ^= config << 4;
  value &= CHEP_RESET;
  value |= state | USB_EP_CTR_RX | USB_EP_CTR_TX;
  *reg = value;
}

void USB_Device_STM32G4::SetEndpointData(unsigned endpoint_no, const void *data, unsigned int length)
{
  CopyToPMA16(endpoint_no, data, length);
  PMA_BUFFER_ENTRY *buff = (PMA_BUFFER_ENTRY *)(USB_PMAADDR + endpoint_no * 8);
  buff->tx_count = length;
  ConfigureEndpointTX(endpoint_no, usb_endpoint_configuration_enabled);
}

void USB_Device_STM32G4::ZeroTransfer(unsigned int endpoint_no)
{
  PMA_BUFFER_ENTRY *buff = (PMA_BUFFER_ENTRY *)(USB_PMAADDR + endpoint_no * 8);
  buff->tx_count = 0;
  ConfigureEndpointTX(endpoint_no, usb_endpoint_configuration_enabled);
}

unsigned int GetEndpointRxLength(unsigned int endpoint)
{
  PMA_BUFFER_ENTRY *buff = (PMA_BUFFER_ENTRY *)(USB_PMAADDR + endpoint * 8);
  return buff->rx_count & 0x3ff;
}

void USB_Device_STM32G4::InterruptHandler()
{
  unsigned int istr = USB->ISTR;
  if (istr & USB_ISTR_CTR)
  {
    unsigned int endpoint = istr & 0x0F;
    volatile unsigned short *reg = &USB->EP0R + endpoint * 2;
    unsigned short value = CHEP_Read(reg);
    if (value & 0x8080) //ctr_tx or ctr_rx
    {
      unsigned int out = value & 0x8000;
      value &= ~0x8080;
      *reg = value;
      if (out)
      {
        if (value & 0x800) // setup
        {
          CopyFromPMA16(endpoint, endpoint_buffers_rx[endpoint], 8);
          manager->SetupPacketReceived(endpoint_buffers_rx[endpoint]);
        }
        else
        {
          unsigned int l = GetEndpointRxLength(endpoint);
          CopyFromPMA16(endpoint, endpoint_buffers_rx[endpoint], l);
          manager->DataPacketReceived(endpoint, endpoint_buffers_rx[endpoint], l);
        }
      }
      else
        manager->ContinueTransfer(endpoint);
    }
    USB->ISTR &= ~USB_ISTR_CTR;
    return;
  }
  if (istr & USB_ISTR_SOF)
  {
    manager->Sof();
    USB->ISTR &= ~USB_ISTR_SOF;
    return;
  }
  if (istr & USB_ISTR_RESET)
  {
    manager->Reset();
    USB->ISTR &= ~USB_ISTR_RESET;
    return;
  }
  if (istr & USB_ISTR_SUSP)
  {
    USB->CNTR |= USB_CNTR_FSUSP;
    USB->ISTR &= ~USB_ISTR_SUSP;
    USB->CNTR |= USB_CNTR_LPMODE;
    manager->Suspend();
    return;
  }
  if (istr & USB_ISTR_WKUP)
  {
    USB->CNTR &= ~(USB_CNTR_FSUSP | USB_CNTR_LPMODE);
    manager->Resume();
  }
  // clear all interrupt requests
  USB->ISTR = 0;
}

void USB_Device_STM32G4::SetAddress(unsigned short address)
{
  USB->DADDR = USB_DADDR_EF | (address & 0x7F);
}

const unsigned int USB_Device_STM32G4::GetEnabledEndpoints()
{
  return 0xFF;
}
