#include "board.h"
#include <usb_device_drd.h>
#include <cstring>

#define PMAADDR_OFFSET 0x40

static void AssignEndpointBuffers(unsigned int endpoint, unsigned int rxaddress, unsigned int txaddress,
                                  unsigned int max_transfer_size)
{
  unsigned int num_blocks = max_transfer_size <= 64 ? 1 : max_transfer_size / 32 - 1;
  USB_DRD_PMABuffDescTypeDef *buf = USB_DRD_PMA_BUFF + endpoint;
  buf->RXBD = rxaddress | (num_blocks << 26) | (1 << 31); // 32 byte blocks
  buf->TXBD = txaddress;
}

void USB_Device_DRD::AssignEndpointsBuffers()
{
  unsigned int address = PMAADDR_OFFSET;
  unsigned char *buf = (unsigned char*)USB_DRD_PMAADDR;
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

USB_Device_DRD::USB_Device_DRD()
{
}

void USB_Device_DRD::Init(USB_DeviceManager *m)
{
  manager = m;
  AssignEndpointsBuffers();
  USB_DRD_FS->CNTR = USB_CNTR_ERRM | USB_CNTR_PMAOVRM | USB_CNTR_CTRM | USB_CNTR_THR512M
    | USB_CNTR_SUSPM | USB_CNTR_DCON | USB_CNTR_WKUPM;
  if (manager->SofShouldBeEnabled())
    USB_DRD_FS->CNTR |= USB_CNTR_SOFM;
}

void USB_Device_DRD::Reset()
{
  USB_DRD_FS->DADDR = USB_DADDR_EF;
}

void USB_Device_DRD::Connect()
{
  USB_DRD_FS->BCDR = 1 << 15; // enable the embedded pull-up on DP line
}

#define CHEP_RESET ~(USB_CHEP_TX_STTX | USB_CHEP_RX_STRX | USB_CHEP_DTOG_RX | USB_CHEP_DTOG_TX)
#define CHEP_Read(reg) *reg & CHEP_RESET

void USB_Device_DRD::SetEndpointTransferType(unsigned int endpoint, USBEndpointTransferType transfer_type)
{
  endpoint &= 0x0F;
  uint32_t epkind;
  switch (transfer_type)
  {
    case usb_endpoint_transfer_type_control: epkind = 1 << 9; break;
    case usb_endpoint_transfer_type_isochronous: epkind = 2 << 9; break;
    case usb_endpoint_transfer_type_interrupt: epkind = 3 << 9; break;
    default: epkind = 0; break;
  }
  volatile uint32_t *reg = &USB_DRD_FS->CHEP0R + endpoint;
  uint32_t value = CHEP_Read(reg);
  value |= endpoint | epkind;
  *reg = value;
}

void USB_Device_DRD::ConfigureEndpoint(unsigned int endpoint_no, USBEndpointConfiguration rx_config,
                                        USBEndpointConfiguration tx_config)
{
  volatile uint32_t *reg = &USB_DRD_FS->CHEP0R + endpoint_no;
  uint32_t value = *reg;
  uint32_t state = value & (USB_CHEP_RX_STRX | USB_CHEP_TX_STTX);
  state ^= (rx_config << 12) | (tx_config << 4);
  value &= CHEP_RESET;
  value |= state;
  *reg = value;
}

void USB_Device_DRD::ConfigureEndpointRX(unsigned int endpoint_no, USBEndpointConfiguration config)
{
  volatile uint32_t *reg = &USB_DRD_FS->CHEP0R + endpoint_no;
  uint32_t value = *reg;
  uint32_t state = value & USB_CHEP_RX_STRX;
  state ^= config << 12;
  value &= CHEP_RESET;
  value |= state;
  *reg = value;
}

void USB_Device_DRD::ConfigureEndpointTX(unsigned int endpoint_no, USBEndpointConfiguration config)
{
  volatile uint32_t *reg = &USB_DRD_FS->CHEP0R + endpoint_no;
  uint32_t value = *reg;
  uint32_t state = value & USB_CHEP_TX_STTX;
  state ^= config << 4;
  value &= CHEP_RESET;
  value |= state;
  *reg = value;
}

void USB_Device_DRD::SetEndpointData(unsigned endpoint_no, const void *data, unsigned int length)
{
  CopyToPMA32(endpoint_no, data, length);
  USB_DRD_PMABuffDescTypeDef *buff = USB_DRD_PMA_BUFF + endpoint_no;
  unsigned int temp = buff->TXBD & 0xFC00FFFF;
  buff->TXBD = temp | (length << 16);
  ConfigureEndpointTX(endpoint_no, usb_endpoint_configuration_enabled);
}

void USB_Device_DRD::ZeroTransfer(unsigned int endpoint_no)
{
  USB_DRD_PMABuffDescTypeDef *buff = USB_DRD_PMA_BUFF + endpoint_no;
  buff->TXBD &= 0xFC00FFFF;
  ConfigureEndpointTX(endpoint_no, usb_endpoint_configuration_enabled);
}

static unsigned int GetEndpointRxLength(unsigned int endpoint)
{
  USB_DRD_PMABuffDescTypeDef *buff = USB_DRD_PMA_BUFF + endpoint;
  return (buff->RXBD & 0x3ff0000) >> 16;
}

void USB_Device_DRD::InterruptHandler()
{
  uint32_t istr = USB_DRD_FS->ISTR;
  if (istr & USB_ISTR_CTR)
  {
    uint32_t endpoint = istr & 0x0F;
    volatile uint32_t *reg = &USB_DRD_FS->CHEP0R + endpoint;
    uint32_t value = CHEP_Read(reg);
    if (value & 0x8080) //vttx or vtrx
    {
      uint32_t out = value & 0x8000;
      value &= ~0x8080;
      *reg = value;
      if (out)
      {
        if (value & 0x800) // setup
        {
          CopyFromPMA32(endpoint, endpoint_buffers_rx[endpoint].buffer, 8);
          manager->SetupPacketReceived(endpoint_buffers_rx[endpoint].buffer);
        }
        else
        {
          unsigned int l = GetEndpointRxLength(endpoint);
          CopyFromPMA32(endpoint, endpoint_buffers_rx[endpoint].buffer, l);
          manager->DataPacketReceived(endpoint, endpoint_buffers_rx[endpoint].buffer, l);
        }
      }
      else
        manager->ContinueTransfer(endpoint);
    }
    USB_DRD_FS->ISTR &= ~USB_ISTR_CTR;
    return;
  }
  if (istr & USB_ISTR_SOF)
  {
    manager->Sof();
    USB_DRD_FS->ISTR &= ~USB_ISTR_SOF;
    return;
  }
  if (istr & USB_ISTR_DCON)
  {
    manager->Reset();
    USB_DRD_FS->ISTR &= ~USB_ISTR_DCON;
    return;
  }
  if (istr & USB_ISTR_SUSP)
  {
    USB_DRD_FS->CNTR |= USB_CNTR_SUSPEN;
    manager->Suspend();
  }
  if (istr & USB_ISTR_WKUP)
    manager->Resume();
  // clear all interrupt requests
  USB_DRD_FS->ISTR = 0;
}

void USB_Device_DRD::SetAddress(unsigned short address)
{
  USB_DRD_FS->DADDR = USB_DADDR_EF | (address & 0x7F);
}

const unsigned int USB_Device_DRD::GetEnabledEndpoints()
{
  return 0xFF;
}
