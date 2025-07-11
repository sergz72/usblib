#ifndef USB_DEVICE_STM32F_H
#define USB_DEVICE_STM32F_H

#include <usb_device.h>

typedef struct
{
  bool vbus_sensing_enable;  /*!< Enable or disable the VBUS Sensing feature.                            */
  bool use_external_vbus;    /*!< Enable or disable the use of the external VBUS.                        */
} USB_OTG_CfgTypeDef;

typedef struct
{
  unsigned char *pointer;
  unsigned int length;
} USB_OTG_EPXferData;

class USB_Device_STM32F: public USB_Device {
  private:
    USB_OTG_GlobalTypeDef *instance;
    const USB_OTG_CfgTypeDef *cfg;
    USB_OTG_EPXferData xfer_data[USB_MAX_ENDPOINTS];

    void USB_CoreInit();
    void USB_DevInit();
    void USB_FIFO_Init() const;
    void USB_EPInit() const;
    void USBRSTHandler();
    void OEPINTHandler();
    void IEPINTHandler();
    void InitXferBuffers();
    void AssignEndpointsBuffers();
  public:
    explicit USB_Device_STM32F(USB_OTG_GlobalTypeDef *_instance, const USB_OTG_CfgTypeDef *_cfg)
    {
      instance = _instance;
      cfg = _cfg;
    };

    void Init(USB_DeviceManager *m) override;
    void ConfigureEndpoint(unsigned int endpoint_no, USBEndpointConfiguration rx_config,
                                  USBEndpointConfiguration tx_config) override;
    void ConfigureEndpointRX(unsigned int endpoint_no, USBEndpointConfiguration config) override;
    void ConfigureEndpointTX(unsigned int endpoint_no, USBEndpointConfiguration config) override;
    void SetEndpointData(unsigned endpoint_no, const void *data, unsigned int length) override;
    void ZeroTransfer(unsigned int endpoint_no) override;
    void InterruptHandler() override;
    void SetEndpointTransferType(unsigned int endpoint, USBEndpointTransferType transfer_type) override;
    void Connect() override;
    void Reset() override;
    void SetAddress(unsigned short address) override;
    const unsigned int GetEnabledEndpoints() override;
};

#endif
