#ifndef USB_DEVICE_STM32F_H
#define USB_DEVICE_STM32F_H

#include <usb_device.h>

typedef struct
{
  unsigned int vbus_sensing_enable;  /*!< Enable or disable the VBUS Sensing feature.                            */
  unsigned int use_external_vbus;    /*!< Enable or disable the use of the external VBUS.                        */
  unsigned int dev_remote_wakeup;
} USB_OTG_CfgTypeDef;

class USB_Device_STM32F: public USB_Device {
  private:
    USB_OTG_GlobalTypeDef *instance;
    const USB_OTG_CfgTypeDef *cfg;

    void USB_CoreInit();
    void USB_DevInit();
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
