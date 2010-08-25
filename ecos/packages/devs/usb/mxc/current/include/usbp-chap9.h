/*
 * Copyright 2004-2006 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/* Modified from Belcarra Linux USB driver by Yi Li */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#define USB_DIR_OUT                     0
#define USB_DIR_IN                      0x80

#define USB_DT_DEVICE                   1
#define USB_DT_CONFIGURATION            2
#define USB_DT_STRING                   3
#define USB_DT_INTERFACE                4
#define USB_DT_ENDPOINT                 5

#define USB_ENDPOINT_NUMBER_MASK        0x0f            
#define USB_ENDPOINT_DIR_MASK           0x80

#define USB_ENDPOINT_MASK               0x03            
#define USB_ENDPOINT_CONTROL            0x00
#define USB_ENDPOINT_ISOCHRONOUS        0x01
#define USB_ENDPOINT_BULK               0x02
#define USB_ENDPOINT_INTERRUPT          0x03

#define USB_PID_OUT                     0xe1
#define USB_PID_ACK                     0xd2
#define USB_PID_DATA0                   0xc3
#define USB_PID_SOF                     0xa5
#define USB_PID_IN                      0x69
#define USB_PID_NAK                     0x5a
#define USB_PID_DATA1                   0x4b
#define USB_PID_ERR                     0x3c            
#define USB_PID_SETUP                   0x2d
#define USB_PID_STALL                   0x1e

#define USB_REQ_GET_STATUS              0x00
#define USB_REQ_CLEAR_FEATURE           0x01
#define USB_REQ_SET_FEATURE             0x03
#define USB_REQ_SET_ADDRESS             0x05
#define USB_REQ_GET_DESCRIPTOR          0x06
#define USB_REQ_SET_DESCRIPTOR          0x07
#define USB_REQ_GET_CONFIGURATION       0x08
#define USB_REQ_SET_CONFIGURATION       0x09
#define USB_REQ_GET_INTERFACE           0x0A
#define USB_REQ_SET_INTERFACE           0x0B
#define USB_REQ_SYNCH_FRAME             0x0C

#define USB_REQ_DIRECTION_MASK          0x80
#define USB_REQ_TYPE_MASK               0x60
#define USB_REQ_RECIPIENT_MASK          0x1f

#define USB_REQ_DEVICE2HOST             0x80
#define USB_REQ_HOST2DEVICE             0x00

#define USB_REQ_TYPE_STANDARD           0x00
#define USB_REQ_TYPE_CLASS              0x20
#define USB_REQ_TYPE_VENDOR             0x40

#define USB_REQ_RECIPIENT_DEVICE        0x00
#define USB_REQ_RECIPIENT_INTERFACE     0x01
#define USB_REQ_RECIPIENT_ENDPOINT      0x02
#define USB_REQ_RECIPIENT_OTHER         0x03

#define USB_STATUS_SELFPOWERED          0x01
#define USB_STATUS_REMOTEWAKEUP         0x02

#define USB_STATUS_HALT                 0x01

#define FEATURE(f)                      (1 << f)

#define USB_ENDPOINT_HALT               0x00
#define USB_DEVICE_REMOTE_WAKEUP        0x01

struct usbd_device_request {
    u8 bmRequestType;
    u8 bRequest;
    u16 wValue;
    u16 wIndex;
    u16 wLength;
};

struct usbd_endpoint_descriptor {
    u8 bLength;
    u8 bDescriptorType;   // 0x5
    u8 bEndpointAddress;
    u8 bmAttributes;
    u16 wMaxPacketSize;
    u8 bInterval;
};

struct usbd_interface_descriptor {
    u8 bLength;
    u8 bDescriptorType;   // 0x04
    u8 bInterfaceNumber;
    u8 bAlternateSetting;
    u8 bNumEndpoints;
    u8 bInterfaceClass;
    u8 bInterfaceSubClass;
    u8 bInterfaceProtocol;
    u8 iInterface;
};

struct usbd_configuration_descriptor {
    u8 bLength;
    u8 bDescriptorType;   // 0x2
    u16 wTotalLength;
    u8 bNumInterfaces;
    u8 bConfigurationValue;
    u8 iConfiguration;
    u8 bmAttributes;
    u8 bMaxPower;
};

struct usbd_device_descriptor {
    u8 bLength;
    u8 bDescriptorType;   // 0x01
    u16 bcdUSB;
    u8 bDeviceClass;
    u8 bDeviceSubClass;
    u8 bDeviceProtocol;
    u8 bMaxPacketSize0;
    u16 idVendor;
    u16 idProduct;
    u16 bcdDevice;
    u8 iManufacturer;
    u8 iProduct;
    u8 iSerialNumber;
    u8 bNumConfigurations;
};

struct usbd_langid_descriptor {
    u8 bLength;
    u8 bDescriptorType;   // 0x03
    u8 bData[2];
};

typedef enum usbd_urb_status {
    USBD_URB_OK = 0,
    USBD_URB_IN_QUEUE,
    USBD_URB_ACTIVE,
    USBD_URB_CANCELLED,
    USBD_URB_ERROR,
    USBD_URB_STALLED,
    USBD_URB_RESET,
    USBD_URB_NOT_READY,
    USBD_URB_DISABLED,
} usbd_urb_status_t;

#define USBD_URB_SENDZLP        0x01    /* send a Zero Length Packet when urb is finished */

/* definition of the structure itself */
struct usbd_urb {
    struct usbd_endpoint_instance   *endpoint;
    u8                              *buffer;        // data received (OUT) or being sent (IN)
    u32                             buffer_length;  // maximum data expected for OUT
    u32                             actual_length;  // actual data received (OUT or being sent (IN)
    u32                             flags;
    usbd_urb_status_t               status;         // what is the current status of the urb
};

struct usbd_endpoint_instance {
    u8   bEndpointAddress;    // logical endpoint address 
    int  bmAttributes;        // endpoint type
    u16  wMaxPacketSize;      // packet size for requested endpoint
    struct usbd_urb *urb;     // active urb
    u32  planed;              // data will be sent in planed packet 
};
