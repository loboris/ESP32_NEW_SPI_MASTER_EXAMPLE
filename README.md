### Modified spi_master driver and example

#### Driver modifications

* Number of the devices attached to the bus which uses **hw** CS can be 3 (*NO_CS*) as in original driver
* Additional devices which does not use **hw** CS can be attached to the bus, up to *NO_DEV*
* Additional parameter **spics_ext_io_num** is added to *spi_device_interface_config_t*, it is automatically handled by **spi_device_select**/**spi_device_deselect** functions
* Devices can have individual **bus_configs**, so different mosi, miso, sck pins can be configured for each device
* Because of that, **spi_bus_add_device()** function has added parameter *bus_config*
* Reconfiguring the bus is done automaticaly when using *spi_device_select()* function
* Device select/deselect functions **spi_device_select**/**spi_device_deselect** are added
* Added **NonDMA/no Transactions** mode for direct transfer on the spi bus
* Added some helper functions

#### Queued mode & DMA transfer

The essence of the interface to a device is a set of queues; one per device.
The idea is that to send something to a SPI device, you allocate a transaction descriptor. It contains some information about the transfer like the lenghth, address, command etc, plus pointers to transmit and receive buffer. The address of this block gets pushed into the transmit queue. 
The SPI driver does its magic, and sends and retrieves the data eventually.
The data gets written to the receive buffers, if needed the transaction descriptor is modified to indicate returned parameters and the entire thing goes into the return queue, where whatever software initiated the transaction can retrieve it.

The entire thing is run from the SPI interrupt handler. If SPI is done transmitting/receiving but nothing is in the queue, it will not clear the SPI interrupt but just disable it. This way, when a new thing is sent, pushing the packet into the send queue and re-enabling the interrupt will trigger the interrupt again, which can then take care of the sending.


#### Non Queued mode without DMA

Direct transfer to SPI device is possible which can coexist with queued data transfer.
Non queued transfers uses the semaphore (taken in select function & given in deselect function) to protect the transfer.

Main function in this mode is **spi_transfer_data()**
* it has **trans** parameter which has to be configured the same way as for *spi_device_transmit()* function.
* uses devices **pre_cb** and **post_cb**
* transfer data bit size must be **8-bit multiples**
* if the divice is configured for half duplex mode (*cfg.flags = SPI_DEVICE_HALFDUPLEX*) data is read after sending (if any), otherwise, data is read while sending
* if device uses hw CS (*spics_io_num > 0*), CS is activated before transfer and deactivated after transfer
* if device uses external CS (*spics_io_num <= 0* & *spics_ext_io_num > 0*) CS must be handled with *spi_device_select*/*spi_device_deselect*
* there is **no limit** for transmit/receive buffer size

**Complete function decsriptions are available in the header file** *spi_master.h*

#### Example

To run the example, attach ILI9341 based display module to ESP32. Default pins used are:
* mosi: 23
* miso: 19
*  sck: 18
*   CS:  5 (display CS)
*   DC: 26 (display DC)
*  TCS: 25 (touch screen CS)

---

**If you want to use different pins, change them in** *tftfunc.h*

**if you dont have the touch screen, comment** *#define USE_TOUCH* in *spi_master_demo.c*

Using *make menuconfig* **select tick rate 1000** ( → Component config → FreeRTOS → Tick rate (Hz) ) to get more accurate timings

---

This code tests accessing ILI9341 based display in various modes and prints some timings.

Some fancy graphics is displayed on the ILI9341-based 320x240 LCD, lines, pixels and rectangles.

In DMA/Transactions mode this example demonstrate the use of both spi_device_transmit as well as spi_device_queue_trans/spi_device_get_trans_result as well as pre-transmit callbacks.

The usage of NonDMA/NoTransaction mode is also demonstrated and the speed is compared.
In this mode, sending individual pixels is more than 10 times faster than when using DMA/Transactions!
 
Reading the display content is demonstrated by comparing sent and read color line.
 
If Touch screen is available, reading the touch coordinates (non calibrated) is also demonstrated. Keep touching the display until the info is printed.
 
Some info about the ILI9341:
It has an C/D line, which is connected to a GPIO here. It expects this line to be low for a command and high for data. We use a pre-transmit callback here to control that line: every transaction has as the user-definable argument the needed state of the D/C line and just before the transaction is sent, the callback will set this line to the correct state.

---

**Example output:**

```
SPI: bus initialized
SPI: attached display device, speed=5000000
SPI: bus uses native pins: true
SPI: attached TS device, speed=2500000
SPI: display init...
OK
-------------
 Disp clock =  5.00 MHz ( 5.00)
 Lines(DMA) =   918  ms (240 lines of 320 pixels)
 Read check      OK
      Lines =  1154  ms (240 lines of 320 pixels)
 Read check      OK
Pixels(DMA) =  2371  ms (160x120)
     Pixels =   694  ms (160x120)
        Cls =   259  ms (320x240)
-------------
-------------
 Disp clock =  8.00 MHz ( 8.00)
 Lines(DMA) =   940  ms (240 lines of 320 pixels)
 Read check     Err
      Lines =  1058  ms (240 lines of 320 pixels)
 Read check      OK
Pixels(DMA) =  4161  ms (160x120)
     Pixels =   488  ms (160x120)
        Cls =   167  ms (320x240)
 Touched at (801,810) [row TS values]
-------------
-------------
 Disp clock = 16.00 MHz (16.00)
 Lines(DMA) =   942  ms (240 lines of 320 pixels)
 Read check     Err
      Lines =   978  ms (240 lines of 320 pixels)
 Read check      OK
Pixels(DMA) =  4363  ms (160x120)
     Pixels =   314  ms (160x120)
        Cls =    89  ms (320x240)
-------------
-------------
 Disp clock = 20.00 MHz (20.00)
 Lines(DMA) =   942  ms (240 lines of 320 pixels)
 Read check     Err
      Lines =   962  ms (240 lines of 320 pixels)
 Read check      OK
Pixels(DMA) =  4374  ms (160x120)
     Pixels =   279  ms (160x120)
        Cls =    73  ms (320x240)
 Touched at (1104,1525) [row TS values]
-------------
-------------
 Disp clock = 26.67 MHz (30.00)
 Lines(DMA) =   953  ms (240 lines of 320 pixels)
 Read check     Err
      Lines =   946  ms (240 lines of 320 pixels)
 Read check      OK
Pixels(DMA) =  5248  ms (160x120)
     Pixels =   247  ms (160x120)
        Cls =    58  ms (320x240)
-------------
-------------
 Disp clock = 40.00 MHz (40.00)
 Lines(DMA) =   953  ms (240 lines of 320 pixels)
 Read check     Err
      Lines =   930  ms (240 lines of 320 pixels)
 Read check      OK
Pixels(DMA) =  5244  ms (160x120)
     Pixels =   211  ms (160x120)
        Cls =    42  ms (320x240)
-------------
```
