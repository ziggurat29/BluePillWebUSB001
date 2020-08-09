@rem There are several hacks made that are outside the 'protected' areas of
@rem generated code, alas, and will be overwritten each time STM32CubeMX is
@rem re-run (e.g. if you change some settings).  This will restore those hacks.

@rem put in our (expanded) heap manager
@rem the F1 freertos middleware does not have the 'advanced' config option
@rem that would otherwise allow us to avoid this delete/copy
del ..\Middlewares\Third_Party\FreeRTOS\Source\portable\MemMang\heap_4.c
@rem really only need to do this once
copy heap_x.c.MyHeap ..\src\heap_x.c

@rem update the linker script to expand to the undocumented 128 KiB flash, and
@rem reserve the last page for our pseudo-eeprom for persistent settings
@rem really only need to do this once
copy STM32F103C8Tx_FLASH.ld.MyLinkerScript ..\STM32F103C8Tx_FLASH.ld

@rem apply the hacks to the CubeMX USB middleware
del ..\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Inc\usbd_cdc.h
copy usbd_cdc.h.MyCDCExt ..\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Inc\usbd_cdc.h

del ..\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Src\usbd_cdc.c
copy usbd_cdc.c.MyCDCExt ..\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Src\usbd_cdc.c

del ..\Src\usbd_cdc_if.c
copy usbd_cdc_if.c.MyCDCExt ..\Src\usbd_cdc_if.c
