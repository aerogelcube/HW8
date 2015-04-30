/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "system_definitions.h"
#include "app.h"

#include "i2c_master_int.h" //use "" to search for the proper directory
#include "i2c_display.h"
#include "accel.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

/* Recieve data buffer */
uint8_t receiveDataBuffer[64] APP_MAKE_BUFFER_DMA_READY;

/* Transmit data buffer */
uint8_t  transmitDataBuffer[64] APP_MAKE_BUFFER_DMA_READY;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

USB_DEVICE_HID_EVENT_RESPONSE APP_USBDeviceHIDEventHandler
(
    USB_DEVICE_HID_INDEX iHID,
    USB_DEVICE_HID_EVENT event,
    void * eventData,
    uintptr_t userData
)
{
    USB_DEVICE_HID_EVENT_DATA_REPORT_SENT * reportSent;
    USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED * reportReceived;

    /* Check type of event */
    switch (event)
    {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_SENT
             * pointer type containing details about the report that was
             * sent. */
            reportSent = (USB_DEVICE_HID_EVENT_DATA_REPORT_SENT *) eventData;
            if(reportSent->handle == appData.txTransferHandle )
            {
                // Transfer progressed.
                appData.hidDataTransmitted = true;
            }
            
            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_RECEIVED
             * pointer type containing details about the report that was
             * received. */

            reportReceived = (USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED *) eventData;
            if(reportReceived->handle == appData.rxTransferHandle )
            {
                // Transfer progressed.
                appData.hidDataReceived = true;
            }
          
            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

            /* For now we just accept this request as is. We acknowledge
             * this request using the USB_DEVICE_HID_ControlStatus()
             * function with a USB_DEVICE_CONTROL_STATUS_OK flag */

            USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* Save Idle rate recieved from Host */
            appData.idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*)eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData.usbDevHandle, & (appData.idleRate),1);

            /* On successfully reciveing Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function drvier returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */

            break;
        default:
            // Nothing to do.
            break;
    }
    return USB_DEVICE_HID_EVENT_RESPONSE_NONE;
}

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Host has de configured the device or a bus reset has happened.
             * Device layer is going to de-initialize all function drivers.
             * Hence close handles to all function drivers (Only if they are
             * opened previously. */
            //LED1 is RB15, LED2 is RB7
            BSP_LEDOn  (APP_USB_LED_1);
            BSP_LEDOn  (APP_USB_LED_2);
            
            appData.deviceConfigured = false;
            appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            break;

        case USB_DEVICE_EVENT_CONFIGURED:
            /* Set the flag indicating device is configured. */
            appData.deviceConfigured = true;

            /* Save the other details for later use. */
            appData.configurationValue = ((USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData)->configurationValue;

            /* Register application HID event handler */
            USB_DEVICE_HID_EventHandlerSet(USB_DEVICE_HID_INDEX_0, APP_USBDeviceHIDEventHandler, (uintptr_t)&appData);

            /* Update the LEDs: device is configured: green LED (RB15 is on)*/
            BSP_LEDOn (APP_USB_LED_1);
            BSP_LEDOff (APP_USB_LED_2);
            

            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* yellow LED (RB7 is on) */
            BSP_LEDOff (APP_USB_LED_1);
            BSP_LEDOn  (APP_USB_LED_2);
            
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */

            USB_DEVICE_Attach (appData.usbDevHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available */
            USB_DEVICE_Detach(appData.usbDevHandle);
            break;

        /* These events are not used in this demo */
        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    appData.usbDevHandle = USB_DEVICE_HANDLE_INVALID;
    appData.deviceConfigured = false;
    appData.txTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    appData.rxTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    appData.hidDataReceived = false;
    appData.hidDataTransmitted = true;
    appData.receiveDataBuffer = &receiveDataBuffer[0];
    appData.transmitDataBuffer = &transmitDataBuffer[0];
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks (void )
{

    /* Check if device is configured.  See if it is configured with correct
     * configuration value  */

    switch(appData.state)
    {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

            if(appData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.usbDevHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            if(appData.deviceConfigured == true)
            {
                /* Device is ready to run the main task */
                appData.hidDataReceived = false;
                appData.hidDataTransmitted = true;
                appData.state = APP_STATE_MAIN_TASK;

                /* Place a new read request. */
                USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                        &appData.rxTransferHandle, appData.receiveDataBuffer, 64);
            }
            break;

        case APP_STATE_MAIN_TASK:

            if(!appData.deviceConfigured)
            {
                /* Device is not configured */
                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else if( appData.hidDataReceived )
            {
                /* Look at the data the host sent, to see what
                 * kind of application specific command it sent. */
                
                switch(appData.receiveDataBuffer[0])
                {
                    case 0x80:
                    {
                        //keep in mind that receiveDataBuffer doesn't take the first byte
                        //of the HID buffer, so the indices are shifted. receiveDataBuffer[1]
                        //takes the value of buf[2]!

                        //row should not be > 64; else will fall off screen!
                        int row = receiveDataBuffer[1];
                        display_clear();

                        /*
                        char check = receiveDataBuffer[2];
                        char check1 = receiveDataBuffer[3];
                        char ones = (row%10)+'0';
                        char tens = row/10 + '0';
                        printOLED(check,row,0);
                        printOLED(ones,row,5);
                        */
                        
                        int i;
                        for (i = 0; i < 25; i++)
                        {
                            char check = receiveDataBuffer[i+2];
                            printOLED(check,row,5*i);
                        }
                        
                        
                        //transmit buffer has the same indices as buffer of HID
                        appData.transmitDataBuffer[1] = row;
                        
                        appData.hidDataTransmitted = false;
                        // Prepare the USB module to send the data packet to the host
                        USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
                                &appData.txTransferHandle, appData.transmitDataBuffer, 64 );

                        appData.hidDataReceived = false;
                        // Place a new read request.
                        USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );

                        break;
                    }
                    case 0x81:
                    {
                        //Accel axes
                        short accels[3]; // accelerations for the 3 axes
                        short mags[3]; // magnetometer readings for the 3 axes
                        short temp;

                        unsigned int elapsedticks = _CP0_GET_COUNT(); // read the core timer
                        //unsigned int elapsedns = elapsedticks * 50; // duration in ns, for 80 MHz SYSCLK
                        if (elapsedticks > 200000) //200000
                        {

                            // read the accelerometer from all three axes
                            // the accelerometer and the pic32 are both little endian by default
                            //(the lowest address has the LSB)
                            // the accelerations are 16-bit twos compliment numbers, the same as a short
                            acc_read_register(OUT_X_L_A, (unsigned char *) accels, 6);
                            // need to read all 6 bytes in one transaction to get an update.
                            acc_read_register(OUT_X_L_M, (unsigned char *) mags, 6);
                            // read the temperature data. Its a right justified 12 bit two's compliment number
                            acc_read_register(TEMP_OUT_L, (unsigned char *) &temp, 2);

                            char data[25];
                            sprintf(data, "%3.2f,%3.2f,%3.2f", accels[0]/16000.0,accels[1]/16000.0,accels[2]/16000.0);
                            //keep in mind that receiveDataBuffer doesn't take the first byte
                            //of the HID buffer, so the indices are shifted. receiveDataBuffer[1]
                            //takes the value of buf[2]!

                            int i=0;
                            while(data[i])
                            {
                                printOLED(data[i],28,5*i);
                                i++;
                            }
                            
                            appData.transmitDataBuffer[1] = 1;
                            //x
                            appData.transmitDataBuffer[2] = accels[0] >> 8;
                            appData.transmitDataBuffer[3] = accels[0] & 0xFF;
                            //y
                            appData.transmitDataBuffer[4] = accels[1] >> 8;
                            appData.transmitDataBuffer[5] = accels[1] & 0xFF;
                            //z
                            appData.transmitDataBuffer[6] = accels[2] >> 8;
                            appData.transmitDataBuffer[7] = accels[2] & 0xFF;
                            //transmit buffer has the same indices as buffer of HID
                            _CP0_SET_COUNT(0);
                        }
                        else
                        {
                            appData.transmitDataBuffer[1] = 0;
                            
                        }
                        
                        appData.hidDataTransmitted = false;
                        // Prepare the USB module to send the data packet to the host
                        USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
                                &appData.txTransferHandle, appData.transmitDataBuffer, 64 );

                        appData.hidDataReceived = false;
                        // Place a new read request.
                        USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                        break;
                    }
                    default:
                        appData.hidDataReceived = false;

                        // Place a new read request.
                        USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                        break;
                }
                
                
            }
        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}
 

/*******************************************************************************
 End of File
 */


