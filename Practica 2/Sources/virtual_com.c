/******************************************************************************
 *
 * Freescale Semiconductor Inc.
 * (c) Copyright 2004-2010 Freescale Semiconductor, Inc.
 * ALL RIGHTS RESERVED.
 *
 **************************************************************************//*!
 *
 * @file virtual_com.c
 *
 * @author
 *
 * @version
 *
 * @date May-28-2009
 *
 * @brief  The file emulates a USB PORT as Loopback Serial Port.
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "hidef.h"          /* for EnableInterrupts macro */
#include "derivative.h"     /* include peripheral declarations */
#include "types.h"          /* Contains User Defined Data Types */
#include "usb_cdc.h"        /* USB CDC Class Header File */
#include "virtual_com.h"    /* Virtual COM Application Header File */
#include <stdio.h>          
#include <math.h>
#include "mma845x.h"
#include "iic.h"

#if (defined _MCF51MM256_H) || (defined _MCF51JE256_H)
#include "exceptions.h"
#endif

/* skip the inclusion in dependency state */
#ifndef __NO_SETJMP
	#include <stdio.h>
#endif
#include <stdlib.h>
#include <string.h>

/*****************************************************************************
 * Constant and Macro's - None
 *****************************************************************************/

/*****************************************************************************
 * Global Functions Prototypes
 *****************************************************************************/
void TestApp_Init(void);

/****************************************************************************
 * Global Variables
 ****************************************************************************/
#if HIGH_SPEED_DEVICE
uint_32 g_cdcBuffer[DIC_BULK_OUT_ENDP_PACKET_SIZE>>1];
#endif

///////////////////////// Demo global variables /* Global variables */
static byte CurrentDemo;                       /* Current demo, e.g. DEMO_FREEFALL */
static byte NewDemo;                           /* New demo requested */

static byte CurrentRange;                      /* Current range setting, e.g. FULL_SCALE_2G */
static byte NewRange;                          /* New range setting requested */

static byte CurrentReadMode;                   /* Current read size, e.g. READ_MODE_14BITS */
static byte NewReadMode;                       /* New read size requested */

static byte last_int_source_reg;               /* Last value of INT_SOURCE_REG as read by 
                                                  read_accelerometer_info() 
                                               */ 
static byte last_sysmod_reg;                   /* Last value of SYSMOD_REG */
static byte last_f_status_reg;                 /* Last value of F_STATUS_REG */
static byte last_transient_src_reg;            /* Last value of TRANSIENT_SRC_REG */
static byte last_pl_status_reg;                /* Last value of PL_STATUS_REG */
static byte last_pulse_src_reg;                /* Last value of PULSE_SRC_REG */
static byte last_ff_mt_src_1_reg;              /* Last value of FF_MT_SRC_1_REG */
static byte last_xyz_values [6];               /* Last values of X, Y and Z */ 


/*****************************************************************************
 * Local Types - None
 *****************************************************************************/

/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/
static void USB_App_Callback(uint_8 controller_ID,
                        uint_8 event_type, void* val);
static void USB_Notify_Callback(uint_8 controller_ID,
                        uint_8 event_type, void* val);
static void Virtual_Com_App(void);

//////////////////Demo fuctions/* Private function prototypes */
static void initialise_hardware (void);
static void initialise_accelerometer (void);
static void read_accelerometer_info (void); 
static void update_green_leds (Bool active);
static void configure_LEDs_for_PWM (void);
static void configure_LEDs_for_GPIO (void);
static void millisecond_delay (int delay);
static void leds_pulse (int delay);
static void leds_rotate (int count, int delay);
static void leds_show_tilt (byte *values);
static void leds_circle (int circle_number);
static void leds_cross (void);
static void leds_plus (void);                  
static void leds_all_on (void);
static void leds_all_off (void);
static void leds_show_shake (byte src);
static void wait_for_movement (void);
static byte bigToLittle (byte data);
/*****************************************************************************
 * Local Variables
 *****************************************************************************/
#ifdef _MC9S08JS16_H
#pragma DATA_SEG APP_DATA
#endif
/* Virtual COM Application start Init Flag */
static volatile boolean start_app = FALSE;
/* Virtual COM Application Carrier Activate Flag */
//static volatile boolean start_transactions = FALSE;
static volatile boolean start_transactions = TRUE;
/* Receive Buffer */
static uint_8 g_curr_recv_buf[DATA_BUFF_SIZE];
/* Send Buffer */
static uint_8 g_curr_send_buf[DATA_BUFF_SIZE];
/* Receive Data Size */
static uint_8 g_recv_size;
/* Send Data Size */
static uint_8 g_send_size;

//////////////////////////Demo accelerometer functions
/* Accelerometer I2C slave address */                                   
#define SlaveAddressIIC       (0x1d << 1)      /* Address = 1D in top 7 bits of byte */

/* Demo modes */
enum {
    DEMO_ORIENTATION,
    DEMO_SHAKE,
    DEMO_TAP,
    DEMO_FREEFALL,
    DEMO_TRANSIENT,
    NUM_DEMOS
};

/* Accelerometer can return 8-bit or 14-bit samples */
enum {
    READ_MODE_8BITS,
    READ_MODE_14BITS
};

/* Maximum value to use for PWM in PWM-controlled LED modes */
#define MAX_PWM_VALUE   500

/* 100ms debounce delay used for SW1 - SW3 push buttons */
#define DEBOUNCE_DELAY  100 

/*****************************************************************************
 * Local Functions
 *****************************************************************************/
 /******************************************************************************
 *
 *   @name        TestApp_Init
 *
 *   @brief       This function is the entry for the Virtual COM Loopback app
 *
 *   @param       None
 *
 *   @return      None
 *****************************************************************************
 * This function starts the Virtual COM Loopback application
 *****************************************************************************/

void TestApp_Init(void)
{
    uint_8   error;

    g_recv_size = 0;
    g_send_size= 0;
    DisableInterrupts;		
    #if (defined _MCF51MM256_H) || (defined _MCF51JE256_H)
     usb_int_dis();
    #endif
    /* Initialize the USB interface */
    error = USB_Class_CDC_Init(CONTROLLER_ID,USB_App_Callback,
                                NULL,USB_Notify_Callback, TRUE);
    if(error != USB_OK)
    {
        /* Error initializing USB-CDC Class */
        return;
    }
    EnableInterrupts;
	#if (defined _MCF51MM256_H) || (defined _MCF51JE256_H)
     usb_int_en();
    #endif
    
    millisecond_delay (1000);    
    /* Initialise system clocks, GPIO, etc */           
    initialise_hardware ();  //Esta función ha sido modificada
  
    /* Delay about 2 seconds to indicate that board is OK */
    leds_all_on ();
    
      
    /* Show that we're live */    
    leds_pulse (60);

    /* Force main loop to initially select DEMO_ORIENTATION, 14-bit accuracy, 2g sensitivity */
    CurrentReadMode = READ_MODE_14BITS;
    CurrentRange = FULL_SCALE_2G;
    
    initialise_accelerometer ();
}

static byte bigToLittle (byte data)
{
   byte tempData = (byte) 0;
   tempData = (data & 0b00000001) | ( (data & 0b00000010) << 1 )   | ( (data & 0b00000100) << 2 ) | ( (data & 0b00001000) << 3 )| ( (data & 0b00010000) << 4 )| ( (data & 0b00100000) << 5 )| ( (data & 0b01000000) << 6 )| ( (data & 0b10000000) << 7 );
   
   return tempData;
 
}
  

/******************************************************************************
 *
 *   @name        TestApp_Task
 *
 *   @brief       Application task function. It is called from the main loop
 *
 *   @param       None
 *
 *   @return      None
 *
 *****************************************************************************
 * Application task function. It is called from the main loop
 *****************************************************************************/
void TestApp_Task(void)
{
#if (defined _MC9S08JE128_H) || (defined _MC9S08JM16_H) || (defined _MC9S08JM60_H) || (defined _MC9S08JS16_H) || (defined _MC9S08MM128_H)
	if(USB_PROCESS_PENDING())
	{
		USB_Engine();
	}
#endif
        /* call the periodic task function */
        USB_Class_CDC_Periodic_Task();

       /*check whether enumeration is complete or not */
        if((start_app==TRUE) && (start_transactions==TRUE))
        {
            Virtual_Com_App();
        }
}

/******************************************************************************
 *
 *    @name       Virtual_Com_App
 *
 *    @brief      Implements Loopback COM Port
 *
 *    @param      None
 *
 *    @return     None
 *
 *****************************************************************************
 * Receives data from USB Host and transmits back to the Host
 *****************************************************************************/
static void Virtual_Com_App(void)
{
    static uint_8 status = 0;
//    uint_16 x, y, z;

  //millisecond_delay (1000);
            /* ---- Check for accelerometer event(s) ---- */
        //read_accelerometer_info (); 
            
        /* Wait for interrupt signalling accelerometer event or button press,
           unless another event has already occurred
        */
        DisableInterrupts;
        last_int_source_reg = IIC_RegRead(SlaveAddressIIC, INT_SOURCE_REG);   
        
        if (last_int_source_reg & SRC_DRDY_MASK) {            
             /* Read 14-bit XYZ results using a 6 byte IIC access */
             IIC_RegReadN(SlaveAddressIIC, OUT_X_MSB_REG, 6, (byte *) &last_xyz_values[0]);
                ////////////////////////////////////////////////////////    

                    /* Loopback Application Code */
                   /* if(g_recv_size)
                    { */
                    	/* Copy Received Buffer to Send Buffer */
                    	/*for (status = 0; status < g_recv_size; status++)
                    	{
                    		g_curr_send_buf[status] = g_curr_recv_buf[status];
                    	}
                    	g_send_size = g_recv_size;
                    	g_recv_size = 0;
                    }  */
                    /////////////////////////////////////////////
                         //x = (uint_16) (last_xyz_values [0] << 8) | last_xyz_values [1];
                         //y = (uint_16) (last_xyz_values [2] << 8) | last_xyz_values [3];
                         //z = (uint_16) (last_xyz_values [4] << 8) | last_xyz_values [5];
                         
                         g_curr_send_buf[0] = last_xyz_values [1];
                         g_curr_send_buf[1] = last_xyz_values [0];
                         g_curr_send_buf[2] = last_xyz_values [3];
                         g_curr_send_buf[3] = last_xyz_values [2];
                         g_curr_send_buf[4] = last_xyz_values [5];
                         g_curr_send_buf[5] = last_xyz_values [4];
                         
                        /* g_curr_send_buf[6] = (uint_8) last_xyz_values [0];
                         g_curr_send_buf[7] = (uint_8) last_xyz_values [1];
                         g_curr_send_buf[8] = (uint_8) last_xyz_values [2];
                         g_curr_send_buf[9] = (uint_8) last_xyz_values [3];
                         g_curr_send_buf[10] = (uint_8) last_xyz_values [4];
                         g_curr_send_buf[11] = (uint_8) last_xyz_values [5];
                         
                         g_curr_send_buf[12] = (uint_8) '\n';
                         g_curr_send_buf[13] = (uint_8) '\r'; */
                         
                         g_send_size = 6;
                         
                    
                    if(g_send_size)
                    {
                        /* Send Data to USB Host*/
                        uint_8 size = g_send_size;
                        g_send_size = 0;
                        status = USB_Class_CDC_Interface_DIC_Send_Data(CONTROLLER_ID,
                        g_curr_send_buf,size);
                        if(status != USB_OK)
                        {
                            /* Send Data Error Handling Code goes here */
                        }
                    }
        }
    EnableInterrupts;      
    

    return;
}

/******************************************************************************
 *
 *    @name        USB_App_Callback
 *
 *    @brief       This function handles Class callback
 *
 *    @param       controller_ID    : Controller ID
 *    @param       event_type       : Value of the event
 *    @param       val              : gives the configuration value
 *
 *    @return      None
 *
 *****************************************************************************
 * This function is called from the class layer whenever reset occurs or enum
 * is complete. After the enum is complete this function sets a variable so
 * that the application can start.
 * This function also receives DATA Send and RECEIVED Events
 *****************************************************************************/

static void USB_App_Callback (
    uint_8 controller_ID,   /* [IN] Controller ID */
    uint_8 event_type,      /* [IN] value of the event */
    void* val               /* [IN] gives the configuration value */
)
{
    UNUSED (controller_ID)
    UNUSED (val)
    if(event_type == USB_APP_BUS_RESET)
    {
        start_app=FALSE;
    }
    else if(event_type == USB_APP_ENUM_COMPLETE)
    {    	
#if HIGH_SPEED_DEVICE
    	// prepare for the next receive event
    	USB_Class_CDC_Interface_DIC_Recv_Data(&controller_ID,
    	                                      (uint_8_ptr)g_cdcBuffer, 
    	                                      DIC_BULK_OUT_ENDP_PACKET_SIZE);
#endif
        start_app=TRUE;
    }
    else if((event_type == USB_APP_DATA_RECEIVED)&&
            (start_transactions == TRUE))
    {
        /* Copy Received Data buffer to Application Buffer */
        USB_PACKET_SIZE BytesToBeCopied;
        APP_DATA_STRUCT* dp_rcv = (APP_DATA_STRUCT*)val;
        uint_8 index;
        BytesToBeCopied = (USB_PACKET_SIZE)((dp_rcv->data_size > DATA_BUFF_SIZE) ?
                                      DATA_BUFF_SIZE:dp_rcv->data_size);
        for(index = 0; index<BytesToBeCopied ; index++)
        {
            g_curr_recv_buf[index]= dp_rcv->data_ptr[index];
        }
        g_recv_size = index;
    }
    else if((event_type == USB_APP_SEND_COMPLETE) && (start_transactions == TRUE))
    {
        /* Previous Send is complete. Queue next receive */
#if HIGH_SPEED_DEVICE
    	(void)USB_Class_CDC_Interface_DIC_Recv_Data(CONTROLLER_ID, g_cdcBuffer, DIC_BULK_OUT_ENDP_PACKET_SIZE);
#else
        (void)USB_Class_CDC_Interface_DIC_Recv_Data(CONTROLLER_ID, NULL, 0);
#endif
    }

    return;
}

/******************************************************************************
 *
 *    @name        USB_Notify_Callback
 *
 *    @brief       This function handles PSTN Sub Class callbacks
 *
 *    @param       controller_ID    : Controller ID
 *    @param       event_type       : PSTN Event Type
 *    @param       val              : gives the configuration value
 *
 *    @return      None
 *
 *****************************************************************************
 * This function handles USB_APP_CDC_CARRIER_ACTIVATED and
 * USB_APP_CDC_CARRIER_DEACTIVATED PSTN Events
 *****************************************************************************/

static void USB_Notify_Callback (
    uint_8 controller_ID,   /* [IN] Controller ID */
    uint_8 event_type,      /* [IN] PSTN Event Type */
    void* val               /* [IN] gives the configuration value */
)
{
    UNUSED (controller_ID)
    UNUSED (val)
    if(start_app == TRUE)
    {
        if(event_type == USB_APP_CDC_CARRIER_ACTIVATED)
        {
            start_transactions = TRUE;
        }
        else if(event_type == USB_APP_CDC_CARRIER_DEACTIVATED)
        {
            //start_transactions = FALSE;
        }
    }
    return;
}

/* EOF */

///////////////////////////////// Definición de las funciones del programa demo

/************************************************************************
*       initialise_hardware - Initialise clock, GPIO, etc               *
*************************************************************************
; This routine initialises the main microprocessor - everything we need
; except the accelerometer which is done separately
*/

//Esta función ha sido modificada!!!!! Solo se ha dejado lo referente al IIC
static void initialise_hardware (void) 
{
        ;
        
    /* Enable clocks to all peripherals...
       Note: To conserve power, clocks to unused peripherals should be disabled.
             However, since this software is likely to be used by programmers
             new to the S08, we enable all the clocks here to make it easier
             for the programmer to get things working
    */
    
    /* SCGC1: CMT=1,TPM2=1,TPM1=1,ADC=1,DAC=1,IIC=1,SCI2=1,SCI1=1 */
    SCGC1 = 0xff;                                      
    
    /* SCGC2: USB=1,PDB=1,IRQ=1,KBI=1,PRACMP=1,TOD=1,SPI2=1,SPI1=1 */
    SCGC2 = 0xff;                                      
    
    /* SCGC3: VREF=1,CRC=1,FLS1=1,TRIAMP2=1,TRIAMP1=1,GPOA2=1,GPOA1=1 */
    SCGC3 = 0xff;                                      
  
 

    /* Initialize Inter-Integrated Circuit (IIC) used to talk to accelerometer */    
    SOPT3_IICPS = 1;                /* PTF3/PTF4 used for IIC comms */
    
    IICF = 0x02;                    /* IIC Frequency Divider Register
                                            MULT = 0b00 : mul = 1
                                            ICR  = 0b000010 : scl = 24
                                      
                                       IIC Frequency = Bus clock x mul / scl
                                                     = 4 MHz x 1 / 24
                                                     = 167kHz
                                    */
    
    IICC1 = 0x80;                   /* IIC Control Register 1 : Enable IIC module
                                         RSTA     =0  : Not generating repeat start
                                    */

    /* Enable keyboard interrupt from push buttons
       labelled 'g-Range Select', 'Data 8/14bit' and 'Channel Select'
    */
    KBI1SC = 0;                     /* Mask keyboard interrupts */
    KBI1ES = 0;                     /* Detect falling edge */
    PTAPE_PTAPE1 = 1;               /* Enable pull up on KBI1P0 (Port A pin 1) */
    PTAPE_PTAPE2 = 1;               /* Enable pull up on KBI1P1 (Port A pin 2) */
    PTAPE_PTAPE3 = 1;               /* Enable pull up on KBI1P2 (Port A pin 3) */

    KBI1PE = 0x07;                  /* Enable the KBI pins */
    KBI1SC_KB1ACK = 1;              /* Clear any false interrupt */
    KBI1SC_KB1IE = 1;               /* Enable keyboard interrupts */


   /* Configure interrupts from accelerometer:
          MMA_INT1 (KBI2P4 input, Port E pin 1)
          MMA_INT2 (KBI2P3 input, Port E pin 0)
    */
    KBI2SC = 0;                     /* Mask keyboard interrupts */
    KBI2ES = 0;                     /* Detect falling edge */
    PTEPE_PTEPE1 = 1;               /* Enable pull up on KBI2P4 (Port E pin 1) */
    PTEPE_PTEPE0 = 1;               /* Enable pull up on KBI2P3 (Port E pin 0) */

    KBI2PE = 0x18;                  /* Enable the KBI pins */
    KBI2SC_KB2ACK = 1;              /* Clear any false interrupt */
    KBI2SC_KB2IE = 0;               /* Disable MMA_INTx interrupts initially */

}

/************************************************************************
*       initialise_accelerometer - Perform demo-specific initialisation *
*************************************************************************
; This routine configures the accelerometer, the TPMs and the LEDs in
; preparation for the selected demo mode
*/
static void initialise_accelerometer (void)  
{ 
    byte    n, int_mask;
   
    /* Disable MMA_INTx interrupts so there's no danger of a spurious interrupt
       during reconfiguration 
    */               
    KBI2SC_KB2IE = 0;               

    /* Put MMA845xQ into Standby Mode */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, (IIC_RegRead(SlaveAddressIIC, CTRL_REG1) & ~ACTIVE_MASK));
    
    /* Configure sensor for:
         Sleep Mode Poll Rate of 1.56Hz (640ms) for maximum power saving 
         System Output Data Rate (ODR) of 200Hz (5ms)
         User-specified read mode (8-bit or 14-bit data)
    */
    n = DATA_RATE_5MS;          
    if (CurrentReadMode == READ_MODE_8BITS)
        n |= FREAD_MASK;
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, n);  
  
    /* Select user-specified sensitivity, 2g/4g/8g */
    IIC_RegWrite(SlaveAddressIIC, XYZ_DATA_CFG_REG, CurrentRange);
    
    /* Configure Sleep/Wake modes 
        SMODS1:0   : 1 1    Low power mode when asleep
        SLPE       : 0      Sleep disabled
        MODS1:0    : 0 0    Normal mode when awake
    */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG2, 0x18);          
    //IIC_RegWrite(SlaveAddressIIC, ASLP_COUNT_REG, 62);      /* Sleep after 20 seconds of inactivity */         

    /* Disable the FIFO */
    IIC_RegWrite(SlaveAddressIIC, F_SETUP_REG, 0x00);

    switch (CurrentDemo) {
    case DEMO_TRANSIENT:
    case DEMO_SHAKE:
    case DEMO_ORIENTATION:
        /* In all three of these demo modes we configure the accelerometer to detect
           movement:
           
                DEMO_TRANSIENT   - We configure the accelerometer to detect small movements
                                   of > 0.063g                   
                                   
                DEMO_SHAKE       - We configure the accelerometer to detect movements
                                   of > 0.5g
                                                      
                DEMO_ORIENTATION - We don't care about the movement data itself, but
                                   we use transient detection so that the accelerometer
                                   can tell us when the board isn't being used. When
                                   it transitions to Sleep mode, we can put the main
                                   processor to sleep too.
                                   
          By using the high-pass filter we can remove the constant effect of gravity,
           and only detect changes in acceleration. See Application Note AN4071.
        */

        /* ELE = 1     : Event latch enabled 
           ZTEFE = 1   : Raise event on any of Z, Y, X axes
           YTEFE = 1
           XTEFE = 1
           HBF_BYP = 0 : High-pass filter enabled
        */
        IIC_RegWrite(SlaveAddressIIC, TRANSIENT_CFG_REG, 0x1e);
        
        /* Set High-pass filter cut-off frequency for best sensitivity */
        IIC_RegWrite(SlaveAddressIIC, HP_FILTER_CUTOFF_REG, 0x03);   

        if (CurrentDemo == DEMO_SHAKE) {
            /* Transient is indicated when acceleration on one of the axes
               is above threshold 8 x 0.063g = 0.5g 
             */   
             IIC_RegWrite(SlaveAddressIIC, TRANSIENT_THS_REG, 8);
        } 
        else {
            /* Transient is indicated when acceleration on one of the axes
               is above threshold 1 x 0.063g - i.e. a small movement 
            */   
            IIC_RegWrite(SlaveAddressIIC, TRANSIENT_THS_REG, 1);
        }

        /* Internal debounce counter. For an ODR of 200Hz in Normal mode,
           motion is indicated after 5 x 1 = 5ms. 
        */
        IIC_RegWrite(SlaveAddressIIC, TRANSIENT_COUNT_REG, 1);

        /* Interrupt signalled on INT1 when transient detected */
        int_mask = INT_EN_TRANS_MASK;

        if (CurrentDemo == DEMO_ORIENTATION) {
            /* Interrupt also signalled when new data is ready */
             int_mask |= INT_EN_DRDY_MASK;
         
            /* Set up TPMs to produce edge-aligned PWM signals */
            configure_LEDs_for_PWM ();    
        }
        break;
        
    case DEMO_TAP:
        /* Configure the accelerometer to detect single and double taps... 
           (See Application Note AN4072)
        */
 
        /* Z configured for Single Tap and Double Tap with Latch enabled */
        IIC_RegWrite(SlaveAddressIIC, PULSE_CFG_REG, 0x70);    
        
        /* Enable low-pass filter */
        IIC_RegWrite(SlaveAddressIIC, HP_FILTER_CUTOFF_REG, PULSE_LPF_EN_MASK);   

        /* Set Z Threshold to 16 x 0.063g = 1.0g 
           
           Note: In a more sophisticated application we could dynamically
           track the orientation and configure X, Y Z with changing thresholds
        */
        IIC_RegWrite(SlaveAddressIIC, PULSE_THSZ_REG, 16);  

        /* Set the Pulse Time Limit for 30 ms at 200 Hz ODR in Normal Mode with the LPF Enabled  
           30 ms/5 ms = 6 counts
        */
        IIC_RegWrite(SlaveAddressIIC, PULSE_TMLT_REG, 6);
        
        /* Set the Pulse Latency Timer to 100 ms, 200 Hz ODR Normal Mode, LPF Enabled  
           100 ms/10 ms = 10 counts
        */
        IIC_RegWrite(SlaveAddressIIC, PULSE_LTCY_REG, 10);
        
        /* Set the Pulse window to 400 ms, 200 Hz Normal Mode, LPF Enabled
           400 ms/10 ms = 50 counts
        */
        IIC_RegWrite(SlaveAddressIIC, PULSE_WIND_REG, 50);

        /* Interrupt signalled on INT1 when pulse detected */
        int_mask = INT_EN_PULSE_MASK;
        break;
        
    case DEMO_FREEFALL:
        /* Configure accelerometer for linear freefall detection... 
           [Note that the accelerometer can also detect tumbling, as
            described in Application Note AN4070]
        */
        
        /* ELE = 1   : Event latch enabled 
           OAE = 0   : Detect Freefall
           ZEFE = 1  : Raise event on any of Z, Y, X axes
           YEFE = 1
           XEFE = 1
        */
        IIC_RegWrite(SlaveAddressIIC, FF_MT_CFG_1_REG, 0xb8);   
        
        /* Freefall is indicated when acceleration on all three axes
           falls below threshold 3 x 0.063g = 0.19g 
        */   
        IIC_RegWrite(SlaveAddressIIC, FT_MT_THS_1_REG, 3);
        
        /* Internal debounce counter. For an ODR of 200Hz in Normal mode,
           freefall is indicated after 5 x 20 = 100ms of falling - a drop
           of a few centimetres - See Application Note AN4070
        */
        IIC_RegWrite(SlaveAddressIIC, FF_MT_COUNT_1_REG, 20);

        /* Interrupt signalled on INT1 when Freefall detected */
        int_mask = INT_EN_FF_MT_1_MASK;
        break;
    }
   
    /* Configure interrupts */
    int_mask |= INT_EN_ASLP_MASK;                           /* Also generate interrupt on 
                                                               Sleep <--> Wake transition 
                                                            */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG4, int_mask);
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG5, 0xfd);         /* All interrupts mapped to MMA_INT1 */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG3, 0x78);         /* Active-low interrupts, all functions 
                                                               wake system 
                                                            */  

    /* Throw away any stale interrupt info */
    last_int_source_reg = 0;
    
    /* Put MMA845xQ into Active Mode */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, (IIC_RegRead(SlaveAddressIIC, CTRL_REG1) | ACTIVE_MASK));

    /* Enable MMA_INTx interrupts */
    KBI2SC_KB2IE = 1;               
}

/************************************************************************
*       read_accelerometer_info - Read any state change info            *
*************************************************************************
; This routine reads the accelerometer's INT_SOURCE_REG to check whether
; any events have occured. It then reads any event-specific data in order
; to clear the interrupts.
;
; Values are returned in the following global variables:
;
;       last_int_source_reg       - INT_SOURCE_REG
;       last_sysmod_reg           - SYSMOD_REG                     
;       last_f_status_reg         - F_STATUS_REG                    
;       last_transient_src_reg    - TRANSIENT_SRC_REG               
;       last_pl_status_reg        - PL_STATUS_REG                   
;       last_pulse_src_reg        - PULSE_SRC_REG                   
;       last_ff_mt_src_1_reg      - FF_MT_SRC_1_REG                 
;       last_xyz_values           - OUT_X_MSB_REG, etc          
;
; See the comments in the routine 'accelerometer_isr' for more information 
*/
static void read_accelerometer_info (void)
{
    /* Which source caused an interrupt, if any ? */
    last_int_source_reg = IIC_RegRead(SlaveAddressIIC, INT_SOURCE_REG);

    /* Sleep <--> Wake transition detected ? */
    if (last_int_source_reg & SRC_ASLP_MASK) {            
        /* Yes - Clear the event */
        last_sysmod_reg = IIC_RegRead(SlaveAddressIIC, SYSMOD_REG);
    } 
    
    /* FIFO event detected ? */
    if (last_int_source_reg & SRC_FIFO_MASK) {            
        /* Yes - Clear the event 
           Note that our demos don't use this event, so no further processing is required
        */
        last_f_status_reg = IIC_RegRead(SlaveAddressIIC, F_STATUS_REG);
    } 
    
    /* Transient detected ? */
    if (last_int_source_reg & SRC_TRANS_MASK) {            
        /* Yes - Clear the transient event */
        last_transient_src_reg = IIC_RegRead(SlaveAddressIIC, TRANSIENT_SRC_REG);
    } 
    
    /* Landscape/portrait orientation change detected ? */
    if (last_int_source_reg & SRC_LNDPRT_MASK) {            
        /* Yes - Clear the event 
           Note that our demos don't use this event, so no further processing is required
        */
        last_pl_status_reg = IIC_RegRead(SlaveAddressIIC, PL_STATUS_REG);
    } 
    
    /* Tap/pulse event detected ? */
    if (last_int_source_reg & SRC_PULSE_MASK) {
        /* Yes - Clear the pulse event */
        last_pulse_src_reg = IIC_RegRead(SlaveAddressIIC, PULSE_SRC_REG);
    }
    
    /* Freefall detected ? */
    if (last_int_source_reg & SRC_FF_MT_1_MASK) {            
        /* Yes - Clear the freefall event */
        last_ff_mt_src_1_reg = IIC_RegRead(SlaveAddressIIC, FF_MT_SRC_1_REG);
    }             

    /* Freefall detected ? */
    if (last_int_source_reg & SRC_DRDY_MASK) {            
        /* Yes - read the XYZ data to clear the event */
         if (CurrentReadMode == READ_MODE_14BITS) {
             /* Read 14-bit XYZ results using a 6 byte IIC access */
             IIC_RegReadN(SlaveAddressIIC, OUT_X_MSB_REG, 6, (byte *) &last_xyz_values[0]);
         }
         else {
             /* Read 8-bit XYZ results using a 3 byte IIC access */
             IIC_RegReadN(SlaveAddressIIC, OUT_X_MSB_REG, 3, (byte *) &last_xyz_values[0]);
         }
    }      
}

/************************************************************************
*       update_green_leds - Update status of green indicator LEDs       *
*************************************************************************
; The green LEDs are used to indicate what mode the board's in:
;
;       LEDs D1-D2 indicate the accelerometer read mode 
;       LEDs D3-D5 indicate the accelerometer sensitivity 
;       LEDs D6-D10 indicate the current demo mode
;
; Arguments:
;   Bool       active        - Specifies whether the board is active 
;                              or asleep (turn all green LEDs off) 
;                           
*/
static void update_green_leds (Bool active)
{
    /* Green LEDs all off initially */
    PTAD_PTAD0 = 1;                             /* 8-bit/14-bit data LEDs */
    PTAD_PTAD4 = 1;             

    PTFD_PTFD6 = 1;                             /* 2g/4g/8g LEDs */
    PTFD_PTFD7 = 1;
    PTGD_PTGD0 = 1;
    
    PTAD_PTAD5 = 1;                             /* Channel select LEDs */
    PTAD_PTAD6 = 1;
    PTAD_PTAD7 = 1;
    PTBD_PTBD0 = 1;
    PTBD_PTBD1 = 1;

    if (active) {
        /* Use green LEDs D1-D2 to indicate current accelerometer read mode. 
           NB: Internally the accelerometer always uses 14 bits, but it can be configured to
           only return 8 bits to achieve a faster IIC transfer speed)
        */
        if (CurrentReadMode == READ_MODE_8BITS) 
            PTAD_PTAD0 = 0;             /* Turn on '8-bit data' LED */
        else
            PTAD_PTAD4 = 0;             /* Turn on '14-bit data' LED */
            
        /* Use green LEDs D3-D5 to indicate current accelerometer sensitivity - 2g, 4g or 8g */
        switch (CurrentRange) {
        case FULL_SCALE_2G:
            /* Turn on '2g' green LED indicator */
            PTFD_PTFD6 = 0;
            break;
             
        case FULL_SCALE_4G:     
            /* Turn on '4g' green LED indicator */
            PTFD_PTFD7 = 0;
            break;
             
        case FULL_SCALE_8G:
            /* Turn on '8g' green LED indicator */
            PTGD_PTGD0 = 0;
            break;
        }
             
        /* Use green LEDs D6-D10 to indicate current demo mode */   
        switch (CurrentDemo) {
        case DEMO_ORIENTATION:
            /* Turn 'Orientation' LED on */
            PTAD_PTAD7 = 0;
            break;
                
        case DEMO_SHAKE:
            /* Turn 'Shake' LED on */
            PTBD_PTBD0 = 0;
            break;
                
        case DEMO_TAP:
            /* Turn 'Tap' LED on */
            PTBD_PTBD1 = 0;
            break;
                
        case DEMO_FREEFALL:
            /* Turn 'Freefall' LED on */
            PTAD_PTAD5 = 0;
            break;
                
        case DEMO_TRANSIENT:
            /* Turn 'Transient' LED on */
            PTAD_PTAD6 = 0;
            break;
                
        }
    }
}

/************************************************************************
*       millisecond_delay - Pause execution for specified time          *
*************************************************************************
; NB: Millisecond timings are approximate only. 
;
; Arguments:
;   int        delay        - Approximate number of milliseconds to delay
*/
static void millisecond_delay (int delay)
{
    unsigned short count;
    
    /* Configure TPM1 as a free-running counter with approximately 32 microsecond tick... */  
    
    while (delay > 0) {
        if (delay > (0xffffUL/32)) {
            /* Need to loop multiple times to achieve full delay because a value 
               of 2048 or more would overflow 16 bits in "count = (count << 5) - count" below 
            */
            count = (0xffffUL/32);
            delay -= (int) (0xffffUL/32);
        }
        else {
            /* Do the whole thing in one hit */  
            count = (unsigned short) delay;
            delay = 0;
        }
        
        /* Multiply by 31, since 31 ticks = 31 x 32 microseconds = 0.992 ms,
           or approximately 1 ms. We don't care about the slight inaccuracy 
        */   
        count = (count << 5) - count;  
            
        TPM1SC = 0;                     /* Stop the timer */  
        TPM1SC_TOF = 0;                 /* Clear the TOF flag */  
             
        TPM1C0SC = 0;                   /* Configure the timer */  
        TPM1MOD = count;           
        TPM1CNT = 0;
        TPM1SC = 0x0f;                  /* Restart the timer: CLKS = 01 (Use BUSCLK), PS=111 (Divide-by-128)
                                           Frequency = fbus / 128, so period = 128 / 4 MHz = 32 microseconds 
                                         */
        
        /* Wait for timer to expire */  
        while (!_TPM1SC.Bits.TOF)  
            ;
    }
}


/************************************************************************
*       leds_show_tilt - Use LEDs to indicate how board is tilted       *
*************************************************************************
; This routine uses the X and Y values to show how the board is tilted
;
; (a) The LEDs are illuminated to indicate the direction of gravity
;     using a PWM technique to illuminate most brightly the 'spoke'
;     that's closest in direction, and dimmly illuminate the others
;
; (b) The number of LEDs illuminated along a spoke indicates how much the
;     board is tipped. (This is also affected by the 2g/4g/8g button)
;      
; The following diagram shows the orientation of the board when held 
; vertically with screen-printed writing the right way up - i.e. "On"
; switch at the top.
;
; This orients the accelerometer in the so-called Portrait Down (PD) position
;
;
;             PTD5            PTD4           PTD3          X
;                  *           *           *           <-------+ 
;                    *         *         *                     |
;                      *       *       *                       | Y
;                        *     *     *                         |
;                          *   *   *                           V
;                            * * *                              
;            PTF2  * * * * * *   * * * * * *  PTD2             
;                            * * *                             | 
;                          *   *   *                           | Gravity
;                        *     *     *                         |
;                      *       *       *                       V
;                    *         *         *
;                  *           *           * 
;             PTF1            PTF0           PTE7
;
;                "FREESCALE"        "Debug S08 Rev.A"
;
;
; The PTxx pins are actually configurable as outputs of the two 
; Timer/Pulse Width Modulator modules (TPMs):
;
;       PTD4 : TPM1CH2
;       PTD3 : TPM1CH1
;       PTD2 : TPM1CH0
;       PTE7 : TPM2CH3
;       PTF0 : TPM2CH2
;       PTF1 : TPM2CH1
;       PTF2 : TPM2CH0
;       PTD5 : TPM1CH3
;
; We first calculate the direction in which gravity is pulling, relative to the board.
; This is then used to calculate PWM duty cycles for the LEDs, so that they illuminate 
; with different brightnesses to show which way is Down.
;
; Note that it's not necessary to go to all this effort if the software just wants to
; detect a change in orientation between Portait and Landscape mode. For this simple
; case, the accelerometer can detect the change automatically, without polling. See 
; Application Note AN4068 for details.
;
; Arguments:
;   byte        *values - Array of 3 pairs of values for X, Y and Z
;                         Each pair is in 14-bit binary form
*/
static void leds_show_tilt (byte *values)
{
    int     x, y, g, r, pwm, spoke; 
    double  theta, angle, brightness, x2, y2, radius;
    
    /* Get X and Y values */
    x = (values [0] << 8) | values [1];
    y = (values [2] << 8) | values [3];
    
    /* What reading would correspond to normal gravity (at FULL_SCALE_2G) ? */
    if (CurrentReadMode == READ_MODE_8BITS)
        g = 0x3f00;                         
    else
        g = 0x3ffc;
        
    /* Indicate how much the board is tipped by turning on more LEDs
       along a spoke as the board is tipped further over
    */
    x2 = (double) x / g; 
    y2 = (double) y / g;
    radius = sqrt (x2 * x2 + y2 * y2); 
    
    PTCD &= ~0x3f;
    for (r = 0; r < 6; r++) {
        if (radius > r / 6.0)
            PTCD |= 1 << r;
    }  
        
    /* Calculate direction in which gravity is pulling. We'll use this to choose
       which spokes to illuminate, and with what duty cycles.
       
       With reference to the diagram above, 0 degrees is at 12 o'clock
       and angles increase clockwise
    */ 
    if (x == 0 && y == 0)
        theta = 0.0;
    else {    
        theta = atan2(-x, -y);
        if (theta < 0.0)
            theta +=  2.0 * _M_PI;
    }
      
    /* Loop to illuminate spokes */
    for (spoke = 0; spoke < 8; spoke++) {
        /* Calculate angle of spoke in radians */
        angle = spoke * 45.0 * _M_PI / 180.0; 
       
        /* Calculate desired brightness */
        angle = fabs (theta - angle);
        if (angle >= _M_PI / 2.0 && angle <= 3 * _M_PI / 2.0) {
            /* More than 90 degress between gravity and this spoke, so always off */
            pwm = 0;
        } else {
            /* Calculate brightness based on angle between spoke and gravity */
            brightness = cos (angle);

            /* Since the eye's response to brightness is non-linear, 
               use a logarithmic dimming curve as used by DALI lighting
            */
            brightness = pow (10.0, 3.0 * (brightness - 1.0));
        
            /* Calculate PWM value */
            pwm = (int) (MAX_PWM_VALUE * brightness);
        }
       
        switch (spoke) {
        case 0: TPM1C2V = pwm;  break;
        case 1: TPM1C1V = pwm;  break;
        case 2: TPM1C0V = pwm;  break;
        case 3: TPM2C3V = pwm;  break;
        case 4: TPM2C2V = pwm;  break;
        case 5: TPM2C1V = pwm;  break;
        case 6: TPM2C0V = pwm;  break;
        case 7: TPM1C3V = pwm;  break;
        }
    }
}

/************************************************************************
*       configure_LEDs_for_PWM - Set up TPM1 and TPM2                   *
*************************************************************************
; Since the LEDs are connected to the Timer/Pulse Width Modulator modules (TPMs),
; we can configure the TPMs to generate an edge-aligned PWM signal. This
; means that the brightness of a 'spoke' of LEDs can be varied by changing
; the PWM duty cycle
*/
static void configure_LEDs_for_PWM (void)
{
    /* Configure TPM1 to generate PWM signals */
    TPM1SC = 0;                                 /* Stop the timer */ 

    /* Configure duty cycles of each channel
     
       NB: We choose a timer frequency = fbus / 16, so a period of 500 (MAX_PWM_VALUE)
       means that an LED will flash
       
            4 x 10E6 / (500 * 16) = 500 flashes/second 
            
       ...i.e. too fast to be detected by the human eye
    
    */
    TPM1MOD = MAX_PWM_VALUE; 
    
    /* Configure TPMxCnSC for each channel as follows
        CHnF    = 0  Not used
        CHnIE   = 0  No interrupt
        MSnB:A  = 10 Edge-aligned PWM
        ELSnB:A = 01 Active-low pulse
    */   
    TPM1C0SC = 0x24;       
    TPM1C1SC = 0x24;       
    TPM1C2SC = 0x24;       
    TPM1C3SC = 0x24;       
                                    
    TPM1SC = 0x0c;                  /* Restart the timer: CLKS = 01 (Use BUSCLK), PS=100 (Divide-by-16) */

    /* Repeat for TPM2 */
    TPM2SC = 0;      
    TPM2MOD = MAX_PWM_VALUE;
     
    TPM2C0SC = 0x24;       
    TPM2C1SC = 0x24;       
    TPM2C2SC = 0x24;       
    TPM2C3SC = 0x24;       
    
    TPM2SC = 0x0c;     
}

/************************************************************************
*       configure_LEDs_for_GPIO - Stop TPMs from driving LEDs           *
*************************************************************************
; Modifies the TPM channel configurations so that the TPMs no longer drive
; the LEDs. Control over the LEDs reverts to the GPIO method
*/
static void configure_LEDs_for_GPIO (void)
{
    /* Stop the TPMs to conserve power */
    TPM1SC = 0;                                  
    TPM2SC = 0; 
                                    
    /* ELSnB:A = 00 : Pin reverts to GPIO */ 
    TPM1C0SC = 0x00;       
    TPM1C1SC = 0x00;       
    TPM1C2SC = 0x00;       
    TPM1C3SC = 0x00;       

    TPM2C0SC = 0x00;       
    TPM2C1SC = 0x00;       
    TPM2C2SC = 0x00;       
    TPM2C3SC = 0x00;       
}

/************************************************************************
*       leds_circle - Turn on specified ring of LEDs                    *
*************************************************************************
; Turns on specified ring of LEDs, where 0 = inner ring, 5 = outer ring
;
; Arguments:
;   int        circle_number        - Ring to turn on
*/
static void leds_circle (int circle_number)
{
    byte    ptc_val;
 
    configure_LEDs_for_GPIO ();
    
    /* All LED 'spokes' active */
    PTDD &= ~0x3c;                  
    PTED &= ~0x80;
    PTFD &= ~0x07;

    /* Turn on requested circle; turn off all others */
    ptc_val = PTCD & ~0x3f;
    ptc_val |= 1 << circle_number;
    PTCD = ptc_val;           
}

/************************************************************************
*       leds_pulse - Show 'pulse' on LEDs                               *
*************************************************************************
; Displays an animated 'pulse' of light on the LEDs
;
; Arguments:
;   int        delay        - Delay between each stage
*/
static void leds_pulse (int delay)
{
    byte    ring, dir;

    configure_LEDs_for_GPIO ();

    ring = 0;                       /* Start off by illuminating inner ring */
    dir = 1;                        /* Start off with pulse spreading outwards */
    
    for (;;) { 
        /* Turn on current 'ring' of LEDs */
        leds_circle (ring);
        millisecond_delay (delay);
        
        /* Figure out which ring to turn on next */
        ring += dir;
        if (ring == 6) {
            /* Reached outer ring, so reverse direction */
            dir = -1;
            ring = 5;
        }
        else if (ring == 0) {
            /* All done. Turn all LEDs off */
            leds_all_off ();
            break;
        }
        
        /* If user has changed demo mode by pressing SW3, exit early
           to improve responsiveness
        */
        if (NewDemo != CurrentDemo)
            break;
    }
}

/************************************************************************
*       leds_cross - Turn on LEDs to form a "x" cross                   *
*************************************************************************
;
;             PTD5            PTD4           PTD3     
;                  *                       *          
;                    *                   *            
;                      *               *              
;                        *           *                
;                          *       *                  
;                            *   *                    
;            PTF2                            PTD2     
;                            *   *                    
;                          *       *                  
;                        *           *                
;                      *               *              
;                    *                   *
;                  *                       * 
;             PTF1            PTF0           PTE7
;
*/
static void leds_cross (void)
{
    configure_LEDs_for_GPIO ();
    
    /* Set active spokes */
    PTDD |= 0x3c; 
    PTDD &= ~(1 << 3);          /* PTD3 on */
    PTDD &= ~(1 << 5);          /* PTD5 on */
                     
    PTED &= ~(1 << 7);          /* PTE7 on */
    
    PTFD |= 0x07;
    PTFD &= ~(1 << 1);          /* PTF1 on */
    
    /* Turn on all LEDs in active spokes */
    PTCD |= 0x3f;         
}

/************************************************************************
*       leds_plus - Turn on LEDs to form a "+" plus sign                *
*************************************************************************
;
;             PTD5            PTD4           PTD3     
;                              *                      
;                              *                      
;                              *                      
;                              *                      
;                              *                      
;            PTF2  * * * * * *   * * * * * * PTD2     
;                              *                      
;                              *                      
;                              *                      
;                              *                      
;                              *          
;                              *             
;             PTF1            PTF0           PTE7
;
*/
static void leds_plus (void)
{
    configure_LEDs_for_GPIO ();
    
    /* Set active spokes */
    PTDD |= 0x3c; 
    PTDD &= ~(1 << 2);          /* PTD2 on */
    PTDD &= ~(1 << 4);          /* PTD4 on */
                     
    PTED |= 0x80;
    
    PTFD |= 0x07;
    PTFD &= ~(1 << 0);          /* PTF0 on */
    PTFD &= ~(1 << 2);          /* PTF2 on */
    
    /* Turn on all LEDs in active spokes */
    PTCD |= 0x3f;         
}

/************************************************************************
*       leds_rotate - Show rotating 'spoke' pattern on LEDs             *
*************************************************************************
; Displays a rotating spoke pattern on the LEDs
;
; Arguments:
;   int        count        - Number of complete rotations to do
;   int        delay        - Delay between each stage
*/
static void leds_rotate (int count, int delay)
{
    int     i;
  
    for (i = 0; i < count * 8; i++) {
        if (i & 0x1)
            leds_cross ();
        else
            leds_plus ();
        
        millisecond_delay (delay);
        
        /* If user has changed demo mode by pressing SW3, exit early
           to improve responsiveness
        */
        if (NewDemo != CurrentDemo)
            break;
    }
    
    /* All spokes off */
    leds_all_off ();
}   

/************************************************************************
*       leds_all_on - Turn all LEDs on                                  *
*************************************************************************/
static void leds_all_on (void)
{
    configure_LEDs_for_GPIO ();
    PTDD &= ~0x3c;
    PTED &= ~0x80;
    PTFD &= ~0x07;
    PTCD |= 0x3f;
}

/************************************************************************
*       leds_all_off - Turn all LEDs off                                *
*************************************************************************/
static void leds_all_off (void)
{
    configure_LEDs_for_GPIO ();
    PTCD &= ~0x3f;
}

/************************************************************************
*       leds_show_shake - Show directional shake                        *
*************************************************************************
; This routine takes 'shake' data as returned in the accelerometer's
; Transient Source Register, and illuminates the LEDs to indicate the
; direction of the initial flick
;
; Reminder: With the board oriented so that the writing is the right way up
; and the On/Off switch is at the top, the accelerometer's X and Y
; directions are as follows:
;
; 
;             PTD5            PTD4           PTD3          X
;                  .           .           .           <-------+ 
;                    .         .         .                     |
;                      .       .       .                       | Y
;                        .     .     .                         |
;                          .   .   .                           V
;                            . . .                              
;            PTF2  . . . . . .   . . . . . .  PTD2             
;                            . . .                        
;                          .   .   .                      
;                        .     .     .                    
;                      .       .       .                  
;                    .         .         .
;                  .           .           . 
;             PTF1            PTF0           PTE7
;
;
; Note that more sophisticated directional shake detection would
; require analysis of the pattern of transients before and after
; the shake event...
;
; When the hand starts the shake, the board will experience 
; acceleration in a certain direction. A short while later the hand will
; stop the shake, causing deceleration. Usually the hand will slightly
; over-correct, so that the board 'bounces' and briefly accelerates
; in the opposite direction. Depending on the selected threshold
; programmed into TRANSIENT_THS_REG, the accelerometer may miss the
; initial shake and only detect the bounce. 
;
; See Application Note AN4071
;
*/
static void leds_show_shake (byte src)
{
    byte    i;
    
    /* All LEDs off */
    configure_LEDs_for_GPIO ();

    PTDD |= 0x3c;
    PTED |= 0x80;
    PTFD |= 0x07;
    PTCD |= 0x3f;

	/* Turn on LEDs to illustrate shake direction */
    if (src & ZTRANSE_MASK) {
        /* Shake in Z axis detected */
        if (src & ZTRANSEPOL_MASK)  
            leds_circle (5);
        else
            leds_circle (0);
    }
    else if (src & YTRANSE_MASK) {
        /* Shake in Y axis detected */
        if (src & YTRANSEPOL_MASK)  
            PTFD &= ~(1 << 0);
        else  
            PTDD &= ~(1 << 4);
   }
    else if (src & XTRANSE_MASK) {
        /* Shake in X axis detected */
        if (src & XTRANSEPOL_MASK)  
            PTFD &= ~(1 << 2);
        else  
            PTDD &= ~(1 << 2);
   }
    
    /* Hold LED pattern for 2 seconds */
    for (i = 0; i < 100; i++) {
        millisecond_delay (20);
        
        /* If user has changed demo mode by pressing SW3, exit early
           to improve responsiveness
        */
        if (NewDemo != CurrentDemo)
            break;
    }
    leds_all_off ();
}

/************************************************************************
*       button_press_isr - Interrupt handler for push buttons           *
*************************************************************************
; Interrupt handler for push buttons SW1-SW3 as follows:
;
;       g-Range Select (SW1) : Select 2g, 4g or 8g mode
;       Data 8/14bit   (SW2) : Configure accelerometer to return 8- or 14-bit data 
;       Channel Select (SW3) : Select a new demo
;
; In each case we just update a global variable, leaving it upto non-ISR
; code to change the configuration at a convenient time. Note that we can't
; directly change the accelerometer here, because a transition from 
; STANDBY -> ACTIVE mode will cause certain registers to be clobbered
*/
interrupt void button_press_isr (void)
{
    byte    val;
    
    /* Acknowledge the interrupt */  
    val = PTAD;
    KBI1SC_KB1ACK = 1;
    
    /* Which button was pressed ?  */
    val = ~val;                         /* Active low */

    if (val & (1 << 1)) {
        /* User pressed 'g-Range Select' button (SW1) 
           Request switch to next range setting
         */
         if (NewRange == CurrentRange) {
            switch (CurrentRange) {
            case FULL_SCALE_2G:
                NewRange = FULL_SCALE_4G;
                break;
            case FULL_SCALE_4G:
                NewRange = FULL_SCALE_8G;
                break;
            case FULL_SCALE_8G:
            default:
                NewRange = FULL_SCALE_2G;
                break;
            }
        }      
        else {
            /* Ignore key bounce. Main loop hasn't yet seen and responded
               to previous key press
            */
        }
    }

    if (val & (1 << 2)) {
        /* User pressed 'Data 8/14bit' button (SW2) 
           Request toggle current mode
        */
        if (NewReadMode == CurrentReadMode) {
            if (CurrentReadMode == READ_MODE_14BITS)  
                NewReadMode = READ_MODE_8BITS;    
            else    
                NewReadMode = READ_MODE_14BITS;            
        }
        else {
            /* Ignore key bounce. Main loop hasn't yet seen and responded
               to previous key press
            */
        }
    }
    
    if (val & (1 << 3)) {
        /* User pressed 'Channel Select' button (SW3)
           Request new demo
        */
        if (NewDemo == CurrentDemo) {
            /* Bump and wrap */
            NewDemo = CurrentDemo + 1;
            if (NewDemo == NUM_DEMOS)
                NewDemo = 0;
        }
        else {
            /* Ignore key bounce. Main loop hasn't yet seen and responded
               to previous key press
            */
        }
    } 
}

/************************************************************************
*       accelerometer_isr - Interrupt handler for accelerometer         *
*************************************************************************
; The accelerometer signals an interrupt by driving MMA_INT1 low
; low, which causes an edge-triggered interrupt via KBI2. The interrupt will
; wake the main processor up if it's sleeping.
;
; To tell the accelerometer that we've seen the interrupt, so that it stops
; driving MMA_INT1 low, it's necessary to read an event-specific register.  
; For example, to clear a 'Transient' interrupt you would read the TRANS_SRC 
; register.
; 
; However, this requires a bit of thought to achieve the best design:
;
; (a) It's good practice to keep ISRs as short as possible. We don't want to
;     perform IIC reads here if we can avoid it
;
; (b) Reading a register like TRANS_SRC also has another effect besides 
;     clearing MMA_INT1. Normally the bits in TRANS_SRC are latched to 
;     indicate which transient occurred. Reading TRANS_SRC un-freezes the bits 
;     and starts the next accelerometer transient detection, so that re-reading
;     it wouldn't return the same result.
; 
; The solution relies on recognising that although the accelerometer will 
; continue to drive MMA_INT1 low until we've cleared the interrupt, this won't
; cause a new falling edge on KBI2.
;
; Thus, provided we acknowledge the KBI2 interrupt here, the processor
; isn't immediately interrupted again. We can leave it up to non-interrupt 
; code in the routine 'read_accelerometer_info' to read TRANS_SRC, etc
*/
interrupt void accelerometer_isr (void)
{
    /* Acknowledge KBI2 interrupt */
    KBI2SC_KB2ACK = 1;
}

/************************************************************************
*       wait_for_movement - Wait until accelerometer indicates movement *
*************************************************************************
; This routine is called when the accelerometer indicates that no event of 
; the current type has occurred for 20 seconds. (For example if we're
; detecting 'taps', the routine is called if the user hasn't tapped the
; board for 20 seconds).
;
; We reconfigure the accelerometer to detect any movement, then put the
; CPU into a low power 'STOP 3' mode
;
; The subroutine doesn't return until the accelerometer indicates
; movement - e.g. the user has picked the board up.
*/
static void wait_for_movement (void)
{
    /* Configure accelerometer for motion detection in low power mode... */
      
    /* Disable MMA_INTx interrupts so there's no danger of a spurious interrupt
       during reconfiguration 
    */               
    DisableInterrupts;
    KBI2SC_KB2IE = 0;                  

    /* Put MMA845xQ into Standby Mode */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, (IIC_RegRead(SlaveAddressIIC, CTRL_REG1) & ~ACTIVE_MASK));
    
    /* ELE = 1     : Event latch enabled 
       ZTEFE = 1   : Raise event on any of Z, Y, X axes
       YTEFE = 1
       XTEFE = 1
       HBF_BYP = 0 : High-pass filter enabled
    */
    IIC_RegWrite(SlaveAddressIIC, TRANSIENT_CFG_REG, 0x1e);
    IIC_RegWrite(SlaveAddressIIC, HP_FILTER_CUTOFF_REG, 0x00);   
    
    /* Transient is indicated when acceleration on one of the axes
       is above threshold 1 x 0.063g  
    */   
    IIC_RegWrite(SlaveAddressIIC, TRANSIENT_THS_REG, 1);
    IIC_RegWrite(SlaveAddressIIC, TRANSIENT_COUNT_REG, 1);

    /* Interrupt signalled on INT1 when transient detected, or on Sleep <--> Wake transition */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG4, (INT_EN_TRANS_MASK | INT_EN_ASLP_MASK));
    
    /* Since reconfiguring the accelerometer here will wake it up again,
       tell it to go back to sleep as soon as possible
    */ 
    IIC_RegWrite(SlaveAddressIIC, ASLP_COUNT_REG, 1);              

    /* Put MMA845xQ back into Active Mode */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, (IIC_RegRead(SlaveAddressIIC, CTRL_REG1) | ACTIVE_MASK));

    /* Enable MMA_INTx interrupts */
    last_int_source_reg = 0;
    KBI2SC_KB2IE = 1;               

    /* Indicate that we're in sleep mode */
    leds_cross ();
    millisecond_delay (2000);
    leds_all_off ();
    
    /* Turn green LEDs off too */
    update_green_leds (FALSE);

    /* Sleep until accelerometer wakes up */
    for (;;) {
    
        /* Wait until we have event from accelerometer or push button */
        DisableInterrupts;
        last_int_source_reg = IIC_RegRead(SlaveAddressIIC, INT_SOURCE_REG);   
        if (last_int_source_reg == 0)  
            asm "stop";
        read_accelerometer_info ();
        EnableInterrupts;

        /* Accelerometer wake-up event? */
        if (last_int_source_reg & SRC_ASLP_MASK) {
            if ((last_sysmod_reg & SYSMOD_MASK) != 0x02) {
                /* Yes */  
                break;
            }
        }
       
        /* Did user press one of the buttons SW1 - SW3 ?
           If so, return to active mode
        */
        if (NewRange != CurrentRange || NewReadMode != CurrentReadMode || NewDemo != CurrentDemo) 
            break;
   }

    /* Indicate that we're awake again */
    leds_plus ();      
    millisecond_delay (2000);    
    leds_all_off ();
}             


