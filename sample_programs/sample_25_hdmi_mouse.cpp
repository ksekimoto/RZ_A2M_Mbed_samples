/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2020 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
#include "sample_select.h"

#if (SAMPLE_PROGRAM_NO == 25)
// SAMPLE_PROGRAM_NO 25 : HDMI output & mouse cursor Sample
//

#include "mbed.h"
#include "EasyAttach_CameraAndLCD.h"
#include "USBHostMouse.h"
#include "r_cache_lld_rza2m.h"
#include "LCD_config_rgb_to_hdmi.h"
#include "DisplayBace.h"
#include "arrow_data.h"

/******************************************************************************
Macro definitions
******************************************************************************/
/* LED blinky */
#define BLINKING_RATE     500ms

/* Frame buffer */
#define FRAME_BUFFER_BYTE_PER_PIXEL0         (2)
#define FRAME_BUFFER_STRIDE0                 (((LCD_PIXEL_WIDTH * FRAME_BUFFER_BYTE_PER_PIXEL0) + 31u) & ~31u)
#define FRAME_BUFFER_BYTE_PER_PIXEL2         (1)
#define FRAME_BUFFER_STRIDE2                 (((LCD_PIXEL_WIDTH * FRAME_BUFFER_BYTE_PER_PIXEL2) + 31u) & ~31u)

/* mouse */
#define MAIN_FLG_MOUSE_IN       (0x00001000)

/* mouse disp */
#define    ARROW_BUFFER_AREA     (32u * 32u)
#define    CAUTION_WINDOW_WIDTH  (576u)
#define    CAUTION_WINDOW_HEIGHT (160u)


/******************************************************************************
Typedef definitions
******************************************************************************/
/* mouse */
typedef struct {
    int32_t x;
    int32_t y;
    uint8_t buttons;
} mouse_info_t;

/* mouse proc */
class Mouse {

public:
    /** Touch position structure */
    typedef struct {
        uint32_t x;          /**< The position of the x-coordinate. */
        uint32_t y;          /**< The position of the y-coordinate. */
        uint8_t  buttons;    /**< buttons. */
        bool     valid;      /**< Whether a valid data.. */
    } touch_pos_t;


    /** Create a Mouse object
     * 
     */
    Mouse(void){
    }

    /** Reset a Mouse object
     * 
     * @param x :The maximum value of the x-coordinate.
     * @param y :The maximum value of the y-coordinate.
     */
    void Reset(int32_t max_x = 800, int32_t max_y = 480);

    /** Attach a function to call when touch panel int
     *
     *  @param fptr A pointer to a void function, or 0 to set as none
     */
    void SetCallback(void (*fptr)(void));

   /** Get the coordinates
     *
     * @param touch_buff_num The number of structure p_touch.
     * @param p_touch Touch position information.
     * @return The number of touch points.
     */
    int GetCoordinates(touch_pos_t * p_touch);
};

/******************************************************************************
Private global variables and functions
******************************************************************************/
/* Main task */
static Thread mainTask(osPriorityNormal,1024*8);


/* Frame buffer */
DisplayBase Display;
static uint8_t user_frame_buffer0[FRAME_BUFFER_STRIDE0 * LCD_PIXEL_HEIGHT]__attribute((aligned(32)));  /* 32 bytes aligned */
static volatile int32_t vsync_count = 0;

static void Start_LCD_Display(void);

/* mouse proc */
static int32_t pos_x = 0;
static int32_t pos_y = 0;
static int32_t maximum_x = 0;
static int32_t maximum_y = 0;
static uint8_t  pressed_buttons = 0;
static void (*cbFunc)(void) = NULL;
static void onMouseEvent(uint8_t buttons, int8_t x, int8_t y, int8_t z);
static void mouse_task(void);
static Thread mouseTask(osPriorityNormal, 1024 * 2);

/* mouse disp */
static uint32_t    arrow_buffer[ARROW_BUFFER_AREA] __attribute__ ((section("NC_BSS")));

static DisplayBase::video_spea_sklym_t arrow_size = { 32,  32};
static DisplayBase::video_spea_skpsm_t arrow_pos  = {  0,   0};

/**************************************************************************//**
 * @fn          mouse_disp_init
 * @brief       Initialize mouse disp
 * @retval      none
 *****************************************************************************/
void mouse_disp_init(void)
{
    DisplayBase::rect_t rect;
    rect.vs = 0;
    rect.vw = LCD_PIXEL_HEIGHT;
    rect.hs = 0;
    rect.hw = LCD_PIXEL_WIDTH;
    Display.Graphics_Read_Setting_SPEA(
        DisplayBase::GRAPHICS_LAYER_3,
        &rect
    );

    memcpy((void *)arrow_buffer, (void *)g_arrow_data, ARROW_SIZE);

    Display.Graphics_Update_Window_SPEA(
        DisplayBase::GRAPHICS_LAYER_3,
        DisplayBase::WINDOW_01,
        DisplayBase::SPEA_ON,
        &arrow_size,
        &arrow_pos,
        arrow_buffer
    );
}

/**************************************************************************//**
 * @fn          set_mouse_disp
 * @brief       Receive mouse information, and move the sprite
 * @param[in]   pos_x          :x-coordinate
 * @param[in]   pos_y          :y-coordinate
 * @param[in]   buttons        :buttons (each bit)
 * @retval      none
 *****************************************************************************/
void set_mouse_disp(int32_t mouse_x, int32_t mouse_y, uint8_t buttons)
{
    arrow_pos.x = mouse_x;
    arrow_pos.y = mouse_y;
    Display.Graphics_Update_Window_SPEA(
        DisplayBase::GRAPHICS_LAYER_3,
        DisplayBase::WINDOW_01,
        DisplayBase::SPEA_ON,
        &arrow_size,
        &arrow_pos,
        arrow_buffer
    );
}

/**************************************************************************//**
 * @fn          set_mouse_info
 * @brief       Receive mouse information
 * @param[in]   pos_x          :x-coordinate
 * @param[in]   pos_y          :y-coordinate
 * @param[in]   buttons        :buttons (each bit)
 * @retval      none
 *****************************************************************************/
static void set_mouse_info(void)
{
    mainTask.flags_set(MAIN_FLG_MOUSE_IN);

}

/**************************************************************************//**
 * @fn          onMouseEvent
 * @brief       Callback function for mouse event
 * @param[in]   buttons        :buttons (each bit)
 * @param[in]   x              :x amount of change
 * @param[in]   y              :y amount of change
 * @param[in]   x              :z amount of change
 * @retval      none
 *****************************************************************************/
static void onMouseEvent(uint8_t buttons, int8_t x, int8_t y, int8_t z) {

    /* X */
    pos_x += x;
    if (pos_x < 0) {
        pos_x = 0;
    }
    if (pos_x > (maximum_x - 1)) {
        pos_x = maximum_x - 1;
    }

    /* Y */
    pos_y += y;
    if (pos_y < 0) {
        pos_y = 0;
    }
    if (pos_y > (maximum_x - 1)) {
        pos_y = (maximum_x - 1);
    }
    
    if( cbFunc != NULL ) {
        cbFunc();
    }
    pressed_buttons = buttons;
    set_mouse_disp(pos_x, pos_y, buttons);

}
/**************************************************************************//**
 * @fn          SetCallback
 * @brief       register callback function for mouse event
 * @retval      none
 *****************************************************************************/
void Mouse::SetCallback(void (*fptr)(void))
{
    cbFunc = fptr;
}
/**************************************************************************//**
 * @fn          SetCallback
 * @brief       register callback function for mouse event
 * @retval      none
 *****************************************************************************/
int Mouse::GetCoordinates(touch_pos_t * p_touch)
{
    if (p_touch!=NULL) {
        p_touch->valid = true;
        /* X */
        if (pos_x > 0) {
            p_touch->x = pos_x;
        }
        else {
            p_touch->valid = false;
        }
        /* Y */
        if (pos_y > 0) {
            p_touch->y = pos_y;
        }
        else {
            p_touch->valid = false;
        }
        p_touch->buttons = pressed_buttons;
        p_touch->y = pos_y;
        return 0;
    }
    return -1;
}
/**************************************************************************//**
 * @fn          mouse_task
 * @brief       mouse task
 * @retval      none
 *****************************************************************************/
static void mouse_task(void) {
    USBHostMouse mouse;
    mouse_disp_init();
    while (1) {
        /* try to connect a USB mouse */
        while (!mouse.connect()) {
            ThisThread::sleep_for(500ms);
        }

        /* when connected, attach handler called on mouse event */
        mouse.attachEvent(onMouseEvent);

        /* wait until the mouse is disconnected */
        while (mouse.connected()) {
            ThisThread::sleep_for(500ms);
        }
    }
}

/**************************************************************************//**
 * @fn          Mouse
 * @brief       Mouse class constructor
 * @retval      none
 *****************************************************************************/
void Mouse::Reset(int32_t max_x, int32_t max_y) {
    

    pos_x = max_x / 2;
    pos_y = max_y / 2;
    maximum_x = max_x;
    maximum_y = max_y;
    mouseTask.start(callback(mouse_task));
}

/**************************************************************************//**
 * @fn          IntCallbackFunc_LoVsync
 * @brief       Callback function for each Vsync
 * @param[in]   int_type          :not used
 * @retval      none
 *****************************************************************************/
static void IntCallbackFunc_LoVsync(DisplayBase::int_type_t int_type) {
    /* Interrupt callback function for Vsync interruption */
    if (vsync_count > 0) {
        vsync_count--;
    }
}

#if 0 /* not used */
/**************************************************************************//**
 * @fn          Wait_Vsync
 * @brief       Waiting Vsync
 * @param[in]   wait_count          :wait count
 * @retval      none
 *****************************************************************************/
static void Wait_Vsync(const int32_t wait_count) {
    /* Wait for the specified number of times Vsync occurs */
    vsync_count = wait_count;
    while (vsync_count > 0) {
        ThisThread::sleep_for(2ms);
    }
}
#endif

/**************************************************************************//**
 * @fn          Start_LCD_Display
 * @brief       Start HDMI display
 * @retval      none
 *****************************************************************************/
static void Start_LCD_Display(void) {
    DisplayBase::rect_t rect;
    DisplayBase::graphics_error_t error;

    /* Interrupt callback function setting (Vsync signal output from scaler 0) */
    EasyAttach_Init(Display);
    error = Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_LO_VSYNC, 0, IntCallbackFunc_LoVsync);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }

    /* Create Layer 0 */
    memset(user_frame_buffer0, 0, sizeof(user_frame_buffer0));
    /* create base graphics */
    {
        #define LATTICE_HIGHT 16
        #define LATTICE_WIDTH 16
        uint32_t x = 0;
        uint32_t y = 0;
        for( y = 0; y < LCD_PIXEL_HEIGHT; y++ )
        {
            if ((y % LATTICE_HIGHT) == 0 )
            {
                for (x = 0; x < LCD_PIXEL_WIDTH * FRAME_BUFFER_BYTE_PER_PIXEL0; x+=2)
                {
                    /* PIXEL_FORMAT_RGB565 */
                    user_frame_buffer0[y * FRAME_BUFFER_STRIDE0 + x + 0] = 0x18;
                    user_frame_buffer0[y * FRAME_BUFFER_STRIDE0 + x + 1] = 0xE5;
                }
            }
            else
            {
                for (x = 0; x < FRAME_BUFFER_STRIDE0; x += LATTICE_WIDTH * FRAME_BUFFER_BYTE_PER_PIXEL0)
                {
                    /* PIXEL_FORMAT_RGB565 */
                    user_frame_buffer0[y * FRAME_BUFFER_STRIDE0 + x + 0] = 0x18;
                    user_frame_buffer0[y * FRAME_BUFFER_STRIDE0 + x + 1] = 0xE5;
                }
            }
        }
    }
    R_CACHE_L1DataCleanLine(user_frame_buffer0, sizeof(user_frame_buffer0));
    rect.vs = 0;
    rect.vw = LCD_PIXEL_HEIGHT;
    rect.hs = 0;
    rect.hw = LCD_PIXEL_WIDTH;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_0,
        (void *)user_frame_buffer0,
        FRAME_BUFFER_STRIDE0,
        DisplayBase::GRAPHICS_FORMAT_RGB565,
        DisplayBase::WR_RD_WRSWA_32_16_8BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_0);

    ThisThread::sleep_for(50ms);
    EasyAttach_LcdBacklight(true);
}

/**************************************************************************//**
 * @fn          main_task
 * @brief       Main task
 * @retval      none
 *****************************************************************************/
void main_task(void)
{
    Start_LCD_Display();
    ThisThread::sleep_for(200ms);
    Mouse mouse;
    mouse.Reset(LCD_PIXEL_WIDTH, LCD_PIXEL_HEIGHT);
    mouse.SetCallback(set_mouse_info);
    Mouse::touch_pos_t pos;
    Mouse::touch_pos_t pos_last = {0};
    printf("Start sample.\n\r");
    
    while (true) {

        ThisThread::flags_wait_any_for(MAIN_FLG_MOUSE_IN, rtos::Kernel::wait_for_u32_forever);
        
        mouse.GetCoordinates(&pos);
        if (pos.valid == true) {
        
            /* print mouse information */
            printf("(%03d,%03d)",(int)pos.x, (int)pos.y);
            if (((pos.buttons & 0x01) == 0) && ((pos_last.buttons & 0x01) != 0)) {
                printf(" click off");
            }
            else if (((pos.buttons & 0x01) != 0) && ((pos_last.buttons & 0x01) == 0)) {
                printf(" click on");
            }
            printf("\n");
            /* backup mouse information */
            pos_last = pos;
        }
    }
}

/**************************************************************************//**
 * @fn          main
 * @brief       Wake up main task, blinks LED
 * @retval      none
 *****************************************************************************/
int main()
{
    mainTask.start(callback(main_task));
    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);

    while (true) {
        led = !led;
        ThisThread::sleep_for(BLINKING_RATE);
    }

}

#endif
