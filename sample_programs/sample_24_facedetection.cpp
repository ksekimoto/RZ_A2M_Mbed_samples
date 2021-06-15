/**********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO
 * THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2020 Renesas Electronics Corporation. All rights reserved.
 *********************************************************************************************************************/
/******************************************************************************************************************//**
 * @file          sample_24_facedetection.cpp
 * @version       1.00
 * @brief         .
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History : 17.12.2020 Version  Description
 *         : 01.01.2020 1.00     First Release
 *********************************************************************************************************************/

#include "sample_select.h"

#if (SAMPLE_PROGRAM_NO == 24)

#include "mbed.h"
#include "SdUsbConnect.h"
#include "EasyAttach_CameraAndLCD.h"
#include "AsciiFont.h"
#include "r_dk2_if.h"
#include "r_drp_simple_isp.h"
#include "face_detector.hpp"
#include "r_cache_lld_rza2m.h"
#include "FATFileSystem.h"
#include "EasyPlayback.h"
#include "EasyDec_WavCnv2ch.h"

#if !defined(TARGET_RZ_A2XX)
#error "DRP and MIPI are not supported."
#endif
#if (MBED_CONF_APP_CAMERA_TYPE != CAMERA_RASPBERRY_PI_832X480)
#error Please set the value of "camera-type" in "mbed_app.json" to "CAMERA_RASPBERRY_PI_832X480" and build.
#endif 

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
#define VIDEO_PIXEL_HW          (832u)
#define VIDEO_PIXEL_VW          (480u)

#define FRAME_BUFFER_STRIDE     (((VIDEO_PIXEL_HW * 1) + 31u) & ~31u)
#define FRAME_BUFFER_STRIDE_2   (((VIDEO_PIXEL_HW * 2) + 31u) & ~31u)
#define FRAME_BUFFER_HEIGHT     (VIDEO_PIXEL_VW)

#define DRP_FLG_CAMER_IN        (0x00000100)
#define AUDIO_FLG               (0x00000100)
#define DRP_NOT_FINISH          (0)
#define DRP_FINISH              (1)
#define TILE_0                  (0)

#define RESULT_BUFFER_BYTE_PER_PIXEL  (2u)

#define DEBUG_BAUDRATE          (115200)
#define FACE_DETECTOR_MODEL     "/storage/lbpcascade_frontalface.xml"
#define MOUNT_NAME              "storage"
#define FILE_NAME_LEN           (64)

using namespace cv;

static DisplayBase Display;
static uint8_t fbuf_bayer[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(128)));
static uint8_t fbuf_yuv[FRAME_BUFFER_STRIDE_2 * FRAME_BUFFER_HEIGHT]__attribute((aligned(32)));
static uint8_t fbuf_grayscale[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(32)));

static r_drp_simple_isp_t param_isp __attribute((section("NC_BSS")));
static void cb_drp_finish(uint8_t id);
static uint8_t drp_lib_id[R_DK2_TILE_NUM] = {0}; 
static volatile uint8_t drp_lib_status[R_DK2_TILE_NUM] = {DRP_NOT_FINISH};
static Thread drpTask(osPriorityNormal,(1024*33));
static Thread audioTask(osPriorityHigh);
static EasyPlayback AudioPlayer;
static Timer  g_detect_timer;
static Rect copysquare;


static bool draw_square = false;

static const uint32_t clut_data_resut[] = {0x00000000, 0xff00ff00};  // ARGB8888

/**********************************************************************************************************************
 * Function Name: intcallbackfunc_vfield
 * Description  : .
 * Argument     : int_type
 * Return Value : .
 *********************************************************************************************************************/
static void intcallbackfunc_vfield(DisplayBase::int_type_t int_type)
{
    drpTask.flags_set(DRP_FLG_CAMER_IN);
}
/******************************************************************************
 End of function intcallbackfunc_vfield
 *****************************************************************************/


/**********************************************************************************************************************
 * Function Name: cb_drp_finish
 * Description  : .
 * Argument     : id
 * Return Value : .
 *********************************************************************************************************************/
static void cb_drp_finish(uint8_t id)
{
    uint32_t tile_no;
    /* Change the operation state of the DRP library notified by the argument to finish */
    for (tile_no = 0; tile_no < R_DK2_TILE_NUM; tile_no++ )
    {
        if (drp_lib_id[tile_no] == id)
        {
            drp_lib_status[tile_no] = DRP_FINISH;
            break;
        }
        else
        {
            /* DO NOTHING */
        }
    }
    return;
}
/**********************************************************************************************************************
 End of function cb_drp_finish
 *********************************************************************************************************************/


/**********************************************************************************************************************
 * Function Name: start_video_camera
 * Description  : .
 * Return Value : .
 *********************************************************************************************************************/
static void start_video_camera(void)
{
    /* Video capture setting (progressive form fixed) */
    Display.Video_Write_Setting(
        DisplayBase::VIDEO_INPUT_CHANNEL_0,
        DisplayBase::COL_SYS_NTSC_358,
        /* Camera image buffer */
        (void *)fbuf_bayer,
        FRAME_BUFFER_STRIDE,
        DisplayBase::VIDEO_FORMAT_RAW8,
        DisplayBase::WR_RD_WRSWA_NON,
        VIDEO_PIXEL_VW,
        VIDEO_PIXEL_HW
    );
    EasyAttach_CameraStart(Display, DisplayBase::VIDEO_INPUT_CHANNEL_0);
}
/**********************************************************************************************************************
 End of function start_video_camera
 *********************************************************************************************************************/


/**********************************************************************************************************************
 * Function Name: start_lcd_display
 * Description  : .
 * Return Value : .
 *********************************************************************************************************************/
static void start_lcd_display(void)
{
    DisplayBase::rect_t rect;                                   //The relative position within the graphics display area


    rect.vs = 0;
    rect.vw = 480;
    rect.hs = 0;
    rect.hw = 800;

    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_0,
        /* Output image buffer on monitor */
        (void *)fbuf_yuv,
        FRAME_BUFFER_STRIDE_2,
        DisplayBase::GRAPHICS_FORMAT_YCBCR422,
        DisplayBase::WR_RD_WRSWA_32_16_8BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_0);

    ThisThread::sleep_for(50ms);
    EasyAttach_LcdBacklight(true);
}
/**********************************************************************************************************************
 End of function start_lcd_display
 *********************************************************************************************************************/


/**********************************************************************************************************************
 * Function Name: drawsquare
 * Description  : .
 * Arguments    : x
 *              : y
 *              : w
 *              : h
 *              : colour
 * Return Value : .
 *********************************************************************************************************************/

static void drawsquare(int x, int y, int w, int h, uint32_t const colour)
{
    uint32_t idx_base;
    uint32_t wk_idx;
    uint32_t i;
    uint32_t coller_pix[RESULT_BUFFER_BYTE_PER_PIXEL];  /* YCbCr 422 */

    idx_base = ((x & 0xfffffffe) + (VIDEO_PIXEL_HW * y)) * RESULT_BUFFER_BYTE_PER_PIXEL;

    /* Select color */
    coller_pix[0] = (colour >> 0) & 0xff;   
    coller_pix[1] = (colour >> 8) & 0xff;   
    coller_pix[2] = (colour >> 16) & 0xff;  
    coller_pix[3] = (colour >> 24) & 0xff;  

    /* top */
    wk_idx = idx_base;
    for (i = 0; (int)i < w; i += 2)
    {
        fbuf_yuv[wk_idx++ ] = coller_pix[0];
        fbuf_yuv[wk_idx++ ] = coller_pix[1];
        fbuf_yuv[wk_idx++ ] = coller_pix[2];
        fbuf_yuv[wk_idx++ ] = coller_pix[3];
    }

    /* middle */
    for (i = 1; (int)i < (h - 1); i++ )
    {
        wk_idx = idx_base + (VIDEO_PIXEL_HW * RESULT_BUFFER_BYTE_PER_PIXEL * i);
        fbuf_yuv[wk_idx + 0] = coller_pix[0];
        fbuf_yuv[wk_idx + 1] = coller_pix[1];
        fbuf_yuv[wk_idx + 2] = coller_pix[2];
        fbuf_yuv[wk_idx + 3] = coller_pix[3];
        wk_idx += (((w & 0xfffffffe) - 2) * RESULT_BUFFER_BYTE_PER_PIXEL);
        fbuf_yuv[wk_idx + 0] = coller_pix[0];
        fbuf_yuv[wk_idx + 1] = coller_pix[1];
        fbuf_yuv[wk_idx + 2] = coller_pix[2];
        fbuf_yuv[wk_idx + 3] = coller_pix[3];
    }

    /* bottom */
    wk_idx = idx_base + (VIDEO_PIXEL_HW * RESULT_BUFFER_BYTE_PER_PIXEL * (h - 1));
    for (i = 0; (int)i < w; i += 2 )
    {
        fbuf_yuv[wk_idx++ ] = coller_pix[0];
        fbuf_yuv[wk_idx++ ] = coller_pix[1];
        fbuf_yuv[wk_idx++ ] = coller_pix[2];
        fbuf_yuv[wk_idx++ ] = coller_pix[3];
    }
    draw_square = true;
}


/**********************************************************************************************************************
 End of function drawsquare
 *********************************************************************************************************************/


/**********************************************************************************************************************
 * Function Name: drp_task
 * Description  : .
 * Return Value : .
 *********************************************************************************************************************/
static void drp_task(void)
{

    uint32_t counter = 0;
    
    /* Display initialization */
    EasyAttach_Init(Display);

    /* DRP initialization */
    R_DK2_Initialize();

    /* Waiting for SD & USB insertion */
    SdUsbConnect storage("storage");
    printf("Finding a storage...");
    /* wait for the storage device to be connected */
    storage.wait_connect();
    printf("done\n");

    /* Face detection initialization */
    detectFaceInit(FACE_DETECTOR_MODEL);
    g_detect_timer.reset();
    g_detect_timer.start();

    /* Display output start */
    start_lcd_display();

    /* Camera activation process */
    /* Interrupt callback function setting (Field end signal for recording function in scaler 0) */
    Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VFIELD, 0, intcallbackfunc_vfield);
    start_video_camera();

    copysquare.width = 0;
    copysquare.height = 0;
    while (true)
    {
        /* Camera image acquisition process */
        ThisThread::flags_wait_all(DRP_FLG_CAMER_IN);
        /************************************/
        /* Load DRP Library                 */
        /*        +-----------------------+ */
        /* tile 0 |                       | */
        /*        +                       + */
        /* tile 1 |                       | */
        /*        +                       + */
        /* tile 2 |                       | */
        /*        + SimpleIsp bayer2yuv_6 + */
        /* tile 3 |                       | */
        /*        +                       + */
        /* tile 4 |                       | */
        /*        +                       + */
        /* tile 5 |                       | */
        /*        +-----------------------+ */
        R_DK2_Load(g_drp_lib_simple_isp_bayer2yuv_6,
                R_DK2_TILE_0,
                R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);

        /************************/
        /* Activate DRP Library */
        /************************/
        R_DK2_Activate(0, 0);

        /* Set ISP parameters */
        memset(&param_isp, 0, sizeof(param_isp));
        /* ISP source: camera image buffer */
        param_isp.src    = (uint32_t)fbuf_bayer; 
        /* ISP destination: output image buffer on monitor */
        param_isp.dst    = (uint32_t)fbuf_yuv;
        param_isp.width  = VIDEO_PIXEL_HW;
        param_isp.height = VIDEO_PIXEL_VW;
        param_isp.gain_r = 0x1266;
        param_isp.gain_g = 0x0CB0;
        param_isp.gain_b = 0x1359;

        drp_lib_status[TILE_0] = DRP_NOT_FINISH;

        /*********************/
        /* Start DRP Library */
        /*********************/
        R_DK2_Start(drp_lib_id[0], (void *)&param_isp, sizeof(r_drp_simple_isp_t));

        /***************************************/
        /* Wait until DRP processing is finish */
        /***************************************/
        while (drp_lib_status[TILE_0] == DRP_NOT_FINISH)
        {
            /* Spin here forever.. */
        }

        /**********************/
        /* Unload DRP Library */
        /**********************/
        R_DK2_Unload(drp_lib_id[TILE_0], &drp_lib_id[0]);

        if ((copysquare.width > 0) && (copysquare.height > 0) && (counter > 0) )
            {
                counter--;
                R_CACHE_L1DataInvalidLine(fbuf_yuv,sizeof(fbuf_yuv));
                drawsquare(copysquare.x, copysquare.y, copysquare.width, copysquare.height, 0x4CF04C55);
                R_CACHE_L1DataCleanInvalidLine(fbuf_yuv,sizeof(fbuf_yuv)); 
            }
        /************************************/
        /* Load DRP Library                 */
        /*        +-----------------------+ */
        /* tile 0 |                       | */
        /*        +                       + */
        /* tile 1 |                       | */
        /*        +                       + */
        /* tile 2 |                       | */
        /*        + SimpleIsp bayer2grayscale_6 + */
        /* tile 3 |                       | */
        /*        +                       + */
        /* tile 4 |                       | */
        /*        +                       + */
        /* tile 5 |                       | */
        /*        +-----------------------+ */
        R_DK2_Load(g_drp_lib_simple_isp_bayer2grayscale_6,
                R_DK2_TILE_0,
                R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);

        /************************/
        /* Activate DRP Library */
        /************************/
        R_DK2_Activate(0, 0);

        /* Set ISP parameters */
        memset(&param_isp, 0, sizeof(param_isp));
        /* ISP source: camera image buffer */
        param_isp.src    = (uint32_t)fbuf_bayer;
        /* ISP destination: Grayscale image buffer for face detecting */
        param_isp.dst    = (uint32_t)fbuf_grayscale;
        param_isp.width  = VIDEO_PIXEL_HW;
        param_isp.height = VIDEO_PIXEL_VW;
        param_isp.gain_r = 0x1266;
        param_isp.gain_g = 0x0CB0;
        param_isp.gain_b = 0x1359;

        /* Initialize variables to be used in termination judgment of the DRP library */
        drp_lib_status[TILE_0] = DRP_NOT_FINISH;

        /*********************/
        /* Start DRP Library */
        /*********************/
        R_DK2_Start(drp_lib_id[0], (void *)&param_isp, sizeof(r_drp_simple_isp_t));

        /***************************************/
        /* Wait until DRP processing is finish */
        /***************************************/
        while (drp_lib_status[TILE_0] == DRP_NOT_FINISH)
        {
            /* DO NOTHING */
        }

        /**********************/
        /* Unload DRP Library */
        /**********************/
        R_DK2_Unload(drp_lib_id[TILE_0], &drp_lib_id[0]);

        
        /* Cache invalidate operation */
        R_CACHE_L1DataInvalidLine(fbuf_grayscale,sizeof(fbuf_grayscale));
        /* Call OpenCV face detection library */
        /* Detect a face in the frame */
        Mat img_gray(VIDEO_PIXEL_VW, VIDEO_PIXEL_HW, CV_8U, fbuf_grayscale);
        Rect face_roi;
        detectFace(img_gray, face_roi);

        /* A face is detected */
        if ((face_roi.width > 0) && (face_roi.height > 0))
        {
            counter = 5;
            printf("Detected a face X:%d Y:%d W:%d H:%d\n",face_roi.x, face_roi.y, face_roi.width, face_roi.height);
            copysquare = face_roi; 
            audioTask.flags_set(AUDIO_FLG);
        }
        else
        {
            printf("No face is detected\n");
        } 
    }
}
/**********************************************************************************************************************
 End of function drp_task
 *********************************************************************************************************************/


/**********************************************************************************************************************
 * Function Name: audio_task
 * Description  : .
 * Return Value : .
 *********************************************************************************************************************/
static void audio_task(void)
{
    /* make a sound */
    DIR  * d;
    uint8_t file_path[sizeof("/" MOUNT_NAME "/") + FILE_NAME_LEN];
    
    /* decoder setting */
    AudioPlayer.add_decoder<EasyDec_WavCnv2ch>(".wav");
    AudioPlayer.add_decoder<EasyDec_WavCnv2ch>(".WAV");
    AudioPlayer.outputVolume(0.5);

    while (true)
    {
        /* When face is detected */
        ThisThread::flags_wait_all(AUDIO_FLG); 
        
        /* file search */
        d = opendir("/" MOUNT_NAME "/");
        if (d != NULL)
        {
            size_t len = strlen("Alarm.wav");
            if (len < FILE_NAME_LEN)
            {
                /* make file path */
                sprintf((char*)file_path, "/%s/%s", MOUNT_NAME, "Alarm.wav");
                printf("%s\r\n", file_path);

                /* play */
                AudioPlayer.play((const char*)file_path);
            }
            closedir(d);
        }
    }

}
/**********************************************************************************************************************
 End of function audio_task
 *********************************************************************************************************************/



/**********************************************************************************************************************
 * Function Name: main
 * Description  : .
 * Return Value : .
 *********************************************************************************************************************/
int main(void)
{
    /* Start DRP task */
    drpTask.start(callback(drp_task));
    audioTask.start(callback(audio_task));
    printf("struct st_vdc=%08x\n\r",sizeof(struct st_vdc));

    ThisThread::sleep_for(rtos::Kernel::wait_for_u32_forever );
}
/**********************************************************************************************************************
 End of function main
 *********************************************************************************************************************/


#endif /* SAMPLE_PROGRAM_NO == 24 */
