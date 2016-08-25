/*
 * Copyright (c) 2012-2013, NVIDIA CORPORATION. All rights reserved.
 * All information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

// edited by Hyundai Autron
// gcc -I. -I./utils `pkg-config opencv --cflags` -I./include  -c -o captureOpenCV.o captureOpenCV.c
// gcc -I. -I./utils `pkg-config opencv --cflags` -I./include  -c -o nvthread.o nvthread.c
// gcc  -o captureOpenCV captureOpenCV.o nvthread.o  -L ./utils -lnvmedia -lnvtestutil_board -lnvtestutil_capture_input -lnvtestutil_i2c -lpthread `pkg-config opencv --libs`

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>

#include <nvcommon.h>
#include <nvmedia.h>

#include <testutil_board.h>
#include <testutil_capture_input.h>

#include "nvthread.h"
#include "car_lib.h"

#include <highgui.h>
#include <cv.h>
#include <ResTable_720To320.h>
#include <pthread.h>
#include <unistd.h>     // for sleep
#define VIP_BUFFER_SIZE 6
#define VIP_FRAME_TIMEOUT_MS 100
#define VIP_NAME "vip"

#define MESSAGE_PRINTF printf
#define CRC32_POLYNOMIAL 0xEDB88320L

//Nak Add
#define MAX(a, b)    a >= b ? a : b
#define MIN(a, b)    a >= b ? b : a
#define E_VAL_CAL(a) (4800*a/26)
//~Nak

#define RESIZE_WIDTH  320
#define RESIZE_HEIGHT 240

// function define
void curveLine(int resHeight, int resWidth, IplImage* imgResult, int last_width);  
void driveLine();
void emergStop();
void parkFirst();
void parkSecond();
void interSection();
void overTaking();
void trafficLight();
void hill();
// ~function define

static NvMediaVideoSurface *capSurf = NULL;
pthread_cond_t      cond  = PTHREAD_COND_INITIALIZER;
pthread_mutex_t     mutex = PTHREAD_MUTEX_INITIALIZER;

int table_298[256];
int table_409[256];
int table_100[256];
int table_208[256];
int table_516[256];

int curve_flag=0;

//Nak Add
int center_total;
int center_num;
int center_avg;
int max;
short corner_check;
short distance_table[21][2] =
{{5, 204}, {6, 199}, {7, 195}, {8, 191}, {9, 188},
    {10, 184}, {11, 180} ,{12, 177}, {13, 174}, {14, 171},
    {15, 169}, {16, 166}, {17, 163}, {18, 161}, {19, 159},
    {20, 156}, {21, 154}, {22, 152}, {23, 150}, {24, 148},
    {25, 147}
};
short encoder_value;
short last_height;
enum section
{
    CURVLINE,
    EMERGSTOP,
    PARKFIRST,
    PARKSECOND,
    INTERSECTION,
    OVERTAKING,
    TRAFFICLIGHT,
    HILL
};

enum section main_flag;
//~Nak

typedef struct
{
    I2cId i2cDevice;                                        //enum

    CaptureInputDeviceId vipDeviceInUse;                    //enum
    NvMediaVideoCaptureInterfaceFormat vipInputtVideoStd;   //enum
    unsigned int vipInputWidth;
    unsigned int vipInputHeight;
    float vipAspectRatio;

    unsigned int vipMixerWidth;
    unsigned int vipMixerHeight;

    NvBool vipDisplayEnabled;
    NvMediaVideoOutputType vipOutputType;
    NvMediaVideoOutputDevice vipOutputDevice[2];
    NvBool vipFileDumpEnabled;
    char * vipOutputFileName;

    unsigned int vipCaptureTime;
    unsigned int vipCaptureCount;
} TestArgs;

typedef struct
{
    NvMediaVideoSurface *surf;
    NvBool last;
} QueueElem;

typedef struct
{
    char *name;

    NvSemaphore *semStart, *semDone;

    NvMediaVideoCapture *capture;
    NvMediaVideoMixer *mixer;
    FILE *fout;

    unsigned int inputWidth;
    unsigned int inputHeight;

    unsigned int timeout;

    NvBool displayEnabled;
    NvBool fileDumpEnabled;

    NvBool timeNotCount;
    unsigned int last;
} CaptureContext;

static NvBool stop = NVMEDIA_FALSE;
/*
   static int user_turn(int distance)//unit : mm
   {
   long start_encoder = 0;
   long current_encoder = 0;
   DesireSpeed_Write(50);
   start_encoder = EncoderCounter_Read();
   while(current_encoder-start_encoder < ((long)(distance/0.5394)))
   {
   current_encoder=EncoderCounter_Read();
   printf("current:%d, start:%d, diff : %d \n",current_encoder,start_encoder,current_encoder-start_encoder);

   if(current_encoder<start_encoder)	//exception
   return 1;
   }
   EncoderCounter_Write(0);
   DesireSpeed_Write(20);
   return 0;
   }
   */
void UserInit(void)
{
    SteeringServoControl_Write(1460);   //initialize Servo Center Position
    SpeedPIDProportional_Write(20);     //speed -> P gain : default:10, range:10~50
    SpeedPIDIntegral_Write(20);         //speed -> I gain : default:10, range:10~50
    SpeedPIDDifferential_Write(20);     //speed -> D gain : default:10, range:10~50
    PositionProportionPoint_Write(10);  //Encoder -> P gain : default:10, range:10~50
    DesireSpeed_Write(0);
    EncoderCounter_Write(0);

    PositionControlOnOff_Write(UNCONTROL);  // position control OFF 
    SpeedControlOnOff_Write(UNCONTROL);     // speed control OFF

    usleep(100000);//wait for 100ms
}

void UserMove(int direction, int steer,int distance)
{
    static int temp=0;
    int state  = 0;
    state = SpeedControlOnOff_Read();
    if(state == 0)
    {
        DesireSpeed_Write(50);
        SpeedControlOnOff_Write(CONTROL);   // speedControl ON
    }
    state = PositionControlOnOff_Read();
    if(state == 0)
        PositionControlOnOff_Write(CONTROL);    // positionControl function ON 

    SteeringServoControl_Write(steer);

    printf("distance : %d\n",distance);

    distance = distance*18/10;
    printf("***************************************\n");
    EncoderCounter_Write(0);

    if(direction == 1)
    {
        DesireEncoderCount_Write(distance);
        //temp = DesireEncoderCount_Read();
        temp = distance;
        while(temp>20)
        {
            temp = DesireEncoderCount_Read();
        }
    }
    else
    {
        DesireEncoderCount_Write(-distance);
        //temp = DesireEncoderCount_Read();
        temp = distance;
        while(temp<-10)
        {
            temp = DesireEncoderCount_Read();
        }
    }

    PositionControlOnOff_Write(UNCONTROL);
}


static void SignalHandler(int signal)
{
    stop = NVMEDIA_TRUE;
    MESSAGE_PRINTF("%d signal received\n", signal);
}

static void GetTime(NvMediaTime *time)
{
    struct timeval t;

    gettimeofday(&t, NULL);

    time->tv_sec = t.tv_sec;
    time->tv_nsec = t.tv_usec * 1000;
}

static void AddTime(NvMediaTime *time, NvU64 uSec, NvMediaTime *res)
{
    NvU64 t, newTime;

    t = (NvU64)time->tv_sec * 1000000000LL + (NvU64)time->tv_nsec;
    newTime = t + uSec * 1000LL;
    res->tv_sec = newTime / 1000000000LL;
    res->tv_nsec = newTime % 1000000000LL;
}

//static NvS64 SubTime(NvMediaTime *time1, NvMediaTime *time2)
static NvBool SubTime(NvMediaTime *time1, NvMediaTime *time2
        )
{
    NvS64 t1, t2, delta;

    t1 = (NvS64)time1->tv_sec * 1000000000LL + (NvS64)time1->tv_nsec;
    t2 = (NvS64)time2->tv_sec * 1000000000LL + (NvS64)time2->tv_nsec;
    delta = t1 - t2;

    //    return delta / 1000LL;
    return delta > 0LL;
}


static void DisplayUsage(void)
{
    printf("Usage : nvmedia_capture [options]\n");
    printf("Brief: Displays this help if no arguments are given. Engages the respective capture module whenever a single \'c\' or \'v\' argument is supplied using default values for the missing parameters.\n");
    printf("Options:\n");
    printf("-va <aspect ratio>    VIP aspect ratio (default = 1.78 (16:9))\n");
    printf("-vmr <width>x<height> VIP mixer resolution (default 800x480)\n");
    printf("-vf <file name>       VIP output file name; default = off\n");
    printf("-vt [seconds]         VIP capture duration (default = 10 secs); overridden by -vn; default = off\n");
    printf("-vn [frames]          # VIP frames to be captured (default = 300); default = on if -vt is not used\n");
}

static int ParseOptions(int argc, char *argv[], TestArgs *args)
{
    int i = 1;

    // Set defaults if necessary - TBD
    args->i2cDevice = I2C4;     // i2c chnnel

    args->vipDeviceInUse = AnalogDevices_ADV7182;
    args->vipInputtVideoStd = NVMEDIA_VIDEO_CAPTURE_INTERFACE_FORMAT_VIP_NTSC;
    args->vipInputWidth = 720;
    args->vipInputHeight = 480;
    args->vipAspectRatio = 0.0f;

    args->vipMixerWidth = 800;
    args->vipMixerHeight = 480;

    args->vipDisplayEnabled = NVMEDIA_FALSE;
    args->vipOutputType = NvMediaVideoOutputType_OverlayYUV;
    args->vipOutputDevice[0] = NvMediaVideoOutputDevice_LVDS;
    args->vipFileDumpEnabled = NVMEDIA_FALSE;
    args->vipOutputFileName = NULL;

    args->vipCaptureTime = 0;
    args->vipCaptureCount = 0;



    if(i < argc && argv[i][0] == '-')
    {
        while(i < argc && argv[i][0] == '-')
        {
            if(i > 1 && argv[i][1] == '-')
            {
                MESSAGE_PRINTF("Using basic and custom options together is not supported\n");
                return 0;
            }

            // Get options
            if(!strcmp(argv[i], "-va"))
            {
                if(++i < argc)
                {
                    if(sscanf(argv[i], "%f", &args->vipAspectRatio) != 1 || args->vipAspectRatio <= 0.0f) // TBC
                    {
                        MESSAGE_PRINTF("Bad VIP aspect ratio: %s\n", argv[i]);
                        return 0;
                    }
                }
                else
                {
                    MESSAGE_PRINTF("Missing VIP aspect ratio\n");
                    return 0;
                }
            }
            else if(!strcmp(argv[i], "-vmr"))
            {
                if(++i < argc)
                {
                    if(sscanf(argv[i], "%ux%u", &args->vipMixerWidth, &args->vipMixerHeight) != 2)
                    {
                        MESSAGE_PRINTF("Bad VIP mixer resolution: %s\n", argv[i]);
                        return 0;
                    }
                }
                else
                {
                    MESSAGE_PRINTF("Missing VIP mixer resolution\n");
                    return 0;
                }
            }
            else if(!strcmp(argv[i], "-vf"))
            {
                args->vipFileDumpEnabled = NVMEDIA_TRUE;
                if(++i < argc)
                    args->vipOutputFileName = argv[i];
                else
                {
                    MESSAGE_PRINTF("Missing VIP output file name\n");
                    return 0;
                }
            }
            else if(!strcmp(argv[i], "-vt"))
            {
                if(++i < argc)
                    if(sscanf(argv[i], "%u", &args->vipCaptureTime) != 1)
                    {
                        MESSAGE_PRINTF("Bad VIP capture duration: %s\n", argv[i]);
                        return 0;
                    }
            }
            else if(!strcmp(argv[i], "-vn"))
            {
                if(++i < argc)
                    if(sscanf(argv[i], "%u", &args->vipCaptureCount) != 1)
                    {
                        MESSAGE_PRINTF("Bad VIP capture count: %s\n", argv[i]);
                        return 0;
                    }
            }
            else
            {
                MESSAGE_PRINTF("%s is not a supported option\n", argv[i]);
                return 0;
            }

            i++;
        }
    }

    if(i < argc)
    {
        MESSAGE_PRINTF("%s is not a supported option\n", argv[i]);
        return 0;
    }

    // Check for consistency
    if(i < 2)
    {
        DisplayUsage();
        return 0;
    }


    if(args->vipAspectRatio == 0.0f)
        args->vipAspectRatio = 1.78f;

    if(!args->vipDisplayEnabled && !args->vipFileDumpEnabled)
        args->vipDisplayEnabled = NVMEDIA_TRUE;


    if(!args->vipCaptureTime && !args->vipCaptureCount)
        args->vipCaptureCount = 300;
    else if(args->vipCaptureTime && args->vipCaptureCount)
        args->vipCaptureTime = 0;



    return 1;
}

static int DumpFrame(FILE *fout, NvMediaVideoSurface *surf)
{
    NvMediaVideoSurfaceMap surfMap;
    unsigned int width, height;

    if(NvMediaVideoSurfaceLock(surf, &surfMap) != NVMEDIA_STATUS_OK)
    {
        MESSAGE_PRINTF("NvMediaVideoSurfaceLock() failed in DumpFrame()\n");
        return 0;
    }

    width = surf->width;
    height = surf->height;

    unsigned char *pY[2] = {surfMap.pY, surfMap.pY2};
    unsigned char *pU[2] = {surfMap.pU, surfMap.pU2};
    unsigned char *pV[2] = {surfMap.pV, surfMap.pV2};
    unsigned int pitchY[2] = {surfMap.pitchY, surfMap.pitchY2};
    unsigned int pitchU[2] = {surfMap.pitchU, surfMap.pitchU2};
    unsigned int pitchV[2] = {surfMap.pitchV, surfMap.pitchV2};
    unsigned int i, j;

    for(i = 0; i < 2; i++)
    {
        for(j = 0; j < height / 2; j++)
        {
            fwrite(pY[i], width, 1, fout);
            pY[i] += pitchY[i];
        }
        for(j = 0; j < height / 2; j++)
        {
            fwrite(pU[i], width / 2, 1, fout);
            pU[i] += pitchU[i];
        }
        for(j = 0; j < height / 2; j++)
        {
            fwrite(pV[i], width / 2, 1, fout);
            pV[i] += pitchV[i];
        }
    }


    NvMediaVideoSurfaceUnlock(surf);

    return 1;
}

static int Frame2Ipl(IplImage* img, IplImage* imgResult, IplImage* imgCenter)
{
    unsigned char status, gain;
    short speed;
    NvMediaVideoSurfaceMap surfMap;
    unsigned int resWidth, resHeight;

    short both_side = 0;    //차선 검출 위한 변수
    char check_side = 0;    //차선 검출 위한 변수 
    short center = 0, left = 0, right = 0;
    int num;
    unsigned char y,u,v;
    int start_cnt = 0;

    if(NvMediaVideoSurfaceLock(capSurf, &surfMap) != NVMEDIA_STATUS_OK)
    {
        MESSAGE_PRINTF("NvMediaVideoSurfaceLock() failed in Frame2Ipl()\n");
        return 0;
    }

    unsigned char *pY[2] = {surfMap.pY, surfMap.pY2};
    unsigned char *pU[2] = {surfMap.pU, surfMap.pU2};
    unsigned char *pV[2] = {surfMap.pV, surfMap.pV2};
    unsigned int pitchY[2] = {surfMap.pitchY, surfMap.pitchY2};
    unsigned int pitchU[2] = {surfMap.pitchU, surfMap.pitchU2};
    unsigned int pitchV[2] = {surfMap.pitchV, surfMap.pitchV2};
    unsigned int i, j, k, x;
    unsigned int stepY, stepU, stepV;
    unsigned int bin_num=0;
    //Nak Add
    int temp1;
    int temp2;
    int min;
    int flag = 0;
    int tmp_center = 0;
    //~Nak
    resWidth = RESIZE_WIDTH;
    resHeight = RESIZE_HEIGHT;

    // Frame2Ipl
    img->nSize = 112;
    img->ID = 0;
    img->nChannels = 3;
    img->alphaChannel = 0;
    img->depth = IPL_DEPTH_8U;
    img->colorModel[0] = 'v';
    img->colorModel[1] = 'U';
    img->colorModel[2] = 'V';
    img->channelSeq[0] = 'Y';
    img->channelSeq[1] = 'U';
    img->channelSeq[2] = 'V';
    img->dataOrder = 0;
    img->origin = 0;
    img->align = 4;
    img->width = resWidth;
    img->height = resHeight;
    img->imageSize = resHeight*resWidth*3;
    img->widthStep = resWidth*3;
    img->BorderMode[0] = 0;
    img->BorderMode[1] = 0;
    img->BorderMode[2] = 0;
    img->BorderMode[3] = 0;
    img->BorderConst[0] = 0;
    img->BorderConst[1] = 0;
    img->BorderConst[2] = 0;
    img->BorderConst[3] = 0;

    stepY = 0;
    stepU = 0;
    stepV = 0;
    i = 0;

    for(j = 0; j < resHeight; j++)
    {
        for(k = 0; k < resWidth; k++)
        {
            imgCenter->imageData[j*imgCenter->widthStep + k]=0;
        }
    }

    for(j = 0; j < resHeight; j++)
    {
        for(k = 0; k < resWidth; k++)
        {
            x = ResTableX_720To320[k];
            y = pY[i][stepY+x];
            u = pU[i][stepU+x/2];
            v = pV[i][stepV+x/2];

            // - 39 , 111 , 51, 241 
            num = 3*k+3*resWidth*(j);
            bin_num = j*imgResult->widthStep + k;
            if( u>-39  &&  u<120  &&  v>45   &&   v<245  ) {
                // 흰색으로
                imgResult->imageData[bin_num] = (char)255;

            }
            else {
                // 검정색으로
                imgResult->imageData[bin_num] = (char)0;

            }            

            img->imageData[num] = y;
            img->imageData[num+1] = u;
            img->imageData[num+2] = v;

        }
        stepY += pitchY[i];
        stepU += pitchU[i];
        stepV += pitchV[i];
    }

    cvDilate(imgResult,imgResult,0,1);
    cvErode(imgResult,imgResult,0,2);

    center_total = 0;
    center_num = 1;
    corner_check = 0;

    for(j =(resHeight/3); j < resHeight; j++)
    {
        left = 0;
        right = 0;
        center =0;
        for( k = (resWidth / 2) ; k >0 ; k--)
        {           
            if(imgResult->imageData[j*imgResult->widthStep + k] == (char) 255)
            {
                left=k;
                break;
            }
        } // 중앙에서 왼쪽으로 움직이며 최초의 255값을 찾는다. 찾으면 break

        for( k = (resWidth / 2) ; k< resWidth ; k++)
        {
            if(imgResult->imageData[j*imgResult->widthStep + k] == (char) 255)
            {
                right=k;
                break;
            }
        } // 

        if(left != 0 && right !=0)
        {
            //TODO 코너 검출 방법 바꿔줘야함
            if(j < (resHeight/3)*2 && j > resHeight/2)
                corner_check++;
            // 화면의 1/2지점에서 2/3 지점까지의 center pixel개수를 센다. 만약 일정 수 미만이면 코너 인식
            center=(right+left)/2;  // 왼쪽과 오른쪽 pixel의 중간값을 계산 = center값
            center_total += center; // center값의 총 합을 더한다.
            center_num++;           // center값의 총합에 center값의 총 수를 나눠 주기 위해 개수를 센다.
            imgCenter->imageData[j*imgCenter->widthStep + center] = (char)255;
                                    // center가 어딘지를 확인하기 위해 영상에 저장
        }
    }

    NvMediaVideoSurfaceUnlock(capSurf);

    return 1;
}

static unsigned int CaptureThread(void *params)
{
    int i = 0;
    static char num;
    NvU64 stime, ctime;
    NvMediaTime t1 = {0}, t2 = {0}, st = {0}, ct = {0};
    CaptureContext *ctx = (CaptureContext *)params;
    NvMediaVideoSurface *releaseList[4] = {NULL}, **relList;
    NvMediaRect primarySrcRect;
    NvMediaPrimaryVideo primaryVideo;

    primarySrcRect.x0 = 0;
    primarySrcRect.y0 = 0;
    primarySrcRect.x1 = ctx->inputWidth;
    primarySrcRect.y1 = ctx->inputHeight;

    primaryVideo.next = NULL;
    primaryVideo.previous = NULL;
    primaryVideo.previous2 = NULL;
    primaryVideo.srcRect = &primarySrcRect;
    primaryVideo.dstRect = NULL;


    NvSemaphoreDecrement(ctx->semStart, NV_TIMEOUT_INFINITE);

    if(ctx->timeNotCount)
    {
        GetTime(&t1);
        AddTime(&t1, ctx->last * 1000000LL, &t1);
        GetTime(&t2);
        printf("timeNotCount\n");
    }
    GetTime(&st);
    stime = (NvU64)st.tv_sec * 1000000000LL + (NvU64)st.tv_nsec;

    while((ctx->timeNotCount? (SubTime(&t1, &t2)): ((unsigned int)i < ctx->last)) && !stop)
    {
        GetTime(&ct);
        ctime = (NvU64)ct.tv_sec * 1000000000LL + (NvU64)ct.tv_nsec;
        //  printf("frame=3d, time=%llu.%09llu[s] \n", i, (ctime-stime)/1000000000LL, (ctime-stime)%1000000000LL);

        pthread_mutex_lock(&mutex);            // for ControlThread()

        if(!(capSurf = NvMediaVideoCaptureGetFrame(ctx->capture, ctx->timeout)))
        { // TBD
            MESSAGE_PRINTF("NvMediaVideoCaptureGetFrame() failed in %sThread\n", ctx->name);
            stop = NVMEDIA_TRUE;
            break;
        }

        if(i%5 == 0)                        // once in three loop = 10 Hz
            pthread_cond_signal(&cond);        // ControlThread() is called

        pthread_mutex_unlock(&mutex);        // for ControlThread()

        primaryVideo.current = capSurf;
        primaryVideo.pictureStructure = NVMEDIA_PICTURE_STRUCTURE_TOP_FIELD;

        if(NVMEDIA_STATUS_OK != NvMediaVideoMixerRender(ctx->mixer, // mixer
                    NVMEDIA_OUTPUT_DEVICE_0, // outputDeviceMask
                    NULL, // background
                    &primaryVideo, // primaryVideo
                    NULL, // secondaryVideo
                    NULL, // graphics0
                    NULL, // graphics1
                    releaseList, // releaseList
                    NULL)) // timeStamp
        { // TBD
            MESSAGE_PRINTF("NvMediaVideoMixerRender() failed for the top field in %sThread\n", ctx->name);
            stop = NVMEDIA_TRUE;
        }

        primaryVideo.pictureStructure = NVMEDIA_PICTURE_STRUCTURE_BOTTOM_FIELD;
        if(NVMEDIA_STATUS_OK != NvMediaVideoMixerRender(ctx->mixer, // mixer
                    NVMEDIA_OUTPUT_DEVICE_0, // outputDeviceMask
                    NULL, // background
                    &primaryVideo, // primaryVideo
                    NULL, // secondaryVideo
                    NULL, // graphics0
                    NULL, // graphics1
                    releaseList, // releaseList
                    NULL)) // timeStamp
        { // TBD
            MESSAGE_PRINTF("NvMediaVideoMixerRender() failed for the bottom field in %sThread\n", ctx->name);
            stop = NVMEDIA_TRUE;
        }

        if(ctx->fileDumpEnabled)
        {
            if(!DumpFrame(ctx->fout, capSurf))
            { // TBD
                MESSAGE_PRINTF("DumpFrame() failed in %sThread\n", ctx->name);
                stop = NVMEDIA_TRUE;
            }

            if(!ctx->displayEnabled)
                releaseList[0] = capSurf;
        }

        relList = &releaseList[0];

        while(*relList)
        {
            if(NvMediaVideoCaptureReturnFrame(ctx->capture, *relList) != NVMEDIA_STATUS_OK)
            { // TBD
                MESSAGE_PRINTF("NvMediaVideoCaptureReturnFrame() failed in %sThread\n", ctx->name);
                stop = NVMEDIA_TRUE;
                break;
            }
            relList++;
        }

        if(ctx->timeNotCount)
            GetTime(&t2);

        i++;
    } // while end

    // Release any left-over frames
    //    if(ctx->displayEnabled && capSurf && capSurf->type != NvMediaSurfaceType_YV16x2) // To allow returning frames after breaking out of the while loop in case of error
    if(ctx->displayEnabled && capSurf)
    {
        NvMediaVideoMixerRender(ctx->mixer, // mixer
                NVMEDIA_OUTPUT_DEVICE_0, // outputDeviceMask
                NULL, // background
                NULL, // primaryVideo
                NULL, // secondaryVideo
                NULL, // graphics0
                NULL, // graphics1
                releaseList, // releaseList
                NULL); // timeStamp

        relList = &releaseList[0];

        while(*relList)
        {
            if(NvMediaVideoCaptureReturnFrame(ctx->capture, *relList) != NVMEDIA_STATUS_OK)
                MESSAGE_PRINTF("NvMediaVideoCaptureReturnFrame() failed in %sThread\n", ctx->name);

            relList++;
        }
    }

    NvSemaphoreIncrement(ctx->semDone);
    return 0;
}

static void CheckDisplayDevice(NvMediaVideoOutputDevice deviceType, NvMediaBool *enabled, unsigned int *displayId)
{
    int outputDevices;
    NvMediaVideoOutputDeviceParams *outputParams;
    int i;

    // By default set it as not enabled (initialized)
    *enabled = NVMEDIA_FALSE;
    *displayId = 0;

    // Get the number of devices
    if(NvMediaVideoOutputDevicesQuery(&outputDevices, NULL) != NVMEDIA_STATUS_OK) {
        return;
    }

    // Allocate memory for information for all devices
    outputParams = malloc(outputDevices * sizeof(NvMediaVideoOutputDeviceParams));
    if(!outputParams) {
        return;
    }

    // Get device information for acll devices
    if(NvMediaVideoOutputDevicesQuery(&outputDevices, outputParams) != NVMEDIA_STATUS_OK) {
        free(outputParams);
        return;
    }

    // Find desired device
    for(i = 0; i < outputDevices; i++) {
        if((outputParams + i)->outputDevice == deviceType) {
            // Return information
            *enabled = (outputParams + i)->enabled;
            *displayId = (outputParams + i)->displayId;
            break;
        }
    }

    // Free information memory
    free(outputParams);
}

static void YUV2RGVtableInit(void)
{
    int i;

    for( i = 0 ; i < 256 ; i++ )
    {
        table_298[i] = 298*(i-16) + 128;
        table_409[i] = 409*(i-128);
        table_100[i] = 100*(i-128);
        table_208[i] = 208*(i-128);
        table_516[i] = 516*(i-128);
    }

}
#define baseAngle 1500

void *ControlThread(void *unused)
{
    unsigned char status, gain;
    short speed;
    int i=0;
    char fileName[30];
    char fileName2[30];
    char fileName3[30];
    char fileName4[30];
    char fileName5[30];
    char fileName6[30];
    char fileName7[30];
    //Nak Add
    int steer_angle;
    int sub_center;
    int last_width;
    int flag;

    static int num;
    unsigned int resWidth, resHeight;
    int j, k;

    resWidth = RESIZE_WIDTH;
    resHeight = RESIZE_HEIGHT;
    //~Nak

    NvMediaTime pt1 ={0}, pt2 = {0};
    NvU64 ptime1, ptime2;
    struct timespec;

    IplImage* imgOrigin;
    IplImage* imgY;
    IplImage* imgU;
    IplImage* imgV;
    IplImage* RGB;  
    IplImage* imgCenter;

    IplImage* imgResult;
    IplImage* tempResult;

    int tempPosition=0;
    int position_now=0;
    int tol;
    int curve_temp=0;

    long start_encoder = 0;
    long current_encoder = 0;

    int tempcounter;
    // 차량 초기화 코드
    CarControlInit();
    UserInit();

    DesireSpeed_Write(50);//Speed, default : 10, 
    SpeedControlOnOff_Write(CONTROL);

    CameraXServoControl_Write(1470);
    CameraYServoControl_Write(1600);
    // ~차량 초기화 코드
    
    // cvCreateImage
    imgOrigin = cvCreateImage(cvSize(RESIZE_WIDTH, RESIZE_HEIGHT), IPL_DEPTH_8U, 3);
    RGB = cvCreateImage(cvGetSize(imgOrigin),IPL_DEPTH_8U, 3);
    imgResult = cvCreateImage(cvGetSize(imgOrigin), IPL_DEPTH_8U, 1);
    imgCenter = cvCreateImage(cvGetSize(imgOrigin), IPL_DEPTH_8U, 1);
    tempResult = cvCreateImage(cvGetSize(imgOrigin), IPL_DEPTH_8U, 1);

    while(1)
    {
        pthread_mutex_lock(&mutex);
        pthread_cond_wait(&cond, &mutex);
        GetTime(&pt1);
        ptime1 = (NvU64)pt1.tv_sec * 1000000000LL + (NvU64)pt1.tv_nsec;

        if(Frame2Ipl(imgOrigin,imgResult,imgCenter) == 0)// save image to IplImage structure & resize image from 720x480 to 320x240
        {
            DesireSpeed_Write(0);
        }

        pthread_mutex_unlock(&mutex);     

        cvCvtColor(imgOrigin, RGB , CV_YUV2BGR);    // YUv값을 rgb로 변환

        sprintf(fileName4, "captureImage/imgRGB%d.png", i);
        sprintf(fileName5, "captureImage/imgResult%d.png", i);
        sprintf(fileName6, "captureImage/imgYUV%d.png", i);
        sprintf(fileName7, "captureImage/imgCenter%d.png", i);  // 이미지파일 이름 지정

        cvSaveImage(fileName4, RGB, 0);
        cvSaveImage(fileName5, imgResult, 0);
        cvSaveImage(fileName6, imgOrigin, 0);
        cvSaveImage(fileName7, imgCenter, 0);       // 이미지파일 저장

        // TODO : control steering angle based on captured image ---------------
        /*
         *  직선 로직
         *   1. 영상에서 좌우 라인을 보고 중간 값을 구한다
         *   2. 중간 값이 영상의 중간이 되도록 조향각을 틀어준다.
         *  곡석 확인 방법
         *   1. center값들의 편차를 구하여 어느 정도의 편차가 발생하면 곡선을 인식
         *   2. 곡선 시작점 까지 주행 후 로직으로 주행
         */

        /*
         * 곡선(curveLine) 
         * 넘겨야할 parameter : resHeight, resWidth, imgResult, last_width
         * 전역 변수 : corner_check, last_height
         * 선언해야할 변수 : curve_flag(int), j, k, curve_temp, center_avg, sub_center, steer_angle
         */

        /*
         * 주행(driveLine)
         * global parameter : center_avg, center_total, center_num
         * define parameter : sub_center, steer_angle
         */


        /* flag 판단
         *  1. curveLine : corner_check변수 값으로 확인
         *  2. emergStop : red pixel 개수를 통해 확인
         *  3. parkFirst : 미정
         *  4. parkSecond : 미정
         *  5. interSection : linetrace를 통해 확인 - 1번째
         *  6. overTaking : gray pixel 개수를 통해 확인
         *  7. trafficLight : linetrace를 통해 확인 - 2번째
         *  8. hill : 미정
         */

        cvErode(imgResult,tempResult,0,1);

        flag = 100;

        if(corner_check <= 4) //4이하일 때만 코너 인식
            flag = CURVLINE;
        else if(/*red_pixel > ? */)
            flag = EMERGSTOP;           //Frame2Ipl에서 red pixel값을 읽어서 check
        else if(/*parkFirst condition*/)
            flag = PARKFIRST;           //미정
        else if(/*parkSecond*/)
            flag = PARKSECOND;          //미정
        else if(/*First Linetrace*/)
            flag = INTERSECTION;        //linetrace를 통해 하얀선 인식
        else if(/*gray_pixel*/)
            flag = OVERTAKING;          //Frame2Ipl에서 gray pixel값을 읽어서 check
        else if(/*Second Linetrace*/)
            flag = TRAFFICLIGHT;        //linetrace를 통해 하얀선 인식
        else if(/*hill*/)
            flag = HILL;                //미정

        //switch()
        switch(flag){
            case CURVLINE:
                curveLine(resHeight, resWidth, imgResult, last_width);  // 코너주행
                break;
            case EMERGSTOP:
                emergStop();
                break;
            case PARKFIRST:
                parkFirst();
                break;
            case PARKSECOND:
                parkSecond();
                break;
            case INTERSECTION:
                interSection();
                break;
            case OVERTAKING:
                overTaking();
                break;
            case TRAFFICLIGHT:
                trafficLight();
                break;
            case HILL:
                hill();
                break;
            default:
                driveLine();
                break;
        }

        GetTime(&pt2);
        ptime2 = (NvU64)pt2.tv_sec * 1000000000LL + (NvU64)pt2.tv_nsec;
        printf("%d:--------------------------------operation time=%llu.%09llu[s]\n",num, (ptime2-ptime1)/1000000000LL, (ptime2-ptime1)%1000000000LL);  
        num++;
        i++;
    }
}


int main(int argc, char *argv[])
{
    int err = -1;
    TestArgs testArgs;

    CaptureInputHandle handle;

    NvMediaVideoCapture *vipCapture = NULL;
    NvMediaDevice *device = NULL;
    NvMediaVideoMixer *vipMixer = NULL;
    NvMediaVideoOutput *vipOutput[2] = {NULL, NULL};
    NvMediaVideoOutput *nullOutputList[1] = {NULL};
    FILE *vipFile = NULL;

    NvSemaphore *vipStartSem = NULL, *vipDoneSem = NULL;
    NvThread *vipThread = NULL;

    CaptureContext vipCtx;
    NvMediaBool deviceEnabled = NVMEDIA_FALSE;
    unsigned int displayId;

    pthread_t cntThread;
    //1
    signal(SIGINT, SignalHandler);

    memset(&testArgs, 0, sizeof(TestArgs));
    if(!ParseOptions(argc, argv, &testArgs))
        return -1;
    //~1. TestArgs initialize and option check

    printf("1. Create NvMedia capture \n");
    // Create NvMedia capture(s)
    switch (testArgs.vipDeviceInUse)
    {
        case AnalogDevices_ADV7180:
            break;
        case AnalogDevices_ADV7182:
            {
                CaptureInputConfigParams params; //cpature 화면 정보 저장

                params.width = testArgs.vipInputWidth;   //720
                params.height = testArgs.vipInputHeight; //480
                params.vip.std = testArgs.vipInputtVideoStd; //NVMEDIA_VIDEO_CAPTURE_INTERFACE_FORMAT_VIP_NTSC,

                if(testutil_capture_input_open(testArgs.i2cDevice, testArgs.vipDeviceInUse, NVMEDIA_TRUE, &handle) < 0)
                {
                    MESSAGE_PRINTF("Failed to open VIP device\n");
                    goto fail;
                }

                if(testutil_capture_input_configure(handle, &params) < 0)
                {
                    MESSAGE_PRINTF("Failed to configure VIP device\n");
                    goto fail;
                }

                break;
            }
        default:
            MESSAGE_PRINTF("Bad VIP device\n");
            goto fail;
    }


    if(!(vipCapture = NvMediaVideoCaptureCreate(testArgs.vipInputtVideoStd, // interfaceFormat
                    NULL, // settings
                    VIP_BUFFER_SIZE)))// numBuffers
    {
        MESSAGE_PRINTF("NvMediaVideoCaptureCreate() failed for vipCapture\n");
        goto fail;
    }


    printf("2. Create NvMedia device \n");
    // Create NvMedia device
    if(!(device = NvMediaDeviceCreate()))
    {
        MESSAGE_PRINTF("NvMediaDeviceCreate() failed\n");
        goto fail;
    }

    printf("3. Create NvMedia mixer(s) and output(s) and bind them \n");
    // Create NvMedia mixer(s) and output(s) and bind them
    unsigned int features = 0;


    features |= NVMEDIA_VIDEO_MIXER_FEATURE_VIDEO_SURFACE_TYPE_YV16X2;
    features |= NVMEDIA_VIDEO_MIXER_FEATURE_PRIMARY_VIDEO_DEINTERLACING; // Bob the 16x2 format by default
    if(testArgs.vipOutputType != NvMediaVideoOutputType_OverlayYUV)
        features |= NVMEDIA_VIDEO_MIXER_FEATURE_DVD_MIXING_MODE;

    if(!(vipMixer = NvMediaVideoMixerCreate(device, // device
                    testArgs.vipMixerWidth, // mixerWidth
                    testArgs.vipMixerHeight, // mixerHeight
                    testArgs.vipAspectRatio, //sourceAspectRatio
                    testArgs.vipInputWidth, // primaryVideoWidth
                    testArgs.vipInputHeight, // primaryVideoHeight
                    0, // secondaryVideoWidth
                    0, // secondaryVideoHeight
                    0, // graphics0Width
                    0, // graphics0Height
                    0, // graphics1Width
                    0, // graphics1Height
                    features , // features
                    nullOutputList))) // outputList
    {
        MESSAGE_PRINTF("NvMediaVideoMixerCreate() failed for vipMixer\n");
        goto fail;
    }

    printf("4. Check that the device is enabled (initialized) \n");
    // Check that the device is enabled (initialized)
    CheckDisplayDevice(
            testArgs.vipOutputDevice[0],
            &deviceEnabled,
            &displayId);

    if((vipOutput[0] = NvMediaVideoOutputCreate(testArgs.vipOutputType, // outputType
                    testArgs.vipOutputDevice[0], // outputDevice
                    NULL, // outputPreference
                    deviceEnabled, // alreadyCreated
                    displayId, // displayId
                    NULL))) // displayHandle
    {
        if(NvMediaVideoMixerBindOutput(vipMixer, vipOutput[0], NVMEDIA_OUTPUT_DEVICE_0) != NVMEDIA_STATUS_OK)
        {
            MESSAGE_PRINTF("Failed to bind VIP output to mixer\n");
            goto fail;
        }
    }
    else
    {
        MESSAGE_PRINTF("NvMediaVideoOutputCreate() failed for vipOutput\n");
        goto fail;
    }



    printf("5. Open output file(s) \n");
    // Open output file(s)
    if(testArgs.vipFileDumpEnabled)
    {
        vipFile = fopen(testArgs.vipOutputFileName, "w");
        if(!vipFile || ferror(vipFile))
        {
            MESSAGE_PRINTF("Error opening output file for VIP\n");
            goto fail;
        }
    }

    printf("6. Create vip pool(s), queue(s), fetch threads and stream start/done semaphores \n");
    // Create vip pool(s), queue(s), fetch threads and stream start/done semaphores
    if(NvSemaphoreCreate(&vipStartSem, 0, 1) != RESULT_OK)
    {
        MESSAGE_PRINTF("NvSemaphoreCreate() failed for vipStartSem\n");
        goto fail;
    }

    if(NvSemaphoreCreate(&vipDoneSem, 0, 1) != RESULT_OK)
    {
        MESSAGE_PRINTF("NvSemaphoreCreate() failed for vipDoneSem\n");
        goto fail;
    }

    vipCtx.name = VIP_NAME;

    vipCtx.semStart = vipStartSem;
    vipCtx.semDone = vipDoneSem;

    vipCtx.capture = vipCapture;
    vipCtx.mixer = vipMixer;
    vipCtx.fout = vipFile;

    vipCtx.inputWidth = testArgs.vipInputWidth;
    vipCtx.inputHeight = testArgs.vipInputHeight;

    vipCtx.timeout = VIP_FRAME_TIMEOUT_MS;

    vipCtx.displayEnabled = testArgs.vipDisplayEnabled;
    vipCtx.fileDumpEnabled = testArgs.vipFileDumpEnabled;

    if(testArgs.vipCaptureTime)
    {
        vipCtx.timeNotCount = NVMEDIA_TRUE;
        vipCtx.last = testArgs.vipCaptureTime;
    }
    else
    {
        vipCtx.timeNotCount = NVMEDIA_FALSE;
        vipCtx.last = testArgs.vipCaptureCount;
    }


    if(NvThreadCreate(&vipThread, CaptureThread, &vipCtx, NV_THREAD_PRIORITY_NORMAL) != RESULT_OK)
    {
        MESSAGE_PRINTF("NvThreadCreate() failed for vipThread\n");
        goto fail;
    }

    printf("wait for ADV7182 ... one second\n");
    sleep(1);

    printf("7. Kickoff \n");
    // Kickoff
    NvMediaVideoCaptureStart(vipCapture);
    NvSemaphoreIncrement(vipStartSem);

    printf("8. Control Thread\n");
    YUV2RGVtableInit();
    pthread_create(&cntThread, NULL, &ControlThread, NULL); 

    printf("9. Wait for completion \n");
    // Wait for completion
    NvSemaphoreDecrement(vipDoneSem, NV_TIMEOUT_INFINITE);


    err = 0;

fail: // Run down sequence
    // Destroy vip threads and stream start/done semaphores
    if(vipThread)
        NvThreadDestroy(vipThread);
    if(vipDoneSem)
        NvSemaphoreDestroy(vipDoneSem);
    if(vipStartSem)
        NvSemaphoreDestroy(vipStartSem);

    DesireSpeed_Write(0);
    printf("10. Close output file(s) \n");
    // Close output file(s)
    if(vipFile)
        fclose(vipFile);

    // Unbind NvMedia mixer(s) and output(s) and destroy them
    if(vipOutput[0])
    {
        NvMediaVideoMixerUnbindOutput(vipMixer, vipOutput[0], NULL);
        NvMediaVideoOutputDestroy(vipOutput[0]);
    }
    if(vipOutput[1])
    {
        NvMediaVideoMixerUnbindOutput(vipMixer, vipOutput[1], NULL);
        NvMediaVideoOutputDestroy(vipOutput[1]);
    }
    if(vipMixer)
        NvMediaVideoMixerDestroy(vipMixer);


    // Destroy NvMedia device
    if(device)
        NvMediaDeviceDestroy(device);

    // Destroy NvMedia capture(s)
    if(vipCapture)
    {
        NvMediaVideoCaptureDestroy(vipCapture);

        // Reset VIP settings of the board
        switch (testArgs.vipDeviceInUse)
        {
            case AnalogDevices_ADV7180: // TBD
                break;
            case AnalogDevices_ADV7182: // TBD
                //testutil_capture_input_close(handle);
                break;
            default:
                break;
        }
    }

    return err;
}

void curveLine(int resHeight, int resWidth, IplImage* imgResult, int last_width)
{
    int curve_flag, curve_temp, sub_center, steer_angle;
    int j, k;

    curve_flag=0;

    printf("Corner in!!!!!!!!!!!!!!!!!!!\n");

    for(j = resHeight-1; j >= 0; j--)
    {
        for(k = resWidth-1; k >= 0; k--)
        {
            if(imgResult->imageData[j*imgResult->widthStep + k] == (char) 255)
            {
                last_height = j;
                //last_width = k;
                curve_flag++;
                break;
            }
        }
        if(curve_flag!=0)
        {
            curve_flag = 0;
            break;
        }
    }   //영상 제일 작은 Height값 검출

    for(j = resHeight-1; j >= 0; j--)
    {
        for(k = resWidth-1; k >= 0; k--)
        {
            if(tempResult->imageData[j*tempResult->widthStep + k] == (char) 255)
            {
                //last_height = j;
                last_width = k;
                curve_flag++;
                break;
            }
        }
        if(curve_flag!=0)
        {
            curve_flag = 0;
            break;
        }
    }   //최초의 width값 검출

    for(j = 0; j < 20; j++)
    {
        if(distance_table[j][1] >= last_height )
        {
            if(distance_table[j+1][1] < last_height)
            {
                encoder_value = distance_table[j][0]+31;
                //Nak
                printf("Stop!!!\n");
                printf("encoder_value : %d\n", encoder_value); //몇 센치 이동하는지 check
                //~Nak
                break;
            }
        }
    } //Table에서 움직인 거리 읽어옴

    UserMove(1, 1470, encoder_value*10);
    curve_temp = 675;  //곡선 인식 후 코너 진입점 까지 주행


    //TODO 각 코너별 flag 만들어서 진행!!!!
    if(last_width<(resWidth/2))
    {
        printf("--------------RIGHT---------------------\n");
        //				CameraXServoControl_Write(1270);
        UserMove(1, 1000, curve_temp);
        printf("---------------------------------finish!\n");
    } //오른쪽으로 회전 
    else
    {	
        printf("---------------LEFT---------------------\n");
        //				CameraXServoControl_Write(1670);
        UserMove(1, 1950, curve_temp);
        printf("---------------------------------finish!\n");
    } // 왼쪽으로 회전

    encoder_value=0;
}

void driveLine()
{
    int sub_center, steer_angle; 

    printf("Go straight!!!!!!!!!!!!!!!!!\n");
    CameraXServoControl_Write(1470);

    center_avg = center_total / center_num;
    sub_center = 160 - center_avg; 
    steer_angle += 4*sub_center;
    SteeringServoControl_Write(steer_angle);
}

void emergStop()
{

}
void parkFirst()
{

}
void parkSecond()
{

}
void interSection()
{

}
void overTaking()
{

}
void trafficLight()
{

}
void hill()
{

}
