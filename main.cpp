#include "mbed.h"
#include "Bmx055.h"
#include "VL53L0X.h"
#include "motion.h"

#include <string>
#include <map>

bool debugMode = false;
bool debugVerbose = false;

Serial host(USBTX, USBRX, 115200);
static DigitalOut led1(LED1);
static DigitalIn initButton(USER_BUTTON); /// F401RE: B1  (blue sw)  0:on, 1:off

static Thread thread(osPriorityNormal1);

static Thread threadAction;
static Semaphore actionPending(0);
void (*action)(void) = NULL;

static LEG_SET currentLegSet = LEG_SET_TRANSIT;
static uint32_t initialBodyHeight = 800;

static DevI2C i2c(PB_9, PB_8);

static Bmx055 gyro(&i2c, 0x19, 0x69, 0x13);

static DigitalOut tofPower(D2, 0);
static VL53L0X tof(&i2c, &tofPower, D9, VL53L0X_DEFAULT_ADDRESS);
static Semaphore tofReady(0);

static uint32_t expectedBodyHeight = 200;
static uint32_t bodyHeight = 0;
static Mutex mutexHeight;
static void setBodyHeight(uint32_t height)
{
    mutexHeight.lock();
    bodyHeight = (bodyHeight * 9 + height) / 10;
    mutexHeight.unlock();
}
static uint32_t getBodyHeight()
{
    uint32_t height;
    mutexHeight.lock();
    height = bodyHeight;
    mutexHeight.unlock();

    return height;
}

static void tof_init()
{
    int status;
    status = tof.init_sensor(VL53L0X_DEFAULT_ADDRESS);  
    if (status != VL53L0X_ERROR_NONE) {
        if (debugMode) {
            printf("//VL53L0X.init_sensor: %d\r\n", status);
        }
    }
}

static void tof_read_thread()
{
    int status;
    VL53L0X_RangingMeasurementData_t data;

    while (true) {
        tofReady.wait();
        status = tof.handle_irq(range_continuous_interrupt, &data);
        if (!status) {
            if (!data.RangeStatus) {
                setBodyHeight(data.RangeMilliMeter);
            } else {
                if (debugMode) {
                    printf("//RangeStatus: %u, RangeMilliMeter: %u\r\n", data.RangeStatus, data.RangeMilliMeter);
                }
            }
        } else {
            if (debugMode) {
                printf("//Status: %d\r\n", status);
            }
        }
    }
}
static void irqReady()
{
    tofReady.release();
}

static void actionInit()
{
    static bool initRunning = false;
    if (initRunning) {
        return;
    }

    initRunning = true;

    int i;
    currentLegSet = LEG_SET_TRANSIT;

    /// 全部縮める.
    for (i = 2; i <= 7; i++) {        legMotion(i, DIR_CONTRACT, 255);    }
    Thread::wait(4500);
    for (i = 2; i <= 7; i++) {        legMotion(i, DIR_STOP,     255);    }

    /// 少し待つ ここで電源を切れば、収納状態
    Thread::wait(4000);

    /// 初期位置まで伸ばす. 実験用
    for (i = 2; i <= 7; i++) {        legMotion(i, DIR_EXPAND,   240);    }
    Thread::wait(1000);
    for (i = 2; i <= 7; i++) {        legMotion(i, DIR_STOP,     100);    }

    initialBodyHeight = getBodyHeight();
    printf("Initial height: %d\r\n", initialBodyHeight);

    initRunning = false;
} ////////////// actionInit()

static void actionII()
{
    int i;
    currentLegSet = LEG_SET_TRANSIT;

    /// 全部縮める.
    for (i = 2; i <= 7; i++) { legMotionf(i, DIR_CONTRACT, 1.0f);    }
    Thread::wait(2000);
    for (i = 2; i <= 7; i++) { legMotionf(i, DIR_STOP,     1.0f);    }
    
    /////////////////////////
    
    int t = 300;
    for (i = 2; i <= 7; i++)  {
        
        legMotionf(i, DIR_EXPAND,   1.0f);
        Thread::wait(t);
  /*      legMotionf(i, DIR_EXPAND,   0.1f);
        Thread::wait(1000);
        legMotionf(i, DIR_EXPAND,   0.01f);
        Thread::wait(1000); */
        legMotion(i, DIR_STOP,     10);
        Thread::wait(t);
        
    }
     for (i = 2; i <= 7; i++)  {      
        legMotionf(i, DIR_EXPAND,   0.5f);
        Thread::wait(t);
        legMotion(i, DIR_STOP,     10);
        Thread::wait(t);     
    }
      for (i = 2; i <= 7; i++)  {      
        legMotionf(i, DIR_EXPAND,   0.2f);
        Thread::wait(t);
 //       legMotion(i, DIR_STOP,     10);
 //       Thread::wait(t);     
    }   
       for (i = 2; i <= 7; i++)  {      
        legMotionf(i, DIR_EXPAND,   1.0f);
        Thread::wait(t);
        legMotion(i, DIR_STOP,     10);
        Thread::wait(t);     
    }  
    initialBodyHeight = getBodyHeight();
    printf("Initial height: %d\r\n", initialBodyHeight);
    
}/////////////// actionII

static void actionAA() //left groupe blue
{
    int i;
    /// 全部縮める.
    for (i = 2; i <= 7; i++) {        legMotionf(i, DIR_CONTRACT, 1.0f);    }
    Thread::wait(500);
    for (i = 2; i <= 7; i++) {        legMotionf(i, DIR_STOP,     1.0f);    }

    /// 少し伸ばす 青 ok
    legMotionf(2, DIR_EXPAND,   1.0f);
    legMotionf(4, DIR_EXPAND,   1.0f);
    legMotionf(6, DIR_EXPAND,   1.0f);
    Thread::wait(500);
    legMotionf(2, DIR_STOP,    0.2f);  /// 一旦全部止めるのはpwmが制御不能にならないようにするための施策 ★★★
    legMotionf(4, DIR_STOP,    0.2f);
    legMotionf(6, DIR_STOP,    0.2f);
    Thread::wait(1);
    legMotionf(2, DIR_EXPAND,  0.14f);
    legMotionf(4, DIR_EXPAND,  0.10f);
    legMotionf(6, DIR_EXPAND,  0.14f); 
    Thread::wait(1);
    legMotionf(2, DIR_STOP,    0.14f);
//  legMotionf(4, DIR_STOP,    0.14f);
    
    //currentLegSet = LEG_SET_A;

}/// AA

static void actionBB() /// right group red
{
    int i;
    /// 全部縮める.
    for (i = 2; i <= 7; i++) {        legMotionf(i, DIR_CONTRACT, 1.0f);    }
    Thread::wait(500);
    for (i = 2; i <= 7; i++) {        legMotionf(i, DIR_STOP,     1.0f);    }

    /// 少し伸ばす 赤
    legMotionf(7, DIR_EXPAND, 1.0f);
    legMotionf(5, DIR_EXPAND, 1.0f);
    legMotionf(3, DIR_EXPAND, 1.0f);
    Thread::wait(500);
    legMotionf(7, DIR_EXPAND, 0.1f);
    legMotionf(5, DIR_EXPAND, 0.1f);
    legMotionf(3, DIR_EXPAND, 0.1f);
    
    //currentLegSet = LEG_SET_B;
}/// BB

static void actionCC() /// 歩行用 短時間で全部縮める
{
    int i;
    /// 全部縮める.
    for (i = 2; i <= 7; i++) { legMotionf(i, DIR_CONTRACT, 1.0f);    }
    Thread::wait(400);
    for (i = 2; i <= 7; i++) { legMotionf(i, DIR_STOP,     1.0f);    }   
     
}/////////////// actionCC

static void actionMM() 
{
    currentLegSet = LEG_SET_MM;
    
}/// mm

static void actionA()
{
    if (currentLegSet == LEG_SET_B) {
        int i;
        currentLegSet = LEG_SET_TRANSIT;
        for (i = 0; i < 6; i++) {
            if( 0==i%2 ){
                legMotion(i + 2, DIR_CONTRACT, 255);
            }
        }
        Thread::wait(5000);
        for (i = 0; i < 6; i++) {
            //   legMotion(i + 2, DIR_STOP, 0);
            legMotion(i + 2, DIR_STOP, 10);
        }
    }
    currentLegSet = LEG_SET_A;
    
}/// left

static void actionB()
{
    if (currentLegSet == LEG_SET_A) {
        int i;
        currentLegSet = LEG_SET_TRANSIT;
        for (i = 0; i < 6; i++) {
            if( 1==i%2 ){
                legMotion(i + 2, DIR_CONTRACT, 255);
            }
        }
        Thread::wait(5000);
        for (i = 0; i < 6; i++) {
            //   legMotion(i + 2, DIR_STOP, 0);
            legMotion(i + 2, DIR_STOP, 10);
        }
    }
    currentLegSet = LEG_SET_B;
}

static int param1UpDown, param2UpDown;

static void actionUp()
{
    currentLegSet = LEG_SET_TRANSIT;
    legMotion(param1UpDown, DIR_EXPAND, param2UpDown);
    Thread::wait(500);
    legMotion(param1UpDown, DIR_STOP, 10);
}

static void actionDown()
{
    currentLegSet = LEG_SET_TRANSIT;
    legMotion(param1UpDown, DIR_CONTRACT, param2UpDown);
    Thread::wait(500);
    legMotion(param1UpDown, DIR_STOP, 10);
}

static void processHostCommand(const char *cmdLine)
{
    char cmd[11];
    int param1, param2;
    int num = sscanf(cmdLine, "%10s %d %d", cmd, &param1, &param2);
    if (num <= 0) {
        if (debugMode) {
            printf("//Invalid command: %s\r\n", cmdLine);
        }
        return;
    }

    if        (string(cmd).compare("init") == 0) {
        action = actionInit;
        actionPending.release();
    } else if (string(cmd).compare("ii") == 0) {
        action = actionII;
        actionPending.release();
    } else if (string(cmd).compare("mm") == 0) { /// motion()を強制的に動かす
        action = actionMM;
        actionPending.release();        
    } else if (string(cmd).compare("aa") == 0) {
        action = actionAA;
        actionPending.release();
    } else if (string(cmd).compare("bb") == 0) {
        action = actionBB;
        actionPending.release();
    } else if (string(cmd).compare("cc") == 0) { /// 歩行用 短時間で全部縮める
        action = actionCC;
        actionPending.release();
    } else if (string(cmd).compare("left") == 0) {
        action = actionA;
        actionPending.release();
    } else if (string(cmd).compare("right") == 0) {
        action = actionB;
        actionPending.release();
    } else if (string(cmd).compare("height") == 0 && num >= 2) {
        expectedBodyHeight = param1;
        if (debugMode) {
            printf("Expected body height changed to %d\r\n", expectedBodyHeight);
        }
    } else if (string(cmd).compare("who") == 0 || string(cmd).compare("who:") == 0) {
        printf("stm\r\n");
    } else if (string(cmd).compare("debug") == 0) {
        if (string(cmdLine).compare("debug on") == 0) {
            debugMode = true;
            debugVerbose = false;
            printf("//Debug mode ON\r\n");
        } else if (string(cmdLine).compare("debug verbose") == 0) {
            debugMode = true;
            debugVerbose = true;
            printf("//Debug mode Verbose\r\n");
        } else {
            // off
            printf("//Debug mode OFF\r\n");
            debugMode = false;
        }
    } else if (string(cmd) == "up" && num >= 3) {
        action = actionUp;
        param1UpDown = param1;
        param2UpDown = param2;
        actionPending.release();
    } else if (string(cmd) == "down" && num >= 3) {
        action = actionDown;
        param1UpDown = param1;
        param2UpDown = param2;
        actionPending.release();
    } else if (string(cmd) == "u" && num >= 3) {
        action = actionUp;
        param1UpDown = param1;
        param2UpDown = param2;
        actionPending.release();
    } else if (string(cmd) == "d" && num >= 3) {
        action = actionDown;
        param1UpDown = param1;
        param2UpDown = param2;
        actionPending.release();        
    } else {
        if (debugMode) {
            printf("//Invalid command: %s\r\n", cmdLine);
        }
    }

    // TODO: add host commands here
}

static void hostSerialCallback()
{
    static string cmdLine("");
    int c = host.getc();
    switch (c) {
        case '\r':
        case '\n':
            // TODO: parse command
            if (!cmdLine.empty()) {
//                host.printf("Echo: %s\r\n", cmdLine.c_str());
                processHostCommand(cmdLine.c_str());
                cmdLine = "";
            }
            break;
        default:
            cmdLine.append(1, c);
            break;
    }
}

static void actionThread()
{
    while (true) {
        actionPending.wait();
        action();
    }
}

int main()
{
    int initButtonState = 1;

    initButton.mode(PullUp); /// 青ボタン
    gyro.Initialize();
    Thread::wait(100);
    threadAction.start(actionThread);
    tof_init();
    thread.start(tof_read_thread);
    tof.start_measurement(range_continuous_interrupt, irqReady);
    host.attach(hostSerialCallback);

    while (true) {
        initButtonState = initButton;
        if (initButtonState == 0) {
            actionInit();
        }

#define ROLL_OFFSET  3.2f
#define PITCH_OFFSET -0.6f
        float roll, pitch, yaw, r0,p0;
        gyro.read(roll, pitch, yaw);
        roll += ROLL_OFFSET;
        pitch += PITCH_OFFSET;
        
        roll  = roll *0.2f + r0*0.8f;
        pitch = pitch*0.2f + p0*0.8f;  
        
////         

          
        r0=roll;
        p0=pitch;
        
        if (debugMode && debugVerbose) {
            printf("//(height, roll, pitch, yaw): %u, %lf, %lf, %lf\r\n", getBodyHeight(), roll, pitch, yaw);

        }
            //printf("r,p: %03.3f, %03.3f\r\n",roll, pitch);

        if (currentLegSet == LEG_SET_A || currentLegSet == LEG_SET_B || currentLegSet == LEG_SET_MM) {
            uint32_t h0=getBodyHeight();
//            printf("%u\r\n", h0);
            motionf(currentLegSet, initialBodyHeight, h0, roll, pitch);
        }
        Thread::wait(20);
    }
}
