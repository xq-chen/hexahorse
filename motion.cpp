#include "motion.h"
#include "mbed.h"
//#include <math.h>

static DigitalOut *dout_dir[][2] = {
    { new DigitalOut(PC_10), new DigitalOut(PC_12) }, /// 0 AIN1,AIN2
    { new DigitalOut(PA_13), new DigitalOut(PA_14) }, /// 1 BIN1,BIN2
    { new DigitalOut(PA_15), new DigitalOut(PB_3) },  /// 2 AIN1,AIN2
    { new DigitalOut(PC_11), new DigitalOut(PD_2) },  /// 3 BIN1,BIN2
    { new DigitalOut(PC_8), new DigitalOut(PA_12) },  /// 4 AIN1,AIN2
    { new DigitalOut(PA_11), new DigitalOut(PB_12) }, /// 5 BIN1,BIN2
    { new DigitalOut(PB_2), new DigitalOut(PB_15) },  /// 6 AIN1,AIN2
    { new DigitalOut(PB_14), new DigitalOut(PB_13) }, /// 7 BIN1,BIN2
};
static PwmOut* pwmLeg[] = {
    new PwmOut(PC_9), /// 0
    new PwmOut(PB_7), /// 1 I2C1_SDA と重複しているので、こちらを空いているpwm portに変更org:PB_9
    new PwmOut(PB_6), /// 2
    new PwmOut(PA_9), /// 3
    new PwmOut(PB_4), /// 4
    new PwmOut(PB_5), /// 5
    new PwmOut(PA_8), /// 6
    new PwmOut(PB_10) /// 7
};

static Semaphore semaphore_pwm(1);  /// pwmの多重アクセスをガードするためのセマフォ

/// 入力：pwm: 0～1.0f 
void   legMotionf(int num, LEG_DIRECTION dir, float pwm)
{
    semaphore_pwm.wait();

    switch (dir) {
        case DIR_EXPAND:
            dout_dir[num][0]->write(0);
            dout_dir[num][1]->write(1);
            break;
        case DIR_CONTRACT:
            dout_dir[num][0]->write(1);
            dout_dir[num][1]->write(0);
            break;
        case DIR_BREAK:
            dout_dir[num][0]->write(1);
            dout_dir[num][1]->write(1);
            break;           
        case DIR_STOP:
            dout_dir[num][0]->write(0);
            dout_dir[num][1]->write(0);
            break;           
        default:
            // Unknown direction
            dout_dir[num][0]->write(0);
            dout_dir[num][1]->write(0);
            break;
    }

    /// 1にクリップ
    const float PWM_MAX = 1.0f;
    if( PWM_MAX < pwm ) { 
        pwm = PWM_MAX;  
    }  

    ///  0に入れない処理
    const float PWM_THRESHOLD = 0.001f;
    if( pwm < PWM_THRESHOLD) {
        pwm = PWM_THRESHOLD;    
    }
    pwmLeg[num]->period(0.0003f); ///default:0.020
    pwmLeg[num]->write(pwm);       /// ★★★
 
    float pwm2;
    pwm2 = pwmLeg[num]->read();
    
    
    if( DIR_CONTRACT == dir ){ /// 縮むときはマイナスを表示
        printf("lmf:- %3.2f\r\n", pwm2);
    } else {
        printf("lmf:  %3.2f\r\n", pwm2); 
    }
       
    semaphore_pwm.release();
}



/// speed: 0 to 255
void legMotion(int num, LEG_DIRECTION dir, int speed)
{
    semaphore_pwm.wait();
    
//    printf("ll[%d] %d %d, ", num, dir, speed);

    if (speed == 0) {
        dir = DIR_STOP;
    }

    switch (dir) {
        case DIR_EXPAND:
            dout_dir[num][0]->write(0);
            dout_dir[num][1]->write(1);
                pwmLeg[num]->write((float)speed / 255.0f);
            break;
        case DIR_CONTRACT:
            dout_dir[num][0]->write(1);
            dout_dir[num][1]->write(0);
            pwmLeg[num]->write((float)speed / 255.0f);
            break;
        case DIR_BREAK:
            dout_dir[num][0]->write(1);
            dout_dir[num][1]->write(1);
            break;
        default:
            // Unknown direction
            dout_dir[num][0]->write(0);
            dout_dir[num][1]->write(0);
            break;
    }
    
    printf("gg[%d] %d %d, ", num, dir, speed);
    
    semaphore_pwm.release();
}


/// motion()関数をベースに、変数pwmをfloat化
int g_stopState=0;  /// 0以外なら駆動停止

void motionf(LEG_SET legSet,
            uint32_t initialBodyHeight,
            uint32_t bodyHeight,
            float roll, float pitch)
{
#define X_RADIUS 140.0f  // 140[mm]
#define Y_RADIUS 190.0f  // 190[mm]
#define PI 3.14f

/*
    // Ignore Roll or Pitch smaller than 1 degree
    if (fabs(roll) < 1)
        roll = 0;
    if (fabs(pitch) < 1)
        pitch = 0;
*/
    
    float delta[8] = {        0,    };
    float pwm = 0.0f;
    

    static int h_ctrl_stat = 0;  /// 水平補正の状態
    
    // 角度異常の検出
    if ( 10 < fabs(roll)  ) {h_ctrl_stat = 1; printf("emargency stop  roll:  %f \r\n", roll ); }
    if ( 10 < fabs(pitch) ) {h_ctrl_stat = 2; printf("emargency stop  pitch: %f \r\n", pitch); }
 
    int i=0;
    if( h_ctrl_stat ){
        for ( i = 2; i <= 7; i++ ) { legMotionf( i, DIR_STOP, pwm );  } /// 緊急停止 emargency stop
        
        return;  /// ★
    }

    uint32_t destBodyHeight = initialBodyHeight + 30; /* TBD: 30mm */

    float heightDelta = (int32_t)destBodyHeight - (int32_t)bodyHeight;
    if (heightDelta > 10.0f) {
        heightDelta = 10.0f;
    } else if (heightDelta < -10.0f) {
        heightDelta = -10.0f;
    }
    heightDelta = 0.0f; //// ★★★
    
    float rollDelta  = 2.0f * PI * X_RADIUS * roll  / 360.0f;
    float pitchDelta = 2.0f * PI * Y_RADIUS * pitch / 360.0f;
    printf("mot %f %f %f %f\r\n", roll, pitch, rollDelta, pitchDelta);

    switch (legSet) {
        case LEG_SET_A:
            // right
            delta[6] = (heightDelta + rollDelta) * 2.0f;
            // left front
            delta[2] = heightDelta  + pitchDelta - rollDelta;
            // left back
            delta[4] = heightDelta  - pitchDelta - rollDelta;
            break;
        case LEG_SET_B:
            // left
            delta[3] = (heightDelta - rollDelta) * 2.0f;
            // right front
            delta[5] = heightDelta + pitchDelta + rollDelta;
            // right back
            delta[7] = heightDelta - pitchDelta + rollDelta;
            break;
        default:
            // Do nothing
            break;
    }

     LEG_DIRECTION dir[8] = { DIR_STOP };

    
    /// 水平補正処理
//    static int h_correct_stat;  ///  0:stop , 1:処理中、 ヒステリシスを得るため

    float maxValue = 0.0f;
    const float delta_threshold   = 10.0f;

    
    for (i = 0; i < 6; i++) {
        maxValue = maxValue > fabs(delta[i]) ? maxValue : fabs(delta[i]);
    }
    
    if( 3.0f < maxValue  ){
        for ( i = 2; i <= 7; i++ ) {        
            if( delta_threshold < delta[i] ){
                dir[i] = DIR_EXPAND;
            }else if ( delta[i] < -delta_threshold){
                dir[i] = DIR_CONTRACT;
            }else {
                dir[i] = DIR_STOP;
            }       
        }/// for 
    } else {
        for ( i = 2; i <= 7; i++ ) {        
                dir[i] = DIR_STOP;      
        }/// for        
        
    }
           
    /// モータの駆動  
    ///      const float kp = 0.05f; 
    for ( i = 2; i <= 7; i++ ) { 
        float pwm = 0.7;   //// ★  
        /// float pwm = delta[i] * kp;   /// ★★★
        legMotionf( i, dir[i],   pwm );  
    }
    
    return;
}///////////// motionf()

#if 1
void motion(LEG_SET legSet,
            uint32_t initialBodyHeight,
            uint32_t bodyHeight,
            float roll, float pitch)
{
#define X_RADIUS 140.0f // 140[mm]
#define Y_RADIUS 190.0f  // 190[mm]
#define PI 3.14f
#define PWM_THRESHOLD 0.02f // 0.02f

    // Ignore Roll or Pitch smaller than 1 degree
    if (fabs(roll) < 1)
        roll = 0;
    if (fabs(pitch) < 1)
        pitch = 0;

    float delta[6] = {
        0.0f,
    };

    uint32_t destBodyHeight = initialBodyHeight + 30; /* TBD: 3cm */

    float heightDelta = (int32_t)destBodyHeight - (int32_t)bodyHeight;
    if (heightDelta > 10.0f) {
        heightDelta = 10.0f;
    } else if (heightDelta < -10.0f) {
        heightDelta = -10.0f;
    }

    heightDelta = 0.0f;

    float rollDelta  = 2.0f * PI * X_RADIUS * roll  / 360.0f;
    float pitchDelta = 2.0f * PI * Y_RADIUS * pitch / 360.0f;
    printf("mot %f %f %f %f\r\n", roll, pitch, rollDelta, pitchDelta);

//  rollDelta = 0; // 20190730
    switch (legSet) {
        case LEG_SET_A:
            // right
            delta[6-2] = (heightDelta + rollDelta) * 2.0f;
            // left front
            delta[2-2] = heightDelta  + pitchDelta - rollDelta;
            // left back
            delta[4-2] = heightDelta  - pitchDelta - rollDelta;
            break;
        case LEG_SET_B:
            // left
            delta[3-2] = (heightDelta - rollDelta) * 2.0f;
            // right front
            delta[5-2] = heightDelta + pitchDelta + rollDelta;
            // right back
            delta[7-2] = heightDelta - pitchDelta + rollDelta;
            break;
        default:
            // Do nothing
            break;
    }

    int i;
    float maxValue = 10.0f;
    // Find the biggest delta
    for (i = 0; i < 6; i++) {
        // printf("%f ", delta[i]);
        maxValue = maxValue > fabs(delta[i]) ? maxValue : fabs(delta[i]);
    }
    printf("\r\n");

    for (i = 0; i < 6; i++) {
        float pwm = 0.0f;
 //       const float kp = 0.05f;

        pwm = delta[i] / maxValue;
        printf("pwm[%d] %f, ", i, pwm);
        //  pwm = delta[i] * delta[i] /20 / maxValue;
        //  pwm = delta[i] * kp;

        /*if (fabs(pwm) < PWM_THRESHOLD) {
            //legMotion(i + 2, DIR_STOP, 0);
            legMotion(i + 2, DIR_STOP, 1);
        } else*/ if (pwm > 0) {
            int pwmInt = floor( pwm * 255);
            legMotion(i + 2, DIR_EXPAND, pwmInt);
        } else {
            // pwm < 0
            int pwmInt = floor(-pwm * 255);
            legMotion(i + 2, DIR_CONTRACT, pwmInt);
        }
    }
    printf("\r\n"); 
}
#endif

/// 20190714 ktanaka   /// まだ、なにも動作に変化はないはず。。。
/// motion(currentLegSet, initialBodyHeight, getBodyHeight(), roll, pitch);
///         A/B,
/* 実装前に仕様を考えてみる。。。 20190714
●脚のID
 Front
[2] [5]
[3] [6]
[4] [7]
 L   R

A組 : [3],[5],[7] ?
B組 : [2],[4],[6] ?

●足の切り替えアルゴリズム
・Right,Leftコマンドで使用するそれぞれ3本の脚の組をA組、B組とする
・Aの脚を伸ばして立つ場合
   1. Bの足を全部ひっこめる
   2. 1.をやりつつ、(ABの足の差は少ないとして多少の不安定は無視し、すぐにAの脚に切り替わる前提で) 3.を行う
   3．Aの足を延ばし側に駆動し、Aによる水平処理を行う
・水平処理
  ・等間隔の周期で繰り返す (必要に応じて)
  ・周期は、1秒間に10回以上くらいは欲しい
  ・処理
    ・3点の距離を暫定的に固定する ⇒ 実測: 前後380mm、幅280mmの二等辺三角形abcとする。（ab=bc, ac=380mm)
    ・a,b,c点のz軸方向の補正量をza,zb,zcとする
    ・zaを固定し、zb, zcを計算する
    ・取得したロール値θrによるb点の駆動距離zbの計算   zb1 = 280 x sinθr
    ・取得したピッチ値θpによる3本脚の駆動距離計算  zc = 380 x sinθp, zb2 = zc/2
    ・3本脚駆動距離の合計 za=0, zb = zb1+zb2, zc
    ・高さの補正 （tofによる補正) za,zb,zcをそれぞれ現状の差分から目的高さ(Min+30mm?)に近づける。近い場合は何もしない。
    ・駆動 za, zb, zc により駆動する。

・駆動処理
  ・補正値がある値を超えればpwm：100%駆動
     ・目標の駆動距離に係数をかけて、pwmが100％になるような係数を決定 10mmくらい？
        ・そこから、2㎜くらいまでは60％くらいまで、比例してpwmが落ちるように
          ・2㎜(暫定値)を下回ると駆動を止める
              ・60％ってのは、駆動方向で変えるのがいいかも。縮める方向ならもっと小さくできる

●モード  (周期処理用のRAM)
  ・A/B どちらの組で立つように駆動するか  drive_groupe:  0:A, 1:B
  ・駆動中、駆動終わり  drive_mode[drive_groupe] off:0, on:1

●フィルタリング
  ・２～４周期分の移動平均なり、LPFが必要かも。。。

■20190730 ToDo
x)pwm変数をfloatに統一
x)pwm周りにセマフォガード
x)AAで6の脚が下がるのを防ぐ調整
x)pwmが制御不能になることを回避する方法を探る ⇒ stopした後にpwm値を代入する
x)IMU,ToFを一旦動かないようにして、pwm動作が改善しないか確認 => 改善しない
)ToF高さ処理
 x)水平処理を一旦、止める
 x)現在高さ表示
 x)向きを確認
 x)gain確認

*/


// Test vectors
//motion(LEG_SET_A, 500, 550, -15.0f, 0);
//motion(LEG_SET_A, 500, 550, 15.0f, 0);
//motion(LEG_SET_A, 500, 550, 0, -15.0f);
//motion(LEG_SET_A, 500, 550, 0, 15.0f);
//motion(LEG_SET_A, 500, 550, -15.0f, 15.0f);
