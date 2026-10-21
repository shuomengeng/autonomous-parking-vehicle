/*
  ESP32 parking car controller
  ----------------------------
  Receives DRIVE / (legacy ALIGN) / PARK / UNPARK / STOP commands over UDP.
  Reads four distance sensors, filters their readings, and runs the
  parking state machine plus the standalone-tested timed UNPARK path.
  UNPARK timings and angles are set/ACKed from the laptop controller.

  Speed note: motorForward() and motorReverse() currently use digitalWrite,
  so their speed argument does not set PWM speed; motion is full power.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// 1. 基本设置：先从这里找需要调整的引脚、舵机角度、距离和时间。

// reverse the car forward 5s and then check the second target colour
unsigned long unparkReverseAlignMs = 4500;

const unsigned long UNPARK_BACK_CHECK_MS = 1600;

const int PARK_WALL_TARGET_MM = 75;

const int PARK_SENSOR_CONFIRMS  = 2;

//enum 就是枚举，uint8,unsign integer 8 bits range is 0 255
enum CarState : uint8_t
{
    STATE_DRIVE  = 1,
    STATE_ALIGN  = 2,
    STATE_PARK   = 3,
    STATE_UNPARK = 4,
    STATE_STOP   = 5
};

struct Filtered
{
    float value = 0;
    int misses = 0;
    bool valid = false;
};

const char* const MSG_READ        = "READ";
const char* const MSG_PARK_DONE   = "PARK_DONE";
const char* const MSG_UNPARK_DONE = "UNPARK_DONE";

// Wi-Fi / UDP：电脑端程序必须使用相同的 Wi-Fi 和 UDP 端口。
const char* const WIFI_SSID     = "autoparkingtest";
const char* const WIFI_PASSWORD = "1111111166";
const uint16_t UDP_PORT = 3333;

const unsigned long COMMAND_TIMEOUT_MS = 30000; //PLACEHOLDER: 30 s for debugging, use ~1500 for real runs
const unsigned long DONE_RESEND_MS = 200;

const int MOTOR_FWD_PIN = 14;
const int MOTOR_REV_PIN = 19;
const int SERVO_PIN     = 26;

const int RIGHT_SENSOR1_PIN = 33;
const int RIGHT_SENSOR2_PIN = 32;
const int BACK_SENSOR_PIN   = 12;
const int BACK_SENSOR2_PIN  = 17;

// 舵机角度（度）：本车角度越大，车轮越向左。
const int SERVO_CENTRE = 102;
const int SERVO_LEFT   = 140;
const int SERVO_RIGHT  = 65;

// 前进对齐参数：R1/R2 同时控制车身平行度和与墙的侧向距离。
// 根据用户最近一组正常沿墙行驶数据校准：R1-R2 约 -4 mm，平均距离约 109.5 mm。
// R1 假设安装在右前方，R2 安装在右后方；舵机角度越小越向右，越大越向左。
const float ALIGN_PARALLEL_DIFF_MM = -4.0f;          // 实车平行时 R1-R2 的基准差值
const float ALIGN_TARGET_DISTANCE_MM = 109.5f;       // 实车正常沿墙行驶时的平均侧向距离
// 控制器内部约定：steeringCommandDeg > 0 表示向右/朝墙转；本车舵机角度越小越向右。
const int   ALIGN_DIRECTION = -1;                     // 1 表示正控制量对应向墙转；舵机角度计算时会减去该量
float ALIGN_GAIN_DEG_PER_MM = 0.60f;                 // laptop 可调：Kh，车身角度误差增益
float ALIGN_DISTANCE_GAIN_DEG_PER_MM = 0.15f;        // laptop 可调：Kd，侧向距离误差增益
float ALIGN_TRIM_KI_DEG_PER_MM_SEC = 0.01f;          // laptop 可调：Ki，learned trim 学习速度
const float ALIGN_GAIN_MIN = 0.05f;
const float ALIGN_GAIN_MAX = 2.00f;
const float ALIGN_DISTANCE_GAIN_MIN = 0.00f;
const float ALIGN_DISTANCE_GAIN_MAX = 1.00f;
const float ALIGN_TRIM_KI_MIN = 0.00f;
const float ALIGN_TRIM_KI_MAX = 0.10f;
const float ALIGN_TRIM_LIMIT_DEG = 6.0f;              // learned trim 限幅，防止积分累积过大
const float ALIGN_MAX_STEER_RATE_DEG_PER_SEC = 45.0f;  // 舵机转向控制量最大变化速率
const int   ALIGN_MAX_DEG = 15;                       // 相对中心角允许的最大转向偏移
const int   ALIGN_MAX_DIFF_MM = 100;                  // 原始差值异常大时忽略该组数据，可能是墙面缺口
const float ALIGN_DEADBAND_MM = 5.0f;                 // 角度误差死区
const float ALIGN_DISTANCE_DEADBAND_MM = 4.0f;        // 平均距离误差死区
const unsigned long ALIGN_STEER_PERIOD_MS = 50;      // 每 100 ms 最多更新一次控制器
const unsigned long ALIGN_DEBUG_INTERVAL_MS = 200;    // 每 0.5 秒输出一次对齐调试值

// 停车转弯角：车位在右侧用 SERVO_RIGHT；在左侧时按实车调整。
const int PARK_STEER_ANGLE = SERVO_RIGHT; //PLACEHOLDER: SERVO_LEFT if bays are on the left

// 电机速度占位参数：目前用 digitalWrite，所以下面的数值不会真正调节速度。
const int DRIVE_SPEED     = 255;
const int ALIGN_SPEED     = 255;
const int MANOEUVRE_SPEED = 255;

// 距离传感器：将脉冲宽度换算成毫米。
const unsigned long SENSOR_TIMEOUT_US   = 45000;
const unsigned long SENSOR_PULSE_MAX_US = 1850;
const long SENSOR_PULSE_OFFSET_US = 1000;
const long SENSOR_MM_PER_US       = 4;
const int  SENSOR_MIN_MM          = 10;   //anything closer than this is treated as a bad reading

const unsigned long SENSOR_PERIOD_MS = 50; //how often the sensors are read
const int   SENSOR_MAX_MISSES = 5;         //bad readings in a row before a sensor counts as "no reading"
const float FILTER_ALPHA_RIGHT = 0.8f;        // 固定传感器滤波系数；本次不通过 laptop 调整它
const float FILTER_ALPHA_BACK  = 0.8f;

// PARK：先倒车转弯；后方传感器确认车身平行后，再直线倒车靠墙。
// PARKING_TURN_OFFSET 由电脑端 Python 程序使用，这个常量不在本文件中。



const unsigned long PARK_SETTLE_MS  = 15;
const unsigned long PARK_ARC_MIN_MS = 3000;
// 默认弧线倒车最短时间（电脑 Python 启动时会通过 UDP 覆盖此值）。
// PARK_ALIGN_MIN_TIME_MS、前进对齐控制增益和 UNPARK 直线前进时间均可由 laptop 启动前通过 UDP 配置。
const unsigned long DEFAULT_PARK_ALIGN_MIN_TIME_MS = 6500;
const unsigned long PARK_ALIGN_MIN_TIME_MIN_MS = 1000;
const unsigned long PARK_ALIGN_MIN_TIME_MAX_MS = 29000;
unsigned long parkAlignMinTimeMs = DEFAULT_PARK_ALIGN_MIN_TIME_MS;
const unsigned long PARK_ARC_MAX_MS = 30000;


const unsigned long PARK_STRAIGHTEN_MS = 1000;

// 后方双传感器对齐：按用户确认的实车平行基准，B - B2 约为 50 mm。
const int BACK_SENSOR_OFFSET_MM = 50;
const int BACK_ALIGN_TOLERANCE_MM = 15;
const int BACK_ALIGN_CONFIRMATIONS = 3;
const int BACK_STRAIGHT_ANGLE = 108;

// 记录本次倒车开始时间，用同一个 30 秒总时限保护整个倒车流程。
int backAlignHits = 0;
unsigned long parkArcStartMs = 0;

// Combined UNPARK motion sequence, with timings sent by the laptop before START.
unsigned long unparkInitialForwardMs = 1300;
unsigned long unparkTurnForwardMs = 9000;

unsigned long unparkFinalForwardMs = 5000;
unsigned long unparkServoSettleMs = 300;
unsigned long unparkAlignTimeoutMs = 12000;
unsigned long unparkTurnAngleDeg = 65;
unsigned long unparkCentreAngleDeg = 106;

// Same side-wall alignment values as the standalone UNPARK test.
const float UNPARK_ALIGN_PARALLEL_DIFF_MM = -4.0f;
const float UNPARK_ALIGN_TARGET_DISTANCE_MM = 109.5f;
const float UNPARK_ALIGN_HEADING_TOLERANCE_MM = 5.0f;
const float UNPARK_ALIGN_DISTANCE_TOLERANCE_MM = 4.0f;
const int UNPARK_ALIGN_DIRECTION = -1;
const float UNPARK_ALIGN_GAIN_DEG_PER_MM = 0.6f;
const float UNPARK_ALIGN_DISTANCE_GAIN_DEG_PER_MM = 0.15f;
const int UNPARK_ALIGN_MAX_DEG = 10;
const float UNPARK_ALIGN_MAX_DIFF_MM = 100.0f;
const unsigned long UNPARK_ALIGN_STEER_PERIOD_MS = 100;
const unsigned long UNPARK_ALIGN_DEBUG_INTERVAL_MS = 500;

// B/B2 correction settings imported from the standalone unpark test.

const unsigned long UNPARK_BACK_ADJUST_TIMEOUT_MS = 20000;
const unsigned long UNPARK_BACK_PULSE_MS = 200;
const unsigned long UNPARK_BACK_SETTLE_MS = 250;
const int UNPARK_BACK_TURN_ANGLE_DEG = 65;
const int UNPARK_BACK_FIRST_MOVE_DIRECTION = 1;
const float UNPARK_BACK_MIN_IMPROVEMENT_MM = 1.5f;

// 2. 电机和舵机控制

Servo steeringServo;
int lastAngle = -1;

//speed is not used: full speed only (digitalWrite). Kept so calls don't change.

// 前进：解除倒车输出，再打开前进输出。当前版本不使用 PWM 调速。
void motorForward(int speed)
{
    (void)speed; // 暂时保留参数，但数字输出模式下不会调节速度。
    digitalWrite(MOTOR_REV_PIN, LOW);
    digitalWrite(MOTOR_FWD_PIN, HIGH);
}

// 倒车：解除前进输出，再打开倒车输出。当前版本不使用 PWM 调速。
void motorReverse(int speed)
{
    (void)speed;
    digitalWrite(MOTOR_FWD_PIN, LOW);
    digitalWrite(MOTOR_REV_PIN, HIGH);
}

// 停车：两个电机控制输入都设为 HIGH，使用驱动板的制动状态。
void motorStop()
{
    digitalWrite(MOTOR_FWD_PIN, HIGH);
    digitalWrite(MOTOR_REV_PIN, HIGH);
}

// 设置舵机角度；角度没变时不重复发送指令。
void steerAngle(int angle)
{
    if (angle == lastAngle) return;
    steeringServo.write(angle);
    lastAngle = angle;
}

// 将前轮回正到预设中心角。
void steerCentre()
{
    steerAngle(SERVO_CENTRE);
}

// 3. 网络通讯

WiFiUDP udp;
IPAddress pcIP;
uint16_t pcPort = 0;
bool pcKnown = false;

// 连接配置好的 Wi-Fi，并启动 UDP 监听；连接成功前会等待。
void networkBegin()
{
    Serial.println();
    Serial.println("UDP START");
    Serial.print("SSID: ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("UDP port: ");
    Serial.println(UDP_PORT);

    //监听port， 负责接收udp
    udp.begin(UDP_PORT);
    Serial.println("UDP READY");
    Serial.println();
}

// 接收一条 UDP 指令，并记录发来指令的电脑地址。
// out：参数名称，用来保存接收到的文本
bool networkReceive(String &out)
{

    //检查有没有 UDP 数据包到达,如果没有，返回 false
    int packetSize = udp.parsePacket();
    if (!packetSize)
    {
        return false;
    }
    //如果接收到udp,create a buffer and read the data in the len(udp 多少字节)
    char buffer[64];
    int len = udp.read(buffer, sizeof(buffer) - 1);
    // check是否成功读取
    if (len <= 0)
    {
        return false;
    }

    //'\0'文字到这里结束，记录发送 UDP 数据的电脑地址
    buffer[len] = '\0';
    pcIP = udp.remoteIP();
    pcPort = udp.remotePort();
    pcKnown = true;
    
    // 将数据转换为 String
    out = String(buffer);
    out.trim();

    //read 是python 请求信息，不打印，只打印cmd,READ arrives 10x a second, so don't print it
    if (out != MSG_READ)
    {
        Serial.print("UDP R: [");
        Serial.print(out);
        Serial.println("]");
    }

    return true;
}

// 把状态/传感器回复发回最近一次发来 UDP 数据的电脑。
void networkSend(const char* message)
{
    if (!pcKnown)
    {
        return;
    }

    udp.beginPacket(pcIP, pcPort);
    udp.print(message);
    udp.endPacket();
}

// 4. 距离传感器与滤波

// 读取一个超声波传感器；返回毫米，-1 表示读数无效。
int readDistanceMm(int pin)
{
    unsigned long t = pulseIn(pin, HIGH, SENSOR_TIMEOUT_US);

    if (t == 0 || t > SENSOR_PULSE_MAX_US)
    {
        return -1;
    }

    long d = ((long)t - SENSOR_PULSE_OFFSET_US) * SENSOR_MM_PER_US;

    if (d < SENSOR_MIN_MM)
    {
        return -1; //too short to be real: glitch or dead zone
    }

    return (int)d;
}

Filtered fR1, fR2, fB, fB2;
int rawBack = -1;                 // latest unfiltered B reading
int rawBack2 = -1;                // latest unfiltered B2 reading
int rawRight1 = -1;               // latest unfiltered R1 reading
int rawRight2 = -1;               // latest unfiltered R2 reading
unsigned long sensorSeq = 0;      //increases with every new set of readings
unsigned long lastSensorRead = 0;

// 更新单个传感器的平滑值；连续无效读数过多时将其标为无效。
void filterUpdate(Filtered &f, int raw, float alpha)
{
    if (raw < 0)
    {
        if (++f.misses > SENSOR_MAX_MISSES)
        {
            f.valid = false;
        }
        return;
    }

    f.misses = 0;

    if (!f.valid)
    {
        f.value = raw;
    }
    else
    {
        f.value += alpha * (raw - f.value);
    }

    f.valid = true;
}

// 取得滤波后的毫米值；没有有效值时返回 -1。
int filteredMm(const Filtered &f)
{
    return f.valid ? (int)(f.value + 0.5f) : -1;
}

// 定时读取四个传感器并更新滤波缓存；每次更新都会增加 sensorSeq。
void sensorsUpdate()
{
    if (millis() - lastSensorRead < SENSOR_PERIOD_MS)
    {
        return;
    }

    lastSensorRead = millis();

    int r1 = readDistanceMm(RIGHT_SENSOR1_PIN);
    int r2 = readDistanceMm(RIGHT_SENSOR2_PIN);
    int b  = readDistanceMm(BACK_SENSOR_PIN);
    int b2  = readDistanceMm(BACK_SENSOR2_PIN);

    rawBack = b;
    rawBack2 = b2;
    rawRight1 = r1;
    rawRight2 = r2;

    filterUpdate(fR1, r1, FILTER_ALPHA_RIGHT);
    filterUpdate(fR2, r2, FILTER_ALPHA_RIGHT);
    filterUpdate(fB,  b,  FILTER_ALPHA_BACK);
    filterUpdate(fB2, b2, FILTER_ALPHA_BACK);
    sensorSeq++;
}

// 5. 停车和驶出车位状态机
// 这些状态按照代码中的定义，默认从 0 开始自动递增
enum ParkStep : uint8_t
{
    P_IDLE,                // 0	停车流程未运行
    P_SETTLE,              // 1	暂停车辆，等待 PARK_SETTLE_MS（当前为 15 ms）
    P_ARC,                 // 2	打方向倒车，利用 B、B2 传感器判断车身是否平行
    P_STRAIGHT_REVERSE,    // 3	舵机转到直行角度，继续倒车
    P_STRAIGHTEN,          // 4	停止电机，将舵机回正
    P_DONE                 // 5	停车完成，重复发送 PARK_DONE 通知
};

enum UnparkStep : uint8_t
{
    U_IDLE,
    U_BACK_WALL_CHECK,
    U_BACK_WALL_ADJUST_MOVE,
    U_BACK_WALL_ADJUST_SETTLE,
    U_INITIAL_FORWARD,
    U_TURN_FORWARD_FIXED,
    U_REVERSE_ALIGN,
    U_POST_REVERSE_SETTLE,
    U_FINAL_FORWARD,
    U_DONE,
    U_FAULT
};

ParkStep parkStep = P_IDLE;
UnparkStep unparkStep = U_IDLE;
unsigned long stepStart = 0;
unsigned long lastDoneSend = 0;
unsigned long parkArcMs = 0;                    // P_ARC 弧线倒车的实际持续时间
unsigned long parkStraightReverseStartMs = 0;    // P_STRAIGHT_REVERSE 开始时间
unsigned long parkStraightReverseMs = 0;         // P_STRAIGHT_REVERSE 的实际持续时间
int backHits = 0;
unsigned long lastParkSeq = 0;

// State for combined UNPARK: back-wall correction, fixed timing, then R1/R2 feedback.
unsigned long lastUnparkSeq = 0;
int unparkBackAlignHits = 0;
int unparkBackMoveDirection = UNPARK_BACK_FIRST_MOVE_DIRECTION;
float unparkBackErrorAbsBeforePulse = -1.0f;
unsigned long unparkBackAdjustStartMs = 0;
int unparkAlignInvalidSamples = 0;
unsigned long unparkAlignLastUpdateMs = 0;
unsigned long unparkAlignLastDebugMs = 0;
float unparkAlignAppliedSteeringDeg = 0.0f;

// PD + learned trim controller state; trim is retained between DRIVE phases.
float alignTrimDeg = 0.0f;
float alignAppliedSteeringDeg = 0.0f;
unsigned long alignLastUpdateMs = 0;
unsigned long alignLastDebugMs = 0;

// 切换停车子状态，并记录进入该状态的时间。
void gotoPark(int step)
{
    parkStep = (ParkStep)step;
    stepStart = millis();
}

// 切换驶出车位子状态，并记录进入该状态的时间。
void gotoUnpark(int step)
{
    unparkStep = (UnparkStep)step;
    stepStart = millis();
}

void gotoUnparkPhase(int step, const char* label)
{
    gotoUnpark(step);
    Serial.print("UNPARK PHASE: ");
    Serial.println(label);
    String message = String("UNPARK_PHASE:") + label;
    networkSend(message.c_str());
}

// 取消正在执行的停车/驶出流程，将两个子状态机都重置为空闲。
void manoeuvresReset()
{
    parkStep = P_IDLE;
    unparkStep = U_IDLE;
}

// 返回停车流程是否正在执行（不包括空闲和完成状态）。
bool parkBusy()
{
    return parkStep != P_IDLE && parkStep != P_DONE;
}

// 返回停车流程是否已经完成。
bool parkDone()
{
    return parkStep == P_DONE;
}

// 返回驶出车位流程是否正在执行。
bool unparkBusy()
{
    return unparkStep != U_IDLE && unparkStep != U_DONE;
}

// 初始化停车流程：先刹车并打好转向角，再进入短暂停车阶段。
void parkStart()
{
    unparkStep = U_IDLE;
    parkArcMs = 0;
    parkStraightReverseStartMs = 0;
    parkStraightReverseMs = 0;
    backHits = 0;
    backAlignHits = 0;
    parkArcStartMs = 0;

    motorStop();
    steerAngle(PARK_STEER_ANGLE);
    gotoPark(P_SETTLE);
    Serial.println("PARK: settling");
}

// 停车状态机：每次 loop 调用一次，不在这里使用长时间 delay。
void parkRun()
{
    const unsigned long stepElapsed = millis() - stepStart;

    switch (parkStep)
    {
        case P_SETTLE:
            motorStop();
            steerAngle(PARK_STEER_ANGLE);

            if (stepElapsed >= PARK_SETTLE_MS)
            {
                // 每次停车只从新的传感器更新开始计数，避免重复使用旧读数。
                lastParkSeq = sensorSeq;
                backHits = 0;
                backAlignHits = 0;
                parkArcStartMs = millis(); // 倒车总计时从这里开始。

                Serial.println("PARK: reversing");
                gotoPark(P_ARC);
            }
            break;

        case P_ARC:
        {
            motorReverse(MANOEUVRE_SPEED);
            steerAngle(PARK_STEER_ANGLE);

            const unsigned long reverseElapsed = millis() - parkArcStartMs;

            // 每组传感器读数只判断一次。
            if (sensorSeq != lastParkSeq)
            {
                lastParkSeq = sensorSeq;
                const int back1 = filteredMm(fB);
                const int back2 = filteredMm(fB2);

                if (back1 >= 0 && back2 >= 0)
                {
                    // 车身与墙平行时，两只后传感器的实测差值基准为 50 mm。
                    const int correctedDiff = (back1 - back2) - BACK_SENSOR_OFFSET_MM;

                    if (abs(correctedDiff) <= BACK_ALIGN_TOLERANCE_MM)
                        backAlignHits++;
                    else
                        backAlignHits = 0;
                }
                else
                {
                    backAlignHits = 0; // 任一读数无效，就重新累计连续确认次数。
                }
            }

            if (backAlignHits >= BACK_ALIGN_CONFIRMATIONS &&
                reverseElapsed >= parkAlignMinTimeMs &&
                reverseElapsed < PARK_ARC_MAX_MS)
            {
                // Retain this parking measurement for diagnostics; UNPARK now uses the standalone timed sequence.
                parkArcMs = reverseElapsed;
                parkStraightReverseStartMs = millis();
                parkStraightReverseMs = 0;
                Serial.printf("PARK: aligned after %lu ms; straight reverse\n", parkArcMs);
                backHits = 0;
                gotoPark(P_STRAIGHT_REVERSE);
                break;
            }

            if (reverseElapsed >= PARK_ARC_MAX_MS)
            {
                parkArcMs = reverseElapsed;
                parkStraightReverseMs = 0;
                motorStop();
                Serial.println("PARK: reverse timeout during arc; no straight-reverse segment recorded");
                gotoPark(P_STRAIGHTEN);
            }
            break;
        }

        case P_STRAIGHT_REVERSE:
        {
            // 保持倒车，只把舵机转到直行角度；这里不会先停电机。
            motorReverse(MANOEUVRE_SPEED);
            steerAngle(BACK_STRAIGHT_ANGLE);

            const unsigned long reverseElapsed = millis() - parkArcStartMs;

            if (sensorSeq != lastParkSeq)
            {
                lastParkSeq = sensorSeq;
                const int back = rawBack;

                // 至少经过 PARK_ARC_MIN_MS 后才允许靠后墙距离停止。
                if (reverseElapsed >= PARK_ARC_MIN_MS &&
                    back >= 0 && back <= PARK_WALL_TARGET_MM)
                {
                    backHits++;
                }
                else
                {
                    backHits = 0;
                }
            }

            const bool wallReached = backHits >= PARK_SENSOR_CONFIRMS;
            const bool timedOut = reverseElapsed >= PARK_ARC_MAX_MS;

            if (wallReached || timedOut)
            {
                parkStraightReverseMs = millis() - parkStraightReverseStartMs;
                motorStop();
                Serial.printf("PARK: straight-reverse duration %lu ms\n", parkStraightReverseMs);
                Serial.println(wallReached
                    ? "PARK: wall reached"
                    : "PARK: reverse timeout during straight reverse");
                gotoPark(P_STRAIGHTEN);
            }
            break;
        }

        case P_STRAIGHTEN:
            motorStop();
            steerCentre();

            if (stepElapsed >= PARK_STRAIGHTEN_MS)
            {
                Serial.println("PARK: done");
                lastDoneSend = 0;
                gotoPark(P_DONE);
            }
            break;

        case P_DONE:
            motorStop();

            // 完成通知会重复发送，避免电脑偶尔漏收 UDP 数据包。
            if (millis() - lastDoneSend >= DONE_RESEND_MS)
            {
                networkSend(MSG_PARK_DONE);
                lastDoneSend = millis();
            }
            break;

        default:
            break;
    }
}

// Calculate absolute B/B2 parallelism error using the calibrated baseline.
float currentUnparkBackErrorAbs()
{
    if (rawBack < 0 || rawBack2 < 0 || !fB.valid || !fB2.valid) return -1.0f;
    return fabsf((fB.value - fB2.value) - BACK_SENSOR_OFFSET_MM);
}

void failUnpark(const char* reason)
{
    motorStop();
    steerAngle((int)unparkCentreAngleDeg);
    unparkStep = U_FAULT;
    stepStart = millis();
    Serial.print("UNPARK FAULT: ");
    Serial.println(reason);
    String message = String("UNPARK_FAULT:") + reason;
    networkSend(message.c_str());
    networkSend("UNPARK_PHASE:FAULT");
}

// Initial check/correction is intentionally retained from the standalone test.
void beginUnparkBackWallPulse()
{
    unparkBackAlignHits = 0;
    unparkBackErrorAbsBeforePulse = currentUnparkBackErrorAbs();
    lastUnparkSeq = sensorSeq;
    gotoUnparkPhase(U_BACK_WALL_ADJUST_MOVE,
        unparkBackMoveDirection >= 0 ? "BACK_WALL_ADJUST_FORWARD_PULSE" : "BACK_WALL_ADJUST_REVERSE_PULSE");
}

// Start the standalone-tested movement sequence after B/B2 is already parallel.
void beginUnparkTimedSequence(bool adjusted)
{
    motorStop();
    steerAngle((int)unparkCentreAngleDeg);
    unparkAlignInvalidSamples = 0;
    unparkAlignLastUpdateMs = 0;
    unparkAlignLastDebugMs = 0;
    unparkAlignAppliedSteeringDeg = 0.0f;
    lastUnparkSeq = sensorSeq;
    Serial.println(adjusted
        ? "UNPARK: back-wall parallel after adjustment"
        : "UNPARK: back-wall already parallel; skip adjustment pulses");
    gotoUnparkPhase(U_INITIAL_FORWARD, "INITIAL_FORWARD");
}

// R1/R2 steering correction imported from the standalone test. Uses alpha=0.8
// filters shared with DRIVE, ALIGN_DIRECTION=-1, and a fail-stop on repeated invalid reads.
bool runUnparkSideWallAlignment()
{
    const unsigned long nowMs = millis();
    if (nowMs - unparkAlignLastUpdateMs < UNPARK_ALIGN_STEER_PERIOD_MS) return true;
    unparkAlignLastUpdateMs = nowMs;

    if (rawRight1 < 0 || rawRight2 < 0 || !fR1.valid || !fR2.valid)
    {
        unparkAlignInvalidSamples++;
        if (unparkAlignInvalidSamples >= 5)
        {
            failUnpark(unparkStep == U_REVERSE_ALIGN
                ? "R1_R2_INVALID_DURING_REVERSE_ALIGN"
                : "R1_R2_INVALID_DURING_FINAL_FORWARD_ALIGN");
            return false;
        }
        return true; // retain the last steering angle briefly while a transient recovers
    }
    unparkAlignInvalidSamples = 0;

    const float rawDiff = fR1.value - fR2.value;
    if (fabsf(rawDiff) > UNPARK_ALIGN_MAX_DIFF_MM)
    {
        steerAngle((int)unparkCentreAngleDeg);
        return true; // a wall gap / echo glitch must not drive a large correction
    }

    float headingError = rawDiff - UNPARK_ALIGN_PARALLEL_DIFF_MM;
    if (fabsf(headingError) <= UNPARK_ALIGN_HEADING_TOLERANCE_MM) headingError = 0.0f;
    const float averageDistance = (fR1.value + fR2.value) / 2.0f;
    float distanceError = averageDistance - UNPARK_ALIGN_TARGET_DISTANCE_MM;
    if (fabsf(distanceError) <= UNPARK_ALIGN_DISTANCE_TOLERANCE_MM) distanceError = 0.0f;

    float offset = headingError * UNPARK_ALIGN_GAIN_DEG_PER_MM
                 + distanceError * UNPARK_ALIGN_DISTANCE_GAIN_DEG_PER_MM;
    offset = constrain(offset, (float)-UNPARK_ALIGN_MAX_DEG, (float)UNPARK_ALIGN_MAX_DEG);
    const int targetAngle = constrain(
        (int)unparkCentreAngleDeg + (int)lroundf(UNPARK_ALIGN_DIRECTION * offset),
        (int)unparkCentreAngleDeg - UNPARK_ALIGN_MAX_DEG,
        (int)unparkCentreAngleDeg + UNPARK_ALIGN_MAX_DEG);
    steerAngle(targetAngle);

    if (nowMs - unparkAlignLastDebugMs >= UNPARK_ALIGN_DEBUG_INTERVAL_MS)
    {
        unparkAlignLastDebugMs = nowMs;
        char line[220];
        snprintf(line, sizeof(line),
            "ALIGN:%s,R1=%.1f,R2=%.1f,diff=%.1f,headingErr=%.1f,avg=%.1f,distanceErr=%.1f,servo=%d,direction=%d",
            unparkStep == U_REVERSE_ALIGN ? "REVERSE" : "FINAL_FORWARD",
            fR1.value, fR2.value, rawDiff, headingError, averageDistance,
            distanceError, targetAngle, UNPARK_ALIGN_DIRECTION);
        Serial.println(line);
        networkSend(line);
    }
    return true;
}

// Initialise the combined UNPARK path. No assumption about the previous PARK
// segment duration is used: the timings are supplied by the laptop configuration.
void unparkStart()
{
    parkStep = P_IDLE;
    motorStop();
    steerAngle((int)unparkCentreAngleDeg);
    unparkBackAlignHits = 0;
    unparkBackMoveDirection = UNPARK_BACK_FIRST_MOVE_DIRECTION;
    unparkBackErrorAbsBeforePulse = -1.0f;
    unparkBackAdjustStartMs = millis();
    unparkAlignInvalidSamples = 0;
    unparkAlignLastUpdateMs = 0;
    unparkAlignLastDebugMs = 0;
    unparkAlignAppliedSteeringDeg = 0.0f;
    lastUnparkSeq = sensorSeq;
    gotoUnparkPhase(U_BACK_WALL_CHECK, "BACK_WALL_INITIAL_CHECK");
}

// Combined unpark: B/B2 check -> 1.3 s straight -> 9 s fixed-angle forward ->
// 8 s reverse with R1/R2 correction -> settle -> 5 s forward with R1/R2 correction.
void unparkRun()
{
    const unsigned long elapsed = millis() - stepStart;

    switch (unparkStep)
    {
        case U_BACK_WALL_CHECK:
        {
            motorStop();
            steerAngle((int)unparkCentreAngleDeg);
            if (sensorSeq != lastUnparkSeq)
            {
                lastUnparkSeq = sensorSeq;
                const float errorAbs = currentUnparkBackErrorAbs();
                if (errorAbs >= 0.0f && errorAbs <= BACK_ALIGN_TOLERANCE_MM)
                    unparkBackAlignHits++;
                else
                    unparkBackAlignHits = 0;
            }
            if (unparkBackAlignHits >= BACK_ALIGN_CONFIRMATIONS)
            {
                beginUnparkTimedSequence(false);
            }
            else if (elapsed >= UNPARK_BACK_CHECK_MS)
            {
                if (rawBack < 0 || rawBack2 < 0 || !fB.valid || !fB2.valid)
                {
                    failUnpark("BACK_WALL_SENSORS_INVALID_AT_START");
                }
                else
                {
                    unparkBackAdjustStartMs = millis();
                    unparkBackMoveDirection = UNPARK_BACK_FIRST_MOVE_DIRECTION;
                    Serial.printf("UNPARK: back wall needs adjustment; diff=%.1f target=%d error=%.1f\n",
                        fB.value - fB2.value, BACK_SENSOR_OFFSET_MM, currentUnparkBackErrorAbs());
                    beginUnparkBackWallPulse();
                }
            }
            break;
        }

        case U_BACK_WALL_ADJUST_MOVE:
            steerAngle(UNPARK_BACK_TURN_ANGLE_DEG);
            if (unparkBackMoveDirection >= 0) motorForward(MANOEUVRE_SPEED);
            else motorReverse(MANOEUVRE_SPEED);
            if (millis() - unparkBackAdjustStartMs >= UNPARK_BACK_ADJUST_TIMEOUT_MS)
            {
                failUnpark("BACK_WALL_ADJUST_TIMEOUT");
            }
            else if (elapsed >= UNPARK_BACK_PULSE_MS)
            {
                motorStop();
                steerAngle((int)unparkCentreAngleDeg);
                unparkBackAlignHits = 0;
                lastUnparkSeq = sensorSeq;
                gotoUnparkPhase(U_BACK_WALL_ADJUST_SETTLE, "BACK_WALL_ADJUST_SETTLE_AND_MEASURE");
            }
            break;

        case U_BACK_WALL_ADJUST_SETTLE:
        {
            motorStop();
            steerAngle((int)unparkCentreAngleDeg);
            if (millis() - unparkBackAdjustStartMs >= UNPARK_BACK_ADJUST_TIMEOUT_MS)
            {
                failUnpark("BACK_WALL_ADJUST_TIMEOUT");
                break;
            }
            if (sensorSeq != lastUnparkSeq)
            {
                lastUnparkSeq = sensorSeq;
                const float errorAbs = currentUnparkBackErrorAbs();
                if (errorAbs >= 0.0f && errorAbs <= BACK_ALIGN_TOLERANCE_MM)
                    unparkBackAlignHits++;
                else
                    unparkBackAlignHits = 0;
            }
            if (unparkBackAlignHits >= BACK_ALIGN_CONFIRMATIONS)
            {
                beginUnparkTimedSequence(true);
            }
            else if (elapsed >= UNPARK_BACK_SETTLE_MS && rawBack >= 0 && rawBack2 >= 0)
            {
                const float nowErrorAbs = currentUnparkBackErrorAbs();
                if (unparkBackErrorAbsBeforePulse >= 0.0f &&
                    nowErrorAbs >= unparkBackErrorAbsBeforePulse - UNPARK_BACK_MIN_IMPROVEMENT_MM)
                    unparkBackMoveDirection = -unparkBackMoveDirection;
                beginUnparkBackWallPulse();
            }
            break;
        }

        case U_INITIAL_FORWARD:
            steerAngle((int)unparkCentreAngleDeg);
            motorForward(MANOEUVRE_SPEED);
            if (elapsed >= unparkInitialForwardMs)
                gotoUnparkPhase(U_TURN_FORWARD_FIXED, "TURN_FORWARD_FIXED");
            break;

        case U_TURN_FORWARD_FIXED:
            steerAngle((int)unparkTurnAngleDeg);
            motorForward(MANOEUVRE_SPEED);
            if (elapsed >= unparkTurnForwardMs)
            {
                unparkAlignInvalidSamples = 0;
                unparkAlignLastUpdateMs = 0;
                unparkAlignLastDebugMs = 0;
                gotoUnparkPhase(U_REVERSE_ALIGN, "REVERSE_ALIGN");
            }
            break;

        case U_REVERSE_ALIGN:
            motorReverse(MANOEUVRE_SPEED);
            if (!runUnparkSideWallAlignment()) break;
            if (unparkStep == U_REVERSE_ALIGN && elapsed >= unparkReverseAlignMs)
            {
                motorStop();
                steerAngle((int)unparkCentreAngleDeg);
                gotoUnparkPhase(U_POST_REVERSE_SETTLE, "POST_REVERSE_SETTLE");
            }
            else if (unparkStep == U_REVERSE_ALIGN && elapsed >= unparkAlignTimeoutMs)
            {
                failUnpark("REVERSE_ALIGNMENT_TIMEOUT");
            }
            break;

        case U_POST_REVERSE_SETTLE:
            motorStop();
            steerAngle((int)unparkCentreAngleDeg);
            if (elapsed >= unparkServoSettleMs)
            {
                unparkAlignInvalidSamples = 0;
                unparkAlignLastUpdateMs = 0;
                unparkAlignLastDebugMs = 0;
                gotoUnparkPhase(U_FINAL_FORWARD, "FINAL_FORWARD");
            }
            break;

        case U_FINAL_FORWARD:
            motorForward(MANOEUVRE_SPEED);
            if (!runUnparkSideWallAlignment()) break;
            if (unparkStep == U_FINAL_FORWARD && elapsed >= unparkFinalForwardMs)
            {
                motorStop();
                steerAngle((int)unparkCentreAngleDeg);
                lastDoneSend = 0;
                gotoUnparkPhase(U_DONE, "COMPLETE");
            }
            else if (unparkStep == U_FINAL_FORWARD && elapsed >= unparkAlignTimeoutMs)
            {
                failUnpark("FINAL_FORWARD_ALIGNMENT_TIMEOUT");
            }
            break;

        case U_DONE:
            motorStop();
            if (millis() - lastDoneSend >= DONE_RESEND_MS)
            {
                networkSend(MSG_UNPARK_DONE);
                lastDoneSend = millis();
            }
            break;

        case U_FAULT:
            motorStop();
            steerAngle((int)unparkCentreAngleDeg);
            break;

        default:
            motorStop();
            steerCentre();
            break;
    }
}

// 6. 主状态与电脑指令处理

CarState currentState = STATE_STOP;
unsigned long lastCommandTime = 0;

// 将状态编号转换为便于查看的文字。
const char* stateName(CarState state)
{
    switch (state)
    {
        case STATE_DRIVE: return "DRIVE";
        case STATE_ALIGN: return "ALIGN";
        case STATE_PARK: return "PARK";
        case STATE_UNPARK: return "UNPARK";
        default: return "STOP";
    }
}

// 切换汽车主状态，并在进入 PARK/UNPARK/STOP 时执行对应初始化。
void setState(CarState newState)
{
    if (newState == currentState)
    {
        return;
    }

    Serial.print("STATE: ");
    Serial.print(stateName(currentState));
    Serial.print(" -> ");
    Serial.println(stateName(newState));

    currentState = newState;

    switch (newState)
    {
        case STATE_PARK:
            parkStart();
            break;

        case STATE_UNPARK:
            unparkStart();
            break;

        case STATE_STOP:
            manoeuvresReset();
            motorStop();
            steerCentre();
            break;

        default:
            manoeuvresReset();
            break;
    }
}

// 判断当前流程是否允许切换到请求的新状态，避免中途打断停车。
bool acceptCommand(CarState requested)
{
    if (requested == STATE_STOP)
    {
        return true;
    }

    if (currentState == STATE_PARK && parkBusy())
    {
        return false;
    }

    if (currentState == STATE_UNPARK && unparkBusy())
    {
        return false;
    }

    if (requested == STATE_PARK)
    {
        return currentState == STATE_DRIVE || currentState == STATE_ALIGN;
    }

    if (requested == STATE_UNPARK)
    {
        return currentState == STATE_PARK && parkDone()
            && parkArcMs > 0 && parkStraightReverseMs > 0;
    }

    return true;
}

// 把最新滤波传感器读数和当前状态打包后发给电脑。
void sendSensorReply()
{
    char message[64];

    snprintf(message, sizeof(message), "R1=%d,R2=%d,B=%d,B2=%d,S=%d",
             filteredMm(fR1), filteredMm(fR2), filteredMm(fB), filteredMm(fB2), (int)currentState);

    networkSend(message);
}

// 处理 laptop 发来的浮点数控制参数。
// 返回 true 表示命令匹配并已处理（成功或拒绝）；false 表示不是此参数命令。
bool handleFloatTuningConfig(const String &command, const char* settingName,
                             float &setting, float minValue, float maxValue)
{
    String prefix = "SET_";
    prefix += settingName;
    prefix += "=";
    if (!command.startsWith(prefix))
    {
        return false;
    }

    String errorPrefix = "ERR_";
    errorPrefix += settingName;
    errorPrefix += "=";

    if (currentState != STATE_STOP)
    {
        String errorMessage = errorPrefix + "CAR_NOT_STOPPED";
        networkSend(errorMessage.c_str());
        Serial.println("CONFIG rejected: car must be STOPPED");
        return true;
    }

    const String valueText = command.substring(prefix.length());
    bool validNumber = valueText.length() > 0;
    bool hasDigit = false;
    bool hasDecimalPoint = false;
    for (unsigned int i = 0; i < valueText.length(); ++i)
    {
        const char ch = valueText[i];
        if (isDigit(ch))
        {
            hasDigit = true;
        }
        else if (ch == '.' && !hasDecimalPoint)
        {
            hasDecimalPoint = true;
        }
        else
        {
            validNumber = false;
            break;
        }
    }

    const float requestedValue = valueText.toFloat();
    if (!validNumber || !hasDigit ||
        requestedValue < minValue || requestedValue > maxValue)
    {
        String errorMessage = errorPrefix + "OUT_OF_RANGE";
        networkSend(errorMessage.c_str());
        Serial.printf("CONFIG rejected: %s must be %.5f..%.5f\n",
                      settingName, minValue, maxValue);
        return true;
    }

    setting = requestedValue;

    char ack[96];
    snprintf(ack, sizeof(ack), "ACK_%s=%.5f", settingName, setting);
    networkSend(ack);
    Serial.printf("CONFIG accepted: %s=%.5f\n", settingName, setting);
    return true;
}

// Laptop-configurable integer UNPARK parameters. Accepted only when the car is STOPPED.
bool handleUnsignedUnparkConfig(const String &command, const char* settingName,
                                unsigned long &setting, unsigned long minValue,
                                unsigned long maxValue)
{
    String prefix = "SET_";
    prefix += settingName;
    prefix += "=";
    if (!command.startsWith(prefix)) return false;
    if (currentState != STATE_STOP)
    {
        String error = String("ERR_") + settingName + "=CAR_NOT_STOPPED";
        networkSend(error.c_str());
        return true;
    }
    const String valueText = command.substring(prefix.length());
    bool valid = valueText.length() > 0;
    for (unsigned int i = 0; i < valueText.length(); ++i)
        if (!isDigit(valueText[i])) { valid = false; break; }
    const unsigned long value = valid ? (unsigned long)valueText.toInt() : 0UL;
    if (!valid || value < minValue || value > maxValue)
    {
        String error = String("ERR_") + settingName + "=OUT_OF_RANGE";
        networkSend(error.c_str());
        return true;
    }
    setting = value;
    char ack[96];
    snprintf(ack, sizeof(ack), "ACK_%s=%lu", settingName, setting);
    networkSend(ack);
    Serial.printf("CONFIG accepted: %s=%lu\n", settingName, setting);
    return true;
}

// 解析电脑发来的 READ、控制增益或状态编号指令，并执行合法状态切换。
void handleCommand(const String &command)
{
    // Full timed UNPARK sequence values come from the laptop and must ACK before motion.
    if (handleUnsignedUnparkConfig(command, "UNPARK_INITIAL_FORWARD_MS", unparkInitialForwardMs, 0, 60000)) return;
    if (handleUnsignedUnparkConfig(command, "UNPARK_TURN_FORWARD_MS", unparkTurnForwardMs, 0, 60000)) return;
    if (handleUnsignedUnparkConfig(command, "UNPARK_TURN_ANGLE_DEG", unparkTurnAngleDeg, 0, 180)) return;
    if (handleUnsignedUnparkConfig(command, "UNPARK_REVERSE_ALIGN_MS", unparkReverseAlignMs, 0, 60000)) return;
    if (handleUnsignedUnparkConfig(command, "UNPARK_FINAL_FORWARD_MS", unparkFinalForwardMs, 0, 60000)) return;
    if (handleUnsignedUnparkConfig(command, "UNPARK_SERVO_SETTLE_MS", unparkServoSettleMs, 0, 10000)) return;
    if (handleUnsignedUnparkConfig(command, "UNPARK_ALIGN_TIMEOUT_MS", unparkAlignTimeoutMs, 1000, 60000)) return;
    if (handleUnsignedUnparkConfig(command, "UNPARK_CENTRE_ANGLE_DEG", unparkCentreAngleDeg, 0, 180)) return;

    // These gains control how strongly the car steers while aligning with the wall.
    // FILTER_ALPHA_RIGHT is deliberately fixed in firmware and is not a runtime tuning control.
    if (handleFloatTuningConfig(command, "ALIGN_GAIN_DEG_PER_MM",
                                ALIGN_GAIN_DEG_PER_MM, ALIGN_GAIN_MIN, ALIGN_GAIN_MAX)) return;
    if (handleFloatTuningConfig(command, "ALIGN_DISTANCE_GAIN_DEG_PER_MM",
                                ALIGN_DISTANCE_GAIN_DEG_PER_MM,
                                ALIGN_DISTANCE_GAIN_MIN, ALIGN_DISTANCE_GAIN_MAX)) return;
    if (handleFloatTuningConfig(command, "ALIGN_TRIM_KI_DEG_PER_MM_SEC",
                                ALIGN_TRIM_KI_DEG_PER_MM_SEC,
                                ALIGN_TRIM_KI_MIN, ALIGN_TRIM_KI_MAX)) return;
    // Laptop startup sends PARK alignment timing and normal DRIVE steering gains.
    // Accept these only while stopped; reply with ACKs.
    const String configPrefix = "SET_PARK_ALIGN_MIN_TIME_MS=";
    if (command.startsWith(configPrefix))
    {
        if (currentState != STATE_STOP)
        {
            networkSend("ERR_PARK_ALIGN_MIN_TIME_MS=CAR_NOT_STOPPED");
            Serial.println("CONFIG rejected: car must be STOPPED");
            return;
        }

        const String valueText = command.substring(configPrefix.length());
        bool validNumber = valueText.length() > 0;
        for (unsigned int i = 0; i < valueText.length(); ++i)
        {
            if (!isDigit(valueText[i]))
            {
                validNumber = false;
                break;
            }
        }

        if (!validNumber)
        {
            networkSend("ERR_PARK_ALIGN_MIN_TIME_MS=INVALID_NUMBER");
            Serial.println("CONFIG rejected: invalid PARK_ALIGN_MIN_TIME_MS number");
            return;
        }

        const unsigned long requestedMs = (unsigned long)valueText.toInt();
        if (requestedMs < PARK_ALIGN_MIN_TIME_MIN_MS ||
            requestedMs > PARK_ALIGN_MIN_TIME_MAX_MS)
        {
            networkSend("ERR_PARK_ALIGN_MIN_TIME_MS=OUT_OF_RANGE");
            Serial.printf("CONFIG rejected: value must be %lu..%lu ms\n",
                          PARK_ALIGN_MIN_TIME_MIN_MS, PARK_ALIGN_MIN_TIME_MAX_MS);
            return;
        }

        parkAlignMinTimeMs = requestedMs;
        char ack[64];
        snprintf(ack, sizeof(ack), "ACK_PARK_ALIGN_MIN_TIME_MS=%lu", parkAlignMinTimeMs);
        networkSend(ack);
        Serial.printf("CONFIG accepted: PARK_ALIGN_MIN_TIME_MS=%lu ms (%.2f s)\n",
                      parkAlignMinTimeMs, parkAlignMinTimeMs / 1000.0f);
        return;
    }

    // READ is a sensor query, not a motion-state command.
    if (command == MSG_READ)
    {
        sendSensorReply();
        return;
    }
    // 检查收到的内容是不是一个数字字符
    if (command.length() != 1 || !isDigit(command[0]))
    {
        Serial.print("Unknown command: ");
        Serial.println(command);
        return;
    }
    // 把字符串转换成整数
    int number = command.toInt();
    // 检查数字是否在合法范围内
    if (number < STATE_DRIVE || number > STATE_STOP)
    {
        Serial.print("Unknown state number: ");
        Serial.println(number);
        return;
    }
    // 记录命令时间，并将数字转换成 CarState
    lastCommandTime = millis();
    CarState requested = (CarState)number;
    // 如果请求的状态和当前状态相同，就不必重复切换
    if (requested == currentState)
    {
        return;
    }
    // 检查是否允许切换状态
    if (!acceptCommand(requested))
    {
        Serial.print("Ignored command: ");
        Serial.println(stateName(requested));
        return;
    }

    setState(requested);
}

// 前进控制：R1-R2 判断车身角度，平均值判断侧向距离。
// PD 比例项 + learned trim（距离误差积分），并对转向命令限速、限幅。
// 控制器约定 steeringCommandDeg > 0 为朝右侧墙转；本车舵机角度越小越向右。
void runAlignandMoveForward()
{
    motorForward(ALIGN_SPEED);

    const unsigned long nowMs = millis();
    if (nowMs - alignLastUpdateMs < ALIGN_STEER_PERIOD_MS)
    {
        return;
    }

    float dtSeconds = (alignLastUpdateMs == 0)
        ? (ALIGN_STEER_PERIOD_MS / 1000.0f)
        : ((nowMs - alignLastUpdateMs) / 1000.0f);
    alignLastUpdateMs = nowMs;

    // 避免暂停或线程延迟后一次性积分过大。
    dtSeconds = constrain(dtSeconds, 0.01f, 0.25f);

    float headingError = 0.0f;
    float distanceError = 0.0f;
    float averageDistance = 0.0f;
    float rawDiff = 0.0f;
    bool readingsUsable = fR1.valid && fR2.valid;
    float desiredSteeringCommandDeg = 0.0f;

    if (readingsUsable)
    {
        rawDiff = fR1.value - fR2.value;
        // 差值异常大时可能是墙面断点/回波异常：不利用本次读数更新 trim。
        if (fabsf(rawDiff) <= ALIGN_MAX_DIFF_MM)
        {
            headingError = rawDiff - ALIGN_PARALLEL_DIFF_MM;
            if (fabsf(headingError) <= ALIGN_DEADBAND_MM)
            {
                headingError = 0.0f;
            }

            averageDistance = (fR1.value + fR2.value) / 2.0f;
            distanceError = averageDistance - ALIGN_TARGET_DISTANCE_MM;
            if (fabsf(distanceError) <= ALIGN_DISTANCE_DEADBAND_MM)
            {
                distanceError = 0.0f;
            }

            // Learned trim：缓慢积分距离误差，补偿持续的电机/地面偏差。
            // 距离误差为正表示太远，需要朝墙转，因此 trim 正方向也表示朝墙转。
            alignTrimDeg += ALIGN_TRIM_KI_DEG_PER_MM_SEC * distanceError * dtSeconds;
            alignTrimDeg = constrain(
                alignTrimDeg,
                -ALIGN_TRIM_LIMIT_DEG,
                ALIGN_TRIM_LIMIT_DEG
            );

            // PD + learned trim：heading 项修正车身方向，distance 项修正平均侧向距离。
            desiredSteeringCommandDeg =
                ALIGN_GAIN_DEG_PER_MM * headingError
                + ALIGN_DISTANCE_GAIN_DEG_PER_MM * distanceError
                + alignTrimDeg;
        }
        else
        {
            // 读数不可信时，不用该读数推动控制器；逐渐将转向命令收回中心。
            desiredSteeringCommandDeg = 0.0f;
        }
    }
    else
    {
        // 任一侧面传感器无有效数据：不积分，逐渐回中心，避免突发大幅舵机变化。
        desiredSteeringCommandDeg = 0.0f;
    }

    desiredSteeringCommandDeg = constrain(
        desiredSteeringCommandDeg,
        (float)-ALIGN_MAX_DEG,
        (float)ALIGN_MAX_DEG
    );

    // 对转向控制量进行速率限制，避免舵机突然大角度跳变。
    const float maxCommandStep = ALIGN_MAX_STEER_RATE_DEG_PER_SEC * dtSeconds;
    const float commandDelta = desiredSteeringCommandDeg - alignAppliedSteeringDeg;
    alignAppliedSteeringDeg += constrain(commandDelta, -maxCommandStep, maxCommandStep);

    // 正 steering command 表示向右/朝墙；本车向右对应舵机角度减小。
    const int targetAngle = constrain(
        SERVO_CENTRE - (int)lroundf(ALIGN_DIRECTION * alignAppliedSteeringDeg),
        SERVO_CENTRE - ALIGN_MAX_DEG,
        SERVO_CENTRE + ALIGN_MAX_DEG
    );
    steerAngle(targetAngle);

    if (nowMs - alignLastDebugMs >= ALIGN_DEBUG_INTERVAL_MS)
    {
        Serial.printf(
            "ALIGN R1=%.1f R2=%.1f diff=%.1f headingErr=%.1f avg=%.1f distanceErr=%.1f trim=%.2f desired=%.2f applied=%.2f servo=%d dt=%.3f valid=%d\n",
            fR1.value, fR2.value, rawDiff, headingError,
            averageDistance, distanceError, alignTrimDeg,
            desiredSteeringCommandDeg, alignAppliedSteeringDeg,
            targetAngle, dtSeconds, readingsUsable ? 1 : 0
        );
        alignLastDebugMs = nowMs;
    }
}

// 7. 初始化与主循环

// Arduino 上电初始化：先确保电机停止，再初始化舵机、传感器和网络。
void setup()
{
    // 先配置并停止电机，避免上电初始化时电机误动作。
    pinMode(MOTOR_FWD_PIN, OUTPUT);
    pinMode(MOTOR_REV_PIN, OUTPUT);
    motorStop();

    Serial.begin(115200);

    steeringServo.attach(SERVO_PIN);
    steerCentre();

    pinMode(RIGHT_SENSOR1_PIN, INPUT);
    pinMode(RIGHT_SENSOR2_PIN, INPUT);
    pinMode(BACK_SENSOR_PIN, INPUT);
    pinMode(BACK_SENSOR2_PIN, INPUT);

    networkBegin();
    motorStop();
    setState(STATE_STOP);

    Serial.println("Ready");
}


// 主循环：收 UDP 指令、更新传感器、检查指令超时并运行当前状态。
void loop()
{
    String command;

    while (networkReceive(command))
    {
        handleCommand(command);
    }

    sensorsUpdate();

    if (currentState != STATE_STOP
        && millis() - lastCommandTime > COMMAND_TIMEOUT_MS)
    {
        Serial.println("PC communication timeout");
        setState(STATE_STOP);
    }

    switch (currentState)
    {
        
        // STATE_ALIGN 保留兼容旧命令；不再要求 Python 在 DRIVE/ALIGN 之间切换。
        // 共用同一个code
        case STATE_DRIVE:
        case STATE_ALIGN:
            runAlignandMoveForward();
            break;

        case STATE_PARK:
            parkRun();
            break;

        case STATE_UNPARK:
            unparkRun();
            if (unparkStep == U_FAULT) setState(STATE_STOP);
            break;

        default:
            motorStop();
            steerCentre();
            break;
    }
}