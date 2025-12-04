# RM2026 舵轮底盘双MCU系统完整指南

## 📋 目录
1. [系统概述](#系统概述)
2. [硬件配置](#硬件配置)
3. [板间通信协议](#板间通信协议)
4. [云台MCU实现](#云台mcu实现)
5. [底盘MCU实现](#底盘mcu实现)
6. [PID配置](#pid配置)
7. [集成步骤](#集成步骤)
8. [故障排查](#故障排查)
9. [性能优化](#性能优化)

---

## 系统概述

### 架构设计

```
┌─────────────────────┐
│   云台MCU (Master)   │
│                     │
│ ┌─────────────────┐ │
│ │  DR16接收器      │ │
│ └────────┬────────┘ │
│          │          │
│ ┌────────▼────────┐ │
│ │ RemoteHandler   │ │  读取RC数据
│ │ (50Hz 任务)     │ │  → 归一化
│ └────────┬────────┘ │  → 速度目标
│          │          │
│ ┌────────▼────────┐ │
│ │板间通信 (CAN1)   │ │  发送 VelocityCommand
│ │                 │ │
└─────────────────────┘
         │ CAN1 (1Mbps)
         │ ├─ VelocityCommand (0x120)
         │ ├─ MotorFeedback (0x121)
         │ └─ Heartbeat (0x122)
         │
┌────────▼────────────────────┐
│   底盘MCU (Servant)          │
│                             │
│ ┌─────────────────────────┐ │
│ │ CAN接收回调/超时保护     │ │
│ └────────┬────────────────┘ │
│          │                  │
│ ┌────────▼──────────────┐   │
│ │ ChassisController     │   │  接收速度目标
│ │ (100Hz 任务)          │   │  → 调用 ikine()
│ └────────┬──────────────┘   │  → 计算目标角度/转速
│          │                  │
│ ┌────────▼──────────────┐   │
│ │ 8× PID 控制环         │   │
│ │ (4个转向 + 4个速度)    │   │
│ └────────┬──────────────┘   │
│          │                  │
│ ┌────────▼──────────────┐   │
│ │ CAN马达驱动           │   │
│ │ CAN1 (方向) + CAN2 (速度) │
│ └───────────────────────┘   │
│                             │
│ ┌─────────────────────────┐ │
│ │ 状态反馈 (可选)         │ │
│ │ MotorFeedback → Cloud  │ │
│ └─────────────────────────┘ │
└─────────────────────────────┘
```

### 关键特性

| 特性 | 说明 |
|------|------|
| **实时性** | 云台MCU 50Hz, 底盘MCU 100Hz |
| **双重冗余** | CAN主通道 + RS485备通道 |
| **超时保护** | 20-50ms 无指令 → 停止马达 |
| **舵轮优化** | ±π 反射角度优化，减少转向时间 |
| **独立控制** | 8个PID独立循环，互不干扰 |

---

## 硬件配置

### CAN总线映射

#### 云台MCU (Master)

```
CAN1 (ID 0x001 - 0x00B):
  - Pitch 达妙 4310: ID 0x001
  - Shooter 3508: ID 0x002, 0x003
  - 预留: 0x004 - 0x00B

CAN2 (ID 0x001 - 0x00B):
  - Shooter 马达反馈接收
  - 功率管理

CAN3:
  - Yaw 达妙 4310/6020
```

#### 底盘MCU (Servant)

```
CAN1 (方向马达 - 3508):
  ID 1: 前左轮 (FL)   - 转向
  ID 2: 后左轮 (BL)   - 转向
  ID 3: 后右轮 (BR)   - 转向
  ID 4: 前右轮 (FR)   - 转向
  ID 5: M2006 弹丸盘   - 可选

CAN2 (速度马达 - 7015):
  ID 1: 前左轮 (FL)   - 驱动
  ID 2: 后左轮 (BL)   - 驱动
  ID 3: 后右轮 (BR)   - 驱动
  ID 4: 前右轮 (FR)   - 驱动
  ID 5: SuperCap 管理 - 功率控制
```

### 轮子配置

```cpp
// 轮子位置图
     0 (FL)       3 (FR)
      ◯─────────────◯
      │             │
      │             │  X轴(前进)
      │    (0,0)    │  ────→
      │    底盘中心   │
      │             │
      ◯─────────────◯
     1 (BL)       2 (BR)
```

#### 配置参数

```cpp
struct SwerveConfig {
    // 轮子到底盘中心的距离 [m]
    float chassis_radius_[4] = {0.3, 0.3, 0.3, 0.3};
    
    // 轮子的切向角 [rad]
    float tangent_angle_[4] = {
        M_PI / 4.0f,      // [0] FL:  45° (π/4)
        -M_PI / 4.0f,     // [1] BL: -45° (-π/4)
        3 * M_PI / 4.0f,  // [2] BR: 135° (3π/4)
        -3 * M_PI / 4.0f  // [3] FR:-135° (-3π/4)
    };
    
    // 速度到转速的转换系数
    // 计算: RPM_max / (π × wheel_diameter)
    // 例: 7015 @ 1000RPM, 直径 0.2m
    //     转换系数 ≈ 1000 / (π × 0.2) ≈ 1592
    float velocity_to_rpm_param_ = 1000.0f;
};
```

---

## 板间通信协议

### 消息结构体

#### 1. VelocityCommand (0x120) - 速度目标

```cpp
struct VelocityCommand {
    // 字节 0-3: vx (前后速度) [m/s]
    float vx;
    
    // 字节 4-7: vy (左右速度) [m/s]  
    float vy;
    
    // 字节 8-11: vw (角速度) [rad/s]
    float vw;
    
    // 字节 12: 控制位
    uint8_t enable;              // bit 0: 是否启用
    uint8_t mode;                // bit 1-2: 控制模式
    
    // 字节 13-15: 时间戳 (ms)
    uint16_t timestamp;
    
    // 总大小: 16 字节
} __attribute__((packed));

// 云台MCU 发送频率: 50Hz (20ms)
// 超时时间: 50ms (允许丢失一帧)
```

#### 2. MotorFeedback (0x121) - 马达反馈

```cpp
struct MotorFeedback {
    // 字节 0: 马达健康状态掩码
    uint8_t motor_health_mask;   // bit 0-3: 马达0-3连接状态
                                 // bit 4-7: 马达4-7连接状态
    
    // 字节 1-2: 平均反馈 (可选)
    uint16_t avg_current;
    
    // 字节 3: 系统状态
    uint8_t system_status;       // bit 0: 超电容充电中
                                 // bit 1: 功率限制中
                                 // bit 2-7: 预留
    
    // 字节 4-15: 预留
    uint8_t reserved[12];
    
    // 总大小: 16 字节
} __attribute__((packed));

// 底盘MCU 发送频率: 20Hz (50ms) - 可选
```

#### 3. Heartbeat (0x122) - 心跳包

```cpp
struct Heartbeat {
    // 字节 0: 源MCU ID
    uint8_t source_id;           // 0x01: Cloud, 0x02: Chassis
    
    // 字节 1-2: 发送计数器
    uint16_t tx_counter;
    
    // 字节 3-4: 接收计数器
    uint16_t rx_counter;
    
    // 字节 5-7: 系统运行时间 (ms)
    uint32_t uptime_ms;
    
    // 字节 8-11: 预留
    uint32_t reserved;
    
    // 总大小: 12 字节
} __attribute__((packed));

// 双向频率: 100ms (10Hz)
```

### 序列化/反序列化

```cpp
// 发送端 (云台MCU)
void sendVelocityCommand(float vx, float vy, float vw) {
    VelocityCommand cmd;
    cmd.vx = vx;
    cmd.vy = vy;
    cmd.vw = vw;
    cmd.enable = 1;
    cmd.mode = 0;
    cmd.timestamp = HAL_GetTick();
    
    // 通过 CAN 发送 (0x120, 16 字节)
    CANManager::getInstance().sendMessage(0x120, 
        (uint8_t*)&cmd, sizeof(cmd));
}

// 接收端 (底盘MCU)
void onVelocityCommandReceived(const uint8_t* data, int len) {
    if (len != sizeof(VelocityCommand)) {
        return;  // 数据长度错误
    }
    
    VelocityCommand cmd;
    memcpy(&cmd, data, sizeof(cmd));
    
    // 存储到共享缓冲区
    velocity_command_buffer_ = cmd;
    last_velocity_update_ms_ = HAL_GetTick();
    
    // 重置超时检测计数器
    timeout_counter_ = 0;
}
```

---

## 云台MCU实现

### RemoteHandler 任务设计

#### 功能

```
读取 DR16
    ↓
  【归一化】
  每个通道 (364-1684) → (-1.0, +1.0)
    ↓
  【映射】
  遥控数据 → 速度目标
    ↓
  【发送】
  VelocityCommand → 底盘MCU
    ↓
  【循环】
  50Hz 重复
```

#### 遥控器通道映射

```cpp
struct RemoteControlData {
    // DR16 模式
    // S1 (三档开关) + S2 (两档开关)
    
    // 状态定义
    enum SwitchPosition {
        UP = 1,     // 上
        MID = 3,    // 中
        DOWN = 2    // 下
    };
};

// 推荐映射方案
enum ControlMode {
    MODE_DISABLED,      // S1=DOWN: 禁用底盘
    MODE_NORMAL,        // S1=MID: 正常摇杆控制
    MODE_GYRO,          // S1=UP: 陀螺仪跟随
};

// 通道分配
ch0: 前后 (vx)
ch1: 左右 (vy)
ch2: 旋转 (vw)
ch3: 不用 (预留)
s1:  工作模式
s2:  功能切换
```

#### 代码实现

```cpp
class RemoteHandler {
private:
    static constexpr float RC_MIN = 364.0f;
    static constexpr float RC_MAX = 1684.0f;
    static constexpr float RC_CENTER = 1024.0f;
    static constexpr float RC_RANGE = 660.0f;
    
    // 速度限制
    static constexpr float VX_MAX = 2.0f;      // m/s
    static constexpr float VY_MAX = 2.0f;      // m/s
    static constexpr float VW_MAX = 3.14f;     // rad/s
    
public:
    RemoteHandler() = default;
    
    // 归一化 RC 值到 [-1, 1]
    float normalizeRCValue(uint16_t raw_value) {
        // 范围: 364 - 1684
        // 公式: (value - 1024) / 660
        float normalized = (raw_value - RC_CENTER) / RC_RANGE;
        
        // 限幅到 [-1, 1]
        if (normalized > 1.0f) normalized = 1.0f;
        if (normalized < -1.0f) normalized = -1.0f;
        
        return normalized;
    }
    
    // 读取遥控并转换为速度目标
    void update() {
        // 读取 DR16 实时数据
        auto rc_data = DR16::getInstance().getRCData();
        
        // 获取工作模式
        ControlMode mode = determineControlMode(rc_data.s1, rc_data.s2);
        
        if (mode == MODE_DISABLED) {
            // 禁用状态 - 清空速度目标
            velocity_target_.vx = 0.0f;
            velocity_target_.vy = 0.0f;
            velocity_target_.vw = 0.0f;
            velocity_target_.enable = 0;
            return;
        }
        
        // 归一化遥控通道
        float ch0_norm = normalizeRCValue(rc_data.ch0);  // 前后
        float ch1_norm = normalizeRCValue(rc_data.ch1);  // 左右
        float ch2_norm = normalizeRCValue(rc_data.ch2);  // 旋转
        
        // 死区处理 (±0.05)
        if (fabsf(ch0_norm) < 0.05f) ch0_norm = 0.0f;
        if (fabsf(ch1_norm) < 0.05f) ch1_norm = 0.0f;
        if (fabsf(ch2_norm) < 0.05f) ch2_norm = 0.0f;
        
        // 映射到速度目标
        velocity_target_.vx = ch0_norm * VX_MAX;
        velocity_target_.vy = ch1_norm * VY_MAX;
        velocity_target_.vw = ch2_norm * VW_MAX;
        velocity_target_.enable = 1;
        velocity_target_.mode = (uint8_t)mode;
        velocity_target_.timestamp = HAL_GetTick();
    }
    
    // 发送速度命令到底盘
    void transmit() {
        // 通过 CAN 或 RS485 发送
        InterboardCommManager::getInstance().sendVelocityCommand(
            velocity_target_.vx,
            velocity_target_.vy,
            velocity_target_.vw,
            velocity_target_.enable
        );
    }
    
    // 50Hz 周期任务
    static void taskEntry(void* arg) {
        RemoteHandler* handler = (RemoteHandler*)arg;
        
        while (1) {
            handler->update();
            handler->transmit();
            
            // 20ms 延迟 (50Hz)
            osDelay(20);
        }
    }
    
private:
    ControlMode determineControlMode(uint8_t s1, uint8_t s2) {
        if (s1 == SwitchPosition::DOWN) {
            return MODE_DISABLED;
        } else if (s1 == SwitchPosition::MID) {
            return MODE_NORMAL;
        } else {  // s1 == SwitchPosition::UP
            return MODE_GYRO;
        }
    }
    
    VelocityCommand velocity_target_{};
};
```

### 云台MCU 主程序框架

```cpp
// main.cpp - 云台MCU
#include "DR16.hpp"
#include "RemoteHandler.hpp"
#include "InterboardCommManager.hpp"

static RemoteHandler* g_remote_handler = nullptr;

void initCloudMCU() {
    // 初始化 CAN 总线
    FDCANManager::getInstance().init();
    
    // 初始化 DR16 接收器
    DR16::getInstance().init();
    
    // 初始化板间通信
    InterboardCommManager::getInstance().init();
    
    // 创建 RemoteHandler 实例
    g_remote_handler = new RemoteHandler();
}

void startCloudMCUTasks() {
    // 启动遥控处理任务 (50Hz)
    osThreadNew(RemoteHandler::taskEntry, 
                g_remote_handler, 
                nullptr);
    
    // 启动板间通信心跳任务 (10Hz)
    osThreadNew(InterboardCommManager::heartbeatTask,
                nullptr,
                nullptr);
}

int main() {
    // 系统初始化
    HAL_Init();
    SystemClock_Config();
    
    // MCU 初始化
    initCloudMCU();
    
    // 创建 RTOS 任务
    osKernelInitialize();
    startCloudMCUTasks();
    osKernelStart();
    
    return 0;
}
```

---

## 底盘MCU实现

### CAN 接收回调 + 超时保护

```cpp
class VelocityCommandReceiver {
private:
    static constexpr uint32_t TIMEOUT_MS = 50;  // 50ms 超时
    static constexpr uint32_t WATCHDOG_THRESHOLD = 3;  // 允许丢3帧
    
public:
    VelocityCommandReceiver() 
        : last_update_ms_(0), timeout_counter_(0) {}
    
    // CAN 接收回调 (中断上下文)
    static void onVelocityCommandReceived(uint32_t msg_id, 
                                         const uint8_t* data, 
                                         uint32_t len) {
        getInstance().handleVelocityCommand(data, len);
    }
    
    // 处理接收的速度命令
    void handleVelocityCommand(const uint8_t* data, uint32_t len) {
        if (len != sizeof(VelocityCommand)) {
            return;  // 数据长度错误，丢弃
        }
        
        // 反序列化
        VelocityCommand cmd;
        memcpy(&cmd, data, sizeof(cmd));
        
        // 检查时间戳 (可选)
        if (isValidTimestamp(cmd.timestamp)) {
            // 更新共享缓冲区
            osMessageQueuePut(velocity_command_queue_, &cmd, 0, 0);
            
            // 重置超时计数器
            timeout_counter_ = 0;
            last_update_ms_ = HAL_GetTick();
        }
    }
    
    // 获取最新速度命令
    bool getLatestCommand(VelocityCommand& cmd) {
        uint32_t now = HAL_GetTick();
        
        // 检查超时
        if ((now - last_update_ms_) > TIMEOUT_MS) {
            timeout_counter_++;
            
            if (timeout_counter_ > WATCHDOG_THRESHOLD) {
                // 超时 - 清空速度命令
                cmd.vx = 0.0f;
                cmd.vy = 0.0f;
                cmd.vw = 0.0f;
                cmd.enable = 0;
                return false;
            }
        }
        
        // 从队列获取
        if (osMessageQueueGet(velocity_command_queue_, &cmd, 
                             nullptr, 0) == osOK) {
            return true;
        }
        
        return false;
    }
    
    static VelocityCommandReceiver& getInstance() {
        static VelocityCommandReceiver instance;
        return instance;
    }
    
private:
    bool isValidTimestamp(uint16_t timestamp) {
        // 简单检查: 时间戳不能太大
        uint32_t now = HAL_GetTick();
        uint16_t time_delta = (uint16_t)(now - (uint32_t)timestamp);
        
        // 允许时间差在 0-500ms
        return time_delta < 500;
    }
    
    uint32_t last_update_ms_;
    uint32_t timeout_counter_;
    osMessageQueueId_t velocity_command_queue_;
};

// 注册接收回调
void initVelocityReceiver() {
    CANManager::getInstance().registerCallback(
        0x120,  // VelocityCommand CAN ID
        VelocityCommandReceiver::onVelocityCommandReceived
    );
}
```

### 底盘控制器 (100Hz)

```cpp
class SwerveChassisController {
private:
    static constexpr uint32_t CONTROL_PERIOD_MS = 10;  // 100Hz
    
    // PID 参数 (见下一章节)
    struct PIDParams {
        float kp, ki, kd;
    };
    
    // 转向 PID (4 个)
    PIDParams steering_params_ = {15.0f, 0.3f, 0.5f};
    
    // 速度 PID (4 个)
    PIDParams speed_params_ = {10.0f, 0.3f, 0.5f};
    
public:
    SwerveChassisController() : 
        chassis_calculator_(),
        current_steering_angles_{0, 0, 0, 0},
        current_wheel_rpms_{0, 0, 0, 0},
        target_steering_angles_{0, 0, 0, 0},
        target_wheel_rpms_{0, 0, 0, 0} {
        
        // 初始化 8 个 PID 控制器
        for (int i = 0; i < 4; i++) {
            steering_pids_[i].init(steering_params_.kp,
                                   steering_params_.ki,
                                   steering_params_.kd,
                                   10);  // 最大输出
            
            speed_pids_[i].init(speed_params_.kp,
                               speed_params_.ki,
                               speed_params_.kd,
                               16384);  // 最大输出 (DJI电机)
        }
    }
    
    // 主控制循环 (100Hz)
    void controlLoop() {
        // 步骤 1: 获取最新速度目标
        VelocityCommand velocity_cmd;
        if (!VelocityCommandReceiver::getInstance()
                .getLatestCommand(velocity_cmd)) {
            // 超时 - 停止所有电机
            stopAllMotors();
            return;
        }
        
        // 步骤 2: 读取当前马达反馈
        readMotorFeedback();
        
        // 步骤 3: 调用反向运动学
        computeMotorTargets(velocity_cmd);
        
        // 步骤 4: 执行 PID 控制
        updatePIDControllers();
        
        // 步骤 5: 发送马达命令
        sendMotorCommands();
        
        // 步骤 6: 可选 - 计算正向运动学用于调试
        // computeChassisState();
    }
    
    // 读取马达反馈
    void readMotorFeedback() {
        // 读取转向马达 (CAN1, ID 1-4)
        for (int i = 0; i < 4; i++) {
            auto motor = DJIMotor::getInstance(1, i + 1);  // CAN1
            current_steering_angles_[i] = motor->getAngle();
        }
        
        // 读取速度马达 (CAN2, ID 1-4)
        for (int i = 0; i < 4; i++) {
            auto motor = DJIMotor::getInstance(2, i + 1);  // CAN2
            current_wheel_rpms_[i] = motor->getRPM();
        }
    }
    
    // 反向运动学: 速度目标 → 马达目标
    void computeMotorTargets(const VelocityCommand& cmd) {
        // 准备输入
        Core::Utils::Container::Triple<float> velocity{cmd.vx, cmd.vy, cmd.vw};
        Core::Utils::Container::Quadruple<float> current_angles{
            current_steering_angles_[0],
            current_steering_angles_[1],
            current_steering_angles_[2],
            current_steering_angles_[3]
        };
        
        // 调用 ChassisCalculator::ikine()
        // 注意: ikine() 返回的是角度差 (delta), 不是绝对角度
        auto result = chassis_calculator_.ikine(
            velocity,
            current_angles,
            0.0f  // gimbal_to_chassis_angle (通常=0)
        );
        
        auto steering_deltas = result.first;
        auto wheel_speeds = result.second;
        
        // 转换角度差为目标绝对角度
        for (int i = 0; i < 4; i++) {
            target_steering_angles_[i] = 
                current_steering_angles_[i] + steering_deltas[i];
            
            target_wheel_rpms_[i] = wheel_speeds[i];
        }
    }
    
    // 更新 PID 控制器
    void updatePIDControllers() {
        // 转向 PID (4 个)
        for (int i = 0; i < 4; i++) {
            float angle_error = 
                target_steering_angles_[i] - current_steering_angles_[i];
            
            steering_pids_[i].update(angle_error);
        }
        
        // 速度 PID (4 个)
        for (int i = 0; i < 4; i++) {
            float speed_error = 
                target_wheel_rpms_[i] - current_wheel_rpms_[i];
            
            speed_pids_[i].update(speed_error);
        }
    }
    
    // 发送马达命令
    void sendMotorCommands() {
        // 发送转向命令 (CAN1)
        for (int i = 0; i < 4; i++) {
            int16_t cmd = (int16_t)steering_pids_[i].getOutput();
            DJIMotor::getInstance(1, i + 1)->setTorque(cmd);
        }
        
        // 发送速度命令 (CAN2)
        for (int i = 0; i < 4; i++) {
            int16_t cmd = (int16_t)speed_pids_[i].getOutput();
            DJIMotor::getInstance(2, i + 1)->setTorque(cmd);
        }
    }
    
    // 停止所有马达
    void stopAllMotors() {
        for (int i = 0; i < 4; i++) {
            DJIMotor::getInstance(1, i + 1)->setTorque(0);
            DJIMotor::getInstance(2, i + 1)->setTorque(0);
        }
        
        // 重置 PID
        for (int i = 0; i < 4; i++) {
            steering_pids_[i].reset();
            speed_pids_[i].reset();
        }
    }
    
    // FreeRTOS 任务入口
    static void taskEntry(void* arg) {
        SwerveChassisController* controller = 
            (SwerveChassisController*)arg;
        
        // 等待底盘MCU初始化完成 (1秒)
        osDelay(1000);
        
        while (1) {
            uint32_t tick = osKernelGetTickCount();
            
            controller->controlLoop();
            
            // 精确延迟到 10ms
            osDelayUntil(tick + 10);
        }
    }
    
private:
    ChassisCalculator chassis_calculator_;
    
    // 当前反馈
    float current_steering_angles_[4];
    float current_wheel_rpms_[4];
    
    // 目标值
    float target_steering_angles_[4];
    float target_wheel_rpms_[4];
    
    // 8 个 PID 控制器
    PID steering_pids_[4];
    PID speed_pids_[4];
};
```

---

## PID配置

### 参数选择原理

```
转向马达 (3508) 参数建议:
  Kp = 15.0  - 负责快速响应角度差
  Ki = 0.3   - 消除稳态误差
  Kd = 0.5   - 阻尼，防止震荡

速度马达 (7015) 参数建议:
  Kp = 10.0  - 转速控制需要更温和的Kp
  Ki = 0.3   - 
  Kd = 0.5   - 
```

### 调试流程

#### 第 1 阶段: 只设 Kp

```
1. 设置 Kp = 5.0, Ki = 0, Kd = 0
2. 给定速度目标，观察响应速度
   - 太慢: 增大 Kp (如 10.0)
   - 太快/震荡: 减小 Kp (如 2.0)
3. 目标: 无震荡地快速响应 (~0.5秒内到达)
```

#### 第 2 阶段: 加入 Kd

```
1. 设置 Kd = 0.1, 逐步增加
2. 减少超调幅度
3. 目标: 无明显超调 (<5%)
```

#### 第 3 阶段: 加入 Ki

```
1. 设置 Ki = 0.1, 逐步增加
2. 消除稳态误差 (~0.05rad/s 以内)
3. 注意: 过大的 Ki 会导致积分饱和
```

### 代码集成

```cpp
// PID 初始化
void initChassisControllers() {
    static SwerveChassisController controller;
    
    // 转向 PID 配置
    for (int i = 0; i < 4; i++) {
        controller.getSteeringPID(i).init(
            15.0f,   // Kp
            0.3f,    // Ki
            0.5f,    // Kd
            10.0f    // 积分限幅
        );
    }
    
    // 速度 PID 配置
    for (int i = 0; i < 4; i++) {
        controller.getSpeedPID(i).init(
            10.0f,     // Kp
            0.3f,      // Ki
            0.5f,      // Kd
            5000.0f    // 积分限幅
        );
    }
}
```

---

## 集成步骤

### 第 1 步: 代码文件组织

```
RM2026-Core/
├── Control/
│   ├── ChassisCalculator.cpp         ✓ 已存在
│   ├── ChassisCalculator.hpp         ✓ 已存在
│   ├── PID.cpp                       ✓ 已存在
│   ├── PID.hpp                       ✓ 已存在
│   └── Chassis/
│       ├── SwerveChassisController.hpp    [新建]
│       └── SwerveChassisController.cpp    [新建]
├── Drivers/
│   ├── DR16.hpp                      ✓ 已存在
│   ├── CANManager.hpp                ✓ 已存在
│   ├── RosComm/
│   └── InterboardComm/
│       ├── InterboardMessage.hpp     [新建]
│       └── InterboardCommManager.hpp [新建]
└── Cloud/                                [新建]
    ├── RemoteHandler.hpp                [新建]
    └── RemoteHandler.cpp                [新建]
```

### 第 2 步: 创建必要的文件

#### InterboardMessage.hpp

```cpp
#pragma once
#include <cstring>

// 速度命令
struct VelocityCommand {
    float vx, vy, vw;
    uint8_t enable;
    uint8_t mode;
    uint16_t timestamp;
} __attribute__((packed));

// 马达反馈
struct MotorFeedback {
    uint8_t motor_health_mask;
    uint16_t avg_current;
    uint8_t system_status;
    uint8_t reserved[12];
} __attribute__((packed));

// 心跳
struct Heartbeat {
    uint8_t source_id;
    uint16_t tx_counter;
    uint16_t rx_counter;
    uint32_t uptime_ms;
    uint32_t reserved;
} __attribute__((packed));
```

### 第 3 步: 配置 CAN 发送/接收

#### CAN1 配置 (云台MCU)

```cpp
void initCloudMCUCAN() {
    FDCANManager& can_mgr = FDCANManager::getInstance();
    
    // 配置 CAN1
    can_mgr.setChannel(FDCAN_CHANNEL_1);  // CAN1
    
    // 初始化
    can_mgr.init();
    
    // 配置过滤器接收板间通信消息
    // 0x120: VelocityCommand (本MCU发送)
    // 0x121: MotorFeedback (接收)
    // 0x122: Heartbeat (接收)
    can_mgr.addFilterRule(0x121, 0x7FF);  // 接收 0x121
    can_mgr.addFilterRule(0x122, 0x7FF);  // 接收 0x122
}
```

#### CAN1/CAN2 配置 (底盘MCU)

```cpp
void initChassisMCUCAN() {
    FDCANManager& can_mgr = FDCANManager::getInstance();
    
    // 配置 CAN1 (转向马达 + 板间通信)
    can_mgr.setChannel(FDCAN_CHANNEL_1);
    can_mgr.init();
    
    // 设置接收滤波器
    can_mgr.addFilterRule(0x120, 0x7FF);  // 接收 0x120 (VelocityCommand)
    can_mgr.addFilterRule(0x200, 0x7FF);  // 接收马达反馈 (0x200-0x203)
    
    // 注册回调
    can_mgr.registerCallback(0x120, 
        VelocityCommandReceiver::onVelocityCommandReceived);
    
    // 配置 CAN2 (速度马达)
    can_mgr.setChannel(FDCAN_CHANNEL_2);
    can_mgr.init();
    can_mgr.addFilterRule(0x200, 0x7FF);  // 接收马达反馈
}
```

### 第 4 步: 任务创建

#### 云台MCU 任务

```cpp
void startCloudMCUTasks() {
    static RemoteHandler remote_handler;
    
    // 遥控处理任务 (50Hz)
    osThreadId_t remote_task = osThreadNew(
        RemoteHandler::taskEntry,
        &remote_handler,
        &(osThreadAttr_t){
            .name = "RemoteHandler",
            .priority = osPriorityHigh,
            .stack_size = 2048
        }
    );
}
```

#### 底盘MCU 任务

```cpp
void initChassisMCUTasks() {
    static SwerveChassisController chassis_controller;
    
    // 初始化接收器
    initVelocityReceiver();
    
    // 控制任务 (100Hz)
    osThreadId_t control_task = osThreadNew(
        SwerveChassisController::taskEntry,
        &chassis_controller,
        &(osThreadAttr_t){
            .name = "ChassisControl",
            .priority = osPriorityRealtime,
            .stack_size = 4096
        }
    );
}
```

### 第 5 步: 验证检查清单

```
准备工作:
  ☐ 确认 ChassisCalculator.hpp 已正确编译
  ☐ DR16 接收器能正常读数
  ☐ CAN 总线能正常通信
  ☐ PID.hpp 已包含

云台MCU:
  ☐ RemoteHandler 能读取 DR16 数据
  ☐ 遥控数据正确归一化 (-1 ~ +1)
  ☐ 速度目标正确映射 (vx, vy, vw)
  ☐ VelocityCommand 正确序列化和发送

底盘MCU:
  ☐ 接收回调正常触发
  ☐ 超时检测正常工作
  ☐ ChassisCalculator::ikine() 返回合理值
  ☐ 8 个 PID 都能正确更新
  ☐ 马达能收到正确的转矩命令
```

---

## 故障排查

### 问题 1: 底盘不动

```
排查步骤:

1. 检查 CAN 通信
   - 云台MCU 是否发送了 VelocityCommand?
   - 底盘MCU 的接收回调是否被触发?
   - 使用 CANTrace 或示波器查看 CAN 总线

2. 检查速度命令
   - print(velocity_command.vx, vy, vw)
   - 值是否为 0? → 检查遥控器
   - 值非 0? → 继续下一步

3. 检查反向运动学
   - print(target_steering_angles, target_wheel_rpms)
   - 是否为 0? → ChassisCalculator 问题
   - 是否不合理? → 参数配置错误

4. 检查 PID 输出
   - print(steering_pids[0].getOutput())
   - print(speed_pids[0].getOutput())
   - 是否为 0? → PID 未初始化或输入为 0
   
5. 检查马达通信
   - 底盘MCU 能否通过 CAN 驱动马达?
   - 测试代码:
     DJIMotor::getInstance(1, 1)->setTorque(1000);
     osDelay(100);
     DJIMotor::getInstance(1, 1)->setTorque(0);
```

### 问题 2: 底盘方向错误

```
排查步骤:

1. 验证轮子配置
   - 确认 chassis_radius_[] 和 tangent_angle_[] 正确
   - 这直接影响 ikine() 计算
   
2. 验证 CAN ID 映射
   - 确认 [ID 1-4] 是转向马达
   - 确认 [ID 1-4] 是速度马达 (不同CAN口)
   
3. 验证反向运动学方向
   - vx = 1.0, vy = 0, vw = 0
   - 应该输出: steering = 0°, speed = max
   - 如果 steering ≠ 0°, 说明配置有误

4. 手动发送命令测试
   - 绕过遥控器, 直接调用:
     velocity.vx = 1.0; velocity.vy = 0; velocity.vw = 0;
     result = calculator.ikine(velocity, current_angles, 0);
   - 查看 result 是否正确
```

### 问题 3: 转向震荡/超调

```
排查步骤:

1. 检查 Kp (比例增益)
   - 太大 → 震荡
   - 太小 → 响应缓慢
   - 建议: 从 5.0 开始逐步增加

2. 检查 Kd (微分增益)
   - 作用: 减少超调
   - 如果 Kd = 0 且有超调, 添加 Kd = 0.1 后测试

3. 检查 Ki (积分增益)
   - 作用: 消除稳态误差
   - 太大 → 积分饱和, 响应变慢
   - 建议: Ki 保持小值 (0.1-0.5)

4. 调试建议
   - 从 Kp 开始, 找到最大无超调值
   - 然后加入 Kd 减少超调
   - 最后加入 Ki 消除稳态误差
```

### 问题 4: 接收超时

```
排查步骤:

1. 检查发送周期
   - 云台MCU RemoteHandler 是否 20ms 发送一次?
   - print(timestamp) 检查时间间隔

2. 检查接收回调
   - 是否注册了 0x120 的回调?
   - 回调是否被调用? (添加调试 print)

3. 检查消息格式
   - sizeof(VelocityCommand) 是否等于 16 字节?
   - 使用 __attribute__((packed)) 确保无填充

4. 测试 CAN 发送
   - 使用 CANTrace 软件观察
   - 或在底盘 MCU 中添加:
     if (can_frame.CanId == 0x120) {
         HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, GPIO_PIN_SET);
     }
```

### 问题 5: 马达不响应

```
排查步骤:

1. 检查转矩命令
   - print(steering_pids[i].getOutput())
   - 值是否在 [-10000, +10000] 范围内?
   - 值是否都是 0?

2. 检查 PID 初始化
   - 是否调用了 init() 方法?
   - Kp/Ki/Kd 是否为 0?

3. 检查马达连接
   - DJI 马达能否单独驱动?
   - CAN ID 是否正确?

4. 测试直接驱动
   - 绕过 PID:
     DJIMotor::getInstance(1, 1)->setTorque(5000);
     osDelay(100);
   - 马达是否转动?
```

---

## 性能优化

### 1. 优先级设置

```cpp
// FreeRTOS 任务优先级分配
enum TaskPriority {
    PRIORITY_REALTIME = osPriorityRealtime,    // 底盘控制 (100Hz)
    PRIORITY_HIGH = osPriorityHigh,            // 遥控处理 (50Hz)
    PRIORITY_NORMAL = osPriorityNormal,        // 通信反馈
    PRIORITY_LOW = osPriorityLow               // 诊断输出
};
```

### 2. CAN 总线优化

```cpp
// 减少 CAN 消息冲突
// 云台MCU:
// - 0x001-0x003: Pitch/Shooter (CAN1)
// - 0x120: VelocityCommand (板间通信 CAN1)

// 底盘MCU:
// - 0x200-0x203: 转向马达反馈 (CAN1)
// - 0x204-0x207: 速度马达反馈 (CAN2)
// - 0x120: 接收速度命令 (板间)

// 避免冲突: 使用不同范围的 CAN ID
```

### 3. 内存优化

```cpp
// 使用 __attribute__((packed)) 减少结构体大小
struct VelocityCommand {
    float vx, vy, vw;      // 12 字节
    uint8_t enable;        // 1 字节
    uint8_t mode;          // 1 字节
    uint16_t timestamp;    // 2 字节
} __attribute__((packed));  // 总计: 16 字节 (无填充)

// 不使用 packed 可能导致 24 字节 (8 字节对齐)
```

### 4. 控制周期优化

```cpp
// 底盘MCU (100Hz)
void chassisControlLoop() {
    uint32_t tick = osKernelGetTickCount();
    
    // 处理时间应 < 5ms (留一半余量)
    controlLoop();
    
    // 精确延迟到下一个周期
    osDelayUntil(tick + 10);  // 精确 10ms
}

// 确保不会出现"漂移"
```

---

## 总结

### 核心流程

```
1. 遥控器 → 云台MCU
   ├─ DR16 读取 (中断)
   ├─ RemoteHandler 处理 (50Hz)
   └─ 发送 VelocityCommand

2. VelocityCommand → 底盘MCU
   ├─ CAN 接收 (中断)
   ├─ 超时检测
   └─ 存储到共享缓冲

3. 底盘控制 (100Hz)
   ├─ 读取速度目标
   ├─ 调用 ikine()
   ├─ 运行 8 个 PID
   └─ 驱动马达

4. 闭环反馈
   ├─ 马达反馈 (CAN)
   ├─ 正向运动学 (可选)
   └─ 状态发送回云台
```

### 关键参数速查表

| 参数 | 值 | 单位 |
|------|-----|------|
| 云台 RemoteHandler 周期 | 20 | ms |
| 底盘控制周期 | 10 | ms |
| CAN 通信间隔 | 20 | ms |
| 超时阈值 | 50 | ms |
| 最大前进速度 | 2.0 | m/s |
| 最大横向速度 | 2.0 | m/s |
| 最大旋转速度 | 3.14 | rad/s |
| 转向 PID Kp | 15.0 | - |
| 速度 PID Kp | 10.0 | - |

### 文件清单

需要创建/修改的文件:

```
[新建]
- Control/Chassis/SwerveChassisController.hpp
- Control/Chassis/SwerveChassisController.cpp
- Drivers/InterboardComm/InterboardMessage.hpp
- Drivers/InterboardComm/InterboardCommManager.hpp
- Cloud/RemoteHandler.hpp
- Cloud/RemoteHandler.cpp

[已存在, 需检查]
- Control/ChassisCalculator.hpp (ikine/fkine)
- Control/PID.hpp
- Drivers/DR16.hpp
- Drivers/CANManager.hpp
- Drivers/FDCANManager.hpp
```

### 下一步行动

```
1. ☐ 根据硬件实际情况调整 CAN ID 映射
2. ☐ 编写并测试 RemoteHandler
3. ☐ 编写并测试 VelocityCommandReceiver
4. ☐ 编写并测试 SwerveChassisController
5. ☐ 调试 PID 参数 (从 Kp 开始)
6. ☐ 集成所有模块
7. ☐ 现场测试和优化
```

有任何疑问或需要具体代码实现, 继续提问!
