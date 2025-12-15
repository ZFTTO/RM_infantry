# ChassisTask 完整实现指南

## 📋 ChassisTask 是什么？

ChassisTask 是**底盘控制任务**，负责：
- ✅ 读取遥控器输入
- ✅ 计算底盘的速度目标 (vx, vy, omega)
- ✅ 通过 CAN 发送命令给底盘 MCU
- ✅ 接收底盘状态反馈
- ✅ 执行速度控制和故障保护

---

## 🎯 ChassisTask 的位置和作用

```
架构图：

┌─────────────────────────────────────┐
│      云台 MCU (STM32G473)           │
│  (你的代码在这里)                   │
├─────────────────────────────────────┤
│                                     │
│  ┌─ rcControlTask                  │
│  │  ├─ 读遥控                       │
│  │  └─ 计算速度                     │
│  │                                 │
│  ├─ gimbalTask (云台)              │
│  │  ├─ 计算PID                      │
│  │  └─ 控制云台电机                 │
│  │                                 │
│  └─ armTask (机械臂)               │
│     ├─ 计算IKine                    │
│     └─ 控制关节电机                 │
│                                     │
└─────────────────────────────────────┘
         ↓ CAN 总线 (0x100)
┌─────────────────────────────────────┐
│      底盘 MCU (STM32F407)           │
│  (底盘厂家提供的代码)                 │
├─────────────────────────────────────┤
│                                     │
│  ┌─ chassisTask ← 这是底盘MCU做的  │
│  │  ├─ 接收速度命令 (vx,vy,omega)  │
│  │  ├─ 计算电机转速                 │
│  │  ├─ 控制4个电机运动              │
│  │  └─ 返回速度反馈                 │
│  │                                 │
│  └─ motorControlTask               │
│     ├─ PID 速度控制                 │
│     └─ 发送PWM给电调                │
│                                     │
└─────────────────────────────────────┘
```

---

## ❓ 你的代码 vs 底盘代码

### **你的云台MCU做什么？**

```cpp
// 在 UserTask.cpp 的 rcControlTask 中

void rcControlTask(void *pvPara) {
    while (true) {
        // 1️⃣ 读遥控
        const RcData& rc = DR16::getRcData();
        
        // 2️⃣ 计算目标速度
        float vx = ch1_normalized * MAX_VX;
        float vy = ch0_normalized * MAX_VY;
        float omega = ch3_normalized * MAX_OMEGA;
        
        // 3️⃣ 打包成 CAN 帧
        uint8_t data[8];
        // ... 编码 vx, vy, omega ...
        
        // 4️⃣ 发送给底盘MCU
        CAN_TXHEADER_T header = getTxHeader(0x100);
        canManager.transmit(header, data);
        //
        // 就这样！你的工作完成了
        // 之后的事交给底盘MCU
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

### **底盘MCU做什么？（你看不到的地方）**

```cpp
// 在底盘 MCU 的 ChassisTask 中（底盘厂家代码）

void chassisTask(void *pvPara) {
    while (true) {
        // 1️⃣ 接收你发来的命令
        // CAN Callback 自动更新了 targetVx, targetVy, targetOmega
        
        // 2️⃣ 计算每个电机的转速
        // 底盘是麦克纳姆轮或舵轮
        // 需要做正向运动学计算
        float wheel_speed[4];
        forwardKinematics(targetVx, targetVy, targetOmega, wheel_speed);
        
        // 3️⃣ 对每个电机做速度PID
        for (int i = 0; i < 4; i++) {
            float motor_output = motorPID[i].update(wheel_speed[i]);
            sendPWM(motor[i], motor_output);
        }
        
        // 4️⃣ 读取电机反馈（电流、温度等）
        // 通过 CAN 反馈给你
        sendMotorFeedback();
        
        vTaskDelay(pdMS_TO_TICKS(5));  // 更频繁的控制
    }
}
```

---

## 📊 完整的数据流和CAN通信

```
你的云台MCU                          底盘MCU
    │                                 │
    │  ┌─ rcControlTask              │
    │  │ 1. 读遥控                    │
    │  │ 2. 计算 vx, vy, omega       │
    │  │                             │
    │  │ CAN Frame (0x100)           │
    │  │ ├─ vx (2bytes)              │
    │  │ ├─ vy (2bytes)              │
    │  │ ├─ omega (2bytes)           │
    │  │ └─ mode (1byte)             │
    │  └──────────────────────────→  │ ┌─ chassisTask
    │                                │ │ 1. 接收命令
    │                                │ │ 2. 计算轮速
    │                                │ │ 3. 电机PID
    │                                │ │ 4. 读电机状态
    │                                │ │
    │  ┌─ motorFeedbackTask         │ │ CAN Frame (0x200)
    │  │ 接收底盘反馈                │ │ ├─ 电机角度
    │  │ ┌──────────────────────────←─└─ ├─ 电机速度
    │  │ │ 更新 chassisFeedback[]   │    └─ 电机电流
    │  │ └──────────────────────────┘
    │  │
    │  └─ 10ms循环
    │
    └─ 连续运行...
```

---

## 🔧 实际代码：底盘相关的你的代码

### **第1部分：发送命令给底盘**

```cpp
// 在 rcControlTask 中

void rcControlTask(void *pvPara)
{
    while (true)
    {
        // 获取遥控数据
        const volatile RcData& rc_data = DR16::getRcData();
        
        if (DR16::isConnected())
        {
            // 提取摇杆
            uint16_t ch0 = rc_data.rc.ch0;  // 左右移动 (vy)
            uint16_t ch1 = rc_data.rc.ch1;  // 前后移动 (vx)
            uint16_t ch3 = rc_data.rc.ch3;  // 旋转 (omega)
            uint8_t switch1 = rc_data.rc.s1;
            
            // 归一化 [-1, 1]
            float ch0_norm = (float)(ch0 - DR16::CH_VALUE_MID) / DR16::CH_VALUE_ABS_RANGE;
            float ch1_norm = (float)(ch1 - DR16::CH_VALUE_MID) / DR16::CH_VALUE_ABS_RANGE;
            float ch3_norm = (float)(ch3 - DR16::CH_VALUE_MID) / DR16::CH_VALUE_ABS_RANGE;
            
            // 限制范围
            if (ch0_norm > 1.0f) ch0_norm = 1.0f;
            if (ch0_norm < -1.0f) ch0_norm = -1.0f;
            if (ch1_norm > 1.0f) ch1_norm = 1.0f;
            if (ch1_norm < -1.0f) ch1_norm = -1.0f;
            if (ch3_norm > 1.0f) ch3_norm = 1.0f;
            if (ch3_norm < -1.0f) ch3_norm = -1.0f;
            
            // ========== 根据开关选择速度档位 ==========
            float max_vx = 2.0f;      // 默认全速
            float max_vy = 2.0f;
            float max_omega = 3.14f;
            
            if (switch1 == DR16::SW_UP)
            {
                // 全速模式
                max_vx = 2.0f;
                max_vy = 2.0f;
                max_omega = 3.14f;
            }
            else if (switch1 == DR16::SW_MID)
            {
                // 半速模式
                max_vx = 1.0f;
                max_vy = 1.0f;
                max_omega = 1.57f;
            }
            else if (switch1 == DR16::SW_DOWN)
            {
                // 停止模式
                max_vx = 0.0f;
                max_vy = 0.0f;
                max_omega = 0.0f;
            }
            
            // ========== 计算目标速度 ==========
            float target_vx = ch1_norm * max_vx;      // 前后
            float target_vy = ch0_norm * max_vy;      // 左右
            float target_omega = ch3_norm * max_omega; // 旋转
            
            // ========== 打包 CAN 帧 ==========
            // CAN ID: 0x100 (底盘命令)
            CAN_TXHEADER_T txHeader = CANManager::getTxHeader(0x100);
            
            uint8_t canData[8];
            
            // 编码 vx (float -2.0~2.0 → int16 -32767~32767)
            int16_t vx_scaled = (int16_t)(target_vx * 16383.5f);
            canData[0] = (vx_scaled >> 8) & 0xFF;
            canData[1] = vx_scaled & 0xFF;
            
            // 编码 vy
            int16_t vy_scaled = (int16_t)(target_vy * 16383.5f);
            canData[2] = (vy_scaled >> 8) & 0xFF;
            canData[3] = vy_scaled & 0xFF;
            
            // 编码 omega (float -3.14~3.14 → int16)
            int16_t omega_scaled = (int16_t)(target_omega * 10430.4f);
            canData[4] = (omega_scaled >> 8) & 0xFF;
            canData[5] = omega_scaled & 0xFF;
            
            // 模式标志
            canData[6] = 0;  // 可以用来表示特殊模式
            canData[7] = 0;  // 预留
            
            // ========== 发送给底盘 MCU ==========
            canManager.transmit(txHeader, canData);
        }
        else
        {
            // ========== 掉线保护：停止底盘 ==========
            CAN_TXHEADER_T txHeader = CANManager::getTxHeader(0x100);
            uint8_t canData[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // 全是0 = 停止
            canManager.transmit(txHeader, canData);
        }
        
        // 10ms 发送一次
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

### **第2部分：接收底盘反馈**

```cpp
// 全局变量：接收底盘反馈
struct ChassisFeedback {
    float vx_actual;       // 实际速度
    float vy_actual;
    float omega_actual;
    float power;           // 功率消耗
    uint8_t status;        // 状态标志
};

ChassisFeedback chassisFeedback = {0};

// CAN 回调：当接收到底盘数据时自动调用
void chassisFeedbackCallback(const uint8_t *rxBuffer, const uint16_t id, const uint8_t canIndex)
{
    if (id == 0x200)  // 底盘反馈数据帧
    {
        // 解码 vx_actual
        int16_t vx_scaled = ((int16_t)rxBuffer[0] << 8) | rxBuffer[1];
        chassisFeedback.vx_actual = vx_scaled / 16383.5f;
        
        // 解码 vy_actual
        int16_t vy_scaled = ((int16_t)rxBuffer[2] << 8) | rxBuffer[3];
        chassisFeedback.vy_actual = vy_scaled / 16383.5f;
        
        // 解码 omega_actual
        int16_t omega_scaled = ((int16_t)rxBuffer[4] << 8) | rxBuffer[5];
        chassisFeedback.omega_actual = omega_scaled / 10430.4f;
        
        // 解码功率
        uint16_t power_raw = (rxBuffer[6] << 8) | rxBuffer[7];
        chassisFeedback.power = power_raw / 10.0f;  // 放大10倍存储
    }
}

// 注册回调（在 startUserTasks 中）
void registerChassisCallback()
{
    CAN_FILTER_T filter = CANManager::getFilter(
        0x7FF, 0x200,
        CANManager::FilterType::MASK,
        CANManager::FilterConfig::FIFO0
    );
    canManager.registerFilterCallback(filter, chassisFeedbackCallback);
}
```

### **第3部分：底盘反馈监控 Task**

```cpp
// 专门处理底盘反馈的 Task

StackType_t chassisMonitorStack[256];
StaticTask_t chassisMonitorTCB;

void chassisMonitorTask(void *pvPara)
{
    while (true)
    {
        // 读取底盘反馈（由回调自动更新）
        float actual_vx = chassisFeedback.vx_actual;
        float actual_vy = chassisFeedback.vy_actual;
        float actual_omega = chassisFeedback.omega_actual;
        float power = chassisFeedback.power;
        
        // ========== 检查异常 ==========
        
        // 1. 功率超限保护
        if (power > 120.0f)  // 功率上限 120W
        {
            // 降低速度指令
            // TODO: 降低遥控灵敏度或自动减速
        }
        
        // 2. 电机卡死检测
        // 如果命令速度 > 0，但实际速度 ≈ 0，说明电机可能卡死
        
        // 3. 超时检测
        // 如果长时间没有收到底盘反馈，说明通信故障
        
        // ========== 可选：速度跟踪 ==========
        // 计算跟踪误差，用于调试
        float vx_error = target_vx - actual_vx;
        float vy_error = target_vy - actual_vy;
        float omega_error = target_omega - actual_omega;
        
        // 如果误差很大，可能需要调整底盘的PID参数
        
        // 100ms 检查一次足够
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

---

## 📝 完整的 startUserTasks 实现

```cpp
void startUserTasks()
{
    // ========== 初始化硬件 ==========
    DR16::init();                    // 遥控接收
    canManager.init(&hfdcan1);       // CAN 通信
    
    // ========== 注册 CAN 回调 ==========
    // 底盘反馈
    CAN_FILTER_T chassisFilter = CANManager::getFilter(
        0x7FF, 0x200,
        CANManager::FilterType::MASK,
        CANManager::FilterConfig::FIFO0
    );
    canManager.registerFilterCallback(chassisFilter, chassisFeedbackCallback);
    
    // 电机反馈 (如果需要)
    CAN_FILTER_T motorFilter = CANManager::getFilter(
        0x7FF, 0x201,
        CANManager::FilterType::MASK,
        CANManager::FilterConfig::FIFO0
    );
    canManager.registerFilterCallback(motorFilter, motorFeedbackCallback);
    
    // ========== 创建 Task ==========
    
    // Task 1: 遥控 → 底盘速度 (最重要，优先级最高)
    xTaskCreateStatic(rcControlTask,
                      "RC_Control",
                      512,
                      NULL,
                      3,              // 高优先级
                      uxRcTaskStack,
                      &xRcTaskTCB);
    
    // Task 2: 底盘反馈监控
    xTaskCreateStatic(chassisMonitorTask,
                      "Chassis_Monitor",
                      256,
                      NULL,
                      2,              // 中等优先级
                      chassisMonitorStack,
                      &chassisMonitorTCB);
    
    // Task 3: 云台控制 (如果有云台)
    xTaskCreateStatic(gimbalTask,
                      "Gimbal",
                      512,
                      NULL,
                      2,
                      gimbalTaskStack,
                      &gimbalTaskTCB);
    
    // Task 4: 机械臂控制 (如果有机械臂)
    xTaskCreateStatic(armTask,
                      "Arm",
                      512,
                      NULL,
                      2,
                      armTaskStack,
                      &armTaskTCB);
    
    // Task 5: 调试输出 (低优先级)
    xTaskCreateStatic(debugTask,
                      "Debug",
                      256,
                      NULL,
                      1,
                      debugTaskStack,
                      &debugTaskTCB);
    
    // Task 6: LED 闪烁 (最低优先级)
    xTaskCreateStatic(blinkTask,
                      "Blink",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      1,
                      blinkTaskStack,
                      &blinkTaskTCB);
}
```

---

## 🎮 遥控的效果

```
摇杆位置 → 你的计算 → CAN 发送 → 底盘动作

推左摇杆前进:
  ch1 = 1400 (向前最大)
  → ch1_norm = 1.0
  → vx = 2.0 m/s
  → 发送 CAN ID=0x100, vx=2.0
  → 底盘MCU收到
  → 底盘向前驱动
  ↓
1000ms 后
  → 底盘反馈 0x200 帧
  → vx_actual = 1.8 m/s (可能有滑动)
  → 你可以在 chassisMonitorTask 中看到
```

---

## 🔄 完整的时序图

```
时间轴：

t=0ms:
  ┌─ 你推左摇杆前进

t=1ms:
  ├─ UART 中断接收遥控数据
  ├─ dr16CompleteCallback 更新 rc_data
  └─ 设置定时器重启

t=10ms:
  ├─ rcControlTask 被唤醒
  ├─ 读 rc_data (ch1=1400)
  ├─ 计算 vx=2.0
  ├─ 打包 CAN 帧
  ├─ canManager.transmit() 发送
  └─ 等待 10ms 后再执行

t=15ms:
  ├─ CAN 帧发送到底盘 MCU
  └─ 底盘 MCU 的 CAN 中断处理

t=20ms:
  ├─ 底盘 MCU 的 chassisTask 接收命令
  ├─ 计算 4 个轮的转速
  ├─ 执行电机 PID 控制
  └─ 向 4 个电机输出 PWM

t=100ms:
  ├─ 底盘电机转了一圈
  ├─ 底盘反馈数据生成
  ├─ 底盘 MCU 发送 CAN 帧 (ID=0x200)
  └─ 包含 vx_actual, vy_actual 等

t=110ms:
  ├─ 你的 CAN 中断接收数据
  ├─ chassisFeedbackCallback 更新 chassisFeedback
  └─ 设置 RX Task 就绪

t=120ms:
  ├─ 你的 rcControlTask 再次执行
  ├─ 如果摇杆还在前进，继续发送相同命令
  └─ 如果摇杆回到中点，发送 vx=0

...循环...
```

---

## 📊 速度参数参考

```cpp
// 常见的速度限制

// 竞速机器人
const float MAX_VX = 2.5f;      // 2.5 m/s
const float MAX_VY = 2.5f;
const float MAX_OMEGA = 4.0f;   // rad/s

// 普通机器人
const float MAX_VX = 1.5f;      // 1.5 m/s
const float MAX_VY = 1.5f;
const float MAX_OMEGA = 2.0f;

// 步兵机器人
const float MAX_VX = 2.0f;      // 2.0 m/s
const float MAX_VY = 2.0f;
const float MAX_OMEGA = 3.14f;  // π rad/s (1 rotation/sec)

// 哨兵机器人（高速）
const float MAX_VX = 3.0f;      // 3.0 m/s
const float MAX_VY = 3.0f;
const float MAX_OMEGA = 5.0f;
```

---

## ✅ 总结：你需要做的

1. ✅ **rcControlTask** - 读遥控，发送给底盘
2. ✅ **chassisMonitorTask** - 监控底盘状态
3. ✅ **注册回调** - 接收底盘反馈
4. ✅ **故障保护** - 掉线停止

**不需要你做的（底盘MCU已经做了）：**
- ❌ 计算轮速 (正向运动学)
- ❌ 电机 PID 控制
- ❌ PWM 输出
- ❌ 功率管理

就这样！简单吧？
