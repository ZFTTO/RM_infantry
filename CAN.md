# FDCANManager 完整指南

## 📋 FDCANManager 是什么？

FDCANManager 是一个**企业级的 CAN 总线驱动库**，用于：
- 管理 **FDCAN (CAN FD)** 硬件通信
- 自动处理 **接收中断、DMA、队列**
- 提供 **过滤器和回调机制** 路由数据
- 使用 **FreeRTOS** 实现高效的多任务通信

---

## 🏗️ 核心架构

### **三层设计**

```
┌─────────────────────────────────────┐
│        用户代码 (UserTask.cpp)       │
│   - transmit()    (发送)             │
│   - callbacks()   (接收)             │
└────────────┬──────────────────────────┘
             │
┌────────────▼──────────────────────────┐
│       FDCANManager 驱动层             │
├─────────────────────────────────────┤
│  RX Task        TX Task        Queue │
│  (接收任务)     (发送任务)     (缓存)│
│                                     │
│  Callbacks      Filters             │
│  (数据路由)     (ID过滤)            │
└────────────┬──────────────────────────┘
             │
┌────────────▼──────────────────────────┐
│     FDCAN 硬件 (STM32 硬件)         │
│   中断、DMA、FIFO 队列              │
└─────────────────────────────────────┘
```

---

## 🔄 数据流

### **接收流程**

```
硬件 FDCAN 接收数据包 (8字节)
    ↓ (FDCAN_RX_FIFO0/1 有数据)
硬件中断 (rxFifoCallback)
    ↓
唤醒 RX Task (Task Notification)
    ↓
RX Task 执行:
    ├─ 循环读取 FIFO0
    ├─ 循环读取 FIFO1
    ├─ 获取 rxHeader.FilterIndex
    ├─ 查找对应的回调函数
    └─ 调用回调: callback(data, id, canIndex)
    ↓
你的回调函数处理数据
```

### **发送流程**

```
调用 transmit(header, data)
    ↓
检查硬件 FIFO 状态:
    │
    ├─ 有空间 → 直接发送 (快速路径)
    │         ↓
    │         硬件立即发送
    │
    └─ 满了 → 加入 FreeRTOS Queue
              ↓
              TX Task 从 Queue 取数据
              ↓
              硬件有空间时发送
```

---

## 🎯 基本使用步骤

### **步骤 1: 初始化**

```cpp
#include "FDCANManager.hpp"

Core::Drivers::CANManager canManager;

void startUserTasks()
{
    // 初始化 CAN (必须在使用前调用)
    canManager.init(&hfdcan1);  // hfdcan1 由 STM32CubeMX 生成
}
```

**关键点：**
- `hfdcan1` 是 STM32CubeMX 生成的 FDCAN 句柄
- 只需调用一次
- 会自动创建 RX/TX Task 和 Queue

---

### **步骤 2: 创建过滤器**

过滤器决定哪些 CAN ID 的数据会被接收和路由到你的回调函数。

#### **过滤器类型**

**A. MASK 模式（掩码）- 推荐用于单个或少量ID**

```cpp
// 只接收 ID 0x201 的数据
CAN_FILTER_T filter = CANManager::getFilter(
    0x7FF,                              // FilterID2: Mask (掩码)
    0x201,                              // FilterID1: ID (要接收的ID)
    CANManager::FilterType::MASK,       // 掩码模式
    CANManager::FilterConfig::FIFO0     // 存储到 FIFO0
);

// 工作原理:
//   接收的ID & Mask == 要求的ID
//   0x201 & 0x7FF = 0x201  ✓ 匹配
//   0x202 & 0x7FF = 0x202  ✗ 不匹配
//   0x200 & 0x7FF = 0x200  ✗ 不匹配
```

**B. RANGE 模式（范围）- 用于连续ID段**

```cpp
// 接收 0x200~0x20F 范围内的所有ID
CAN_FILTER_T filter = CANManager::getFilter(
    0x20F,                              // FilterID2: End ID (结束)
    0x200,                              // FilterID1: Start ID (起始)
    CANManager::FilterType::RANGE,      // 范围模式
    CANManager::FilterConfig::FIFO0
);

// 工作原理:
//   0x200 <= ID <= 0x20F
//   0x201 ✓  0x20A ✓  0x20F ✓
//   0x1FF ✗  0x210 ✗
```

#### **FIFO 选择**

```cpp
CANManager::FilterConfig::FIFO0  // FIFO 队列 0
CANManager::FilterConfig::FIFO1  // FIFO 队列 1

// 通常都用 FIFO0，除非需要优先级隔离
```

---

### **步骤 3: 定义接收回调**

回调函数在 RX Task 中调用，有充足的处理时间。

```cpp
// 定义回调函数类型
void motorFeedbackCallback(
    const uint8_t *rxBuffer,    // 接收的 8 字节数据
    const uint16_t id,          // CAN ID (0x200-0xFFFF)
    const uint8_t canIndex      // CAN 编号 (0=FDCAN1, 1=FDCAN2...)
)
{
    // 例：解析电机数据
    // 通常数据格式为大端序 (Big Endian)
    
    uint16_t angle = (rxBuffer[0] << 8) | rxBuffer[1];
    int16_t speed = (int16_t)((rxBuffer[2] << 8) | rxBuffer[3]);
    int16_t current = (int16_t)((rxBuffer[4] << 8) | rxBuffer[5]);
    uint8_t temp = rxBuffer[6];
    
    // 保存到全局变量或发送到队列
    motorData[id - 0x201].angle = angle;
    motorData[id - 0x201].speed = speed;
    motorData[id - 0x201].current = current;
}
```

---

### **步骤 4: 注册过滤器和回调**

```cpp
// 创建过滤器
CAN_FILTER_T filter = CANManager::getFilter(
    0x7FF, 0x201,
    CANManager::FilterType::MASK,
    CANManager::FilterConfig::FIFO0
);

// 注册：将过滤器和回调绑定
canManager.registerFilterCallback(filter, motorFeedbackCallback);

// 现在当接收到 ID 0x201 的数据时，会自动调用 motorFeedbackCallback()
```

**重要：** 最多支持 8 个过滤器 (由 `CAN_FILTER_NUM` 定义)

---

### **步骤 5: 发送数据**

#### **创建 TX Header**

```cpp
// 创建标准 CAN 帧头 (11位ID, 8字节数据)
CAN_TXHEADER_T txHeader = CANManager::getTxHeader(0x300);

// 或创建 FDCAN 帧头 (支持不同数据长度)
// CAN_TXHEADER_T txHeader = CANManager::getTxHeader(
//     0x300,
//     CANManager::DataLength::BYTES_8
// );
```

#### **准备数据并发送**

```cpp
uint8_t txData[8] = {
    0x12, 0x34,           // Bytes 0-1
    0x56, 0x78,           // Bytes 2-3
    0xAA, 0xBB,           // Bytes 4-5
    0xCC, 0xDD            // Bytes 6-7
};

// 发送 (会自动选择快速路径或加入队列)
canManager.transmit(txHeader, txData);

// 函数立即返回，数据会被发送 (不阻塞)
```

---

## 💡 实际例子：电机控制

### **场景：控制 4 个 DJI 电机**

```cpp
// 1. 定义数据结构
struct MotorData {
    uint16_t angle;
    int16_t speed;
    int16_t current;
    uint8_t temperature;
};

MotorData motors[4];

// 2. 定义接收回调
void motorFeedbackCallback(const uint8_t *data, const uint16_t id, const uint8_t canIndex)
{
    uint8_t motorIdx = id - 0x201;
    if (motorIdx >= 4) return;
    
    motors[motorIdx].angle = (data[0] << 8) | data[1];
    motors[motorIdx].speed = (int16_t)((data[2] << 8) | data[3]);
    motors[motorIdx].current = (int16_t)((data[4] << 8) | data[5]);
    motors[motorIdx].temperature = data[6];
}

// 3. 初始化
void setupMotors()
{
    canManager.init(&hfdcan1);
    
    // 监听 4 个电机的反馈 (ID: 0x201~0x204)
    CAN_FILTER_T filter = CANManager::getFilter(
        0x7FF, 0x201,
        CANManager::FilterType::MASK,
        CANManager::FilterConfig::FIFO0
    );
    canManager.registerFilterCallback(filter, motorFeedbackCallback);
}

// 4. 发送控制命令
void sendMotorCommand(uint8_t motorId, int16_t targetCurrent)
{
    CAN_TXHEADER_T txHeader = CANManager::getTxHeader(0x200);
    
    uint8_t txData[8] = {0};
    
    // DJI 电机格式: 每个电流值占 2 字节
    // 电机 0: Bytes 0-1
    // 电机 1: Bytes 2-3
    // 电机 2: Bytes 4-5
    // 电机 3: Bytes 6-7
    
    txData[motorId * 2] = (targetCurrent >> 8) & 0xFF;
    txData[motorId * 2 + 1] = targetCurrent & 0xFF;
    
    canManager.transmit(txHeader, txData);
}

// 5. 在 Task 中使用
void motorControlTask(void *pvPara)
{
    while (true) {
        // 读取电机反馈 (由回调自动更新)
        uint16_t angle = motors[0].angle;
        int16_t speed = motors[0].speed;
        
        // 计算目标电流
        int16_t targetCurrent = pidController.update(angleError);
        
        // 发送给所有电机
        for (int i = 0; i < 4; i++) {
            sendMotorCommand(i, targetCurrent);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

---

## 📊 过滤器配置表

| 应用场景 | FilterType | FilterID1 | FilterID2 | 说明 |
|---------|-----------|----------|----------|------|
| 单个ID | MASK | 0x201 | 0x7FF | 只接收 0x201 |
| 多个单独ID | MASK | 各ID | 0x7FF | 需要多个过滤器 |
| ID 范围 | RANGE | 0x200 | 0x20F | 接收 0x200-0x20F |
| 所有 ID | MASK | 0x000 | 0x000 | 接收所有ID (不推荐) |

---

## ⚙️ 配置参数

在 `AppConfig.h` 中配置：

```cpp
#define USE_CAN_MANAGER 1              // 启用 CAN Manager

#if USE_CAN_MANAGER
    #define CAN_NUM 1                  // CAN 数量 (最多 3: FDCAN1/2/3)
    #define CAN_FILTER_NUM 8           // 过滤器数量 (最多 8 个)
    #define CAN_TX_MAX_MESSAGE_NUM_PER_TICK 7  // 每周期最大 TX 消息数
#endif
```

---

## 🧪 调试和监控

### **监控发送状态**

```cpp
// 在 FDCANManager 内部有这些计数器：

volatile uint32_t txCountFifoQ;     // ✅ 成功发送的消息数
volatile uint32_t txCountFifoQFail; // ❌ 发送失败的消息数
volatile uint32_t txCountAbortFifoQ;// ⚠️ 丢弃的消息数
volatile uint8_t qLevel;            // 📊 当前队列深度
volatile uint32_t busOffCount;      // 🔴 总线离线次数

// 在调试器中监控这些值来评估 CAN 总线健康状况
```

### **监控接收状态**

```cpp
volatile uint32_t rxCountTask;      // ✅ 接收处理的消息数

// 正常情况下，rxCountTask 应该稳定增长
// 如果没有增长，说明没有收到数据
```

---

## 🚨 常见错误和解决方案

| 问题 | 原因 | 解决 |
|------|------|------|
| 收不到数据 | 过滤器ID设置错误 | 检查 FilterID1 值是否与发送端匹配 |
| 回调不执行 | 没注册过滤器 | 确保调用 registerFilterCallback() |
| 发送数据丢失 | TX Queue 满 | 减少发送频率或增加 CAN_TX_QUEUE_LENGTH |
| 数据错乱 | 字节序错误 | 检查大小端转换 (<<8, >>8) |
| CAN 总线错误 | 硬件连接问题 | 检查 CAN 线、终端电阻 |
| 初始化失败 | init() 调用两次 | 只能调用一次，检查代码逻辑 |

---

## 🔍 CAN 数据格式标准

### **大端序 (Big Endian) - DJI 标准**

```cpp
// 发送 16 位数据: 0x1234
uint8_t data[2] = {0x12, 0x34};  // 高字节在前

// 接收并解析
uint16_t value = (data[0] << 8) | data[1];  // 得到 0x1234

// 发送 16 位有符号数: -1000 (0xFC18)
int16_t value = -1000;
uint8_t data[2] = {
    (value >> 8) & 0xFF,  // 高字节
    value & 0xFF          // 低字节
};

// 接收
int16_t received = (int16_t)((data[0] << 8) | data[1]);  // 得到 -1000
```

### **小端序 (Little Endian) - 某些设备**

```cpp
// 如果是小端序，字节顺序相反
uint8_t data[2] = {0x34, 0x12};  // 低字节在前
uint16_t value = data[0] | (data[1] << 8);  // 得到 0x1234
```

---

## 📈 性能优化建议

### **1. 任务优先级**

```cpp
// FDCANManager 创建的任务优先级都是 15 (最高)
// 这确保 CAN 通信的实时性
// 你的应用 Task 优先级应该是 1-14
```

### **2. Stack 大小**

```cpp
// RX Task: 512 字节  (接收和回调处理)
// TX Task: 256 字节  (发送处理)
// 如果回调逻辑复杂，可能需要更大的 stack
```

### **3. 发送批量数据**

```cpp
// ❌ 不好：每次发送一个字节
for (int i = 0; i < 8; i++) {
    transmit(header, data);  // 调用 8 次
}

// ✅ 好：一次打包 8 字节
uint8_t data[8] = {...};
transmit(header, data);  // 调用 1 次
```

---

## 📌 关键API总结

```cpp
// 初始化
canManager.init(FDCAN_HandleTypeDef *handle);

// 创建过滤器
CAN_FILTER_T getFilter(
    uint16_t filterID2, uint16_t filterID1,
    FilterType type, FilterConfig config
);

// 注册回调
canManager.registerFilterCallback(filter, callback);

// 发送数据
canManager.transmit(txHeader, txData);

// 创建 TX Header
CAN_TXHEADER_T getTxHeader(uint16_t id);
CAN_TXHEADER_T getTxHeader(uint16_t id, DataLength len);
```

---

## ✅ 完整工作流程

```
1. AppConfig.h 配置
   ↓
2. FDCANManager canManager;
   ↓
3. canManager.init(&hfdcan1);
   ↓
4. 创建过滤器: CAN_FILTER_T filter = getFilter(...);
   ↓
5. 注册回调: registerFilterCallback(filter, callback);
   ↓
6. 编写回调函数: void callback(data, id, canIndex)
   ↓
7. 在 Task 中调用: canManager.transmit(header, data);
   ↓
✅ 完成！CAN 通信工作
```

---

现在你已经掌握了 FDCANManager 的使用！可以参考 `RC_to_Chassis_CAN_Protocol.md` 看实际应用例子。


# 遥控 → 底盘 CAN 通信协议

## 📊 通信架构

```
遥控器 (DR16)
    ↓ UART (DMA)
云台MCU (STM32G473)
    ├─ rcControlTask 读取遥控数据
    ├─ 计算速度目标
    └─ 发送给底盘MCU
    ↓ FDCAN (CAN总线)
底盘MCU (STM32F407/其他)
    ├─ 接收速度命令
    ├─ 控制四个电机
    └─ 反馈当前状态 (可选)
```

---

## 📝 CAN 帧格式定义

### **发送帧：云台MCU → 底盘MCU**

```
CAN ID: 0x100 (云台控制命令)
Data Length: 8 bytes
Frequency: 100Hz (10ms)

┌─────────┬─────────┬──────────────────────────┐
│ Byte    │ 含义    │ 数值范围                 │
├─────────┼─────────┼──────────────────────────┤
│ 0-1     │ vx      │ -2.0 ~ +2.0 m/s          │
│ 2-3     │ vy      │ -2.0 ~ +2.0 m/s          │
│ 4-5     │ omega   │ -3.14 ~ +3.14 rad/s      │
│ 6       │ mode    │ bit0=特殊, bit1=视觉     │
│ 7       │ reserve │ 预留                     │
└─────────┴─────────┴──────────────────────────┘
```

### **字节编码方式**

#### **vx (前后速度) - Byte 0-1**
```
原始值: -2.0 ~ +2.0 m/s
编码: int16_t = (float值) * 16383.5
解码: float = (int16_t值) / 16383.5

例:
  vx = 1.0 m/s  → int16 = 16383  → canData[0:1] = 0x3FFF
  vx = 0.0 m/s  → int16 = 0      → canData[0:1] = 0x0000
  vx = -1.0 m/s → int16 = -16383 → canData[0:1] = 0xC001
```

#### **vy (左右速度) - Byte 2-3**
```
原始值: -2.0 ~ +2.0 m/s (同 vx)
编码: int16_t = (float值) * 16383.5
```

#### **omega (旋转速度) - Byte 4-5**
```
原始值: -3.14 ~ +3.14 rad/s
编码: int16_t = (float值) * 10430.4  (= 32767 / 3.14)
解码: float = (int16_t值) / 10430.4

例:
  omega = 1.57 rad/s (π/2)  → int16 = 16367 → canData[4:5] = 0x3FAF
  omega = 0.0 rad/s         → int16 = 0     → canData[4:5] = 0x0000
```

#### **mode (模式标志) - Byte 6**
```
bit0: 特殊模式 (S1==UP)
bit1: 视觉自瞄 (鼠标左键)
bit2~7: 预留

例: 普通模式 mode = 0x00
    视觉模式 mode = 0x02
```

---

## 🎮 遥控映射表

### **摇杆到速度的映射**

```
通道映射:
Ch0 (Roll)    → vy (左右移动)
Ch1 (Pitch)   → vx (前后移动)
Ch3 (Yaw)     → omega (旋转)

范围转换:
遥控: [364, 1024, 1684]
归一化: [-1.0, 0.0, 1.0]
速度: [-MAX, 0.0, +MAX]
```

### **开关映射**

```
Switch1 位置:
┌──────┬────────────────────────────┐
│UP    │ 全速模式 (vx/vy max=2.0)  │
│MID   │ 半速模式 (vx/vy max=1.0)  │
│DOWN  │ 停止模式 (vx/vy=0)        │
└──────┴────────────────────────────┘

Switch2 位置:
┌──────┬────────────────────────────┐
│UP    │ 特殊模式 (mode bit0=1)    │
│MID   │ 普通模式 (mode bit0=0)    │
│DOWN  │ 普通模式 (mode bit0=0)    │
└──────┴────────────────────────────┘
```

### **鼠标映射**

```
鼠标左键: 视觉自瞄模式 (mode bit1=1)
```

---

## 💾 代码实现详解

### **1. CAN 帧打包**

```cpp
// 创建 CAN TX Header
CAN_TXHEADER_T txHeader = CANManager::getTxHeader(0x100);

// 准备 8 字节数据
uint8_t canData[8];

// 编码 vx
int16_t vx_scaled = (int16_t)(chassisCmd.vx * 16383.5f);
canData[0] = (vx_scaled >> 8) & 0xFF;   // 高字节
canData[1] = vx_scaled & 0xFF;          // 低字节

// 编码 vy
int16_t vy_scaled = (int16_t)(chassisCmd.vy * 16383.5f);
canData[2] = (vy_scaled >> 8) & 0xFF;
canData[3] = vy_scaled & 0xFF;

// 编码 omega
int16_t omega_scaled = (int16_t)(chassisCmd.omega * 10430.4f);
canData[4] = (omega_scaled >> 8) & 0xFF;
canData[5] = omega_scaled & 0xFF;

// 编码 mode
canData[6] = (switch2 == DR16::SW_UP) ? 0x01 : 0x00;
if (mouse_left) canData[6] |= 0x02;

canData[7] = 0;  // Reserved

// 发送
canManager.transmit(txHeader, canData);
```

### **2. 底盘 MCU 接收和解码**

```cpp
// 在底盘 MCU 的 CAN 接收回调中

void chassisRxCallback(const uint8_t *rxBuffer, const uint16_t id, const uint8_t canIndex)
{
    if (id == 0x100)  // 云台命令
    {
        // 解码 vx
        int16_t vx_scaled = ((int16_t)rxBuffer[0] << 8) | rxBuffer[1];
        float vx = vx_scaled / 16383.5f;
        
        // 解码 vy
        int16_t vy_scaled = ((int16_t)rxBuffer[2] << 8) | rxBuffer[3];
        float vy = vy_scaled / 16383.5f;
        
        // 解码 omega
        int16_t omega_scaled = ((int16_t)rxBuffer[4] << 8) | rxBuffer[5];
        float omega = omega_scaled / 10430.4f;
        
        // 解码 mode
        bool special_mode = (rxBuffer[6] & 0x01) != 0;
        bool vision_mode = (rxBuffer[6] & 0x02) != 0;
        
        // 调用底盘控制器
        chassisController.setVelocity(vx, vy, omega);
        
        if (special_mode) {
            // 执行特殊模式
        }
        if (vision_mode) {
            // 启用视觉自瞄
        }
    }
}
```

---

## ⚙️ AppConfig.h 配置

确保已启用 CAN：

```cpp
/*=================*
   CAN CONFIG
 *=================*/
#define USE_CAN_MANAGER 1

#if USE_CAN_MANAGER
    #define CAN_NUM 1           // 使用 1 个 CAN (FDCAN1)
    #define CAN_FILTER_NUM 8    // 最多 8 个过滤器
#endif

/*=============*
   DR16 CONFIG
 *=============*/
#define USE_DR16 1

#if USE_DR16
    #if defined(STM32G473xx)
        #define DR16_UART huart1  // 遥控接收器 UART
    #endif
#endif
```

---

## 🧪 测试步骤

### **1. 硬件连接检查**
```
□ DR16 遥控器 → STM32G473 (huart1)
□ STM32G473 → 底盘MCU (FDCAN)
□ CAN 总线有 120Ω 终端电阻
```

### **2. 初始化检查**
```
□ DR16::init() 调用成功
□ canManager.init(&hfdcan1) 调用成功
□ rcControlTask 创建成功
```

### **3. 数据流检查**
```
□ DR16 LED 闪烁 (接收到遥控数据)
□ 调试器监控 chassisCmd 值变化
□ CAN 总线有数据发送 (监控 txCountFifoQ)
□ 底盘 MCU 接收到数据
```

### **4. 功能测试**
```
□ 推动左摇杆前后 → vx 变化
□ 推动左摇杆左右 → vy 变化
□ 推动右摇杆 → omega 变化
□ 切换开关 S1 → 速度限制变化
□ RC 断连 → 立即停止
```

---

## 📈 性能指标

```cpp
// 可以在调试器中监控

// DR16 性能
uint32_t dr16_receive_counter;      // 接收计数 (应该每10ms+1)
bool rc_connected;                  // 连接状态

// CAN 性能
volatile uint32_t txCountFifoQ;     // 发送成功计数
volatile uint8_t qLevel;            // 队列深度
volatile uint32_t busOffCount;      // 总线离线次数
```

---

## 🐛 常见问题排查

| 问题 | 原因 | 解决方案 |
|------|------|--------|
| CAN 无数据发送 | init() 未调用 | 检查 startUserTasks() |
| 数据乱码 | 字节序错误 | 检查高低字节顺序 |
| 底盘不动 | 速度值未到达 | 监控 CAN 总线 |
| 断连后不停止 | failsafe 未实现 | 检查 else 分支 |

---

## 📌 关键代码位置

```
UserTask.cpp:
├─ ChassisCommand 结构 (定义速度)
├─ rcControlTask() (读遥控, 发CAN)
└─ startUserTasks() (初始化)

AppConfig.h:
├─ USE_DR16 = 1
└─ USE_CAN_MANAGER = 1
```

完成！遥控信号现在流向底盘MCU了。
