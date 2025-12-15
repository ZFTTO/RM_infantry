# 为什么不需要直接调用 FDCANManager？- 驱动封装讲解

## 📋 核心概念

**你的理解是对的！**

```
使用层级：

你的代码 (UserTask.cpp)
    ↓
┌─────────────────────────────────┐
│ 高级驱动（已封装好的）          │
├─────────────────────────────────┤
│  ├─ DJIMotor      (DJI电机)     │
│  ├─ LKMotor       (LK电机)      │
│  ├─ InterboardComm (板间通信)   │
│  └─ ... 其他驱动                │
└────────────┬────────────────────┘
             ↓ 自动使用
┌────────────────────────────────┐
│  低级驱动层 (FDCANManager)      │
│  ├─ CAN 中断处理               │
│  ├─ 数据打包/解包              │
│  ├─ 过滤器管理                 │
│  └─ Queue 缓存                 │
└────────────┬───────────────────┘
             ↓
┌────────────────────────────────┐
│  硬件层 (FDCAN 硬件)           │
└────────────────────────────────┘

你只需要用高级驱动！
FDCANManager 的复杂性被隐藏了
```

---

## 🎯 三层驱动架构详解

### **第1层：硬件层**
```cpp
// 你看不到这层
FDCAN 硬件
├─ 物理 CAN 总线
├─ FIFO 缓冲区
└─ 中断控制器
```

### **第2层：FDCANManager（底层驱动）**
```cpp
// 一般也不需要直接调用
class CANManager {
    ├─ init()                    // 初始化
    ├─ transmit()               // 发送
    ├─ registerFilterCallback()  // 注册过滤器
    └─ ...其他低级函数
};

// 这层做的事：
// ✓ 创建 RX/TX Task
// ✓ 管理 CAN Queue
// ✓ 处理中断
// ✓ 数据格式转换
```

### **第3层：高级驱动（你直接用这个）**
```cpp
// 你直接调用这些！
namespace Core {
namespace Drivers {
    class DJIMotor {
        void setTorque(float torque);      // 设置转矩
        void setSpeed(float speed);        // 设置速度
        void setAngle(float angle);        // 设置角度
    };
    
    class LKMotor {
        void setVelocity(float vel);       // 设置速度
        void setTorque(float torque);      // 设置转矩
    };
    
    class InterboardComm {
        void send(uint8_t *data);         // 发送数据
        void receive();                    // 接收数据
    };
}}
```

---

## 💡 为什么要用高级驱动？

### **原因1：简化接口**

```cpp
// ❌ 直接用 FDCANManager（很复杂）
void controlMotor() {
    CAN_TXHEADER_T txHeader = CANManager::getTxHeader(0x200);
    uint8_t data[8];
    
    // 需要手动编码数据
    int16_t torque_scaled = (int16_t)(targetTorque * 2048.0f / 20.0f);
    data[0] = (torque_scaled >> 8) & 0xFF;
    data[1] = torque_scaled & 0xFF;
    // ... 还有很多字节要编码 ...
    
    canManager.transmit(txHeader, data);
}

// ✅ 用 DJIMotor（很简单）
void controlMotor() {
    motor.setTorque(5.0f);  // 就这么简单！
}
```

### **原因2：自动处理协议**

```cpp
// 内部发生了什么（你不需要关心）：
class DJIMotor {
    void setTorque(float torque) {
        // 1. 验证数值范围
        if (torque > 20.0f) torque = 20.0f;
        
        // 2. 转换数据格式（浮点 → 整数）
        int16_t scaled = (int16_t)(torque * 2048.0f / 20.0f);
        
        // 3. 打包成 CAN 帧
        uint8_t data[8] = {0};
        data[0] = (scaled >> 8) & 0xFF;
        data[1] = scaled & 0xFF;
        
        // 4. 创建 CAN Header
        CAN_TXHEADER_T header = 
            CANManager::getTxHeader(motorCanId);
        
        // 5. 调用 FDCANManager 发送
        canManager.transmit(header, data);  // ← 你不需要这样做！
    }
};
```

### **原因3：统一的反馈处理**

```cpp
// DJIMotor 内部会注册 CAN 回调
class DJIMotor {
public:
    DJIMotor(uint16_t canId) : motorCanId(canId) {
        // 在初始化时，自动注册回调
        CAN_FILTER_T filter = CANManager::getFilter(...);
        canManager.registerFilterCallback(filter, 
                                         &DJIMotor::feedbackCallback);
    }
    
    // 你可以直接读反馈
    uint16_t getAngle() { return angle; }
    int16_t getSpeed() { return speed; }
    int16_t getTorque() { return torque; }
    
private:
    // 回调自动被调用，更新内部变量
    static void feedbackCallback(const uint8_t *data, 
                                uint16_t id, uint8_t idx) {
        // 自动解析数据，更新 angle, speed, torque
    }
};

// 你的代码很简洁：
DJIMotor motor(0x201);

void motorTask(void *pvPara) {
    while (true) {
        // 设置转矩
        motor.setTorque(10.0f);
        
        // 读反馈
        uint16_t angle = motor.getAngle();
        int16_t speed = motor.getSpeed();
        
        vTaskDelay(10);
    }
}
```

---

## 🔧 实际使用例子

### **场景1：使用 DJI 电机控制云台**

```cpp
// ========== 高级方式（推荐）==========

#include "DJI_Motor.hpp"

// 创建两个电机对象
Core::Drivers::DJIMotor yawMotor(0x205);     // 偏航
Core::Drivers::DJIMotor pitchMotor(0x206);   // 俯仰

void gimbalTask(void *pvPara) {
    float yawTarget = 0, pitchTarget = 0;
    
    while (true) {
        // 1️⃣ 读取目标（来自遥控或其他 Task）
        const RcData& rc = DR16::getRcData();
        yawTarget += rc.mouse.x * 0.01f;
        pitchTarget += rc.mouse.y * 0.01f;
        
        // 2️⃣ 计算 PID
        float yawError = yawTarget - yawMotor.getAngle();
        float pitchError = pitchTarget - pitchMotor.getAngle();
        
        float yawOutput = pidYaw.update(yawError);
        float pitchOutput = pidPitch.update(pitchError);
        
        // 3️⃣ 发送控制命令（一行代码！）
        yawMotor.setTorque(yawOutput);
        pitchMotor.setTorque(pitchOutput);
        
        // 4️⃣ 可以直接读反馈
        uint16_t yawAngle = yawMotor.getAngle();
        int16_t yawSpeed = yawMotor.getSpeed();
        int16_t yawCurrent = yawMotor.getTorque();
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ========== 这是多少代码？==========
// 大约 30 行 ✓ 很简洁


// ========== 低级方式（不推荐）==========
// 如果直接用 FDCANManager...

void gimbalTask_BadWay(void *pvPara) {
    while (true) {
        // ... 读取目标 ...
        
        // 需要为每个电机手动编码
        {
            CAN_TXHEADER_T header1 = CANManager::getTxHeader(0x205);
            uint8_t data1[8] = {0};
            int16_t yaw_scaled = (int16_t)(yawOutput * 2048.0f / 20.0f);
            data1[0] = (yaw_scaled >> 8) & 0xFF;
            data1[1] = yaw_scaled & 0xFF;
            canManager.transmit(header1, data1);
        }
        
        // 重复同样的代码给第二个电机
        {
            CAN_TXHEADER_T header2 = CANManager::getTxHeader(0x206);
            uint8_t data2[8] = {0};
            int16_t pitch_scaled = (int16_t)(pitchOutput * 2048.0f / 20.0f);
            data2[0] = (pitch_scaled >> 8) & 0xFF;
            data2[1] = pitch_scaled & 0xFF;
            canManager.transmit(header2, data2);
        }
        
        // 还需要手动注册回调读反馈...
        // ...更多代码...
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ========== 这是多少代码？==========
// 大约 100+ 行 ✗ 很冗长
```

### **场景2：使用 LK 电机**

```cpp
// ========== 高级方式 ==========

#include "LK_Motor.hpp"

Core::Drivers::LKMotor motor1(0x141);  // LK 电机
Core::Drivers::LKMotor motor2(0x142);

void motorControlTask(void *pvPara) {
    while (true) {
        // 设置目标速度
        motor1.setVelocity(10.0f);  // 10 rad/s
        motor2.setVelocity(-10.0f);
        
        // 读反馈
        float vel1 = motor1.getVelocity();
        float torque1 = motor1.getTorque();
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
```

### **场景3：板间通信**

```cpp
// ========== 高级方式 ==========

#include "InterboardComm.hpp"

Core::Drivers::InterboardComm intercomm;

// 发送数据给底盘 MCU
void sendToChassisBoard() {
    uint8_t command[8] = {
        0x01, 0x02, 0x03, 0x04,
        0x05, 0x06, 0x07, 0x08
    };
    intercomm.send(command);  // ← 很简单
}

// 接收底盘反馈
void receiveFromChassisBoard() {
    const uint8_t *feedback = intercomm.receive();
    // 使用反馈数据...
}
```

---

## 🏗️ DJIMotor 内部结构（你不需要知道细节）

```cpp
// 这是 DJI_Motor.cpp 的简化版本

class DJIMotor {
private:
    uint16_t motorCanId;
    uint16_t angle = 0;
    int16_t speed = 0;
    int16_t current = 0;
    
    // 内部注册的回调
    static void motorFeedbackCallback(const uint8_t *data, 
                                      uint16_t id, 
                                      uint8_t canIndex) {
        // 找到对应的 motor 对象
        DJIMotor *pMotor = DJIMotor::getMotorByCanId(id);
        
        // 自动解析数据
        pMotor->angle = (data[0] << 8) | data[1];
        pMotor->speed = (int16_t)((data[2] << 8) | data[3]);
        pMotor->current = (int16_t)((data[4] << 8) | data[5]);
    }
    
public:
    DJIMotor(uint16_t canId) : motorCanId(canId) {
        // 初始化时自动设置
        // 注册 CAN 过滤器和回调
        CAN_FILTER_T filter = CANManager::getFilter(
            0x7FF, canId,
            CANManager::FilterType::MASK,
            CANManager::FilterConfig::FIFO0
        );
        CANManager::getInstance().registerFilterCallback(
            filter, motorFeedbackCallback
        );
    }
    
    void setTorque(float torque) {
        // 自动编码并发送
        // 你不需要关心细节
        CAN_TXHEADER_T header = 
            CANManager::getTxHeader(motorCanId);
        uint8_t data[8];
        // ... 编码逻辑 ...
        CANManager::getInstance().transmit(header, data);
    }
    
    uint16_t getAngle() { 
        return angle;  // 返回已解析的值
    }
};
```

---

## 📊 什么时候需要直接用 FDCANManager？

```
用高级驱动：
├─ DJI 电机 (GM6020, M2006, M3508)
├─ LK 电机 (MF7015, MG4005 等)
├─ 板间通信 (InterboardComm)
└─ 其他已封装的驱动
    ✓ 99% 的情况都在这里

直接用 FDCANManager：
├─ 自定义硬件 (协议自己定义)
├─ 第三方设备 (没有封装好的驱动)
├─ 快速原型开发 (不想写驱动)
└─ 调试目的
    ✗ 很少需要

你的项目：
├─ DJI 云台电机？ → 用 DJIMotor
├─ LK 关节电机？ → 用 LKMotor
├─ 板间通信？ → 用 InterboardComm
└─ 其他？ → 可能有现成驱动
    ✓ 不需要直接用 FDCANManager
```

---

## 🎯 实际项目代码示例

```cpp
// UserTask.cpp - 完整示例

#include "DJI_Motor.hpp"
#include "InterboardComm.hpp"

// ========== 创建电机对象（全局） ==========
Core::Drivers::DJIMotor yaw_motor(0x205);
Core::Drivers::DJIMotor pitch_motor(0x206);
Core::Drivers::InterboardComm intercomm;

// ========== 云台 Task ==========
StackType_t gimbalTaskStack[512];
StaticTask_t gimbalTaskTCB;

void gimbalTask(void *pvPara) {
    float target_yaw = 0, target_pitch = 0;
    float yaw_integral = 0, pitch_integral = 0;
    
    while (true) {
        const RcData& rc = DR16::getRcData();
        
        // 更新目标
        target_yaw += rc.mouse.x * 0.001f;
        target_pitch -= rc.mouse.y * 0.001f;
        
        // 限制范围
        if (target_yaw > 6.28f) target_yaw -= 6.28f;
        if (target_pitch > 1.57f) target_pitch = 1.57f;
        if (target_pitch < -1.57f) target_pitch = -1.57f;
        
        // 读取当前状态
        float current_yaw = yaw_motor.getAngle() * 0.001f;
        float current_pitch = pitch_motor.getAngle() * 0.001f;
        
        // 计算误差
        float yaw_error = target_yaw - current_yaw;
        float pitch_error = target_pitch - current_pitch;
        
        // PID 控制
        yaw_integral += yaw_error * 0.01f;
        pitch_integral += pitch_error * 0.01f;
        
        float yaw_output = 15.0f * yaw_error + 2.0f * yaw_integral;
        float pitch_output = 15.0f * pitch_error + 2.0f * pitch_integral;
        
        // 限制输出
        if (yaw_output > 20.0f) yaw_output = 20.0f;
        if (yaw_output < -20.0f) yaw_output = -20.0f;
        if (pitch_output > 20.0f) pitch_output = 20.0f;
        if (pitch_output < -20.0f) pitch_output = -20.0f;
        
        // 发送命令
        yaw_motor.setTorque(yaw_output);
        pitch_motor.setTorque(pitch_output);
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ========== 底盘通信 Task ==========
StackType_t chassisTaskStack[512];
StaticTask_t chassisTaskTCB;

void chassisTask(void *pvPara) {
    while (true) {
        const RcData& rc = DR16::getRcData();
        
        // 计算底盘速度
        float vx = (rc.rc.ch1 - 1024) / 330.0f * 2.0f;
        float vy = (rc.rc.ch0 - 1024) / 330.0f * 2.0f;
        float omega = (rc.rc.ch3 - 1024) / 330.0f * 3.14f;
        
        // 打包数据
        uint8_t chassis_cmd[8];
        int16_t vx_s = (int16_t)(vx * 1000);
        int16_t vy_s = (int16_t)(vy * 1000);
        int16_t omega_s = (int16_t)(omega * 1000);
        
        chassis_cmd[0] = (vx_s >> 8) & 0xFF;
        chassis_cmd[1] = vx_s & 0xFF;
        chassis_cmd[2] = (vy_s >> 8) & 0xFF;
        chassis_cmd[3] = vy_s & 0xFF;
        chassis_cmd[4] = (omega_s >> 8) & 0xFF;
        chassis_cmd[5] = omega_s & 0xFF;
        chassis_cmd[6] = 0;
        chassis_cmd[7] = 0;
        
        // 发送给底盘 MCU
        intercomm.send(chassis_cmd);
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ========== 初始化 ==========
void startUserTasks() {
    DR16::init();
    // ← 不需要 canManager.init()！
    // ← DJIMotor 和 InterboardComm 会自动初始化
    
    xTaskCreateStatic(gimbalTask, "Gimbal", 512, NULL, 3,
                      gimbalTaskStack, &gimbalTaskTCB);
    xTaskCreateStatic(chassisTask, "Chassis", 512, NULL, 3,
                      chassisTaskStack, &chassisTaskTCB);
}
```

---

## ✅ 总结

| 场景 | 用什么 | 代码量 | 复杂度 |
|------|--------|--------|--------|
| 控制 DJI 电机 | DJIMotor | ⭐ | ⭐ |
| 控制 LK 电机 | LKMotor | ⭐ | ⭐ |
| 板间通信 | InterboardComm | ⭐⭐ | ⭐⭐ |
| 自定义硬件 | FDCANManager | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |

---

## 🎯 记住这一点

```
高级驱动 = FDCANManager 的"好朋友"

        你的代码
            ↓
      高级驱动（简单）
            ↓
      FDCANManager（复杂，但隐藏了）
            ↓
          硬件

你只需要看到第一层和第二层
第三层的复杂性被完全隐藏了！
```

**结论：使用高级驱动，不需要直接调用 FDCANManager！**
