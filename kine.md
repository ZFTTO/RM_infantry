# 舵轮 (Swerve) 正/反向运动学详解

## 📋 目录
1. [基本概念](#基本概念)
2. [坐标系统](#坐标系统)
3. [反向运动学 (ikine)](#反向运动学-ikine)
4. [正向运动学 (fkine)](#正向运动学-fkine)
5. [关键算法详解](#关键算法详解)
6. [实际计算示例](#实际计算示例)

---

## 基本概念

### 什么是舵轮底盘？

```
     0       3          舵轮：每个轮子既能转向，又能驱动
     ◯═══◯             （转向角 + 转速）
     ║   ║
     ║   ║  X(前进)
     ║ O ║────→
     ║   ║
     ║   ║  Y(左)
     ◯═══◯   ↓
     1       2
```

**舵轮的特点：**
- ✅ 4 个轮子**都能转向** (±180°)
- ✅ 4 个轮子**都能独立驱动** (可以反向)
- ✅ 可以做到任意方向运动 + 原地旋转

### 两个方向的运动学

| 方向 | 中文名 | 英文名 | 功能 | 输入 | 输出 |
|------|--------|--------|------|------|------|
| **正向** | 正向运动学 | fkine | 根据马达反馈推断底盘速度 | 转向角 + 转速 | 速度 (vx, vy, vw) |
| **反向** | 反向运动学 | ikine | 根据速度目标计算马达命令 | 速度 (vx, vy, vw) | 转向角 + 转速 |

```
┌─────────────────────┐
│ 速度目标            │
│ (vx, vy, vw)        │  ←── 你想要的速度
└──────────┬──────────┘
           │
           ↓
    【反向运动学】
    ikine() 函数
           ↓
┌──────────────────────────────┐
│ 马达命令                      │
│ [转向角×4] + [转速×4]        │  ←── 告诉马达做什么
└──────────┬───────────────────┘
           │
           ↓
      [驱动马达]
           ↓
┌──────────────────────────────┐
│ 马达反馈                      │
│ [当前转向角×4] + [当前转速×4] │  ←── 马达的实际状态
└──────────┬───────────────────┘
           │
           ↓
    【正向运动学】
    fkine() 函数
           ↓
┌──────────────────────────────┐
│ 推断的底盘速度                │
│ (vx_actual, vy_actual, vw_actual) │
└──────────────────────────────┘
```

---

## 坐标系统

### 舵轮的 4 个轮子位置

```
     0 (FL)         3 (FR)        FL = Front-Left (前左)
      ◯─────────────◯             FR = Front-Right (前右)
      │             │             BL = Back-Left (后左)
      │             │  X(前)      BR = Back-Right (后右)
      │    (0,0)    │  ────→
      │    底盘中心   │
      │             │  Y(左)
      ◯─────────────◯              ↓
     1 (BL)        2 (BR)
```

### 每个轮子的配置

```cpp
struct ChassisConfig {
    float chassis_radius_[4];      // 轮子到中心的距离 [m]
                                   // [0] FL, [1] BL, [2] BR, [3] FR
    float tangent_angle_[4];       // 每个轮子相对于"向前"的角度偏移
                                   // FL: +45° (北东)
                                   // BL: -45° (南东)  
                                   // BR: +135° (南西)
                                   // FR: -135° (北西)
};

例子（对称舵轮）：
chassis_radius_[0] = 0.3;  // 所有轮子到中心距离都是 0.3m
tangent_angle_[0] = 45° = π/4
tangent_angle_[1] = -45° = -π/4
tangent_angle_[2] = 135° = 3π/4
tangent_angle_[3] = -135° = -3π/4
```

---

## 反向运动学 (ikine)

### 反向运动学做什么？

```
输入：
  vx = 2.0 m/s  (前进)
  vy = 0.0       (不横向)
  vw = 0.0       (不旋转)
  
输出：
  steering_angle[4] = {0°, 0°, 0°, 0°}  (所有轮子都指向前)
  wheel_rpm[4] = {3000, 3000, 3000, 3000}  (都以相同速度转)

结果：机器直线向前跑
```

### ikine() 代码详解

#### 第 1 步：坐标系变换

```cpp
// 参数
float _target_velocity[3];      // 要达到的速度 (vx, vy, vw)
float _gimbal_to_chassis_angle; // 云台和底盘的夹角（通常=0）

// 步骤 1：根据云台方向变换速度目标
// 如果云台朝向和底盘不一致，需要旋转坐标系
float target[3];
target[0] = cosf(_gimbal_to_chassis_angle) * _target_velocity[0] 
          - sinf(_gimbal_to_chassis_angle) * _target_velocity[1];
target[1] = sinf(_gimbal_to_chassis_angle) * _target_velocity[0] 
          + cosf(_gimbal_to_chassis_angle) * _target_velocity[1];
target[2] = _target_velocity[2];  // 旋转不影响角速度

// 简单场景（_gimbal_to_chassis_angle = 0）：
// target[0] = vx
// target[1] = vy
// target[2] = vw
```

**为什么需要这个变换？**
```
假设云台转了 45°，你摇前进杆想直线向前。
但"直线向前"对底盘来说不是 (vx, vy) = (1, 0)
而是要考虑云台的旋转角度！

                ◯ 云台朝向 45°
               ╱│
              ╱ │  
             ╱  │  如果你要相对云台向前
            ╱   │  对底盘来说就是"向前+向左"
           ╱    │  需要用这个公式转换
          ─────○ 底盘
```

#### 第 2 步：计算每个轮子的运动

```cpp
// 对每个轮子（i = 0, 1, 2, 3）
for (int i = 0; i < 4; i++)
{
    // 轮子 i 到底盘中心的向量
    float R = target[2] * chassis_radius_[i];
    // ↑ 如果底盘在旋转 (vw != 0)
    //   这个轮子需要的切向速度就是 R = vw × radius
    
    // X, Y 是轮子在底盘坐标系中"应该要有的速度"
    float Y = target[1];  // 基础速度（平移成分）
    float X = target[0];
    
    // 根据轮子的位置调整（考虑旋转）
    // 每个轮子位置不同，所以旋转产生的速度方向不同
    if (i == 0)  // 前左轮 (FL)
    {
        Y += (R * sinf(chassis_config_.tangent_angle_[i]));
        X -= (R * cosf(chassis_config_.tangent_angle_[i]));
    }
    else if (i == 1)  // 后左轮 (BL)
    {
        Y -= (R * sinf(chassis_config_.tangent_angle_[i]));
        X -= (R * cosf(chassis_config_.tangent_angle_[i]));
    }
    // ...
    
    // 现在 (X, Y) 是轮子应该有的速度向量
    
    // 计算转向角（向量指向）
    steer_output[i] = atan2f(Y, X);  // 转向角 (弧度)
    // atan2(Y, X) 得到向量 (X, Y) 的角度
    // 范围：[-π, +π]
    
    // 计算转速（向量大小）
    wheel_output[i] = sqrtf(X * X + Y * Y) * chassis_config_.velocity_to_rpm_param_;
    // velocity_to_rpm_param_ 是速度到转速的转换系数
    // 例如：如果轮子直径 0.2m，则转换系数 = 30/(π×0.2) ≈ 47.75
}
```

**本质上：**
```
对每个轮子，计算：
1. 它需要指向哪个方向？(atan2)
2. 它需要转多快？(sqrt 得到向量长度)
```

#### 第 3 步：关键！角度优化（±π 反射）

```cpp
// 问题：转向马达有限制范围 ±180°
// 如果计算出的角度需要转 170°，但还有 10° 就到反向
// 我们可以反向+反转轮子转速（效果一样）
// 这样只需转 10°，更快更省力！

angleDiff[i][0] = steer_output[i] - _steer_angle[i];  // 直接路线
// ↑ 从当前角度到目标角度需要转多少

float tempTargetAngle = steer_output[i] + (float)M_PI;  // 反向 180°
tempTargetAngle = Core::Utils::Math::wrapMinMax(tempTargetAngle, -(float)M_PI, (float)M_PI);
angleDiff[i][1] = tempTargetAngle - _steer_angle[i];  // 反向路线
// ↑ 从当前角度到反向角度需要转多少

// 选择更近的那条路
if (fabs(angleDiff[i][0]) <= fabs(angleDiff[i][1]))
{
    // 直接走
    direction_check[i] = 1;           // 轮子转速符号不变
    steer_output[i] = angleDiff[i][0]; // 角度目标
}
else
{
    // 反向走
    direction_check[i] = -1;          // 轮子转速反向
    steer_output[i] = angleDiff[i][1]; // 反向角度
}

wheel_output[i] *= direction_check[i];  // 应用反向
```

**举例：**
```
当前角度：170°
目标角度：0°

方案 A（直接）：170° → 0°，需要转 -170°（逆时针 170°）❌ 太远

方案 B（反向）：170° → 180°（反向），需要转 +10°（顺时针 10°）✅ 更近
            同时把轮子转速反向
            
结果一样：轮子最终还是往 0° 方向推动（因为反向 180° + 转速反向 = 原方向）
```

#### 第 4 步：返回结果

```cpp
return Core::Utils::Container::DoubleQuadruple<float>(steer_output, wheel_output);
// steer_output[4]  = 每个轮子的目标转向角 (弧度)
// wheel_output[4]  = 每个轮子的目标转速 (RPM)
```

### ikine() 流程图

```
输入：速度目标 (vx, vy, vw) + 当前转向角

                   ↓
         【坐标系变换】
      考虑云台和底盘夹角
           ↓
  ┌────────────────────────┐
  │ 对每个轮子 (i=0,1,2,3)  │
  └────┬───────────────────┘
       ↓
   【计算轮子速度向量】
   X = vx + 旋转补偿
   Y = vy + 旋转补偿
       ↓
   【计算转向角和转速】
   angle = atan2(Y, X)
   speed = sqrt(X²+Y²)
       ↓
   【优化：选择 ±π 反射】
   如果反向更近，就反向
       ↓
   └────────────────────┘
           ↓
输出：转向角[4] + 转速[4]
```

---

## 正向运动学 (fkine)

### 正向运动学做什么？

```
输入：
  当前转向角[4] = {0°, 0°, 0°, 0°}
  当前转速[4] = {3000 RPM, 3000 RPM, 3000 RPM, 3000 RPM}
  
输出：
  vx = 2.0 m/s
  vy = 0.0
  vw = 0.0
  
结果：推断出机器正在直线向前跑
```

### fkine() 代码详解

#### 第 1 步：分解速度

```cpp
// 参数
float _current_rpm[4];      // 每个轮子的当前转速
float _steer_angle[4];      // 每个轮子的当前转向角

// 步骤 1：把转速分解为 X, Y 方向分量
// 转向角告诉我们轮子指向哪里，转速告诉我们有多快
float wx[4], wy[4];
for (int i = 0; i < 4; i++)
{
    // RPM 沿着转向角的方向分解
    wx[i] = _current_rpm[i] * cosf(_steer_angle[i]);  // X 分量
    wy[i] = _current_rpm[i] * sinf(_steer_angle[i]);  // Y 分量
}

// 例子（转向角 = 0°，即指向前方）：
// wx[i] = RPM × 1 = RPM （全部朝 X 方向）
// wy[i] = RPM × 0 = 0   （没有 Y 分量）
```

#### 第 2 步：计算平移速度

```cpp
// 4 个轮子的平均速度 = 底盘的平移速度
chassis_output[0] = (wx[0] + wx[1] + wx[2] + wx[3]) / 4.0f 
                  / chassis_config_.velocity_to_rpm_param_;
// 取 X 方向的平均
// 除以转换系数（从 RPM 转回 m/s）

chassis_output[1] = (wy[0] + wy[1] + wy[2] + wy[3]) / 4.0f 
                  / chassis_config_.velocity_to_rpm_param_;
// 取 Y 方向的平均
```

#### 第 3 步：计算旋转速度

```cpp
// 最复杂的部分！
// 原理：轮子离中心越远，同样的转速意味着旋转越快
// 所以需要：
// 1. 计算每个轮子的"旋转贡献"
// 2. 平均化并归一化

chassis_output[2] = (
    (-wx[0] + wy[0]) / chassis_config_.chassis_radius_[0] + 
    (-wx[1] - wy[1]) / chassis_config_.chassis_radius_[1] +
    (wx[2] - wy[2]) / chassis_config_.chassis_radius_[2] + 
    (wx[3] + wy[3]) / chassis_config_.chassis_radius_[3]
) / 4.0f / SQRT2 / chassis_config_.velocity_to_rpm_param_;

// 这个公式的含义：
// (-wx[0] + wy[0]) / radius
// ↑ 轮子 0 (FL) 的"旋转速度"
// 
// 对于不同轮子位置，符号不同：
// FL (0): (-wx + wy) / r   ← 这个轮子的切向速度分量
// BL (1): (-wx - wy) / r
// BR (2): (wx - wy) / r
// FR (3): (wx + wy) / r
// 
// 然后平均，除以 SQRT2（几何归一化）
```

**为什么这个公式这么复杂？**

```
旋转速度的计算基于：速度 = 距离 × 角速度

对于舵轮，每个轮子可能有"平行移动" + "旋转"的合成速度

轮子在底盘框架中的位置：
  0       3
  ◯───────◯   如果整个底盘旋转（vw != 0）
  │       │   那么轮子 0 应该有指向"切向"的速度分量
  │   O   │   这个切向方向取决于轮子的相对位置
  │       │   
  ◯───────◯
  1       2

对于轮子 i：
  速度向量 = (wx[i], wy[i])
  相对位置到"旋转贡献"的转换需要复杂的几何
  
  简化：我们取"垂直分量"来代表"转动"贡献
  FL (0): 右上角，垂直分量是 (-wx + wy)
  BL (1): 左上角，垂直分量是 (-wx - wy)
  BR (2): 左下角，垂直分量是 (wx - wy)
  FR (3): 右下角，垂直分量是 (wx + wy)
```

#### 第 4 步：坐标系变换（反向）

```cpp
// 从底盘坐标系转回云台坐标系
result[0] = chassis_output[0] * cosf(_gimbal_to_chassis_angle) 
          + chassis_output[1] * sinf(_gimbal_to_chassis_angle);
result[1] = -chassis_output[0] * sinf(_gimbal_to_chassis_angle) 
          + chassis_output[1] * cosf(_gimbal_to_chassis_angle);
result[2] = chassis_output[2];  // 角速度不变

// 逆变换回去，得到云台看到的速度
```

### fkine() 流程图

```
输入：转向角[4] + 转速[4]

              ↓
   【分解为 X, Y 分量】
   wx[i] = RPM × cos(angle)
   wy[i] = RPM × sin(angle)
              ↓
   【计算平移速度】
   vx = 平均(wx) / 转换系数
   vy = 平均(wy) / 转换系数
              ↓
   【计算旋转速度】（最复杂）
   vw = 各轮子垂直分量的平均 / 转换系数
              ↓
   【坐标系逆变换】
              ↓
输出：底盘速度 (vx, vy, vw)
```

---

## 关键算法详解

### 算法 1：角度优化（±π 反射）

**问题：为什么需要？**

舵轮转向马达的转速有限制。如果直接转 170° 需要 2 秒，
但反向转 10° 只需 0.1 秒，显然应该选反向！

```cpp
// 计算两条路线
angleDiff[0] = target_angle - current_angle;  // 直接
angleDiff[1] = (target_angle + π) - current_angle;  // 反向 180°

// 规范化到 [-π, +π]
angleDiff[0] = wrapMinMax(angleDiff[0], -π, +π);
angleDiff[1] = wrapMinMax(angleDiff[1], -π, +π);

// 选择更近的（角度差更小的）
if (|angleDiff[0]| <= |angleDiff[1]|) {
    use direct path, wheel speed stays same
} else {
    use reverse path, wheel speed reverses
}
```

### 算法 2：向量分解

**ikine：**
```
速度目标 (vx, vy, vw)
    ↓
对每个轮子计算它应该有的速度向量 (X, Y)
    ↓
把 (X, Y) 转换为 (角度, 大小)
```

**fkine：**
```
每个轮子的 (角度, 转速)
    ↓
把 (角度, 转速) 转换为 (X, Y)
    ↓
平均得到底盘速度 (vx, vy, vw)
```

---

## 实际计算示例

### 场景：机器直线向前

**目标：** vx = 2.0 m/s, vy = 0, vw = 0

#### ikine 计算过程

```
步骤 1：坐标变换（gimbal_angle = 0）
target[0] = vx = 2.0
target[1] = vy = 0
target[2] = vw = 0

步骤 2：对轮子 0 (FL)
R = vw × radius = 0 × 0.3 = 0
X = 2.0 - 0 = 2.0
Y = 0 + 0 = 0
angle = atan2(0, 2.0) = 0°
speed = sqrt(4 + 0) = 2.0 m/s

步骤 3：角度优化
current_angle = 0° (假设之前是这个)
direct: 0° - 0° = 0° ✓ 不用转
reverse: (0°+180°) - 0° = 180° ✗ 需要转很多
→ 选择 direct，direction = 1

步骤 4：所有轮子都相同（对称）
steering_angle[0,1,2,3] = 0°
wheel_rpm[0,1,2,3] = 2.0 * 47.75 ≈ 955 RPM

结果：所有轮子都指向前，以相同速度转
机器直线向前 ✓
```

#### fkine 反馈过程

```
输入：
steering_angle[0,1,2,3] = 0°
wheel_rpm[0,1,2,3] = 955 RPM

步骤 1：分解速度
wx[i] = 955 × cos(0°) = 955
wy[i] = 955 × sin(0°) = 0

步骤 2：平移速度
vx = (955+955+955+955) / 4 / 47.75 = 955 / 47.75 = 2.0 m/s ✓
vy = (0+0+0+0) / 4 / 47.75 = 0 ✓

步骤 3：旋转速度
分子 = (-955+0) + (-955-0) + (955-0) + (955+0) = 0
vw = 0 / (4 × √2 × 47.75) = 0 ✓

结果：推断出机器以 vx=2.0 m/s 直线向前
与目标一致 ✓
```

### 场景：机器原地旋转

**目标：** vx = 0, vy = 0, vw = 3.14 rad/s

#### ikine 计算过程

```
步骤 1：坐标变换
target[0] = 0
target[1] = 0
target[2] = 3.14

步骤 2：对轮子 0 (FL, radius=0.3)
R = 3.14 × 0.3 ≈ 0.94
X = 0 - 0.94×cos(45°) = -0.94 × 0.707 = -0.67
Y = 0 + 0.94×sin(45°) = 0.94 × 0.707 = 0.67
angle = atan2(0.67, -0.67) = 135°
speed = sqrt(0.67² + 0.67²) = 0.95 m/s

对轮子 1 (BL, radius=0.3)
R = 0.94
X = 0 - 0.94×cos(-45°) = -0.94 × 0.707 = -0.67
Y = 0 - 0.94×sin(-45°) = -0.94 × (-0.707) = 0.67
angle = atan2(0.67, -0.67) = 135°
speed = 0.95 m/s

... (对称分布)

结果：
steering_angle = {135°, 135°, 135°, 135°}
所有轮子指向外侧
wheel_rpm = 相同速度

机器原地旋转（所有轮子都朝"切向"推动）✓
```

---

## 总结

| 概念 | 说明 |
|------|------|
| **反向运动学 (ikine)** | 速度目标 → 转向角 + 转速 |
| **正向运动学 (fkine)** | 转向角 + 转速 → 推断速度 |
| **坐标变换** | 考虑云台和底盘夹角 |
| **向量分解** | (X, Y) ↔ (角度, 大小) |
| **角度优化** | 选择最近的路线（直接 vs 反向） |
| **旋转补偿** | 添加旋转产生的速度分量 |

### 代码框架

```cpp
// 在底盘控制任务中使用
void chassisControl() {
    // 接收速度目标
    float vx = 2.0, vy = 0, vw = 0;
    
    // 反向运动学：速度目标 → 马达命令
    auto result = calculator.ikine({vx, vy, vw}, current_angles, gimbal_offset);
    auto target_angles = result.first;   // 目标转向角
    auto target_rpms = result.second;    // 目标转速
    
    // 驱动马达...
    
    // 读马达反馈
    float current_angles[4] = {...};
    float current_rpms[4] = {...};
    
    // 正向运动学：马达反馈 → 推断速度
    auto actual_velocity = calculator.fkine(current_rpms, current_angles, gimbal_offset);
    // actual_velocity = {vx_actual, vy_actual, vw_actual}
    
    // 用于调试或高级控制算法...
}
```

现在你应该理解舵轮的正/反向运动学了！有具体问题继续问。
