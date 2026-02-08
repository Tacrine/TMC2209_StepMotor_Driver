# FPNU IC

用于 TI G3507 和 STM32 HAL 平台开发的基于 TMC2209 的步进电机驱动，可以用于电赛。

## 使用案例

### 云台

#### 1. 预先声明云台结构体
```c
Tac_StepMotor Step_Motor_X;
```
![alt text](examples/Hal/declare.png)

#### 2. 初始化
```c
Tac_StepMotor_Init(&Step_Motor_X, &htim1, GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_5);
```
![alt text](examples/Hal/init.png)

#### 3. 控制
```c
Motor_Forward_Angle(&Step_Motor_X, 32.1, 64, 'T', 1000);
```
![alt text](examples/Hal/control.png)
