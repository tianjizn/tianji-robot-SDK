# FX Robot SDK - 人形机器人软件开发工具包

## 版本管理与兼容性

控制器版本和SDK版本由 **MAJOR_VERSION**、**MINOR_VERSION**、**PATCH_VERSION** 三部分组成。

- **MAJOR** 和 **MINOR** 一致情况下，SDK 可以和控制器连接
- SDK 和控制器版本不兼容时，连接会报错误 `-4: "Version incompatible"`

获取SDK版本是一个非建立连接的操作：

| 接口 | C 接口 | Python 接口 |
|------|--------|-------------|
| 获取 SDK 版本 | `int FX_L1_System_GetSDKVersion();` | `get_sdk_version()` |

当连接错误为 `-4` 时，可先调用获取SDK版本接口，再根据SDK版本对控制器进行升级或降级。
---

## 1. 机器人简介

FX 人形机器人是面向科研、工业协作和服务应用的先进双臂多自由度机器人系统。产品线包括：

- **Marvin Pro M3 / M6** — 通用人形平台
- **Gento Skye / Gento Luna** — 双臂协作人形机器人

每台机器人具备以下能力：

- 7 自由度机械臂（左右各一）
- 头部、躯干和升降模块
- 高带宽实时控制（1 kHz 反馈）
- 力/力矩传感、阻抗控制、拖拽示教
- CAN FD / RS485 外设通信通道

系统专为安全人机协作、动态运动规划和基于 UDP 网络的低延迟控制而设计。


## 2. SDK 概述

FX Robot SDK 提供了一套高层 C API，用于控制、监控和编程 FX 人形机器人。它封装了底层通信（L0），为系统管理、运动控制、状态切换、参数配置、运动学和轨迹规划提供了直观的接口， SDK另外提供了机器人运动学、运动规划、碰撞检测、工具动力学参数辨识等功能。

### 2.1 核心功能

| 模块 | 说明 |
|------|------|
| 系统管理 | 连接/断开机器人控制器、设置日志级别、重启、固件升级、文件传输 |
| 状态机 | 在位置、阻抗（关节/笛卡尔/力）、拖拽示教（关节/X/Y/Z/R）和协作释放模式之间切换 |
| 实时反馈 | 获取 1 kHz 实时数据（关节位置、速度、力矩、IMU、F/T 传感器）和 500 Hz 慢组数据（配置、诊断） |
| 参数管理 | 按名称读写整型、浮点型和字符串型参数 |
| 终端通信 | 通过 CAN FD 或 RS485 与外部设备收发数据 |
| 硬件配置 | 刹车锁定/解锁、编码器偏移复位、软件限位禁用 |
| 运行时运动 | 急停、关节位置指令、力/力矩控制、缩放比例、刚度/阻尼调节 |
| 运动学与规划 | 正/逆运动学、雅可比矩阵、工具变换、机身运动学（Skye，Luna）、MoveJ/MoveL 规划、多段笛卡尔规划、双臂同步规划 |
| 动力学辨识 | 采集→辨识解耦流程：`FX_ToolDyn_Start_Sampling` 采样、`FX_ToolDyn_Run_Load_Identification` 辨识负载质量/质心/惯量 |
| 灵巧手数据 | `FX_L1_Hand_GetData/SetData`（按手读写 512 字节数据，通过 `FXObjType` 指定手） |
| 用户反馈通道 | `FX_L1_System_SetUserFbkType` 配置 4 路用户自定义反馈源（如 FFD 力矩指令），数据在 `ROBOT_RT.m_SYSTEM` 中 |



### 2.2 核心类型与结构体

- **FXObjType** — 标识机器人部件：左臂（`FX_OBJ_ARM0`）、右臂（`FX_OBJ_ARM1`）、头部、躯干、升降台。
- **FXStateType** — 高层控制状态：空闲、位置、关节/笛卡尔/力阻抗、拖拽模式、释放、错误。
- **ROBOT_RT** — 实时反馈（关节位置/速度/力矩、IMU、F/T 传感器、用户反馈通道 `SYSTEM_RT`）。
- **ROBOT_SG** — 慢组配置和扩展反馈。
- **SYSTEM_RT** — 用户自定义反馈通道（4 路 × 8 浮点，通过 `FX_L1_System_SetUserFbkType` 配置源）。
- **LoadDynamicPara** — 工具动力学辨识结果（质量 `m`、质心 `r[3]`、惯量 `I[6]`）。
- **ToolDynTaskStatus** — 辨识任务状态（`TASK_IDLE/SAMPLING/IDENTIFYING/DONE/ERROR/STOPPED`）。
- **FXUserFbkType** — 用户反馈通道源类型枚举（如 `FX_USER_FBK_ARM0_CMD_FFDTOR`）。
- **FXFuncReturn** — 标准化返回值（0 = 成功，负值表示具体错误）。

### 2.3 核心控制模式介绍
- 位置模式（Position Mode）

  基于伺服闭环控制，机器人各关节按照预设的目标位置、速度和加速度轨迹运动，实现高精度的点到点或连续路径跟踪。该模式适用于对轨迹重复性要求高的场景（如搬运、涂胶、焊接），不对外部接触力做主动调节，位置偏差主要由刚度决定。

    切换代码示例：

    C/C++(省略连接仅展示切换代码)：
    ```c
    FXObjType ctrl_obj=FX_OBJ_ARM0;
    double vel_ratio = 10.0;             
    double acc_ratio = 10.0;          
    if (FX_L1_State_SwitchToPositionMode(ctrl_obj, 2000, vel_ratio, acc_ratio) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_POSITION state\n");
        return -1;
    }
    ```
    python(省略连接仅展示切换代码)：
    ```python
    target_state="Position"
    ctrl_obj=FXObjType.OBJ_ARM0
    vel=10
    acc=10
    ret=robot.switch_to_position_mode(ctrl_obj,2000,vel,acc)
    if ret !=0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    ```


- 关节阻抗模式（Joint Impedance Mode）

    特别注意：在关节阻抗模式下，为了限位安全，各个关节会在ini配置范围内正负限位各回缩1.5度。

    在关节空间内建立力矩与位置偏差的动态关系，表现为“弹簧-阻尼”特性。

    阻抗参数：

        每个关节的刚度（范围0~22， 单位N*m/deg），刚度越高关节“越硬”

        每个关节的阻尼系数（范围0~1，建议值0.3）

    阻尼越大，物体振幅减小越快，但对力、位移的响应迟缓，运动时感觉阻力大，有粘滞感； 阻尼越小，减震效果减弱，但运动阻力小，更流畅，停止到位置时有余震感。

    关节阻抗下阻尼解释：，关节阻抗下阻尼为在模态空间中计算的阻尼，可以看成关节空间中的2阶系统的阻尼响应  输入1就为临界阻尼，大于1为过阻尼，小于1为欠阻尼系统，这个针对阶跃响应分析，连续系统，欠阻尼就可以保证一定的稳定性

    该模式适用于需要关节级柔顺性的装配、抛光和避障任务，可吸收冲击并适应不规则表面。

    
    切换代码示例：

    C/C++(省略连接仅展示切换代码)：
    ```c
    FXObjType ctrl_obj=FX_OBJ_ARM0;
    double vel_ratio = 10.0;             
    double acc_ratio = 10.0;    
    double k[7] = { 3, 3, 3, 2, 1, 1, 1 };
    double d[7] = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 };      
   if (FX_L1_State_SwitchToImpJointMode(
            FX_OBJ_ARM0,
            2000,
            vel_ratio,
            acc_ratio,
            k,
            d) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_IMP_JOINT state\n");
        return -1;
    }
    ```
    python(省略连接仅展示切换代码)：
    ```python
    target_state="ImpJoint"
    ctrl_obj=FXObjType.OBJ_ARM0
    vel=10
    acc=10
    k=[3, 3, 3, 2, 1, 1, 1]
    d=[0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
    ret=robot.switch_to_imp_joint_mode(ctrl_obj,2000,vel,acc,k,d)
    if ret !=0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    ```

    

- 笛卡尔阻抗模式（Cartesian Impedance Mode）

    在末端笛卡尔空间（X/Y/Z方向及旋转轴以及零空间）构建柔顺控制模型，使末端对外力呈现可调的刚度和阻尼特性。

    阻抗参数：

        平移刚度（范围 0~1200 N*m）和阻尼（范围0~1，建议0.3）；

        旋转刚度（范围 0~600 N*m/rad）和阻尼（范围0~1，建议0.3 ）

        零空间总和刚度参数（范围20~100 N*m/rad）和零空间总和阻尼系数（范围0~1，建议0.3）。

    笛卡尔阻抗下阻尼解释：笛卡尔阻抗下阻尼为在笛卡尔模态空间中计算的阻尼，可以看成笛卡尔节空间中的2阶系统的阻尼响应  输入1就为临界阻尼，大于1为过阻尼，小于1为欠阻尼系统，这个针对阶跃响应分析，连续系统，欠阻尼就可以保证一定的稳定性 

    该模式适用于末端与外部环境交互（如打磨、去毛刺、力控装配），可在保持轨迹精度的同时主动顺应外力，提高接触安全性。

    
    切换代码示例：

    C/C++(省略连接仅展示切换代码)：
    ```c

    FXObjType ctrl_obj=FX_OBJ_ARM0;
    double vel_ratio = 10.0;             
    double acc_ratio = 10.0;  
    double cart_k[7] = { 2000, 2000, 2000, 100, 100, 100, 50 };
    double cart_d[7] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0 };  
    if (FX_L1_State_SwitchToImpCartMode(
            FX_OBJ_ARM0,
            1000,
            FX_REFORI_TYPE_ANYBASE, 
            NULL,
            vel_ratio,
            acc_ratio,
            cart_k,
            cart_d) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_IMP_CART state\n");
        return -1;
    }
    ```
    python(省略连接仅展示切换代码)：
    ```python
    target_state="ImpCart"
    ctrl_obj=FXObjType.OBJ_ARM0
    vel=10
    acc=10
    k=[3000.0,3000.0,3000.0,300.0,300.0,300.0,50.0]
    d=[0.2,0.2,0.2,0.2,0.2,0.2,0.11] 
    ori_type=FXRefOriType.FX_REFORI_TYPE_ANYBASE
    default_ori=[0,0,0]
    ret=robot.switch_to_imp_cart_mode(ctrl_obj,2000,ori_type,default_ori,vel,acc,k,d)
    if ret !=0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    ```
    

-  力控阻抗模式（Force-controlled Impedance Mode）

    在指定笛卡尔方向（X、Y、Z 三轴）上直接以期望接触力为目标进行闭环控制，同时保留阻抗柔顺特性。

    力控参数：

        力控方向，可指定单方向或者复合方向；

        力控制范围 0~50 N；

        力作用距离（即允许的位移偏差窗口）为 -50 mm ~ +50 mm；

    该模式适用于恒定力跟踪应用，通过调整参数适应接触表面起伏，确保接触力稳定可控。

        
    切换代码示例：

    C/C++(省略连接仅展示切换代码)：
    ```c
    FXObjType ctrl_obj=FX_OBJ_ARM0;
    double vel_ratio = 10.0;             
    double acc_ratio = 10.0;  
    double force_ctrl[5] = { 0,1,0,25,25};
    double torque_ctrl[5] = { 0,1,0,5,10};  
    if (FX_L1_State_SwitchToImpForceMode(
            FX_OBJ_ARM0,
            1000,
            FX_REFORI_TYPE_ANYBASE, 
            NULL,
            force_ctrl,
            torque_ctrl) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_IMP_FORCE state\n");
        return -1;
    }
    ```
    python(省略连接仅展示切换代码)：
    ```python
    target_state="ImpForce"
    ctrl_obj=FXObjType.OBJ_ARM0
    force_ctrl=[0,1,0,25,25]
    torque_ctrl=[0,1,0,5,10] 
    ori_type=FXRefOriType.FX_REFORI_TYPE_ANYBASE
    default_ori=[0,0,0]
    ret=robot.switch_to_imp_force_mode(ctrl_obj,2000,ori_type,default_ori,force_ctrl,torque_ctrl)
    if ret !=0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    ```
    


- PD前馈模式

    PD模式时一种极低延时跟踪模式兼具关节阻抗模式的柔顺性能，主要适用于遥操场景。

    用户的关节轨迹每个关节速度不能超过180度/秒

    开启条件:

        - 配置参数robot.ini文件中 JointPIDCtlType=1; 

        - 参数设置：

            阻抗参数：

                刚度参数（N*m/deg）, 最大[20, 20, 20, 15, 8, 8, 8],  最小[2, 2, 2, 1.5, 0.8, 0.8, 0.8], 常用[14, 14, 14, 10.5, 5.6, 5.6, 5.6]        

                阻尼系数 [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]

            设置速度和加速度为100，以免限制轨迹


        - 发送轨迹前，通过FX_L1_Config_SetPDCmdCycleTime设置前馈控制周期时间

    PD模式下阻尼解释：这里阻尼为一个实际阻尼倍率放缩，和最大扭矩和最大速度有关系。实际阻尼可以看成按照如下公式计算：D=1.5*(1+useD)*(Torque/Velmax)，useD为用户给定的阻尼参数，D实际计算阻尼，最后会乘速度误差
    切换代码示例：

    C/C++(省略连接仅展示切换代码)：
    ```c
    FXObjType ctrl_obj=FX_OBJ_ARM0;
    int pd_cycle_time = 0; 
    double vel_ratio = 10.0;             
    double acc_ratio = 10.0;    
    double k[7] = { 3, 3, 3, 2, 1, 1, 1 };
    double d[7] = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 };      

    if (FX_L1_Config_SetPDCmdCycleTime(pd_cycle_time) != FUNC_RET_SUCCESS) {
        printf("Failed to set PD command cycle time\n");
        return -1;
    }

    if (FX_L1_State_SwitchToPDMode(
        FX_OBJ_ARM0,
        2000,
        vel_ratio,
        acc_ratio,
        k,
        d) != FUNC_RET_SUCCESS) {
        printf("Failed to switch arm0 to STATE_PD\n");
        return -1;
    }
    ```
    python(省略连接仅展示切换代码)：
    ```python
    target_state = "PD"
    ctrl_obj=FXObjType.OBJ_ARM0
    pd_cycle_time = 0
    vel=10
    acc=10
    k=[3, 3, 3, 2, 1, 1, 1]
    d=[0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]

    ret = robot.config_set_pd_cmd_cycle_time(pd_cycle_time)
    if ret != 0:
        print(f"Set PD command cycle time failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    ret = robot.switch_to_pd_mode(ctrl_obj, 2000, vel_ratio, acc_ratio, k, d)
    if ret != 0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    ```




- 协作释放模式（Collaborative Release Mode）

    专为人机协作安全设计的安全响应模式。当检测到碰撞或外力超过阈值时，机器人立即停止运动并主动释放所有关节制动力矩，使各轴处于“零力漂浮”状态，从而最大限度降低碰撞冲击能量。该模式也可由操作员手动触发，用于紧急脱离或手动拖拽示教，恢复后需重新上使能方可继续运行。
    
        C/C++(省略连接仅展示切换代码)：
    ```c
    FXObjType ctrl_obj=FX_OBJ_ARM0;
   if (FX_L1_State_SwitchToCollaborativeRelease(
            FX_OBJ_ARM0,
            2000) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_CR state\n");
        return -1;
    }
    ```
    python(省略连接仅展示切换代码)：
    ```python
    target_state = "CR"
    ctrl_obj=FXObjType.OBJ_ARM0
    ret=robot.switch_to_collab_release(ctrl_obj,2000)
    if ret !=0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    ```


- 笛卡尔拖动模式（Cartesian drag Mode）

    在基于基座的笛卡尔空间（X/Y/Z方向及旋转轴的四种拖动）。

    需要先设置笛卡尔阻抗的参数：

        平移刚度（范围 0~1200 N*m）和阻尼（范围0~1，建议0.3）

        旋转刚度（范围 0~600 N*m/rad）和阻尼（范围0~1，建议0.3 ）

        零空间总和刚度参数（范围20~100 N*m/rad）和零空间总和阻尼系数（范围0~1，建议0.3）

    该模式适用于拖动示教。
    
    
    切换代码示例：

    C/C++(省略连接仅展示切换代码)：
    ```c

    FXObjType ctrl_obj=FX_OBJ_ARM0;
    double vel_ratio = 10.0;             
    double acc_ratio = 10.0;  
    double cart_k[7] = { 2000, 2000, 2000, 100, 100, 100, 50 };
    double cart_d[7] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0 };  
    if (FX_L1_State_SwitchToDragCartX(FX_OBJ_ARM0, 2000, FX_REFORI_TYPE_ANYBASE, NULL, cart_k, cart_d) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_DRAG_CART state\n");
        return -1;
    }
    ```
    python(省略连接仅展示切换代码)：
    ```python
    ctrl_obj=FXObjType.OBJ_ARM0
    vel=10
    acc=10
    k=[3000.0,3000.0,3000.0,300.0,300.0,300.0,50.0]
    d=[0.2,0.2,0.2,0.2,0.2,0.2,0.11] 
    ori_type=FXRefOriType.FX_REFORI_TYPE_ANYBASE
    default_ori=[0,0,0]
    target_state="DragCartX"
    ret=robot.switch_to_drag_cart_x(ctrl_obj,2000,ori_type,default_ori,k,d)
    if ret !=0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    ```


    

- 关节拖动模式（Joints drag Mode）

    在关节空间拖动手臂。
    
    需要先设置关节阻抗的参数：

         每个关节的刚度（范围0~22， 单位N*m/deg），刚度越高关节“越硬”

        每个关节的阻尼系数（范围0~1，建议值0.3）

    该模式适用于拖动示教。

    
    切换代码示例：

    C/C++(省略连接仅展示切换代码)：
    ```c
    FXObjType ctrl_obj=FX_OBJ_ARM0;
    double vel_ratio = 10.0;             
    double acc_ratio = 10.0;    
    double k[7] = { 3, 3, 3, 2, 1, 1, 1 };
    double d[7] = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 };      
    if (FX_L1_State_SwitchToDragJoint(FX_OBJ_ARM0, 2000, k, d) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_DRAG_JOINT state\n");
        return -1;
    }
    ```
    python(省略连接仅展示切换代码)：
    ```python
    ctrl_obj=FXObjType.OBJ_ARM0
    vel=10
    acc=10
    k=[3, 3, 3, 2, 1, 1, 1]
    d=[0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
    target_state="DragJoint"
    ret=robot.switch_to_drag_joint(ctrl_obj,2000,k,d)
    if ret !=0:
        print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return
    ```


### 2.4 API 命名规范

所有 L1 函数以 `FX_L1_` 为前缀，后跟模块名和操作：

- `FX_L1_System_*` — 系统级操作
- `FX_L1_State_*` — 状态机切换
- `FX_L1_Fbk_*` — 反馈查询
- `FX_L1_Runtime_*` — 实时运动指令
- `FX_L1_Kinematics_*` — 运动学和运动规划
- `FX_L1_Param_*`、`FX_L1_Config_*`、`FX_L1_Terminal_*`
- `FX_ToolDyn_*` — 工具动力学辨识（采集/辨识），使用独立导出宏 `TOOLDYN_SDK_API`

## 3. SDK 更新历史

| 版本    | 日期         | 说明 |
|-------|------------|------|
| 4.7.1 | 2026-09-09 | 增加SDO的读写接口
| 4.7.0 | 2026-08-20 | 增加FXUserFbkType
| 4.6.1 | 2026-08-17 | 修复 OnMovL_ZSP ,添加边界保护机制
| 4.6.0 | 2026-08-13 | 修复 FX_L1_Config_DisableSoftLimit 必须在位置模式下修改
| 4.6.0 | 2026-08-03 | 新增工具动力学辨识接口 FX_ToolDyn_*  前缀，导出宏改为 TOOLDYN_SDK_API
| 4.6.0 | 2026-08-01 | 新增用户反馈通道 FX_L1_System_SetUserFbkType，ROBOT_RT 增加 SYSTEM_RT；灵巧手数据接口改为 FX_L1_Hand_GetData/SetData（按手 512 字节）；末端通讯参数 FXTerminalType → FXObjType； 增加 Luna 机身运动学 FX_L1_Kinematics_LunaBody*
| 4.6.0 | 2026-07-30 | 增加动力学辨识接口C_SDK\ToolDynaIdent
| 4.6.0 | 2026-07-22 | 增加接口FX_L1_Runtime_SetJointMITCmd；修改接口FX_L1_State_SwitchToImpCartMode，FX_L1_State_SwitchToImpForceMode，FX_L1_State_SwitchToDragCart*；修改ROBOT_RT数据结构
| 4.5.0 | 2026-07-09 | 修改基座IMU数据为单独RT
| 4.5.0 | 2026-07-06 | 反馈数据增加手臂/身体/头部的温度反馈；碰撞检测Interf变更为C函数接口
| 4.4.2 | 2026-06-29 | 增加接口：FX_L1_System_SetSystemTime,FX_L1_System_GetSystemTime,FX_L1_System_SetSystemIP,FX_L1_System_GetSystemIP,FX_L1_Fbk_ResetSystemMsg,FX_L1_Fbk_GetSystemMsg； 修改接口：FX_L1_Fbk_GetCtrlObjServoVersion
| 4.4.2 | 2026-06-16 | 增加用户数据采集接口：FX_L1_Fbk_GetUserData，FX_L1_Fbk_ResetUserDataSet，FX_L1_Fbk_RegisterUserDataSet，FX_L1_Fbk_CheckUserDataSet
| 4.4.2 | 2026-06-12 | FX_L1_Runtime_*接口支持多线程安全，最多7个线程同时调用|
| 4.4.1 | 2026-06-05 | 修复接口：FX_L1_Runtime_StopTraj
| 4.4.0 | 2026-05-29 | 增加连接状态接口：FX_L1_System_GetLinkState
| 4.3.0 | 2026-05-28 | 增加PD控制接口：FX_L1_Config_SetPDCmdCycleTime，FX_L1_Config_GetPDCmdCycleTime，FX_L1_State_SwitchToPDMode，FX_L1_Runtime_SetJointPosPDCmd；增加数据打标接口：FX_L1_Runtime_SetTag；增加灵巧手控制接口：FX_L1_Runtime_SetHandAction，FX_L1_Runtime_SetHandPos，FX_L1_Runtime_SetHandP，FX_L1_Runtime_SetHandD，FX_L1_Runtime_SetHandMaxTor
| 4.2.1 | 2026-05    | 新增 FX_L1_Kinematics_PlanLinearMove_MultiPoints_* 多段笛卡尔规划 API；改进逆运动学稳定性；修复 UDP 连接超时处理
| 4.2.0 | 2026-05    | 引入 Skye 机身运动学`FX_L1_Kinematics_SkyeBody*`；新增双臂同步规划FX_L1_Kinematics_ArmsSynchronousPlanning；新错误码 FUNC_RET_KINE_PLAN_JOINT_LIMIT、FUNC_RET_KINE_SYNC_POINT_MISMATCH
| 4.1.0 | 2026-04    | 新增力/力矩控制运行时 API: FX_L1_Runtime_SetForceCtrl、SetTorqueCtrl；新状态 FX_STATE_IMP_FORCE
| 4.0.0 | 2026-04    | 重大重构：统一 FX_MotionHandle  运动学接口；支持 Marvin Pro M6 和 Gento Luna；改进日志位掩码FX_LOG_*_FLAG|



## 4. 项目结构

```
GENTO_SDK/
├── C_SDK/                          # SDK 源码
│   ├── Common/                     # 公共头文件（FXCommon.h、FXErrorCode.h）
│   ├── FileClient/                 # 文件传输客户端
|   |── FXConfig/                   # 配置文件
│   │   ├── InterferenceCfg/        # 接口文件
│   │   ├── ToolDynIdentData/       # 工具动力学辨识配置文件
│   │   ├── URDF/                   # URDF 
|   |── FXUtility/                  # 公用功能，跨层
│   │   ├── FXCfg/                  # 解析文件库
│   │   ├── FXMath/                 # 公共数学库
|   |── Interf/                     # 公用功能，碰撞检测
│   ├── Kinematics/                 # 运动学子模块
│   │   ├── ArmKinematics/          # 机械臂运动学
│   │   ├── KineCommon/             # 运动学通用类型
│   │   ├── LunaBodyKinematics/     # Luna 机身运动学
│   │   ├── MotionPlanner/          # 轨迹规划器
│   │   └── SkyeBodyKinematics/     # Skye 机身运动学
│   ├── L0Control/                  # 底层通信与控制
│   ├── L1Robot/                    # 上层 API
│   └── ToolDynaIdent/              # 工具动力学参数辨识
├── FX_PLATFORM/                    # 上位机 UI 源码（Tkinter）
│   ├── setup.py                    # 软件打包脚本
│   ├── UI.py                       # 上位机源码
├── C_EXAMPLE/                      # C++ 示例 — 直接调用 C_SDK 源码
│   ├── build_windows.bat           # Windows 示例编译脚本
│   └── build_linux.sh              # Linux 示例编译脚本
├── C_EXAMPLE_USE_DLL_SO/           # C++ 示例 — 调用编译后的 DLL/SO 库
│   ├── build_windows_dll.bat       # Windows 示例编译脚本
│   └── build_linux_so.sh           # Linux 示例编译脚本
├── PYTHON_SDK/                     # Python 封装层（基于 DLL/SO）
│   └── GentoRobot.py               # Python 主入口类
├── PYTHON_EXAMPLE/                 # Python 示例程序
|── FXPlatform                      # linux上位机，ubuntu20.04及以上可使用
|── FXPlatform.exe                  # windows上位机
├── win_auto_compile.bat            # Windows 一键编译脚本（源码 → DLL）
├── linux_auto_compile.sh           # Linux 一键编译脚本（源码 → SO）
├── linux_cross_compile_arm.sh      # Linux 交叉编译脚本（x86_64 → ARM）
├── dist_aarch64/                    # Linux 交叉编译的库，如果是arrch64的机器可直接使用。
└── README.md                       # SDK文档
```

## 5. 快速开始

### 5.1 环境要求

- **操作系统**：Linux（Ubuntu 20.04(glibc=2.31) 及以上）或 Windows 10/11
- **编译器**：GCC 9+（Linux）、MinGW g++ / MSYS2（Windows）
- **PYTHON版本**： python>=3.10
- **网络**：以太网连接机器人控制器（建议使用静态 IP）
- **机器人控制器 IP**：默认 6.6.7.190（请以实际机器人文档为准）

### 5.2 基本使用示例

**C 语言**

```c
#include <stdio.h>
#include "L1Robot.h"

int main()
{
    /* 获取 SDK 版本 */
    int sdk_version = FX_L1_System_GetSDKVersion();
    printf("SDK version: 0x%08x\n", sdk_version);

    /* 连接机器人控制器（IP: 6.6.7.190，全日志级别） */
    int ret = FX_L1_System_Link(6, 6, 7, 100, FX_LOG_ALL_FLAG);
    if (ret < 0)
    {
        printf("连接失败，错误码: %d\n", ret);
        return -1;
    }
    printf("连接成功，延迟: %d ms\n", ret);

    /* 获取控制器版本 */
    int ctrl_version = FX_L1_System_GetControllerVersion();
    printf("控制器版本: 0x%08x\n", ctrl_version);

    /* 断开连接 */
    FX_L1_System_Unlink();
    printf("已断开连接。\n");
    return 0;
}
```

**Python**

```python
from GentoRobot import GentoRobot, FXLogMask

# 创建机器人实例
robot = GentoRobot()

# 获取 SDK 版本
print(f"SDK 版本: {robot.get_sdk_version()}")

# 连接机器人控制器
ret = robot.link(6, 6, 7, 100, FXLogMask.FX_LOG_ALL_FLAG)
if ret < 0:
    print(f"连接失败，错误码: {ret}")
    exit(-1)
print(f"连接成功，延迟: {ret} ms")

# 获取控制器版本
print(f"控制器版本: {robot.get_controller_version()}")

# 断开连接
robot.unlink()
print("已断开连接。")
```

更多示例请参考 [C_EXAMPLE/](C_EXAMPLE/) 和 [PYTHON_EXAMPLE/](PYTHON_EXAMPLE/) 目录。

### 5.3 构建方式

SDK 支持两种使用方式：

**方式一：直接调用源码（不编译库）**

Windows：
```bash
# 例如： `C_EXAMPLE/example_basic_LinkSystem.cpp` 与 `C_SDK/` 下的所有 `.cpp` 文件一起编译。
# 第一步 编译
./C_EXAMPLE/build_windows.bat example_basic_LinkSystem.cpp

# 第二步 执行
./C_EXAMPLE/example_basic_LinkSystem.exe


# 编译全部
./C_EXAMPLE/build_windows.bat all                #-> builds every example_*.cpp
```

Linux：
```bash
# 例如： `C_EXAMPLE/example_basic_LinkSystem.cpp` 与 `C_SDK/` 下的所有 `.cpp` 文件一起编译。
# 第一步 编译
./C_EXAMPLE/build_linux.sh example_basic_LinkSystem.cpp

# 第二步 执行
./C_EXAMPLE/example_basic_LinkSystem


# 编译全部
./C_EXAMPLE/build_linux.sh all                #-> builds every example_*.cpp
```

**方式二：使用自动化编译脚本，编译为 DLL/SO 库后调用**

项目主目录下提供了两个一键编译脚本，自动完成编译， 在C_EXAMPLE_USE_DLL_SO/下提供两个将库和使用代码一起编译的脚本实现使用代码自动编译。

Windows：
```bash
# 第一步：编译 DLL（C/C++/python调用用）
win_auto_compile.bat
- 编译 `libGentoSDKPY.dll`（Python 用，静态链接 libgcc）→ 自动复制到 [PYTHON_SDK/](PYTHON_SDK/)
- 编译 `libGentoSDK.dll`（C/C++ 用）→ 自动复制到 [C_EXAMPLE_USE_DLL_SO/](C_EXAMPLE_USE_DLL_SO/)

# 第二步：编译你的代码，注意实际路径，这里假设你的代码路径为 C_EXAMPLE_USE_DLL_SO\test_link.cpp
./C_EXAMPLE_USE_DLL_SO/build_windows_dll.bat test_link.cpp

# 第三步 执行
./C_EXAMPLE_USE_DLL_SO/test_link.exe

# 将 ../C_EXAMPLE/下所有demo全部编译
./C_EXAMPLE_USE_DLL_SO/build_windows_dll.bat all                   #-> builds every ..\C_EXAMPLE\example_*.cpp
```
Linux：

```bash
# 第一步：编译 SO（C/C++/python调用用）
./linux_auto_compile.sh
- 编译 `libGentoSDK.so`（通用）→ 自动复制到 [C_EXAMPLE_USE_DLL_SO/](C_EXAMPLE_USE_DLL_SO/)
- 编译 `libGentoSDKPY.so`（Python 用，兼容多 glibc 版本）→ 自动复制到 [PYTHON_SDK/](PYTHON_SDK/)

# 第二步：编译你的代码，注意实际路径，这里假设你的代码路径为 C_EXAMPLE_USE_DLL_SO\test_link.cpp
./C_EXAMPLE_USE_DLL_SO/build_linux_so.sh test_link.cpp

# 第三步 执行
./C_EXAMPLE_USE_DLL_SO/test_link

# 将 ../C_EXAMPLE/下所有demo全部编译
./C_EXAMPLE_USE_DLL_SO/build_linux_so.sh all                   #-> builds every ..\C_EXAMPLE\example_*.cpp
```

linux兼容编译:

- 在较新系统上编译的SO库，在旧机器使用，可能出现如下错误：
    "version `GLIBC_2.xx' not found"

- 解决方案：选择一台比较老的x_86架构的linux机器编译，以实现在比较新的x_86和arm架构的机器下直接使用so，推荐在Ubuntu 20.04 编译（glibc 2.31，覆盖主流系统）。

```bash
./linux_auto_compile_compitable.sh
```
- 编译 `libGentoSDK.so`（通用）→ 复制到 [C_EXAMPLE_USE_DLL_SO/](C_EXAMPLE_USE_DLL_SO/)
- 编译 `libGentoSDKPY.so`（Python 用，兼容多 glibc 版本）→ 复制到 [PYTHON_SDK/](PYTHON_SDK/)


### 5.4 Python SDK 使用

Python SDK 通过 [GentoRobot.py](PYTHON_SDK/GentoRobot.py) 封装底层 DLL/SO，提供面向对象的 Python 接口。主入口类为 `GentoRobot`。

**库加载机制**：
- Windows：自动加载 `PYTHON_SDK/libGentoSDKPY.dll`（通过 `ctypes.WinDLL`）
- Linux：自动加载 `PYTHON_SDK/libGentoSDKPY.so`（通过 `ctypes.CDLL`）

**常用 API 分类**：

| 类别 | 方法 | 说明 |
|------|------|------|
| 系统管理 | `link()`, `unlink()`, `reboot()`, `system_update()` | 连接、断开、重启、固件升级 |
| 版本信息 | `get_sdk_version()`, `get_controller_version()`, `get_robot_type()` | 获取版本和型号 |
| 状态切换 | `switch_to_position_mode()`, `switch_to_imp_joint_mode()`, `switch_to_drag_joint()` 等 | 切换控制模式 |
| 实时数据 | `rt`, `sg`, `get_rt_dict()`, `get_sg_dict()` | 读取实时反馈数据 |
| 运动控制 | `runtime_set_joint_pos_cmd()`, `runtime_run_traj()`, `runtime_stop_traj()` | 位置指令和轨迹执行 |
| 末端工具参数 | `runtime_set_tool_kd()` | 运动学/动力学参数设定 |
| 动力学 | `runtime_set_force_ctrl()`, `runtime_set_torque_ctrl()` | 力/力矩控制 |
| 阻抗参数 | `runtime_set_joint_kd()`, `runtime_set_cart_kd()`| 刚度/阻尼调节 |
| 运动学 | `forward_kinematics()`, `inverse_kinematics()`, `jacobian()` | 正/逆运动学 |
| 轨迹规划 | `plan_joints()`, `plan_linear()`, `plan_linear_synchronous()` | 关节/笛卡尔/同步规划 |
| 参数管理 | `param_get_int()`, `param_set_float()`, `param_get_string()` | 读写参数 |
| 硬件配置 | `config_brake_lock()`, `config_reset_enc_offset()`, `config_disable_soft_limit()` | 刹车、编码器、限位 |
| 末端通信 | `terminal_clear()`, `terminal_get()`, `terminal_set()` | CAN FD / RS485 通信（参数为 `FXObjType`） |
| 文件传输 | `send_file()`, `recv_file()` | 文件收发 |
| 动力学辨识 | `start_sampling()`, `get_sampling_status()`, `stop_sampling()`, `destroy_sampling()`, `run_load_identification()` | 采集→辨识解耦：先采样轨迹再辨识负载参数 |
| 灵巧手数据 | `hand_get_data()`, `hand_set_data()` | 按手（`FXObjType`）读写 512 字节数据 |
| 用户反馈通道 | `system_set_user_fbk_type()` | 配置 4 路用户自定义反馈源 |
| Luna 机身运动学 | `luna_body_forward_kinematics()`, `luna_body_inverse_kinematics()` | Luna 双臂机身正/逆运动学 |

**使用示例**：参见上方 [5.2 基本使用示例](#52-基本使用示例) 中的 Python 部分，以及 [PYTHON_EXAMPLE/](PYTHON_EXAMPLE/) 目录中的完整示例。

### 5.5 跨平台编译注意事项

**CPU 架构是硬性限制**：ARM64（如 Jetson Nano、树莓派）编译的 .so 无法在 x86_64 机器上运行，反之亦然。如果需要支持多架构，必须分别在每架构上编译，或使用交叉编译工具链。

**glibc 版本兼容性**：即使架构相同，较新系统编译的 .so 也可能因 glibc 版本不兼容而在旧系统上报错。本项目的解决策略：

| 库文件 | 策略 | 说明 |
|--------|------|------|
| `libGentoSDKPY.so` | `-static-libgcc -static-libstdc++` | 将 C++ 运行时静态链接进 .so，不依赖系统版本 |
| `libGentoSDK.so` | 默认动态链接 | 如需兼容旧系统，加 `--static` 参数 |

**交叉编译（x86_64 主机 → ARM）**：项目提供 [linux_cross_compile_arm.sh](linux_cross_compile_arm.sh)，可在 Ubuntu 20.04 x86_64 主机上交叉编译出 ARM 可用的 `.so`，无需在 ARM 机器上编译。

```bash
# 1. 安装交叉工具链（按目标机架构二选一，或都装）
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu       # 64位 ARM (aarch64)
sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf    # 32位 ARM (armhf)

# 2. 确认目标机架构：在 ARM 机器上执行 uname -m
#    aarch64 → 64位；armv7l → 32位

# 3. 交叉编译（默认 aarch64）
./linux_cross_compile_arm.sh              # = aarch64
./linux_cross_compile_arm.sh armhf        # 32位
./linux_cross_compile_arm.sh all          # 两个都编

# 产物输出到 dist_<arch>/（不覆盖 x86 版本）
# 部署：把 dist_aarch64/libGentoSDKPY.so 拷到 ARM 机器的 GentoRobot.py 旁边
```

**交叉编译注意事项**：

- `libGentoSDKPY.so` 是 ctypes 加载的纯 C++ 库（不链接 libpython），交叉编译**不需要**目标机的 Python。
- 交叉 g++ 自带目标 sysroot，`-lrt`/`-pthread` 等自动指向 ARM 的 libc，`timer_settime` 等符号在链接期即可解析；脚本内置 `--no-undefined` 校验和 `readelf` 架构自检，缺符号或误用本机 g++ 会立即报错。
- 交叉编译**不要加** `-march=native`（那是针对编译机 x86 的）。
- 仅适用于 glibc 系统（Ubuntu/Debian/树莓派 OS 等）。目标机若为 **musl（Alpine）**或 **uclibc**，libc 不同仍可能报错，需改用线程循环代替 POSIX 定时器（见下方 FAQ Q8）。

**最佳实践**：为每种目标架构+系统组合单独编译一份 .so，发布时用文件夹区分，例如：
```
lib/
├── aarch64-jetson-ubuntu2004/
│   └── libGentoSDKPY.so
├── x86_64-ubuntu2004/
│   └── libGentoSDKPY.so
└── x86_64-ubuntu2204/
    └── libGentoSDKPY.so
```

### 5.6 编译环境与依赖差异说明

**参考编译环境**：仓库内提供的 `.so` 在 **Ubuntu 20.04（glibc 2.31）/ x86_64** 上编译。这是向下兼容的目标基准——在此环境编译的库可直接用于 Ubuntu 20.04 及以上的 x86_64 / ARM64 主流系统。

> 不同机器编译出的 `.so`，用 `ldd *.so` 查看到的依赖项可能不一样。**这是正常现象**，多数差异无害，但需要能区分。

**常见 `ldd` 差异与含义**：

| `ldd` 看到的差异 | 原因 | 是否需要处理 |
|------------------|------|--------------|
| 出现 / 不出现 `libstdc++.so.6` | `libGentoSDKPY.so` 使用了 `-static-libgcc -static-libstdc++`，C++ 运行时被静态打进库；**不出现 = 静态成功** | 参考环境产物不应出现，确认即可 |
| 出现 / 不出现 `libpthread.so`、`libdl.so`、`librt.so` | glibc 2.34+ 将它们并入 `libc.so.6`，叠加新版链接器默认的 `--as-needed` 会丢弃未被引用的库 | 无需处理，纯表象差异 |
| `libc.so.6` 实际要求的 `GLIBC_x.x` 版本号不同 | 由编译机的 glibc 版本决定，**决定库能跑多老的系统** | **关键指标**：参考环境为 2.31 |

**只有最后一行（GLIBC 最大版本号）是真正的可移植性指标**，前两行不影响库在目标机器上的运行。

**自查命令**（在编译机上对产物执行）：

```bash
# 查看直接依赖了哪些 .so（等价于 ldd，但更准确）
readelf -d libGentoSDKPY.so | grep NEEDED

# 确认 C++ 运行时是否已静态（无输出 = 已静态）
readelf -d libGentoSDKPY.so | grep -E 'libstdc|libgcc'

# 查看库要求的最高 glibc 版本（参考环境应为 2.31）
objdump -T libGentoSDKPY.so | grep -oE 'GLIBC_[0-9.]+' | sort -V | tail
```

**结论**：

- 在 **Ubuntu 20.04** 编译 → 最高要求 `GLIBC_2.31` → 可在 Ubuntu 20.04 及以上运行。
- 在更新的系统（如 Ubuntu 22.04，glibc 2.35）编译 → 最高要求可能升至 `GLIBC_2.34` → **无法在 Ubuntu 20.04 上运行**，会报 `version 'GLIBC_2.34' not found`。
- 因此**需要兼容旧系统时，请按 5.5 节在 Ubuntu 20.04 或更老的 x86_64 环境编译**，而不是在最新系统上编译。

## 6. 重要使用说明

### 6.1 网络与连接

- SDK 使用带专有可靠协议的 UDP。请确保防火墙允许配置端口（默认 50000–50010）上的 UDP 通信。
- `FX_L1_System_Link()` 成功时返回正延迟值（毫秒），而非零值。负值视为错误。
- 务必在程序退出前调用 `FX_L1_System_Unlink()` 释放套接字和资源。

### 6.2 状态机

- 有效的状态切换由控制器强制执行。例如，从 IDLE 状态必须先进到 POSITION 状态，然后才能发送运动指令。
- 状态切换函数使用超时值（毫秒）。典型超时：模式切换 3000 ms，复位 5000 ms。
- 发生错误后，使用对应的对象类型调用 `FX_L1_State_ResetError()` 并捕获系统错误码。

### 6.3 实时反馈

- `FX_L1_Fbk_GetRT()` 返回指向内部管理数据的指针，不要手动释放。
- 实时数据以 1 kHz 更新；慢组数据（`FX_L1_Fbk_GetSG()`）以 500 Hz 更新。
- 关节位置数组长度为 7——非臂对象的未使用自由度会被填充。

实时反馈结构体见 [FXCommon.h中的ROBOT_RT和ROBOT_SG](C_SDK/Common/FXCommon.h)

### 6.4 运动学与规划

- `FX_MotionHandle` 必须通过 `FX_L1_Kinematics_Create()` 创建，并通过 `FX_L1_Kinematics_Destroy()` 销毁。
- 使用以下两种方式之一初始化机械臂环境：
  - `FX_L1_Kinematics_InitSingleArm_ByIniConfig()` — 从控制器读取参数（需要已建立连接）
  - `FX_L1_Kinematics_InitSingleArm_ByInputParams()` — 手动传入 DH 参数、质量、惯量表格
- 规划 API（如 `FX_L1_Kinematics_PlanJointMove`）输出点集句柄。该句柄为 `CPointSet` 对象指针——须使用 L0 函数管理其生命周期（参考 `L0Robot.h` 中的 `FX_L0_CPointSet_Create/Destroy`）。

### 6.5 线程安全

L1 SDK 默认多线程安全（L0及以下统一使用第0号线程）支持1~7号线程并发调用 API 函数。

### 6.6 错误处理

务必检查返回值并枚举对比。常见错误码：

控制错误码 [FXErrorCode](C_SDK/Common/FXErrorCode.h#L50)
运动和规划错误码 [FXFuncReturn](C_SDK/Common/FXErrorCode.h#L101)

## 7. 常见问题（FAQ）

**Q1：`FX_L1_System_Link()` 返回 -2 或 -3，如何排查？**

- 确认机器人控制器 IP 地址正确，且电脑与控制器在同一子网内。
- 确认防火墙未阻止 UDP 端口（Linux 下可使用 `netstat -uan` 查看）。
- 检查机器人控制器是否已上电，网线是否已连接。
- 部分控制器需要特殊网络配置——请参考机器人手册。


**Q2：逆运动学调用返回 `FUNC_RET_KINE_IK_UNREACHABLE`，怎么办？**

- 目标位姿超出了机器人工作空间，或过于靠近奇异点。
- 尝试调整参考关节配置（`FX_InvKineSolvePara` 中的 `ref_joints`）。
- 使用 `FX_L1_Kinematics_ForwardKinematics()` 测试目标位姿是否可达。
- 放宽关节限位检查的容差（部分 IK 求解器支持边界余量）。

**Q3：`FX_L1_Runtime_SetJointPosCmd` 和轨迹规划有什么区别？**

- `SetJointPosCmd` 发送单一目标位置，机器人使用内部控制器运动（在当前速度/加速度限制下平滑逼近目标）。
- 规划 API（如 `PlanJointMove`）生成完整的时间参数化轨迹（含插值点），需要通过 `FX_L1_Runtime_RunTraj()` 执行点集。

**Q4：如何控制 SDK 日志级别？**

SDK日志分为控制日志和运动规划日志， 日志分为5个级别：
  
  C/C++日志掩码:
  ```C
  FX_LOG_NULL_FLAG (0)       /**< No log output */
  FX_LOG_DEBG_FLAG (1 << 0)  /**< Debug log messages */
  FX_LOG_INFO_FLAG (1 << 1)  /**< Informational log messages */
  FX_LOG_WARN_FLAG (1 << 2)  /**< Warning log messages */
  FX_LOG_ERROR_FLAG (1 << 3) /**< Error log messages */
  FX_LOG_ALL_FLAG (FX_LOG_DEBG_FLAG | FX_LOG_INFO_FLAG | \
                  FX_LOG_WARN_FLAG | FX_LOG_ERROR_FLAG)
  ```

C/C++日志设置:

- 在连接机器人时设置控制的日志级别             
```c
unsigned int log_level=FX_LOG_INFO_FLAG;
ret=FX_L1_System_Link(6,6,7,190,log_level);
```
- 连接后，更换设置控制的日志级别             
```c
unsigned int log_level=FX_LOG_INFO_FLAG;
unsigned int back=FX_L1_System_GetLogLevel();
if back!=log_level
{
  FX_L1_System_SetLogLevel(log_level);
}
```
- 设置运动和规划的日志级别
```c
unsigned int log_level=FX_LOG_INFO_FLAG;
FX_L1_Kinematics_SetLogLevel(log_level);
```

  PYTHON日志掩码:
  ```python
  class FXLogMask:
    """Log level masks."""
    FX_LOG_DEBG_FLAG = 1 << 0
    FX_LOG_INFO_FLAG = 1 << 1
    FX_LOG_WARN_FLAG = 1 << 2
    FX_LOG_ERROR_FLAG = 1 << 3
    FX_LOG_ALL_FLAG = (FX_LOG_DEBG_FLAG | FX_LOG_INFO_FLAG | FX_LOG_WARN_FLAG | FX_LOG_ERROR_FLAG)
  ```

PYTHON日志设置:
- 在连接机器人时设置控制的日志级别   
```python
ret=robot.link(6,6,7,190, log_level=FXLogMask.FX_LOG_INFO_FLAG)
```
- 连接后，更换设置控制的日志级别             
```python
log_level=FXLogMask.FX_LOG_INFO_FLAG
back=robot.get_log_level();
if back!=log_level
{
  robot.set_log_level(log_level);
}
```
- 设置运动和规划的日志级别
```python
log_level=FXLogMask.FX_LOG_INFO_FLAG
robot.kine_log_level(log_level)
```

**Q5：连接时报 `FUNC_RET_VERSION_INCOMPATIABLE` 错误。**

SDK 版本必须与控制器固件兼容。请将 SDK 或机器人控制器更新到匹配的大版本号。可使用 `FX_L1_System_GetControllerVersion()` 和 `FX_L1_System_GetSDKVersion()` 对比版本。

**Q6：能否同时控制双臂？**

可以。分别使用 `FX_OBJ_ARM0` 和 `FX_OBJ_ARM1`，或使用位掩码（如 `FX_OBJ_ARM0_FLAG | FX_OBJ_ARM1_FLAG`）调用 `FX_L1_Runtime_EmergencyStop()` 或 `FX_L1_Runtime_RunTraj()` 等函数。如需同步笛卡尔规划，使用 `FX_L1_Kinematics_ArmsSynchronousPlanning()`。

**Q7：`git pull` 显示已是最新，但 VSCode 里仍满屏显示文件已修改（绿色标记）？**

这通常是编辑器的「保存时自动格式化」（Format On Save）造成的：VSCode 保存时会按本地规则重排代码（如指针 `*` 的位置、行尾空格、`case` 换行等），产生大量非功能性 diff，让 git 误以为文件被修改。此外，Windows 下 CRLF/LF 行尾符差异也会让部分文件显示为已修改（内容其实与服务器相同）。

建议在各自本地的 `.vscode/settings.json` 中关闭（该目录已被 `.gitignore` 忽略，不会进入版本库）：

```json
{
    "editor.formatOnSave": false,
    "editor.formatOnPaste": false
}
```

若需让工作区与服务器完全一致，可先执行 `git stash` 备份本地改动，再执行 `git reset --hard origin/develop`。

**Q8：导入 libGentoSDKPY.so 时报 `undefined symbol: timer_settime`？**

这是因为加载的 `.so` 没有解析到 POSIX 定时器符号。常见原因与排查：

- **最常见：加载的不是新编译的库。** `GentoRobot.py` 按其自身所在目录加载 `.so`，若旧 `.so` 还残留在 `site-packages` 或别的目录，就会加载到旧库。重新编译后务必把新 `.so` 放到 `GentoRobot.py` 旁边。
- **确认 `-lrt` 是否生效**：对实际加载的那个 `.so` 执行 `readelf -d libGentoSDKPY.so | grep NEEDED`，若结果中没有 `librt.so.1`，说明编译时 `-lrt` 没起作用。注意 `g++ -shared` 默认对未定义符号不报错，所以"编译成功"不代表链接正确；本项目脚本已加 `-Wl,--no-undefined`，缺符号会在编译期直接失败。
- **目标机 libc 真的没这个符号**：现代 glibc（≥2.17）已把 `timer_settime` 并入 `libc`，正常情况下不会报此错。若仍报错，说明目标机用的是 musl/uclibc 或被裁剪的 libc——此时可用 [交叉编译脚本](linux_cross_compile_arm.sh) 在 glibc 主机上重新编译，或修改 `C_SDK/L0Control/RobotCtrl.cpp` 用线程循环代替 `timer_create`/`timer_settime`。


## 8. 其他重要信息

### 8.1 安全注意事项

- 执行任何运动指令时，务必确保急停电路随时可用。
- 不要单纯依赖软件限位；建议使用物理限位或外部安全系统。
- 拖拽示教模式（`FX_DRAG_TYPE_*`）会降低刚度——注意远离夹持点。

### 8.2 性能考量

- 实时指令（`FX_L1_Runtime_SetJointPosCmd`）通过 UDP 发送，不保证在硬实时截止时间内到达。对时间要求苛刻的应用，建议使用 L0 实时接口。
- 轨迹规划函数在 PC 端执行计算；点集不能超过 5000 个点， 可能导致内存占用大幅增加，导致规划失败。

### 8.3 支持的机器人型号与自由度

| 机器人型号 | 臂 DOF | 躯干 DOF | 头部 DOF | 升降台 DOF |
|------------|--------|----------|----------|-------------|
| Marvin Pro M3 | 7+7 | 0 | 0 | 0 |
| Marvin Pro M6 | 7+7 | 0 | 0 | 0 |
| Gento Skye | 7+7 | 3 | 3 | 2 |
| Gento Luna | 7+7 | 6 | 3 | 0 |

可在运行时通过 `FX_L1_Fbk_GetCtrlObjDof()` 查询实际 DOF。

### 8.4 文件传输

`FX_L1_System_SendFile()` 和 `RecvFile()` 使用专有的可靠 UDP 文件传输协议。文件过大将会传输失败。请确保远程路径可写（如控制器上的 `/home/FUSION/`）。

### 8.5 固件升级

使用 `FX_L1_System_Update()` 并传入有效的升级包（`.UPDATE`）和 INI 配置文件。升级成功后机器人会自动重启。升级过程中切勿断电。

### 8.6 文档与支持

- **SDK 源码**：[C_SDK/](C_SDK/)
- **C/C++ 示例**：[C_EXAMPLE/](C_EXAMPLE/)（直接调用源码）、[C_EXAMPLE_USE_DLL_SO/](C_EXAMPLE_USE_DLL_SO/)（调用库）
- **Python 示例**：[PYTHON_EXAMPLE/](PYTHON_EXAMPLE/)
- **自动化编译脚本**：[win_auto_compile.bat](win_auto_compile.bat)（Windows）、[linux_auto_compile.sh](linux_auto_compile.sh)（Linux）、[linux_cross_compile_arm.sh](linux_cross_compile_arm.sh)（x86_64 → ARM 交叉编译）

## 9. 许可与免责声明

本 SDK 按"现状"提供，仅供与 FX Robotics 产品配合使用。未经书面许可，禁止重新分发或修改 SDK 二进制文件。FX Robotics 对因误用软件或硬件而造成的任何损害或伤害不承担责任。

---

*FX Robotics SDK/API Reference | Version 4.7.1 | © FX Robotics. All rights reserved.*
