# AGENTS.md — 林-Ⅰ外骨骼二次开发 AI 协作约束

> 本文件约束所有参与本工程开发的 AI 助手（Copilot / 其他 agent）的行为准则。
> **开工前必须完整阅读本文件与 `H5平台知识库.md`，违反"硬性铁律"视为返工。**

## 一、模板工程构成（先认识这个工程）

本工程是驭介林-Ⅰ髋关节外骨骼（STM32H563VGT6 主控）的**二次开发模板**，复制自官方"历程0 空闲程序"并做了最小改造。分层架构 `Core—Platform—Device—APP—Sys` 全部为**平台内部实现，禁止修改**；用户代码只写在 `User_Data/` 下。

```
Templete/
├── AGENTS.md              ← 本文件：AI 协作约束
├── H5平台知识库.md         ← 平台已验证事实全库（必读）
├── README.md              ← 模板使用说明
├── Record.md              ← 改动日志（AI 每次提交后必须更新，不入 git）
├── Core/ Platform/ Device/ APP/ Sys/ Drivers/  ← 平台层，禁止修改
├── MDK-ARM/CORE.uvprojx   ← Keil 工程（新增 .c 需在此注册）
└── User_Data/             ← ★ 唯一可自由修改区
    ├── Inc/
    │   ├── User_Data.h       平台公开数据/接口（右腿坐标说明见头文件注释）
    │   ├── User_main.h       用户入口声明
    │   ├── awalk_control.h   控制算法模板：句柄/常量/接口
    │   └── transmission.h    数据回传封装：JustFloat 发送
    └── Src/
        ├── User_Data.c       平台数据刷新（右腿坐标统一已内置）
        ├── User_main.c       ★ 主入口：模式使能→初始化→注册任务
        ├── awalk_control.c   ★ 控制算法入口（Awalk_Control_Task，1ms）
        └── transmission.c    数据回传实现
```

**三个"入口"记住即可：**
| 入口 | 位置 | 作用 |
|---|---|---|
| 控制算法入口 | `awalk_control.c` 的 `Awalk_Control_Task()` | 1ms 周期执行，算法写 `torque_des`，末尾经 `Awalk_Output_Torque()` 统一下发 |
| 数据回传入口 | `User_main.c` 的 `User_Debug_Task()` | 20ms 周期，调 `Transmission_Send` 回传 |
| 程序主入口 | `User_main.c` 的 `User_main()` | 一次性初始化 + 注册任务，while(1) 空转 |

## 二、硬性铁律（违反 = 返工）

1. **禁止修改** `Core/ Platform/ Device/ APP/ Sys/ Drivers/` 任何文件。新增代码一律放 `User_Data/`，新增 .c 必须同步加入 `MDK-ARM/CORE.uvprojx` 的 User_Data 分组。
2. **1ms 任务禁止阻塞**：`Awalk_Control_Task` 运行在 TIM7 1ms 中断上下文，禁止 `HAL_Delay`/阻塞等待/长循环。只有 `User_main()` 的初始化阶段（一次性）可用阻塞式命令。
3. **必须 git 管理**：每次修改代码后 `git add -A; git commit`（提交全部未提交文件，中文信息）；commit 后**立即**用中文更新 `Record.md`（短 hash + 改动内容 + 是否完整），`Record.md` 不入 git。回复用户前用 `git log -1 --oneline` 确认已提交。
4. **力矩必须走统一出口**：算法只生成左右前踢为正的目标力矩，周期输出必须经 `Awalk_Output_Torque(left, right)`；禁止绕过出口直接调用底层力矩接口。该函数执行门控、NaN/Inf检查、`±AWALK_TORQUE_LIMIT` 限幅（默认10 N·m，配置须>0且≤15）和右腿命令取反。底层接口仍零限幅。禁止在算法中重复取反右腿或提前return跳过每拍输出。
5. **模式/使能只在初始化调用**：`User_Motor_MIT_Mode_Set`/`User_Motor_Enable` 等设置类命令有 100ms ACK 超时，只能在 `User_main()` 初始化阶段调用；周期任务内只发实时命令（`User_Motor_MIT_Set`/`User_Motor_Torque_Set`）。

6. **工程配置必须随代码提交**：修改源文件/库列表或路径时，同步保存并提交 `.uvprojx/.uvoptx`；必需 `.sct`、项目自带 `.lib/.a`、RTE组件和相关配置不得忽略。必需库不要放在Objects/Listings等产物目录。新增忽略规则放在 `.gitignore` 末尾保留规则之前；检查文件确实已被Git跟踪。分支切换前保存并关闭Keil，切换后重新打开工程并Rebuild，详见知识库第10节。

## 三、编程规范（严格照抄）

**文件头模板**（.c 与 .h 分别使用，版权行 "Copyright (c) 2026 DRVEXO."）：

```c
/**
  ******************************************************************************
  * @file           : xxx.c
  * @brief          : xxx文件
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 DRVEXO.
  * All rights reserved.
  *
  *
  *
  ******************************************************************************
  */
```

.h 文件额外要求 include guard + extern "C" 包裹（参考 `awalk_control.h`）。

1. **每个函数必须有中文注释**：`@brief` 说明用途、`@param` 参数、`@return` 返回值、必要时 `@note` 补充约束。
2. **函数返回错误用负值**，成功返回 0（对齐平台 `LY_OK=0`、`-LY_ERR_*` 负值约定）。
3. **结构体优先 `struct xxx` 形式**。
4. **传输协议用联合体**（参考 `transmission` 与平台 `User_UART_Send` 的 JustFloat 帧，`#pragma pack(push,1)`）。
5. **不要拆分简单函数**，能合写就合写。
6. **关键失败路径累计错误计数**（参考平台 `sys_err.xxx_cnt++`）。

## 四、平台关键知识（写算法前必读，详见 H5平台知识库.md）

- **数据读取**：电机反馈 `User_Motor_Data.left/right`（1ms 刷新，遥测源周期 7ms）；IMU `User_IMU_Data.user_euler`（**pitch/roll 单位是度**，用弧度需 ×0.01745f）；电池 `User_Battery_Level`。多字段组合先快照再计算。
- **坐标约定**（本模板已统一）：左右髋角均为**前踢为正、范围 [0,π] 弧度**。右腿已在 `User_Data.c` 出口完成换算，读 `User_Motor_Data.right.*` 即可，**不要自行再取反**。
- **单位**：角度 rad、速度 rad/s、力矩 N·m。
- **下发力矩**：统一调用 `Awalk_Output_Torque(left, right)`，左右输入均前踢为正。**用户已确认右腿命令必须取反**，只由该函数以 `AWALK_RIGHT_COMMAND_SIGN=-1.0f` 处理。默认MIT纯力矩（kp=kd=0）；`AWALK_USE_MIT_OUTPUT` 同时决定初始化模式与发送接口。修改为非零kp/kd时须重新设计总力矩保护。
- **任务注册**：`Scheduler_CreateStaticTask(fn, "名字", 周期ms, 优先级, &句柄)`。优先级 0~31 数值越小越先执行；**用户控制任务建议 ≥23**（排在 FDCAN(21)/LowPower(22) 之后）；1ms 任务周期填 1、优先级 23；回传任务 20ms、优先级 31。
- **控制周期**：1ms（后台 TIM7 中断驱动），与调度器同拍。任务总执行时间需留裕量（同一拍所有到期任务串行执行，最坏叠加勿超 1ms）。
- **安全门控**：任一电机未就绪、故障、非舒适模式，或统一反馈角度超出[2°,178°]（原范围两端各内缩2°，含边界），统一出口实际向双侧发送零力矩；目标力矩/反馈角度NaN或Inf、非法保护配置也清零双侧。角度回到范围内且其他门控通过即可恢复；不得截断反馈角度掩盖越界。即使门控关闭也必须执行每拍输出，不能仅清变量后return。零力矩发送成功不等于失能，发送失败也不能保证电机已清零；应观察错误计数与实机状态。

## 五、典型开发流程

1. **读**：`H5平台知识库.md` + 本文件 + `User_Data/Inc/User_Data.h` 接口面。
2. **写控制算法**：在 `awalk_control.c` 的 `Awalk_Control_Task` 内 TODO 处实现，输入快照→算统一坐标 `torque_des`→末尾 `Awalk_Output_Torque()`（门控/限幅/右腿取反/发送）；保留每拍清零与统一出口。
3. **加数据回传**：在 `User_main.c` 的 `User_Debug_Task` 增删通道，调 `Transmission_Send(send_data, N)`，同步改 `USER_DEBUG_CHANNEL_NUM`。
4. **新增模块**：文件放 `User_Data/Inc|Src`，按第三节规范写文件头与注释，加入 uvprojx。
5. **验证与提交**：编译零错误 → 观察 VOFA+ 数据 → `git commit` → 更新 `Record.md`。

## 六、安全红线（人身安全相关，务必遵守）

- 首次验证**必须空载/固定平台**，禁止直接人体穿戴测试未经验证的控制参数。
- 力矩从小值起步（建议先 1 N·m 内）；当前统一出口固定MIT的kp=kd=0，若另行开发阻抗控制，再从小增益起步并补充总力矩保护。
- 电机最大力矩 15 N·m、最大转速 120 rad/s，勿超。
- 发现异常振动/异响/过热立即停止并失能电机（`User_Motor_Disable`）。
- 长按电机按键会触发整机 Standby（控制任务静默消失），试验时避免误触。
