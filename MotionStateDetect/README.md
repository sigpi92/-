# 运动模式判断 Keil 工程（可直接打开编译）

本文件夹是一份完整的 STM32H5_FT 工程（STM32H563VGT6），已把
“静止 / 行走 / 上坡 / 下坡”运动模式判断模块集成进去并接好线，
打开工程直接编译即可，无需再手动加文件、改代码。

## 怎么用

1. 用 Keil 打开 `MDK-ARM/CORE.uvprojx`。
2. 点击 Rebuild（或 F7）。
3. 烧录（首次可能因主控休眠失败，按住任意电机按键再下载）。
4. 用 USB-TTL（PA9/PA10，230400，共地）接 VOFA+，选 JustFloat，通道数 15，
   即可观察运动状态。

## 已集成的内容

- `User_Data/Src/awalk_control.c`：1ms 控制任务里已调用
  `MotionState_Port_Task(dT_ms)`，每拍更新运动状态。
- `User_Data/Src/User_main.c`：初始化里已调用 `MotionState_Port_Init()`，
  回传任务已从 10 通道扩到 15 通道。
- `User_Data/Inc|Src` 新增 `motion_state.h/.c`、`motion_state_port.h/.c`，
  并已在 `CORE.uvprojx` 的 User_Data 分组注册。

## 运动状态编码（回传第 11 通道 CH10）

| 值 | 状态 |
|---|---|
| 0 | INIT 初始化 |
| 1 | STATIC 静止 |
| 2 | WALKING 平地行走 |
| 3 | UPHILL 上坡 |
| 4 | DOWNHILL 下坡 |
| 5 | UNKNOWN 未知 |

回传通道：CH10=状态编码，CH11=步态活动评分(0~1)，CH12=俯仰坡度(度)，
CH13=气压计高度变化率(m/s)，CH14=气压计原始高度(m)。

## 调参

所有阈值都在 `User_Data/Inc/motion_state.h` 的 `Motion_Config_t`（带注释），
默认值由 `User_Data/Src/motion_state.c` 的 `Motion_DefaultConfig()` 填充。
常见调整：`vel_static_rms`/`vel_walk_rms`（静止/行走灵敏度）、
`pitch_up_deg`/`pitch_down_deg`（俯仰坡度阈值）、
`alt_rate_up_ms`/`alt_rate_down_ms`（高度变化率上/下坡进入阈值，默认±0.10）、
`alt_rate_up_exit_ms`/`alt_rate_down_exit_ms`（上/下坡退出迟滞，默认±0.04）、
`slope_hold`（坡度去抖拍数，默认400ms）。

当前默认参数已用 LSR 的 6 份实测数据（LSR行走1/2、LSR上1/2、LSR下1/2，
位于 EricTool 目录）标定：平地行走不再误判上/下坡，上/下坡方向判定正确。
`tools/offline_validate.py` 可用新数据复现此验证。

## 注意

- 首次务必空载/固定平台验证，力矩等控制逻辑仍按平台规范从小值起步。
- 坡度判断在气压计高度有效时以高度变化率为准，否则退化为 IMU 俯仰；
  俯仰的方向依赖 IMU 安装，实机前倾/后仰各采一次确认符号。
- 本工程只做运动状态判断与回传，不改变原有助力/控制逻辑。
- 运动状态每次变化会调用一次弱函数 `MotionState_OnStateChanged(new_state)`，
  默认空实现；在任意 .c 里定义同名函数即可联动语音/LED/控制模式。
