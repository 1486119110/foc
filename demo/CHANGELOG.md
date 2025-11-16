# demo 位置环改动说明

## 1. 绝对编码器计数
- 在 `demo/Src/Main.c` 中新增 `Place_now`、`PosRevCnt`、`yk` 等状态变量，将 QEP 当前脉冲值与圈计数结合得到绝对位置。
- **原因**：位置环需要和 xptos 一样使用连续的位置反馈，避免原实现里靠速度 PI 推算导致的累积误差。

## 2. 外环调度
- `OutLoop_Control` 的速度分支保持原始 PI 算法，仅在 `PosEnable = 0` 时清零位置累加，确保调速模式下逻辑与改动前一致。
- 位置分支则改为取 `Place_now` 标定后的 `yk` 作为 `pos_ctrl.Fdb`，让 PTOC 控制器在同一函数中直接使用绝对位置；输出仍然通过 `IQ_Given` 驱动电流环。
- **原因**：满足“保留速度控制，只修改位置控制”的要求，让位置环可以复用 xptos 的结构，同时不影响原有速度环调节。

## 3. 数据监控与指标
- DLOG 通道和 `performance_metrics_update` 改为跟踪新的位置反馈与控制量，便于观察调试。
- **原因**：新的位置算法带来不同的量程，需要同步更新可视化和性能统计，否则图形、指标无法反映真实位置。

## 4. QEP 采样位置
- 把 `UpdatePositionFeedback()` 嵌入 PWM 中断中，让 `RawTheta`、速度和绝对位置在 10 kHz 采样率下实时刷新，外环只消费这些数据而不再重复读取 QEP。
- **原因**：xptos 也是在功率中断中维护 QEP 累计，从而保证速度和位置反馈更平滑；demo 侧同步这一机制后，速度环仍保持原样，而位置环获得一致的高带宽测量。

## 5. 位置环控制对齐 xptos
- 将 `OutLoop_Control` 中的位置模式改为与 xptos 相同的 `Place_now + PosCount` 绝对脉冲累计方式，去掉 `PosRevCnt/PosInRev` 推算，直接以 `Place_now * PosRevScale` 作为 RCNS/PTOC 的反馈量。
- **原因**：确保 demo 与 xptos 完全一致的结构和调试逻辑，避免两套位置控制实现长期漂移，便于统一维护。
