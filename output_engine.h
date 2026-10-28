#pragma once
#include "config.h"

// 初始化输出引擎（创建软件定时器等）
void engineInit();

// 应用某一通道的新配置。立即生效；模式/参数变化时输出从起点重新开始。
// 调用前请先用 sanitizeConfig() 校验。
// 线程安全：可在任意任务中调用（内部有互斥锁保护）。
void engineApplyChannel(int ch, const ChannelConfig& cfg);

// 读取某通道当前配置（线程安全，返回副本）
ChannelConfig engineGetChannel(int ch);
