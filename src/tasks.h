#pragma once

#include <Arduino.h>

// ★ A8：FreeRTOS 四任务架构。
//
// 只在【常驻模式】启动。logger 模式是深睡眠 —— 醒 20s、睡 10s，
// 99% 的时间芯片是断电的，给它开任务只是白付几 KB 栈内存，
// 没有任何并发可言。所以那条路径保持单线程，不动。

// 启动四个任务。成功返回 true；任何一个建不起来就全部回收并返回 false，
// 调用方可以退回原来的单线程 loop()。
bool tasksBegin();

// 取【内存里】最新的一条测量。有数据返回 true。
// 这是四任务改造真正的用户可见收益：以前网页只能去存储里读上一条
// 已落盘的记录，现在能读到刚采到、还没写卡的那条。
bool tasksGetLatest(String& out);

// 常驻模式的 loop() 在任务起来后没事干，交出 CPU 即可。
void tasksIdle();

// ★ 存储锁。SD / FFat 的底层是 SPI + FATFS，【不是线程安全的】，
// 而 net 任务要读文件渲染页面、storage 任务要写文件落盘 —— 会撞。
//
// 四任务没起来时（logger 模式、或任务创建失败退回单线程），
// 这两个函数是空操作，所以调用点不需要分情况写。
bool storageLock(uint32_t timeout_ms);
void storageUnlock();
