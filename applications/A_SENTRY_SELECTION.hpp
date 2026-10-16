#ifndef SENTRY_CHASSIS_SELECTION_HPP
#define SENTRY_CHASSIS_SELECTION_HPP

//todo
//1.pid
//2.test new code
//2.1
//2.2
//2.3super_cap.cap_energy?电容始终维持在16v？嗯呢。
//2.4是否有必要加动能回收的速度阈值？据说很有效果。
//3.return是啥意思
//4.动能回收阈值——ok
//5.wz死区
//6.65能过
//n.重构

//备忘录
//1.
//2.
//3.

//SELECTION
// #define SENTRY_CHASSIS_TEST
#define SENTRY_CHASSIS
#define RMUC_TEST

#ifdef SENTRY_CHASSIS
#define HKUST_POWER_CONTROL

#define GAME_START_STATUS



#endif

#ifdef SENTRY_CHASSIS_TEST
// #define HKUST_POWER_CONTROL
// #define CURRENT_POWER_CONTROL
#endif

#ifdef GAME_START_STATUS
#define variable_speed_spin
// #define constant_speed_spin
#endif

// ★ SLIP: 基于滑移率的功率分配闭环控制
// 启用后: 打滑轮自动降低功率分配权重, 功率自动流向有抓地力的轮子
// #define SLIP_BASED_POWER_CONTROL

#ifdef RMUC_TEST
#define REMOTE_VARY
#endif

#ifdef CHASSIS_FOLLOW
// #define CHASSIS_SINGLE_FOLLOW //开启后：单次式跟随，中途不校准
#endif

#endif