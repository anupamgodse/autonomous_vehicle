/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2010 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: sample1.h 2416 2012-09-07 08:06:20Z ertl-hiro $
 */

/*
 *		サンプルプログラム(1)のヘッダファイル
 */

/*
 *  ターゲット依存の定義
 */
#include "target_test.h"

/**
 *  * Task priorities (smaller number has higher priority)
 *   */

#define PRIORITY_LIGHT_SENSOR_GROUND_TASK   TMIN_APP_TPRI
#define PRIORITY_LIGHT_SENSOR_AMBIANCE_TASK TMIN_APP_TPRI + 1
#define PRIORITY_SONAR_SENSOR_TASK          TMIN_APP_TPRI + 2
#define PRIORITY_MOTOR_TASK                 TMIN_APP_TPRI + 3
#define PRIORITY_MAIN_TASK                  TMIN_APP_TPRI + 4

/**
 *  * Task periods
 *   */

#define PERIOD_LIGHT_SENSOR_GROUND_TASK   2
#define PERIOD_LIGHT_SENSOR_AMBIANCE_TASK 4
#define PERIOD_SONAR_SENSOR_TASK          8
#define PERIOD_MOTOR_TASK                 16

/**
 * These values were defined for use in our program in order to track the state of our machine, particularly which phase of evaision we are in
 */

#define LEFT -1
#define RIGHT 1

#define MOVE_FORWARD 0
#define REVERSE 1
#define STEER 2
#define TURN 3
#define STEER_BACK 4
#define REVERSE_EVADE 5
#define STEER_EVADE 6
#define TURN_EVADE 7
#define GET_AWAY 8
#define TURN_BACK_EVADE 9
#define GET_IN 10
#define REVERSE_GET_LINE 11
#define RETURN 12
#define RETURN_STEER_BACK 13
#define REALIGN 14


/*
 *  ターゲットに依存する可能性のある定数の定義
 */

#ifndef STACK_SIZE
#define	STACK_SIZE		4096		/* タスクのスタックサイズ */
#endif /* STACK_SIZE */

#ifndef LOOP_REF
#define LOOP_REF		ULONG_C(1000000)	/* 速度計測用のループ回数 */
#endif /* LOOP_REF */

/**
 *  * Prototypes for configuration
 *   */
#ifndef TOPPERS_MACRO_ONLY

extern void	main_task(intptr_t exinf);
extern void update_motor(intptr_t unused);
extern void light_sensor_ground(intptr_t unused);
extern void light_sensor_ambiance(intptr_t unused);
extern void sonar_sensor(intptr_t unused);
extern void bluetooth(void);
extern void task_activator(intptr_t tskid);

extern void gpio_irq_dispatcher(intptr_t exinf);
#endif
