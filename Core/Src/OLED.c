/**
 ******************************************************************************
 * @file    OLED.c
 * @brief   0.96寸OLED显示屏驱动程序（4针脚I2C接口）—— F407/HAL/FreeRTOS 移植版
 * @note    由江协科技原版（STM32F103 标准库 + 软件I2C PB8/PB9）移植而来。
 *
 *          【移植修改点（相对原版）】
 *            1. #include "stm32f10x.h" 移除，改用 HAL：经由 i2c.h 使用 hi2c2
 *            2. 删除软件I2C位操作（OLED_W_SCL / OLED_W_SDA / OLED_GPIO_Init /
 *               OLED_I2C_Start / Stop / SendByte），改为 HAL_I2C_Master_Transmit
 *               一次事务发送 [控制字节+数据]，地址 0x78（7bit 0x3C 左移1位）
 *            3. OLED_Init 中上电稳定延时改用 HAL_Delay(100)；
 *               GPIO/时钟/复用功能全部由 CubeMX 的 MX_I2C2_Init 完成，此处不再配置
 *            4. 增加 OLED_I2C_TIMEOUT_MS 超时保护：总线异常时函数直接返回，
 *               避免 FreeRTOS 任务被 HAL 超时前的轮询长时间阻塞
 *          【保持不变】显存模型 OLED_DisplayBuf、全部显示/绘图/字模引用逻辑，
 *          上层 API 用法与原版完全一致。
 *
 *          【引脚与配置（对齐本工程 CubeMX）】
 *            I2C2：PB10=SCL / PB11=SDA，100kHz 标准模式，AF 开漏，外接 4.7k 上拉
 *
 *          【2026-08-24 传输层重构（修复"黑屏 / 花屏错位 / 复位后无显示"）】
 *            原 DMA+信号量非阻塞方案整体弃用，改为"轮询 + 失败重试 + 总线自救"：
 *            1. 所有写命令/写数据统一走 HAL_I2C_Master_Transmit 轮询，
 *               单事务 50ms 硬超时，失败自动重试 3 次，抗杜邦线偶发干扰；
 *            2. OLED_Init 前先做 I2C2 总线恢复（GPIO 位拨 9 个 SCL 脉冲，
 *               逼从机释放被拉低的 SDA），再 DeInit/Init 重置外设状态——
 *               解决"按键复位不断 OLED 电源 → SDA 残留低电平 → HAL_BUSY →
 *               初始化序列全丢 → 充电泵不开 → 黑屏，只能断电恢复"的老问题；
 *            3. 运行期显示任务每帧调用 OLED_I2C_SelfCheck()，探测到外设
 *               BUSY/ERR 或 SDA 被拉死时，自动恢复总线并重发完整初始化序列，
 *               花屏/黑屏可在一帧内自愈，不再积累错位乱码；
 *            4. 轮询仅在总线异常时才阻塞(≤50ms)；正常 100kHz 下一页 129 字节
 *               约 11ms，全刷 8 页约 90ms，配合 50ms 任务周期约 7~10FPS，
 *               满足仪表刷新且任何路径都不会让任务永久挂死。
 *
 *          原程序版权归江协科技所有（jiangxiekeji.com），本移植版仅替换通信底层。
 ******************************************************************************
 */

#include "i2c.h"      /* hi2c2：CubeMX 生成的 I2C2 句柄 */
#include "main.h"     /* GPIO 位操作（总线自救）与 HAL_Delay */
#include "OLED.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

/**
  * 数据存储格式：
  * 纵向8点，高位在下，先从左到右，再从上到下
  * 每一个Bit对应一个像素点
  *
  * 坐标轴定义：
  * 左上角为(0, 0)点
  * 横向向右为X轴，取值范围：0~127
  * 纵向向下为Y轴，取值范围：0~63
  */


/*全局变量*********************/

/**
  * OLED显存数组
  * 所有的显示函数，都只是对此显存数组进行读写
  * 随后调用OLED_Update函数或OLED_UpdateArea函数
  * 才会将显存数组的数据发送到OLED硬件，进行显示
  */
uint8_t OLED_DisplayBuf[8][128];

/*********************全局变量*/


/*通信协议（HAL 硬件 I2C2，轮询模式）*********************/

/* OLED 的 8bit 写地址：7bit 地址 0x3C 左移 1 位 = 0x78 */
#define OLED_I2C_ADDR_8BIT      0x78u
/* 单次 I2C 事务超时(ms)：总线异常时快速返回，防止任务长时间阻塞 */
#define OLED_I2C_TIMEOUT_MS     50u

/**
  * 函    数：I2C2 总线自救（SDA 被从机拉死时的标准恢复流程）
  * 参    数：无
  * 说    明：背景——按键复位/NRST 复位不会切断 OLED 电源。若复位瞬间 OLED 正处于
  *           一次传输的"应答位"上，它会把 SDA 一直拉低等待后续时钟，于是 I2C2 外设
  *           上电后看到 SDA=0，直接进入 BUSY，之后所有发送返回 HAL_BUSY，
  *           初始化序列一条都进不去（充电泵 0x8D/0x14 没发 → 黑屏），只有整机电源
  *           断开让 OLED 掉电复位才能恢复——这正是"复位后无显示、断电重上电就好"。
  *           恢复步骤（SSD1306 手册推荐）：
  *             1. 暂时把 SCL/SDA 切成普通开漏 GPIO（断开 I2C 外设对引脚的控制）；
  *             2. 手动拨 9 个 SCL 时钟脉冲，从机在第 9 个脉冲后释放 SDA；
  *             3. 用 GPIO 手动发一个 STOP（SDA 低→高，期间 SCL 保持高）；
  *             4. HAL_I2C_DeInit+Init 把引脚切回 AF 并重置 I2C2 外设状态。
  */
static void OLED_I2C_BusRecovery(void)
{
	GPIO_InitTypeDef gpio = {0};
	uint8_t i;

	/*SDA(PB11) 先置输出高（开漏：高=释放，靠外部上拉拉起），SCL(PB10) 同样释放*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);

	/*SCL/SDA 切为普通开漏 GPIO，脱离 I2C2 外设*/
	gpio.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
	gpio.Mode  = GPIO_MODE_OUTPUT_OD;
	gpio.Pull  = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &gpio);

	/*如果 SDA 已经是高，说明总线没被拉死，无需拨脉冲（直接交给 DeInit/Init 重置外设）*/
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
	{
		/*拨 9 个 SCL 脉冲，逼从机走完残余位并释放 SDA*/
		for (i = 0; i < 9u; i++)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);	/*SCL 拉低*/
			for (volatile uint16_t d = 0; d < 100; d++) { __NOP(); }	/*约几微秒*/
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	/*SCL 释放*/
			for (volatile uint16_t d = 0; d < 100; d++) { __NOP(); }
			/*SDA 一旦释放就提前结束，减少总线扰动*/
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET) { break; }
		}

		/*手动 STOP 条件：SCL 为高期间，SDA 由低跳高*/
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);	/*SDA 拉低*/
		for (volatile uint16_t d = 0; d < 100; d++) { __NOP(); }
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	/*SCL 保持高*/
		for (volatile uint16_t d = 0; d < 100; d++) { __NOP(); }
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);	/*SDA 释放→STOP*/
		for (volatile uint16_t d = 0; d < 100; d++) { __NOP(); }
	}

	/*引脚切回 I2C2 复用开漏，并重置外设寄存器（清 BUSY/ERR 等残留状态）*/
	HAL_I2C_DeInit(&hi2c2);
	HAL_I2C_Init(&hi2c2);
}

/**
  * 函    数：OLED 发送一帧 I2C 数据（轮询 + 失败重试，永不挂死）
  * 参    数：Buf 发送缓冲（含控制字节）；Len 字节数
  * 说    明：2026-08-24 起统一采用轮询方案（原 DMA+信号量方案已移除，原因见文件头）：
  *           - 单事务 50ms 硬超时，失败重试 3 次，重试间插入短延时让总线喘息；
  *           - 连续失败且检测到外设 BUSY/ERR 时，触发一次总线自救后重试；
  *           - 初始化期与运行期行为完全一致，不存在"两条路径两套毛病"；
  *           - 正常情况 100kHz 下 2 字节命令约 0.3ms、129 字节一页约 11ms，
  *             轮询阻塞时间可控，不影响 FreeRTOS 任务调度。
  */
static void OLED_I2C_Send(uint8_t *Buf, uint16_t Len)
{
	uint8_t t;
	for (t = 0; t < 3u; t++)
	{
		if (HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR_8BIT, Buf, Len,
		                            OLED_I2C_TIMEOUT_MS) == HAL_OK)
		{
			return;		/*发送成功，直接返回*/
		}
		/*失败：若外设卡在 BUSY/ERR，先做一次总线自救再继续重试*/
		if ((hi2c2.State == HAL_I2C_STATE_BUSY) ||
		    (hi2c2.State == HAL_I2C_STATE_BUSY_TX) ||
		    (hi2c2.ErrorCode != HAL_I2C_ERROR_NONE))
		{
			OLED_I2C_BusRecovery();
		}
		for (volatile uint16_t d = 0; d < 200; d++) { __NOP(); }	/*重试间隔*/
	}
}

/**
  * 函    数：OLED I2C 链路自检（供显示任务每帧调用，实现运行期自愈）
  * 参    数：无
  * 说    明：检测两类故障并自愈：
  *           1. I2C2 外设状态异常（BUSY/ERR/超时残留）→ 总线自救 + 重发初始化序列；
  *           2. SDA 被从机拉死（空闲时读到低电平）→ 同上。
  *           自愈后 OLED 重新走完整初始化+清屏，花屏/错位/黑屏在一帧内恢复。
  *           正常时本函数只读两个状态，开销可忽略。
  */
void OLED_I2C_SelfCheck(void)
{
	/* 2026-09-05 新增【限频】：自救+重初始化最多每 2 秒做一次。
	 * 原缺陷：若 SDA 持续读到低电平（屏未插好/外部上拉缺失/接线松动时 PB11
	 * 作为开漏 AF 会浮空读低），本函数会在【每一帧】都判定异常 →
	 * 每帧执行 BusRecovery + OLED_Init（内含 HAL_Delay(100) 与整屏重绘），
	 * 表现为 OLED 持续闪烁，且 TaskDisplay 每帧被白白阻塞 100ms 以上。
	 * 限频后：真故障仍能在 2 秒内自愈，误判时不再刷屏闪烁。 */
	static uint32_t s_last_fix_tick = 0u;
	uint8_t need_reinit = 0u;

	if ((hi2c2.State != HAL_I2C_STATE_READY) && (hi2c2.State != HAL_I2C_STATE_BUSY_TX))
	{
		need_reinit = 1u;		/*外设不在就绪态：BUSY/ERROR/TIMEOUT 等*/
	}
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
	{
		need_reinit = 1u;		/*空闲时 SDA 应为高：被拉低=从机卡死*/
	}

	/*HAL_GetTick 在调度器启动前后都可用（SysTick 由 HAL_Init 起就递增），
	  故此处不用 osKernelGetTickCount，避免 OLED_Init 阶段调用出错*/
	if (need_reinit && ((HAL_GetTick() - s_last_fix_tick) >= 2000u))
	{
		s_last_fix_tick = HAL_GetTick();
		OLED_I2C_BusRecovery();	/*恢复总线+重置外设*/
		OLED_Init();			/*重发完整初始化序列并清屏，屏幕自愈*/
	}
}

/**
  * 函    数：OLED写命令
  * 参    数：Command 要写入的命令值，范围：0x00~0xFF
  * 返 回 值：无
  * 说    明：一次 I2C 事务发送 [控制字节0x00 + 命令]
  */
void OLED_WriteCommand(uint8_t Command)
{
	uint8_t Buf[2];
	Buf[0] = 0x00;			//控制字节，0x00 表示后续为命令
	Buf[1] = Command;
	OLED_I2C_Send(Buf, 2);
}

/**
  * 函    数：OLED写数据
  * 参    数：Data 要写入数据的起始地址
  * 参    数：Count 要写入数据的数量，范围：0~128
  * 返 回 值：无
  * 说    明：一次 I2C 事务发送 [控制字节0x40 + 连续数据]，
  *           比逐字节启停总线效率更高，且符合 SSD1306 连续写规范
  */
void OLED_WriteData(uint8_t *Data, uint8_t Count)
{
	uint8_t Buf[129];		//1 控制字节 + 最多 128 数据
	uint8_t i;

	Buf[0] = 0x40;			//控制字节，0x40 表示后续为数据
	for (i = 0; i < Count; i ++)
	{
		Buf[i + 1] = Data[i];
	}
	OLED_I2C_Send(Buf, (uint16_t)(Count + 1));
}

/*********************通信协议*/


/*硬件配置*********************/

/**
  * 函    数：OLED初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：使用前，需要调用此初始化函数。
  *           本移植版不再配置 GPIO：I2C2 的时钟、引脚(AF开漏)、速率
  *           已由 CubeMX 的 MX_I2C2_Init() 在 main() 中完成初始化，
  *           且调用顺序早于本函数（MX_FREERTOS_Init 阶段）。
  *           注意：本函数应在调度器启动前或任务内调用，HAL_Delay 依赖
  *           TIM1 时基中断（FreeRTOS 场景下 SysTick 已让给内核）。
  */
void OLED_Init(void)
{
	/*在初始化前，加入适量延时，待OLED供电稳定*/
	HAL_Delay(100);

	/*先做总线自救+外设重置：清除"复位不断OLED电源"遗留的 SDA 拉低 / BUSY 状态，
	  保证下面的初始化命令序列能真正发进 OLED（否则充电泵不开=黑屏）*/
	OLED_I2C_BusRecovery();

	/*写入一系列的命令，对OLED进行初始化配置*/
	OLED_WriteCommand(0xAE);	//设置显示开启/关闭，0xAE关闭，0xAF开启

	OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);	//0x00~0xFF

	OLED_WriteCommand(0xA8);	//设置多路复用率
	OLED_WriteCommand(0x3F);	//0x0E~0x3F

	OLED_WriteCommand(0xD3);	//设置显示偏移
	OLED_WriteCommand(0x00);	//0x00~0x7F

	OLED_WriteCommand(0x40);	//设置显示开始行，0x40~0x7F

	OLED_WriteCommand(0xA1);	//设置左右方向，0xA1正常，0xA0左右反置

	OLED_WriteCommand(0xC8);	//设置上下方向，0xC8正常，0xC0上下反置

	OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand(0x12);

	OLED_WriteCommand(0x81);	//设置对比度
	OLED_WriteCommand(0xCF);	//0x00~0xFF

	OLED_WriteCommand(0xD9);	//设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭

	OLED_WriteCommand(0xA6);	//设置正常/反色显示，0xA6正常，0xA7反色

	OLED_WriteCommand(0x8D);	//设置充电泵
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);	//开启显示

	OLED_Clear();				//清空显存数组
	OLED_Update();				//更新显示，清屏，防止初始化后未显示内容时花屏
}

/**
  * 函    数：OLED设置显示光标位置
  * 参    数：Page 指定光标所在的页，范围：0~7
  * 参    数：X 指定光标所在的X轴坐标，范围：0~127
  * 返 回 值：无
  * 说    明：OLED默认的Y轴，只能8个Bit为一组写入，即1页等于8个Y轴坐标
  */
void OLED_SetCursor(uint8_t Page, uint8_t X)
{
	/*如果使用此程序驱动1.3寸的OLED显示屏，则需要解除此注释*/
	/*因为1.3寸的OLED驱动芯片（SH1106）有132列*/
	/*屏幕的起始列接在了第2列，而不是第0列*/
	/*所以需要将X加2，才能正常显示*/
//	X += 2;

	/*通过指令设置页地址和列地址*/
	OLED_WriteCommand(0xB0 | Page);					//设置页位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			//设置X位置低4位
}

/*********************硬件配置*/


/*工具函数*********************/

/*工具函数仅供内部部分函数使用*/

/**
  * 函    数：次方函数
  * 参    数：X 底数
  * 参    数：Y 指数
  * 返 回 值：等于X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//结果默认为1
	while (Y --)			//累乘Y次
	{
		Result *= X;		//每次把X累乘到结果上
	}
	return Result;
}

/**
  * 函    数：判断指定点是否在指定多边形内部
  * 参    数：nvert 多边形的顶点数
  * 参    数：vertx verty 包含多边形顶点的x和y坐标的数组
  * 参    数：testx testy 测试点的X和y坐标
  * 返 回 值：指定点是否在指定多边形内部，1：在内部，0：不在内部
  */
uint8_t OLED_pnpoly(uint8_t nvert, int16_t *vertx, int16_t *verty, int16_t testx, int16_t testy)
{
	int16_t i, j, c = 0;

	/*此算法由W. Randolph Franklin提出*/
	/*参考链接：https://wrfranklin.org/Research/Short_Notes/pnpoly.html*/
	for (i = 0, j = nvert - 1; i < nvert; j = i++)
	{
		if (((verty[i] > testy) != (verty[j] > testy)) &&
			(testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
		{
			c = !c;
		}
	}
	return c;
}

/**
  * 函    数：判断指定点是否在指定角度内部
  * 参    数：X Y 指定点的坐标
  * 参    数：StartAngle EndAngle 起始角度和终止角度，范围：-180~180
  *           水平向右为0度，水平向左为180度或-180度，下方为正数，上方为负数，顺时针旋转
  * 返 回 值：指定点是否在指定角度内部，1：在内部，0：不在内部
  */
uint8_t OLED_IsInAngle(int16_t X, int16_t Y, int16_t StartAngle, int16_t EndAngle)
{
	int16_t PointAngle;
	PointAngle = atan2(Y, X) / 3.14 * 180;	//计算指定点的弧度，并转换为角度表示
	if (StartAngle < EndAngle)	//起始角度小于终止角度的情况
	{
		/*如果指定角度在起始终止角度之间，则判定指定点在指定角度*/
		if (PointAngle >= StartAngle && PointAngle <= EndAngle)
		{
			return 1;
		}
	}
	else			//起始角度大于于终止角度的情况
	{
		/*如果指定角度大于起始角度或者小于终止角度，则判定指定点在指定角度*/
		if (PointAngle >= StartAngle || PointAngle <= EndAngle)
		{
			return 1;
		}
	}
	return 0;	//不满足以上条件，则判断判定指定点不在指定角度
}

/*********************工具函数*/


/*功能函数*********************/

/**
  * 函    数：将OLED显存数组更新到OLED屏幕
  * 参    数：无
  * 返 回 值：无
  * 说    明：所有的显示函数，都只是对OLED显存数组进行读写
  *           随后调用OLED_Update函数或OLED_UpdateArea函数
  *           才会将显存数组的数据发送到OLED硬件，进行显示
  *           故调用显示函数后，要想真正地呈现在屏幕上，还需调用更新函数
  */
void OLED_Update(void)
{
	uint8_t j;
	/*遍历每一页*/
	for (j = 0; j < 8; j ++)
	{
		/*设置光标位置为每一页的第一列*/
		OLED_SetCursor(j, 0);
		/*连续写入128个数据，将显存数组的数据写入到OLED硬件*/
		OLED_WriteData(OLED_DisplayBuf[j], 128);
	}
}

/**
  * 函    数：将OLED显存数组部分更新到OLED屏幕
  * 参    数：X 指定区域左上角的横坐标，范围：-32768~32767，屏幕区域：0~127
  * 参    数：Y 指定区域左上角的纵坐标，范围：-32768~32767，屏幕区域：0~63
  * 参    数：Width 指定区域的宽度，范围：0~128
  * 参    数：Height 指定区域的高度，范围：0~64
  * 返 回 值：无
  * 说    明：此函数会至少更新参数指定的区域
  *           如果更新区域Y轴只包含部分页，则同一页的剩余部分会跟随一起更新
  */
void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
	int16_t j;
	int16_t Page, Page1;

	/*负数坐标在计算页地址时需要加一个偏移*/
	/*(Y + Height - 1) / 8 + 1的目的是(Y + Height) / 8并向上取整*/
	Page = Y / 8;
	Page1 = (Y + Height - 1) / 8 + 1;
	if (Y < 0)
	{
		Page -= 1;
		Page1 -= 1;
	}

	/*遍历指定区域涉及的相关页*/
	for (j = Page; j < Page1; j ++)
	{
		if (X >= 0 && X <= 127 && j >= 0 && j <= 7)		//超出屏幕的内容不显示
		{
			/*设置光标位置为相关页的指定列*/
			OLED_SetCursor(j, X);
			/*连续写入Width个数据，将显存数组的数据写入到OLED硬件*/
			OLED_WriteData(&OLED_DisplayBuf[j][X], Width);
		}
	}
}

/**
  * 函    数：将OLED显存数组全部清零
  * 参    数：无
  * 返 回 值：无
  * 说    明：调用此函数后，要想真正地呈现在屏幕上，还需调用更新函数
  */
void OLED_Clear(void)
{
	uint8_t i, j;
	for (j = 0; j < 8; j ++)				//遍历8页
	{
		for (i = 0; i < 128; i ++)			//遍历128列
		{
			OLED_DisplayBuf[j][i] = 0x00;	//将显存数组数据全部清零
		}
	}
}

/**
  * 函    数：将OLED显存数组部分清零
  * 参    数：X 指定区域左上角的横坐标，范围：-32768~32767，屏幕区域：0~127
  * 参    数：Y 指定区域左上角的纵坐标，范围：-32768~32767，屏幕区域：0~63
  * 参    数：Width 指定区域的宽度，范围：0~128
  * 参    数：Height 指定区域的高度，范围：0~64
  * 返 回 值：无
  * 说    明：调用此函数后，要想真正地呈现在屏幕上，还需调用更新函数
  */
void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
	int16_t i, j;

	for (j = Y; j < Y + Height; j ++)	//遍历指定页
	{
		for (i = X; i < X + Width; i ++)	//遍历指定列
		{
			if (i >= 0 && i <= 127 && j >=0 && j <= 63)				//超出屏幕的内容不显示
			{
				OLED_DisplayBuf[j / 8][i] &= ~(0x01 << (j % 8));	//将显存数组指定数据清零
			}
		}
	}
}

/**
  * 函    数：将OLED显存数组全部取反
  * 参    数：无
  * 返 回 值：无
  * 说    明：调用此函数后，要想真正地呈现在屏幕上，还需调用更新函数
  */
void OLED_Reverse(void)
{
	uint8_t i, j;
	for (j = 0; j < 8; j ++)				//遍历8页
	{
		for (i = 0; i < 128; i ++)			//遍历128列
		{
			OLED_DisplayBuf[j][i] ^= 0xFF;	//将显存数组数据全部取反
		}
	}
}

/**
  * 函    数：将OLED显存数组部分取反
  * 参    数：X 指定区域左上角的横坐标，范围：-32768~32767，屏幕区域：0~127
  * 参    数：Y 指定区域左上角的纵坐标，范围：-32768~32767，屏幕区域：0~63
  * 参    数：Width 指定区域的宽度，范围：0~128
  * 参    数：Height 指定区域的高度，范围：0~64
  * 返 回 值：无
  * 说    明：调用此函数后，要想真正地呈现在屏幕上，还需调用更新函数
  */
void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
	int16_t i, j;

	for (j = Y; j < Y + Height; j ++)	//遍历指定页
	{
		for (i = X; i < X + Width; i ++)	//遍历指定列
		{
			if (i >= 0 && i <= 127 && j >=0 && j <= 63)			//超出屏幕的内容不显示
			{
				OLED_DisplayBuf[j / 8][i] ^= 0x01 << (j % 8);	//将显存数组指定数据取反
			}
		}
	}
}

/**
  * 函    数：OLED显示一个字符
  * 参    数：X 指定字符左上角的横坐标
  * 参    数：Y 指定字符左上角的纵坐标
  * 参    数：Char 指定要显示的字符，范围：ASCII码可见字符
  * 参    数：FontSize 指定字体大小
  *           范围：OLED_8X16		宽8像素，高16像素
  *                 OLED_6X8		宽6像素，高8像素
  * 返 回 值：无
  * 说    明：调用此函数后，要想真正地呈现在屏幕上，还需调用更新函数
  */
void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize)
{
	/*2026-08-24 边界保护：坐标完全出屏或字符越出ASCII字模表时直接忽略。
	  防止 snprintf 拼出的异常内容（控制字符、超长串）造成字模数组越界读，
	  越界读到的脏数据画进显存就是"错位乱码"的典型来源之一。*/
	if ((Char < ' ') || (Char > '~')) { return; }		/*只接受可见ASCII*/
	if ((X <= -8) || (X >= 128) || (Y <= -16) || (Y >= 64)) { return; }

	if (FontSize == OLED_8X16)		//字体为宽8像素，高16像素
	{
		/*将ASCII字模库OLED_F8x16的指定数据以8*16的图像格式显示*/
		OLED_ShowImage(X, Y, 8, 16, OLED_F8x16[Char - ' ']);
	}
	else if(FontSize == OLED_6X8)	//字体为宽6像素，高8像素
	{
		/*将ASCII字模库OLED_F6x8的指定数据以6*8的图像格式显示*/
		OLED_ShowImage(X, Y, 6, 8, OLED_F6x8[Char - ' ']);
	}
}

/**
  * 函    数：OLED显示字符串（支持ASCII码和中文混合写入）
  * 参    数：X 指定字符串左上角的横坐标
  * 参    数：Y 指定字符串左上角的纵坐标
  * 参    数：String 指定要显示的字符串，范围：ASCII码可见字符或中文字符组成的字符串
  * 参    数：FontSize 指定字体大小
  * 返 回 值：无
  * 说    明：显示的中文字符需要在OLED_Data.c里的OLED_CF16x16数组定义
  *           未找到指定中文字符时，会显示默认图形（一个方框，内部一个问号）
  */
void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize)
{
	uint16_t i = 0;
	char SingleChar[5];
	uint8_t CharLength = 0;
	uint16_t XOffset = 0;
	uint16_t pIndex;

	while (String[i] != '\0')	//遍历字符串
	{
		/*2026-08-24 宽度保护：当前绘制位置已出右边界则停止，
		  避免超长字符串继续往显存右界外写（配合 ShowChar 的边界检查，
		  双保险杜绝横向错位/越界）*/
		if ((X + (int16_t)XOffset) >= 128) { break; }

#ifdef OLED_CHARSET_UTF8						//定义字符集为UTF8
		/*此段代码的目的是，提取UTF8字符串中的一个字符，转存到SingleChar子字符串中*/
		/*判断UTF8编码第一个字节的标志位*/
		if ((String[i] & 0x80) == 0x00)			//第一个字节为0xxxxxxx
		{
			CharLength = 1;						//字符为1字节
			SingleChar[0] = String[i ++];		//将第一个字节写入SingleChar第0个位置，随后i指向下一个字节
			SingleChar[1] = '\0';				//为SingleChar添加字符串结束标志位
		}
		else if ((String[i] & 0xE0) == 0xC0)	//第一个字节为110xxxxx
		{
			CharLength = 2;						//字符为2字节
			SingleChar[0] = String[i ++];
			if (String[i] == '\0') {break;}		//意外情况，跳出循环，结束显示
			SingleChar[1] = String[i ++];
			SingleChar[2] = '\0';
		}
		else if ((String[i] & 0xF0) == 0xE0)	//第一个字节为1110xxxx
		{
			CharLength = 3;						//字符为3字节
			SingleChar[0] = String[i ++];
			if (String[i] == '\0') {break;}
			SingleChar[1] = String[i ++];
			if (String[i] == '\0') {break;}
			SingleChar[2] = String[i ++];
			SingleChar[3] = '\0';
		}
		else if ((String[i] & 0xF8) == 0xF0)	//第一个字节为11110xxx
		{
			CharLength = 4;						//字符为4字节
			SingleChar[0] = String[i ++];
			if (String[i] == '\0') {break;}
			SingleChar[1] = String[i ++];
			if (String[i] == '\0') {break;}
			SingleChar[2] = String[i ++];
			if (String[i] == '\0') {break;}
			SingleChar[3] = String[i ++];
			SingleChar[4] = '\0';
		}
		else
		{
			i ++;			//意外情况，i指向下一个字节，忽略此字节，继续判断下一个字节
			continue;
		}
#endif

#ifdef OLED_CHARSET_GB2312						//定义字符集为GB2312
		/*此段代码的目的是，提取GB2312字符串中的一个字符，转存到SingleChar子字符串中*/
		/*判断GB2312字节的最高位标志位*/
		if ((String[i] & 0x80) == 0x00)			//最高位为0
		{
			CharLength = 1;						//字符为1字节
			SingleChar[0] = String[i ++];
			SingleChar[1] = '\0';
		}
		else									//最高位为1
		{
			CharLength = 2;						//字符为2字节
			SingleChar[0] = String[i ++];
			if (String[i] == '\0') {break;}
			SingleChar[1] = String[i ++];
			SingleChar[2] = '\0';
		}
#endif

		/*显示上述代码提取到的SingleChar*/
		if (CharLength == 1)	//如果是单字节字符
		{
			/*使用OLED_ShowChar显示此字符*/
			OLED_ShowChar(X + XOffset, Y, SingleChar[0], FontSize);
			XOffset += FontSize;
		}
		else					//否则，即多字节字符
		{
			/*遍历整个字模库，从字模库中寻找此字符的数据*/
			/*如果找到最后一个字符（定义为空字符串），则表示字符未在字模库定义，停止寻找*/
			for (pIndex = 0; strcmp(OLED_CF16x16[pIndex].Index, "") != 0; pIndex ++)
			{
				/*找到匹配的字符*/
				if (strcmp(OLED_CF16x16[pIndex].Index, SingleChar) == 0)
				{
					break;		//跳出循环，此时pIndex的值为指定字符的索引
				}
			}
			if (FontSize == OLED_8X16)		//给定字体为8*16点阵
			{
				/*将字模库OLED_CF16x16的指定数据以16*16的图像格式显示*/
				OLED_ShowImage(X + XOffset, Y, 16, 16, OLED_CF16x16[pIndex].Data);
				XOffset += 16;
			}
			else if (FontSize == OLED_6X8)	//给定字体为6*8点阵
			{
				/*空间不足，此位置显示'?'*/
				OLED_ShowChar(X + XOffset, Y, '?', OLED_6X8);
				XOffset += OLED_6X8;
			}
		}
	}
}

/**
  * 函    数：OLED显示数字（十进制，正整数）
  */
void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i;
	for (i = 0; i < Length; i++)		//遍历数字的每一位
	{
		/*Number / OLED_Pow(10, Length - i - 1) % 10 可以十进制提取数字的每一位*/
		/*+ '0' 可将数字转换为字符格式*/
		OLED_ShowChar(X + i * FontSize, Y, Number / OLED_Pow(10, Length - i - 1) % 10 + '0', FontSize);
	}
}

/**
  * 函    数：OLED显示有符号数字（十进制，整数）
  */
void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i;
	uint32_t Number1;

	if (Number >= 0)						//数字大于等于0
	{
		OLED_ShowChar(X, Y, '+', FontSize);	//显示+号
		Number1 = Number;					//Number1直接等于Number
	}
	else									//数字小于0
	{
		OLED_ShowChar(X, Y, '-', FontSize);	//显示-号
		Number1 = -Number;					//Number1等于Number取负
	}

	for (i = 0; i < Length; i++)			//遍历数字的每一位
	{
		OLED_ShowChar(X + (i + 1) * FontSize, Y, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0', FontSize);
	}
}

/**
  * 函    数：OLED显示十六进制数字（十六进制，正整数）
  */
void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)	//遍历数字的每一位
	{
		/*以十六进制提取数字的每一位*/
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;

		if (SingleNumber < 10)			//单个数字小于10
		{
			OLED_ShowChar(X + i * FontSize, Y, SingleNumber + '0', FontSize);
		}
		else							//单个数字大于10
		{
			OLED_ShowChar(X + i * FontSize, Y, SingleNumber - 10 + 'A', FontSize);
		}
	}
}

/**
  * 函    数：OLED显示二进制数字（二进制，正整数）
  */
void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i;
	for (i = 0; i < Length; i++)	//遍历数字的每一位
	{
		/*Number / OLED_Pow(2, Length - i - 1) % 2 可以二进制提取数字的每一位*/
		OLED_ShowChar(X + i * FontSize, Y, Number / OLED_Pow(2, Length - i - 1) % 2 + '0', FontSize);
	}
}

/**
  * 函    数：OLED显示浮点数字（十进制，小数）
  * 参    数：IntLength 整数位长度；FraLength 小数位长度，小数四舍五入显示
  */
void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize)
{
	uint32_t PowNum, IntNum, FraNum;

	if (Number >= 0)						//数字大于等于0
	{
		OLED_ShowChar(X, Y, '+', FontSize);	//显示+号
	}
	else									//数字小于0
	{
		OLED_ShowChar(X, Y, '-', FontSize);	//显示-号
		Number = -Number;					//Number取负
	}

	/*提取整数部分和小数部分*/
	IntNum = Number;						//直接赋值给整型变量，提取整数
	Number -= IntNum;						//将Number的整数减掉，防止之后把小数乘到整数时因数过大造成错误
	PowNum = OLED_Pow(10, FraLength);		//根据指定小数的位数，确定乘数
	FraNum = round(Number * PowNum);		//将小数乘到整数，同时四舍五入，避免显示误差
	IntNum += FraNum / PowNum;				//若四舍五入造成了进位，则需要再加给整数

	/*显示整数部分*/
	OLED_ShowNum(X + FontSize, Y, IntNum, IntLength, FontSize);

	/*显示小数点*/
	OLED_ShowChar(X + (IntLength + 1) * FontSize, Y, '.', FontSize);

	/*显示小数部分*/
	OLED_ShowNum(X + (IntLength + 2) * FontSize, Y, FraNum, FraLength, FontSize);
}

/**
  * 函    数：OLED显示图像
  * 参    数：X Y 图像左上角坐标；Width Height 图像宽高；Image 图像数据
  * 返 回 值：无
  * 说    明：调用此函数后，要想真正地呈现在屏幕上，还需调用更新函数
  */
void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image)
{
	uint8_t i = 0, j = 0;
	int16_t Page, Shift;

	/*将图像所在区域清空*/
	OLED_ClearArea(X, Y, Width, Height);

	/*遍历指定图像涉及的相关页*/
	/*(Height - 1) / 8 + 1的目的是Height / 8并向上取整*/
	for (j = 0; j < (Height - 1) / 8 + 1; j ++)
	{
		/*遍历指定图像涉及的相关列*/
		for (i = 0; i < Width; i ++)
		{
			if (X + i >= 0 && X + i <= 127)		//超出屏幕的内容不显示
			{
				/*负数坐标在计算页地址和移位时需要加一个偏移*/
				Page = Y / 8;
				Shift = Y % 8;
				if (Y < 0)
				{
					Page -= 1;
					Shift += 8;
				}

				if (Page + j >= 0 && Page + j <= 7)	//超出屏幕的内容不显示
				{
					/*显示图像在当前页的内容*/
					OLED_DisplayBuf[Page + j][X + i] |= Image[j * Width + i] << (Shift);
				}

				/*2026-08-24：Shift==0 时图像恰好页对齐，下一页无内容；
				  且 uint8_t 右移 8 位属未定义行为，必须跳过*/
				if ((Shift != 0) && (Page + j + 1 >= 0) && (Page + j + 1 <= 7))	//超出屏幕的内容不显示
				{
					/*显示图像在下一页的内容*/
					OLED_DisplayBuf[Page + j + 1][X + i] |= (uint8_t)(Image[j * Width + i] >> (8 - Shift));
				}
			}
		}
	}
}

/**
  * 函    数：OLED使用printf函数打印格式化字符串（支持ASCII码和中文混合写入）
  * 说    明：调用此函数后，要想真正地呈现在屏幕上，还需调用更新函数
  */
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...)
{
	char String[256];						//定义字符数组
	va_list arg;							//定义可变参数列表数据类型的变量arg
	va_start(arg, format);					//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);			//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);							//结束变量arg
	OLED_ShowString(X, Y, String, FontSize);//OLED显示字符数组（字符串）
}

/**
  * 函    数：OLED在指定位置画一个点
  */
void OLED_DrawPoint(int16_t X, int16_t Y)
{
	if (X >= 0 && X <= 127 && Y >=0 && Y <= 63)		//超出屏幕的内容不显示
	{
		/*将显存数组指定位置的一个Bit数据置1*/
		OLED_DisplayBuf[Y / 8][X] |= 0x01 << (Y % 8);
	}
}

/**
  * 函    数：OLED获取指定位置点的值
  * 返 回 值：指定位置点是否处于点亮状态，1：点亮，0：熄灭
  */
uint8_t OLED_GetPoint(int16_t X, int16_t Y)
{
	if (X >= 0 && X <= 127 && Y >=0 && Y <= 63)		//超出屏幕的内容不读取
	{
		/*判断指定位置的数据*/
		if (OLED_DisplayBuf[Y / 8][X] & 0x01 << (Y % 8))
		{
			return 1;	//为1，返回1
		}
	}

	return 0;		//否则，返回0
}

/**
  * 函    数：OLED画线（横线/竖线特判，斜线用Bresenham算法）
  */
void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1)
{
	int16_t x, y, dx, dy, d, incrE, incrNE, temp;
	int16_t x0 = X0, y0 = Y0, x1 = X1, y1 = Y1;
	uint8_t yflag = 0, xyflag = 0;

	if (y0 == y1)	//横线单独处理
	{
		/*0号点X坐标大于1号点X坐标，则交换两点X坐标*/
		if (x0 > x1) {temp = x0; x0 = x1; x1 = temp;}

		/*遍历X坐标*/
		for (x = x0; x <= x1; x ++)
		{
			OLED_DrawPoint(x, y0);	//依次画点
		}
	}
	else if (x0 == x1)	//竖线单独处理
	{
		/*0号点Y坐标大于1号点Y坐标，则交换两点Y坐标*/
		if (y0 > y1) {temp = y0; y0 = y1; y1 = temp;}

		/*遍历Y坐标*/
		for (y = y0; y <= y1; y ++)
		{
			OLED_DrawPoint(x0, y);	//依次画点
		}
	}
	else				//斜线
	{
		/*使用Bresenham算法画直线，可以避免耗时的浮点运算，效率更高*/

		if (x0 > x1)	//0号点X坐标大于1号点X坐标
		{
			/*交换两点坐标*/
			temp = x0; x0 = x1; x1 = temp;
			temp = y0; y0 = y1; y1 = temp;
		}

		if (y0 > y1)	//0号点Y坐标大于1号点Y坐标
		{
			/*将Y坐标取负*/
			y0 = -y0;
			y1 = -y1;

			/*置标志位yflag，记住当前变换，在后续实际画线时，再将坐标换回来*/
			yflag = 1;
		}

		if (y1 - y0 > x1 - x0)	//画线斜率大于1
		{
			/*将X坐标与Y坐标互换*/
			temp = x0; x0 = y0; y0 = temp;
			temp = x1; x1 = y1; y1 = temp;

			/*置标志位xyflag，记住当前变换，在后续实际画线时，再将坐标换回来*/
			xyflag = 1;
		}

		/*以下为Bresenham算法画直线*/
		/*算法要求，画线方向必须为第一象限0~45度范围*/
		dx = x1 - x0;
		dy = y1 - y0;
		incrE = 2 * dy;
		incrNE = 2 * (dy - dx);
		d = 2 * dy - dx;
		x = x0;
		y = y0;

		/*画起始点，同时判断标志位，将坐标换回来*/
		if (yflag && xyflag){OLED_DrawPoint(y, -x);}
		else if (yflag)		{OLED_DrawPoint(x, -y);}
		else if (xyflag)	{OLED_DrawPoint(y, x);}
		else				{OLED_DrawPoint(x, y);}

		while (x < x1)	//遍历X轴的每个点
		{
			x ++;
			if (d < 0)	//下一个点在当前点东方
			{
				d += incrE;
			}
			else		//下一个点在当前点东北方
			{
				y ++;
				d += incrNE;
			}

			/*画每一个点，同时判断标志位，将坐标换回来*/
			if (yflag && xyflag){OLED_DrawPoint(y, -x);}
			else if (yflag)		{OLED_DrawPoint(x, -y);}
			else if (xyflag)	{OLED_DrawPoint(y, x);}
			else				{OLED_DrawPoint(x, y);}
		}
	}
}

/**
  * 函    数：OLED矩形
  * 参    数：IsFilled OLED_UNFILLED不填充 / OLED_FILLED填充
  */
void OLED_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled)
{
	int16_t i, j;
	if (!IsFilled)		//指定矩形不填充
	{
		/*遍历上下X坐标，画矩形上下两条线*/
		for (i = X; i < X + Width; i ++)
		{
			OLED_DrawPoint(i, Y);
			OLED_DrawPoint(i, Y + Height - 1);
		}
		/*遍历左右Y坐标，画矩形左右两条线*/
		for (i = Y; i < Y + Height; i ++)
		{
			OLED_DrawPoint(X, i);
			OLED_DrawPoint(X + Width - 1, i);
		}
	}
	else				//指定矩形填充
	{
		/*遍历X坐标*/
		for (i = X; i < X + Width; i ++)
		{
			/*遍历Y坐标*/
			for (j = Y; j < Y + Height; j ++)
			{
				/*在指定区域画点，填充满矩形*/
				OLED_DrawPoint(i, j);
			}
		}
	}
}

/**
  * 函    数：OLED三角形
  * 参    数：IsFilled OLED_UNFILLED不填充 / OLED_FILLED填充
  */
void OLED_DrawTriangle(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint8_t IsFilled)
{
	int16_t minx = X0, miny = Y0, maxx = X0, maxy = Y0;
	int16_t i, j;
	int16_t vx[] = {X0, X1, X2};
	int16_t vy[] = {Y0, Y1, Y2};

	if (!IsFilled)			//指定三角形不填充
	{
		/*调用画线函数，将三个点用直线连接*/
		OLED_DrawLine(X0, Y0, X1, Y1);
		OLED_DrawLine(X0, Y0, X2, Y2);
		OLED_DrawLine(X1, Y1, X2, Y2);
	}
	else					//指定三角形填充
	{
		/*找到三个点最小的X、Y坐标*/
		if (X1 < minx) {minx = X1;}
		if (X2 < minx) {minx = X2;}
		if (Y1 < miny) {miny = Y1;}
		if (Y2 < miny) {miny = Y2;}

		/*找到三个点最大的X、Y坐标*/
		if (X1 > maxx) {maxx = X1;}
		if (X2 > maxx) {maxx = X2;}
		if (Y1 > maxy) {maxy = Y1;}
		if (Y2 > maxy) {maxy = Y2;}

		/*最小最大坐标之间的矩形为可能需要填充的区域*/
		/*遍历此区域中所有的点*/
		for (i = minx; i <= maxx; i ++)
		{
			for (j = miny; j <= maxy; j ++)
			{
				/*调用OLED_pnpoly，判断指定点是否在指定三角形之中*/
				if (OLED_pnpoly(3, vx, vy, i, j)) {OLED_DrawPoint(i, j);}
			}
		}
	}
}

/**
  * 函    数：OLED画圆（Bresenham算法）
  * 参    数：IsFilled OLED_UNFILLED不填充 / OLED_FILLED填充
  */
void OLED_DrawCircle(int16_t X, int16_t Y, uint8_t Radius, uint8_t IsFilled)
{
	int16_t x, y, d, j;

	d = 1 - Radius;
	x = 0;
	y = Radius;

	/*画每个八分之一圆弧的起始点*/
	OLED_DrawPoint(X + x, Y + y);
	OLED_DrawPoint(X - x, Y - y);
	OLED_DrawPoint(X + y, Y + x);
	OLED_DrawPoint(X - y, Y - x);

	if (IsFilled)	//指定圆填充
	{
		/*遍历起始点Y坐标*/
		for (j = -y; j < y; j ++)
		{
			/*在指定区域画点，填充部分圆*/
			OLED_DrawPoint(X, Y + j);
		}
	}

	while (x < y)	//遍历X轴的每个点
	{
		x ++;
		if (d < 0)	//下一个点在当前点东方
		{
			d += 2 * x + 1;
		}
		else		//下一个点在当前点东南方
		{
			y --;
			d += 2 * (x - y) + 1;
		}

		/*画每个八分之一圆弧的点*/
		OLED_DrawPoint(X + x, Y + y);
		OLED_DrawPoint(X + y, Y + x);
		OLED_DrawPoint(X - x, Y - y);
		OLED_DrawPoint(X - y, Y - x);
		OLED_DrawPoint(X + x, Y - y);
		OLED_DrawPoint(X + y, Y - x);
		OLED_DrawPoint(X - x, Y + y);
		OLED_DrawPoint(X - y, Y + x);

		if (IsFilled)	//指定圆填充
		{
			/*遍历中间部分*/
			for (j = -y; j < y; j ++)
			{
				OLED_DrawPoint(X + x, Y + j);
				OLED_DrawPoint(X - x, Y + j);
			}

			/*遍历两侧部分*/
			for (j = -x; j < x; j ++)
			{
				OLED_DrawPoint(X - y, Y + j);
				OLED_DrawPoint(X + y, Y + j);
			}
		}
	}
}

/**
  * 函    数：OLED画椭圆（中点算法）
  * 参    数：A 横向半轴；B 纵向半轴；IsFilled 是否填充
  */
void OLED_DrawEllipse(int16_t X, int16_t Y, uint8_t A, uint8_t B, uint8_t IsFilled)
{
	int16_t x, y, j;
	int16_t a = A, b = B;
	float d1, d2;

	x = 0;
	y = b;
	d1 = b * b + a * a * (-b + 0.5);

	if (IsFilled)	//指定椭圆填充
	{
		/*遍历起始点Y坐标*/
		for (j = -y; j < y; j ++)
		{
			OLED_DrawPoint(X, Y + j);
		}
	}

	/*画椭圆弧的起始点*/
	OLED_DrawPoint(X + x, Y + y);
	OLED_DrawPoint(X - x, Y - y);
	OLED_DrawPoint(X - x, Y + y);
	OLED_DrawPoint(X + x, Y - y);

	/*画椭圆中间部分*/
	while (b * b * (x + 1) < a * a * (y - 0.5))
	{
		if (d1 <= 0)	//下一个点在当前点东方
		{
			d1 += b * b * (2 * x + 3);
		}
		else			//下一个点在当前点东南方
		{
			d1 += b * b * (2 * x + 3) + a * a * (-2 * y + 2);
			y --;
		}
		x ++;

		if (IsFilled)	//指定椭圆填充
		{
			/*遍历中间部分*/
			for (j = -y; j < y; j ++)
			{
				OLED_DrawPoint(X + x, Y + j);
				OLED_DrawPoint(X - x, Y + j);
			}
		}

		/*画椭圆中间部分圆弧*/
		OLED_DrawPoint(X + x, Y + y);
		OLED_DrawPoint(X - x, Y - y);
		OLED_DrawPoint(X - x, Y + y);
		OLED_DrawPoint(X + x, Y - y);
	}

	/*画椭圆两侧部分*/
	d2 = b * b * (x + 0.5) * (x + 0.5) + a * a * (y - 1) * (y - 1) - a * a * b * b;

	while (y > 0)
	{
		if (d2 <= 0)	//下一个点在当前点东方
		{
			d2 += b * b * (2 * x + 2) + a * a * (-2 * y + 3);
			x ++;
		}
		else			//下一个点在当前点东南方
		{
			d2 += a * a * (-2 * y + 3);
		}
		y --;

		if (IsFilled)	//指定椭圆填充
		{
			/*遍历两侧部分*/
			for (j = -y; j < y; j ++)
			{
				OLED_DrawPoint(X + x, Y + j);
				OLED_DrawPoint(X - x, Y + j);
			}
		}

		/*画椭圆两侧部分圆弧*/
		OLED_DrawPoint(X + x, Y + y);
		OLED_DrawPoint(X - x, Y - y);
		OLED_DrawPoint(X - x, Y + y);
		OLED_DrawPoint(X + x, Y - y);
	}
}

/**
  * 函    数：OLED画圆弧（借用画圆算法 + 角度过滤）
  * 参    数：StartAngle EndAngle 角度范围：-180~180
  * 参    数：IsFilled 填充后为扇形
  */
void OLED_DrawArc(int16_t X, int16_t Y, uint8_t Radius, int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled)
{
	int16_t x, y, d, j;

	d = 1 - Radius;
	x = 0;
	y = Radius;

	/*在画圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理*/
	if (OLED_IsInAngle(x, y, StartAngle, EndAngle))	{OLED_DrawPoint(X + x, Y + y);}
	if (OLED_IsInAngle(-x, -y, StartAngle, EndAngle)) {OLED_DrawPoint(X - x, Y - y);}
	if (OLED_IsInAngle(y, x, StartAngle, EndAngle)) {OLED_DrawPoint(X + y, Y + x);}
	if (OLED_IsInAngle(-y, -x, StartAngle, EndAngle)) {OLED_DrawPoint(X - y, Y - x);}

	if (IsFilled)	//指定圆弧填充
	{
		/*遍历起始点Y坐标*/
		for (j = -y; j < y; j ++)
		{
			if (OLED_IsInAngle(0, j, StartAngle, EndAngle)) {OLED_DrawPoint(X, Y + j);}
		}
	}

	while (x < y)	//遍历X轴的每个点
	{
		x ++;
		if (d < 0)	//下一个点在当前点东方
		{
			d += 2 * x + 1;
		}
		else		//下一个点在当前点东南方
		{
			y --;
			d += 2 * (x - y) + 1;
		}

		/*在画圆的每个点时，判断指定点是否在指定角度内*/
		if (OLED_IsInAngle(x, y, StartAngle, EndAngle)) {OLED_DrawPoint(X + x, Y + y);}
		if (OLED_IsInAngle(y, x, StartAngle, EndAngle)) {OLED_DrawPoint(X + y, Y + x);}
		if (OLED_IsInAngle(-x, -y, StartAngle, EndAngle)) {OLED_DrawPoint(X - x, Y - y);}
		if (OLED_IsInAngle(-y, -x, StartAngle, EndAngle)) {OLED_DrawPoint(X - y, Y - x);}
		if (OLED_IsInAngle(x, -y, StartAngle, EndAngle)) {OLED_DrawPoint(X + x, Y - y);}
		if (OLED_IsInAngle(y, -x, StartAngle, EndAngle)) {OLED_DrawPoint(X + y, Y - x);}
		if (OLED_IsInAngle(-x, y, StartAngle, EndAngle)) {OLED_DrawPoint(X - x, Y + y);}
		if (OLED_IsInAngle(-y, x, StartAngle, EndAngle)) {OLED_DrawPoint(X - y, Y + x);}

		if (IsFilled)	//指定圆弧填充
		{
			/*遍历中间部分*/
			for (j = -y; j < y; j ++)
			{
				if (OLED_IsInAngle(x, j, StartAngle, EndAngle)) {OLED_DrawPoint(X + x, Y + j);}
				if (OLED_IsInAngle(-x, j, StartAngle, EndAngle)) {OLED_DrawPoint(X - x, Y + j);}
			}

			/*遍历两侧部分*/
			for (j = -x; j < x; j ++)
			{
				if (OLED_IsInAngle(-y, j, StartAngle, EndAngle)) {OLED_DrawPoint(X - y, Y + j);}
				if (OLED_IsInAngle(y, j, StartAngle, EndAngle)) {OLED_DrawPoint(X + y, Y + j);}
			}
		}
	}
}

/*********************功能函数*/


/* 注：原 HAL_I2C_MasterTxCpltCallback（DMA 完成回调）随 DMA 方案一并移除。
 * I2C2 现在不配置任何中断与 DMA，NVIC 中 I2C2_EV/ER 是否使能均不影响轮询工作。 */

/*****************江协科技|版权所有****************/
/*****************jiangxiekeji.com*****************/
