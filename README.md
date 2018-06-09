# autoblance_platform
  自动平衡台的一个毕业设计，使用STM32f103作为处理器，sca60c作角度传感器，步进电机作控制平衡台俯仰角度。

接口定义：
UART调试口（UART1）：
	
	PA9			TX
	PA10		RX
	使用printf输出调试信息，角度
    
ADC：											
	PA5			ADC12_IN5		channel_5		hadc1			x
	PB0			ADC12_IN8		channel_8		hadc2			y

电机X轴:
	PA0			TIM2_CHAN1		PWM输出
	PB3						DIR正反转
	PB4						EN使能
	
电机Y轴:
	PA1			TIM2_CHAN2		PWM输出
	PB5						DIR正反转
	PB6						EN使能
