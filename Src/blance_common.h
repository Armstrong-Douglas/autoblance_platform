

#ifndef ADC_DEBUG
#define ADC_DEBUG
#endif

#define ADC_MAX_CNT	5


#define hal_uint32	uint32_t
#define hal_double	double
#define hal_int8	char
#define hal_uint8	unsigned char

#define MAX_CAL_NUM	(10)
#define DIVISOR		(MAX_CAL_NUM - 2)

#define ABS(a)	((a) >= 0 ? (a):(-a) )

/*	blance_adc.c function	*/
extern void adc_init(void);
extern void angle_calculate(double *x_angle,double *y_angle);

