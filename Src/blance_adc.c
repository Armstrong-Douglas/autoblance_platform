
#include "blance_common.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "math.h"


/* Private function prototypes -----------------------------------------------*/
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void get_adc_value(hal_uint32 *ADC_value1,hal_uint32 *ADC_value2);

/* Private variables ---------------------------------------------------------*/
static ADC_HandleTypeDef hadc1;
static ADC_HandleTypeDef hadc2;


void adc_init()
{
	MX_ADC1_Init();
	MX_ADC2_Init();
}

void angle_calculate(double *x_angle,double *y_angle)
{
	hal_uint32	adc_x = 0;
	hal_uint32	adc_y = 0;
 	hal_double	temp_volt_1	= 0;
	hal_double  temp_volt_2	= 0;
	hal_double	mid_sin = 0;

	hal_double	x_degree = 0;
	hal_double	y_degree = 0;

	get_adc_value(&adc_x,&adc_y);									// get adc 5 times then return average value

/*	X-axis calculate 	*/
	temp_volt_1 = ((double)adc_x*(3.3/4096));
	temp_volt_1	= ( (float)( (int)( (temp_volt_1+0.0005)*100 ) ) )/100;
	mid_sin = (0.5 * temp_volt_1) - (0.25 * 4.90);					// arcsin(0.5* Vcc - 0.25*Vdd)
	mid_sin = ( (float)( (int)( (mid_sin+0.0005)*1000 ) ) )/1000;
	x_degree = (asin(mid_sin) )* (180/3.14);

#ifdef	ADC_DEBUG
		printf("temp_volt_1 = %lf  mid_sin = %lf\r\n",temp_volt_1,mid_sin);
		printf("X-degree = %f\r\n",x_degree);
#endif

	mid_sin = 0;
	
/*	Y-axis calculate	*/
	temp_volt_2 = ((double)adc_y*(3.3/4096));
	temp_volt_2	= ( (float)( (int)( (temp_volt_2+0.0005)*100 ) ) )/100;
	mid_sin = (0.5 * temp_volt_2) - (0.25 * 4.90);					// arcsin(0.5* Vcc - 0.25*Vdd)
	mid_sin = ( (float)( (int)( (mid_sin+0.0005)*1000 ) ) )/1000;
	y_degree = (asin(mid_sin) )* (180/3.14);

#ifdef	ADC_DEBUG
	printf("temp_volt_2 = %lf  mid_sin = %lf\r\n",temp_volt_2,mid_sin);
	printf("Y-degree = [%lf]\r\n",y_degree);
#endif

	*x_angle =	x_degree;
	*y_angle =	y_degree;
}

static hal_uint32 math_aver(uint32_t *array,uint32_t num)
{
	hal_uint32 i = 0;
	hal_uint32 sum = 0;
	hal_uint32 aver_num = 0;

	for (i = 0;i < num;i++)
	{
		sum += array[i];
	}

	aver_num = sum / 5;

	return aver_num;
}

static void get_adc_value(hal_uint32 *ADC_value1,hal_uint32 *ADC_value2)
{
	hal_uint32 i = 0;
	hal_uint32 adc_value_array_1[5] = {0};
	hal_uint32 adc_value_array_2[5] = {0};
		
	for (i = 0;i < ADC_MAX_CNT;i++)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_Start(&hadc2);

		HAL_ADC_PollForConversion(&hadc1, 50);
		HAL_ADC_PollForConversion(&hadc2, 50);
	 
		adc_value_array_1[i] = HAL_ADC_GetValue(&hadc1);
		adc_value_array_2[i] = HAL_ADC_GetValue(&hadc2);\

		HAL_Delay(10);
	}
	
	*ADC_value1 = math_aver(adc_value_array_1 ,ADC_MAX_CNT);
	*ADC_value2 = math_aver(adc_value_array_2 ,ADC_MAX_CNT);

#ifdef ADC_DEBUG
	printf("The Average ADC:\r\n");
	printf("ADC_1(x) = %d, ADC_1(y) = %d\r\n",*ADC_value1,*ADC_value2);
#endif

	return ;
}




/* ADC1 init function */
static void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef sConfig;

/*Common config*/
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}

/*Configure Regular Channel*/
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}

	return ;
}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{
	ADC_ChannelConfTypeDef sConfig;

/*Common config*/
	hadc2.Instance = ADC2;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}

/*Configure Regular Channel*/
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}

	return ;
}

