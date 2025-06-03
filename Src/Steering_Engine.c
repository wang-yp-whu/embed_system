#include "stm32f4xx_hal.h"
// ɾ������ #include "tim.h" 
// ��Ϊ������׼��ʱ��ͷ�ļ�
#include "tim.h"

// ������ƺ���
void SteeringEngine_Rotate(float angle) {
    // �Ƕ�ת��ΪPWMռ�ձ�
    // �������Ƕȷ�Χ0-180�ȶ�ӦPWMռ�ձ�2.5%-12.5%
    // ʹ��TIM3ͨ��1���PWM
    uint16_t pulse = (uint16_t)(500 + angle * (2000/180.0));  // 500-2500��Ӧ0-180��
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pulse);
}

// �����תһȦ����
void SteeringEngine_RotateFullCircle(void) {
    /* for(float angle = 0; angle <= 180; angle += 5) {
        SteeringEngine_Rotate(angle);
        HAL_Delay(50);
    }
    for(float angle = 180; angle >= 0; angle -= 5) {
        SteeringEngine_Rotate(angle);
        HAL_Delay(50);
    }
		*/
		 	for (int i = 2500; i >= 500; i--) {
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, i);
			HAL_Delay(1);
		}
         // ��λ���м�λ��
    HAL_Delay(100);
}
