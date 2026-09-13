#include "Encoder.h"

void Encoder_Init()
{
  encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // ��ʼ��������ģ�������� �������������ģʽ
  encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // ��ʼ��������ģ�������� �������������ģʽ
  encoder_quad_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B); // ��ʼ��������ģ�������� �������������ģʽ
  encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B); // ��ʼ��������ģ�������� �������������ģʽ
}

void Get_Encoder(void)
{
  // 编码器值融合低通滤波
  static int last_encoder_count_L1 = 0;
  static int last_encoder_count_R1 = 0;
  static int last_encoder_count_L2 = 0;
  static int last_encoder_count_R2 = 0;
  encoder_data_quaddec_L1 = DITONG_NUM * encoder_get_count(ENCODER_4) + (1 - DITONG_NUM) * last_encoder_count_L1;
  encoder_data_quaddec_R1 = -DITONG_NUM * encoder_get_count(ENCODER_3) + (1 - DITONG_NUM) * last_encoder_count_R1;
  encoder_data_quaddec_L2 = DITONG_NUM * encoder_get_count(ENCODER_1) + (1 - DITONG_NUM) * last_encoder_count_L2;
  encoder_data_quaddec_R2 = -DITONG_NUM * encoder_get_count(ENCODER_2) + (1 - DITONG_NUM) * last_encoder_count_R2;
  encoder_clear_count(ENCODER_1);
  encoder_clear_count(ENCODER_2);
  encoder_clear_count(ENCODER_3);
  encoder_clear_count(ENCODER_4);
  last_encoder_count_L1 = encoder_data_quaddec_L1;
  last_encoder_count_R1 = encoder_data_quaddec_R1;
  last_encoder_count_L2 = encoder_data_quaddec_L2;
  last_encoder_count_R2 = encoder_data_quaddec_R2;
}
