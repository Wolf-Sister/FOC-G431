#include "foc.h"

/* 内部辅助函数 */
static void set_pwm_duty(float d_u, float d_v, float d_w) {
	d_u = _constrain(d_u, 0.0f, 0.9f);
    d_v = _constrain(d_v, 0.0f, 0.9f);
    d_w = _constrain(d_w, 0.0f, 0.9f);
	
	motor_control.du = d_u;
	motor_control.dv = d_v;
	motor_control.dw = d_w;
	
	__disable_irq();
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d_u * htim1.Instance->ARR);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d_v * htim1.Instance->ARR);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, d_w * htim1.Instance->ARR);
	__enable_irq();
}

static int SVM(float alpha, float beta, float* tA, float* tB, float* tC) {
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (_1_SQRT3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2

        } else {
            //quadrant II
            if (-_1_SQRT3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-_1_SQRT3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (_1_SQRT3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - _1_SQRT3 * beta;
            float t2 = _2_SQRT3 * beta;

            // PWM timings
            *tA = (1.0f - t1 - t2) * 0.5f;
            *tB = *tA + t1;
            *tC = *tB + t2;
        } break;

        // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + _1_SQRT3 * beta;
            float t3 = -alpha + _1_SQRT3 * beta;

            // PWM timings
            *tB = (1.0f - t2 - t3) * 0.5f;
            *tA = *tB + t3;
            *tC = *tA + t2;
        } break;

        // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = _2_SQRT3 * beta;
            float t4 = -alpha - _1_SQRT3 * beta;

            // PWM timings
            *tB = (1.0f - t3 - t4) * 0.5f;
            *tC = *tB + t3;
            *tA = *tC + t4;
        } break;

        // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + _1_SQRT3 * beta;
            float t5 = -_2_SQRT3 * beta;

            // PWM timings
            *tC = (1.0f - t4 - t5) * 0.5f;
            *tB = *tC + t5;
            *tA = *tB + t4;
        } break;

        // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - _1_SQRT3 * beta;
            float t6 = alpha - _1_SQRT3 * beta;

            // PWM timings
            *tC = (1.0f - t5 - t6) * 0.5f;
            *tA = *tC + t5;
            *tB = *tA + t6;
        } break;

        // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -_2_SQRT3 * beta;
            float t1 = alpha + _1_SQRT3 * beta;

            // PWM timings
            *tA = (1.0f - t6 - t1) * 0.5f;
            *tC = *tA + t1;
            *tB = *tC + t6;
        } break;
    }

    // if any of the results becomes NaN, result_valid will evaluate to false
    int result_valid =
            *tA >= 0.0f && *tA <= 1.0f
         && *tB >= 0.0f && *tB <= 1.0f
         && *tC >= 0.0f && *tC <= 1.0f;
    return result_valid ? 0 : -1;
}

void foc_forward(float d, float q, float angle_el) {
    float d_u = 0;
    float d_v = 0;
    float d_w = 0;
	
	// 回归电压模型
	float mod_to_V = (2.0f / 3.0f) * motor_config.voltage_supply;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d = V_to_mod * d;
    float mod_q = V_to_mod * q;
	motor_control.mod_q = mod_q;
	
	float mod_scalefactor = 0.80f * SQRT3_BY_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
    if (mod_scalefactor < 1.0f) 
	{
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
    } 
	
	//帕克逆变换
	float mod_alpha = mod_d * fast_cos_f32(angle_el) - mod_q * fast_sin_f32(angle_el);
    float mod_beta  = mod_d * fast_sin_f32(angle_el) + mod_q * fast_cos_f32(angle_el);

    SVM(mod_alpha, mod_beta, &d_u, &d_v, &d_w);
	
	set_pwm_duty(d_u, d_v, d_w);
}

