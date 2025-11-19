#include "stm32f4xx_hal.h"
#include <math.h>

/**
 * Example firmware fragment demonstrating how to drive a PMSM using the STM32 HAL.
 * The code configures TIM1 PWM outputs (PWM1/PWM2), samples phase currents
 * and voltages through ADC1, performs a basic FOC current loop, and modulates
 * the duty cycle according to the dq current regulator output.
 */

#define PWM_TIMER               TIM1
#define PWM_CHANNEL_U           TIM_CHANNEL_1
#define PWM_CHANNEL_V           TIM_CHANNEL_2
#define PWM_CHANNEL_W           TIM_CHANNEL_3

#define ADC_CURRENT_CHANNEL_U   ADC_CHANNEL_1
#define ADC_CURRENT_CHANNEL_V   ADC_CHANNEL_2
#define ADC_CURRENT_CHANNEL_W   ADC_CHANNEL_3
#define ADC_VOLTAGE_CHANNEL_U   ADC_CHANNEL_4
#define ADC_VOLTAGE_CHANNEL_V   ADC_CHANNEL_5
#define ADC_VOLTAGE_CHANNEL_W   ADC_CHANNEL_6

#define SQRT3_INV               (0.57735026919f)
#define TWO_PI                  (6.28318530718f)

typedef struct
{
    float i_alpha;
    float i_beta;
    float v_alpha;
    float v_beta;
} ClarkeFrame;

typedef struct
{
    float d;
    float q;
} DqFrame;

typedef struct
{
    float ki;
    float kp;
    float integral;
} PIController;

static TIM_HandleTypeDef htim1;
static ADC_HandleTypeDef hadc1;
static PIController id_controller = {.ki = 0.02f, .kp = 0.4f, .integral = 0.0f};
static PIController iq_controller = {.ki = 0.02f, .kp = 0.4f, .integral = 0.0f};
static float electrical_angle = 0.0f;
static const float electrical_speed = 100.0f; // rad/s reference speed
static const float dt = 0.0001f;              // 10 kHz control period

static void Error_Handler(void);
static void SystemClock_Config(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void PWM_SetDuty(float duty_u, float duty_v, float duty_w);
static void ADC_ReadPhaseValues(float *i_u, float *i_v, float *i_w,
                                float *v_u, float *v_v, float *v_w);
static ClarkeFrame clarke_transform(float i_u, float i_v, float i_w,
                                    float v_u, float v_v, float v_w);
static DqFrame park_transform(const ClarkeFrame *clarke, float angle);
static ClarkeFrame inverse_park(const DqFrame *dq, float angle);
static float pi_run(PIController *controller, float reference, float measured);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_TIM1_Init();
    MX_ADC1_Init();

    HAL_TIM_PWM_Start(&htim1, PWM_CHANNEL_U);
    HAL_TIM_PWM_Start(&htim1, PWM_CHANNEL_V);
    HAL_TIM_PWM_Start(&htim1, PWM_CHANNEL_W);

    while (1)
    {
        float i_u, i_v, i_w;
        float v_u, v_v, v_w;
        ADC_ReadPhaseValues(&i_u, &i_v, &i_w, &v_u, &v_v, &v_w);

        ClarkeFrame clarke = clarke_transform(i_u, i_v, i_w, v_u, v_v, v_w);
        DqFrame current_dq = park_transform(&clarke, electrical_angle);

        const float id_ref = 0.0f;             // Field weakening disabled
        const float iq_ref = 3.0f;             // Torque reference
        float vd = pi_run(&id_controller, id_ref, current_dq.d);
        float vq = pi_run(&iq_controller, iq_ref, current_dq.q);

        DqFrame voltage_dq = {.d = vd, .q = vq};
        ClarkeFrame voltage_alpha_beta = inverse_park(&voltage_dq, electrical_angle);

        float duty_u = 0.5f + voltage_alpha_beta.v_alpha;
        float duty_v = 0.5f + (-0.5f * voltage_alpha_beta.v_alpha + SQRT3_INV * voltage_alpha_beta.v_beta);
        float duty_w = 0.5f + (-0.5f * voltage_alpha_beta.v_alpha - SQRT3_INV * voltage_alpha_beta.v_beta);
        PWM_SetDuty(duty_u, duty_v, duty_w);

        electrical_angle += electrical_speed * dt;
        if (electrical_angle > TWO_PI)
        {
            electrical_angle -= TWO_PI;
        }

        HAL_Delay((uint32_t)(dt * 1000.0f));
    }
}

static void SystemClock_Config(void)
{
    // Configure PLL/clock tree per project requirements.
}

static void MX_TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period = 4200 - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = htim1.Init.Period / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, PWM_CHANNEL_U) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, PWM_CHANNEL_V) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, PWM_CHANNEL_W) != HAL_OK)
        Error_Handler();
}

static void MX_ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 6;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t channels[6] = {ADC_CURRENT_CHANNEL_U, ADC_CURRENT_CHANNEL_V, ADC_CURRENT_CHANNEL_W,
                            ADC_VOLTAGE_CHANNEL_U, ADC_VOLTAGE_CHANNEL_V, ADC_VOLTAGE_CHANNEL_W};

    for (uint32_t rank = 1; rank <= 6; ++rank)
    {
        sConfig.Channel = channels[rank - 1];
        sConfig.Rank = rank;
        sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
    }

    HAL_ADC_Start(&hadc1);
}

static void PWM_SetDuty(float duty_u, float duty_v, float duty_w)
{
    const float min_duty = 0.0f;
    const float max_duty = 1.0f;

    float duties[3] = {duty_u, duty_v, duty_w};
    uint32_t channels[3] = {PWM_CHANNEL_U, PWM_CHANNEL_V, PWM_CHANNEL_W};

    for (int i = 0; i < 3; ++i)
    {
        if (duties[i] < min_duty)
            duties[i] = min_duty;
        if (duties[i] > max_duty)
            duties[i] = max_duty;

        __HAL_TIM_SET_COMPARE(&htim1, channels[i], (uint32_t)(duties[i] * htim1.Init.Period));
    }
}

static void ADC_ReadPhaseValues(float *i_u, float *i_v, float *i_w,
                                float *v_u, float *v_v, float *v_w)
{
    float samples[6];

    for (int i = 0; i < 6; ++i)
    {
        if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
            Error_Handler();
        samples[i] = (float)HAL_ADC_GetValue(&hadc1);
    }

    const float scale_current = 1.0f / 4095.0f * 10.0f; // Example 10A full-scale
    const float scale_voltage = 1.0f / 4095.0f * 400.0f;

    *i_u = samples[0] * scale_current;
    *i_v = samples[1] * scale_current;
    *i_w = samples[2] * scale_current;
    *v_u = samples[3] * scale_voltage;
    *v_v = samples[4] * scale_voltage;
    *v_w = samples[5] * scale_voltage;
}

static ClarkeFrame clarke_transform(float i_u, float i_v, float i_w,
                                    float v_u, float v_v, float v_w)
{
    ClarkeFrame frame;
    frame.i_alpha = i_u;
    frame.i_beta = (i_u + 2.0f * i_v) * SQRT3_INV;
    frame.v_alpha = v_u;
    frame.v_beta = (v_u + 2.0f * v_v) * SQRT3_INV;
    return frame;
}

static DqFrame park_transform(const ClarkeFrame *clarke, float angle)
{
    float sin_a = sinf(angle);
    float cos_a = cosf(angle);

    DqFrame dq;
    dq.d = clarke->i_alpha * cos_a + clarke->i_beta * sin_a;
    dq.q = -clarke->i_alpha * sin_a + clarke->i_beta * cos_a;
    return dq;
}

static ClarkeFrame inverse_park(const DqFrame *dq, float angle)
{
    float sin_a = sinf(angle);
    float cos_a = cosf(angle);

    ClarkeFrame alpha_beta;
    alpha_beta.v_alpha = dq->d * cos_a - dq->q * sin_a;
    alpha_beta.v_beta = dq->d * sin_a + dq->q * cos_a;
    return alpha_beta;
}

static float pi_run(PIController *controller, float reference, float measured)
{
    float error = reference - measured;
    controller->integral += controller->ki * error * dt;
    const float integral_limit = 10.0f;
    if (controller->integral > integral_limit)
        controller->integral = integral_limit;
    else if (controller->integral < -integral_limit)
        controller->integral = -integral_limit;

    float output = controller->kp * error + controller->integral;
    return output;
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        // Trap CPU for debugging
    }
}
