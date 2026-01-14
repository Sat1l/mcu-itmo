#include "main.h"  
#include <stdio.h>  
#include <string.h>

#define MIN_DUTY    80u   
#define MAX_DUTY    999u


// глобальные переменные, волатайл потому что они меняются в прерывании
volatile int16_t g_target_pos = 0;   // целевая позиция (задается внешним энкодером)
volatile int16_t g_current_pos = 0;  // текущая позиция (считает энкодер на моторе)
volatile int16_t g_speed = 0;        // скорость (путь за 10мс)
volatile int16_t g_error = 0;        // ошибка (целевое - текущее)
volatile int16_t g_output = 0;       // мощность пида
volatile uint8_t g_send_telemetry = 0; // флаг для синхронизации прерывания и main

// функция управления мотором
void motor_set_speed(int16_t speed)
{
    if (speed == 0) {       // если скорость ноль
        TIM1->CCR1 = 0;    // выключаем правый канал
        TIM1->CCR2 = 0;    // выключаем левый канал
        return;
    }
    
    // ограничиваем входную скорость для безопасности
    if (speed > 60) speed = 60;
    if (speed < -60) speed = -60;

    // масштабируем в диапазон MIN_DUTY - MAX_DUTY чтобы пид работал с удобными числами, а таймер с реальными тиками
    uint32_t abs_speed = (speed > 0) ? speed : -speed;
    uint32_t duty = MIN_DUTY + (abs_speed * (MAX_DUTY - MIN_DUTY) / 100);

    if (speed > 0) {        // если крутим вперед
        TIM1->CCR1 = duty; // шим на RPWM
        TIM1->CCR2 = 0;    // LPWM 0
    } else {                // если крутим назад
        TIM1->CCR1 = 0;    // RPWM 0
        TIM1->CCR2 = duty; //  шим на LPWM
    }
}

// настройка UART для телемы
static void uart_init(void)
{
    // подаем питание на модуль USART2 (шина APB1) и порт GPIOA (шина APB2)
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // настраиваем PA2 - на TX
    GPIOA->CRL &= ~(0xFu << 8);  // очищаем биты настройки только для PA2
    GPIOA->CRL |= (0xBu << 8);   // PA2: Alternate Function PP (нужно для UART TX)

    /**
     * настройка баудрейта
     * скорость = F_bus / (16 * USARTDIV). Для 115200 при шине 18МГц (APB1 с делителем 4):
     * 18,000,000 / (16 * 115200) = 9.76. Мантисса = 9, дробь = 0.76 * 16 = 12.
     */
    USART2->BRR = (9u << 4) | 12u;
    
    // включаем только передатчик (TE) и сам модуль USART (UE)
    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

// функция для отправки строки, ждет окончания передачи каждого символа
void uart_send_string(const char *s)
{
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE)); // ждем пустой регистр передачи
        USART2->DR = *s++; // записываем символ в буфер
    }
}

// настройка GPIO
static void gpio_init(void)
{
    // включаем питание портов и AFIO (нужно для переназначения функций пинов)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
    
    // PA8, PA9 (TIM1) PWM. шим должен быть "альтернативной функцией"
    GPIOA->CRH &= ~(0xFFu << 0);
    GPIOA->CRH |= (0xBBu << 0); // af push-pull, 50MHz

    // PA0, PA1 (TIM2) энкодер 1. в режиме энкодера пины должны быть "входами"
    // очищаем 8 бит (по 4 на пин) и ставим режим 0x44 (floating input для обоих)
    GPIOA->CRL &= ~(0xFFu << 0);
    GPIOA->CRL |= (0x44u << 0);

    // PA6, PA7 (TIM3) Энкодер 2
    // Делаем то же самое для пинов 6 и 7 (сдвиг 24 бита: 6 пинов * 4 бита)
    GPIOA->CRL &= ~(0xFFu << 24);
    GPIOA->CRL |= (0x44u << 24);

    // PC13 светодиод на плате
    GPIOC->CRH &= ~(0xFu << 20);
    GPIOC->CRH |= (0x2u << 20); // Output Push-Pull, 2MHz
}

// настройка таймеров
static void tim_init_all(void)
{
    // 1. TIM1 - шим на1кГц
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 71;    // 72МГц / (71+1) = 1МГц тики
    TIM1->ARR = 999;   // 1000 тиков = 1кГц
    TIM1->CCMR1 |= (6u << TIM_CCMR1_OC1M_Pos) | (6u << TIM_CCMR1_OC2M_Pos); // PWM Mode 1
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; // Разрешить выход сигнала
    TIM1->BDTR |= TIM_BDTR_MOE; // Main Output Enable (нужно только для TIM1/TIM8)
    TIM1->CR1 |= TIM_CR1_CEN;   // Пуск

  
    // таймеры в режиме энкодера - аппаратно следят за двумя фазами и сами меняют счетчик CNT (четырехкратное увеличение точности)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
    
    // TIM2 - встроенный энкодер мотора
    TIM2->ARR = 0xFFFF;
    TIM2->SMCR |= (3u << TIM_SMCR_SMS_Pos); // SMS=011 -> Encoder Mode 3
    TIM2->CCMR1 |= (1u << TIM_CCMR1_CC1S_Pos) | (1u << TIM_CCMR1_CC2S_Pos); // Связать каналы с пинами
    TIM2->CR1 |= TIM_CR1_CEN;

    // TIM3 - внешний энкодер
    TIM3->ARR = 0xFFFF;
    TIM3->SMCR |= (3u << TIM_SMCR_SMS_Pos);
    TIM3->CCMR1 |= (1u << TIM_CCMR1_CC1S_Pos) | (1u << TIM_CCMR1_CC2S_Pos);
    TIM3->CR1 |= TIM_CR1_CEN;

    // 3. TIM4 - таймер управления, генерирует прерывание каждые 10мс
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 7199; // 72МГц / 7200 = 10кгц тики
    TIM4->ARR = 99;   // 100 тиков = 0.01с
    TIM4->DIER |= TIM_DIER_UIE; // разрешить прерывание update
    TIM4->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM4_IRQn); // включить TIM4 в контроллере прерываний
}

// работаем с интерраптом TIM4, технически он рулит всем процессом
void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_UIF) { // проверка флага прерывания
        TIM4->SR &= ~TIM_SR_UIF; // обязательный сброс флага вручную

        static int16_t last_pos = 0;   // позиция на прошлом шаге для расчета скорости
        static uint8_t telemetry_cnt = 0;
        
        // ПИД ПАРАМЕТРЫ
        static float kp = 0.8f;   // сила отклика на текущую ошибку
        static float ki = 0.05f;  // дожим ошибки (копит ошибку во времени)
        static float kd = 1.2f;   // демпфирование (сопротивление скорости ошибки)
        static float integral = 0, last_error = 0;

        // 1. читаем данные. CNT возвращает текущее значение счетчика энкодера
        g_target_pos = (int16_t)TIM3->CNT;
        g_current_pos = (int16_t)TIM2->CNT;
        
        // 2. скорость. разница позиций за фиксированный интервал времени (10мс)
        g_speed = g_current_pos - last_pos;
        last_pos = g_current_pos;

        // 3. а-ля пид регулятор
        float err = (float)(g_target_pos - g_current_pos); // основная ошибка
        
        // дедзона: игнорируем шум в 4 тика, чтобы не так сильно шуметь в покое
        if (err > -4 && err < 4) err = 0; 
        
        if (err != 0) {
            integral += err; // накопление интеграла (интегрирование ошибки)
        } else {
            integral *= 0.8f; // быстрое обнуление интеграла, когда мы на месте
        }
        
        // ограничиваем накопленный интеграл, чтобы не было "перелетов"
        if (integral > 300) integral = 300;
        if (integral < -300) integral = -300;

        // дифференциал: разность текущей и прошлой ошибки (производная)
        float derivative = err - last_error;
        last_error = err;

        // результат ПИД: сумма трех компонент
        float res = (kp * err) + (ki * integral) + (kd * derivative);
        
        g_error = (int16_t)err;
        g_output = (int16_t)res;
        
        // 4. управление. передаем вычисленное значение на ШИМ
        motor_set_speed(g_output);

        // 5. телеметрия. ставим флаг раз в 50мс (каждый 5й тик таймера)
        if (++telemetry_cnt >= 5) {
            telemetry_cnt = 0;
            g_send_telemetry = 1;
        }
    }
}

int main(void)
{
    SystemInit(); // настройка системы тактирования (уже сделана в startup)
    gpio_init(); // настройка GPIO
    uart_init(); // настройка UART для телеметрии
    tim_init_all(); // настройка и запуск всех таймеров

    while (1)
    {
        // проверяем флаг из прерывания
        if (g_send_telemetry) {
            g_send_telemetry = 0;
            char buf[128];
            // телеметрия
            sprintf(buf, "%d,%d,%d,%d,%d\r\n", g_target_pos, g_current_pos, g_speed, g_error, g_output);
            uart_send_string(buf); // отправка по юарту
            
            // sanity-check в виде моргания диодом
            static uint32_t last_blink = 0;
            uint32_t blink_p = (g_error == 0) ? 500 : 100;
            if ((millis - last_blink) > blink_p) {
                GPIOC->ODR ^= (1u << 13); // переключить состояние PC13
                last_blink = millis;
            }
        }
    }
}
