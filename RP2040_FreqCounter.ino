/*
 * RP2040-Zero Frequency Counter
 * REF=GPIO5, DUT=GPIO1
 * 10 MHz REF, 10 s gate, 5-window sliding avg
 * 12 significant digits output
 *
 * 编译环境：Arduino IDE + Arduino-Pico (earlephilhower)
 */

#include <Arduino.h>
#include <hardware/pwm.h>
#include <hardware/irq.h>

// ============ 引脚定义 ============
#define REF_GPIO     5    // 奇数 → Slice 2, Channel B ✅
#define DUT_GPIO     1    // 奇数 → Slice 0, Channel B ✅

#define GATE_SECONDS     10
#define REF_CLK_HZ       10000000UL
#define AVG_WINDOW       5   // 5 × 10s = 50s 滑动窗口

// ============ 全局变量 ============
volatile uint64_t ref_overflow = 0;
volatile uint64_t dut_overflow = 0;
volatile bool measuring = false;

uint slice_ref;   // Slice 2
uint slice_dut;   // Slice 0

// 滑动平均缓冲区
uint64_t ref_buf[AVG_WINDOW];
uint64_t dut_buf[AVG_WINDOW];
uint8_t  avg_idx = 0;
uint8_t  avg_count = 0;

// ============ PWM 外部边沿计数初始化 ============
void init_pwm_counter(uint gpio, uint *slice_out)
{
    uint slice = pwm_gpio_to_slice_num(gpio);
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);  // 上升沿计数
    pwm_config_set_clkdiv(&cfg, 1.0f);
    pwm_init(slice, &cfg, false);

    pwm_set_counter(slice, 0);
    pwm_clear_irq(slice);
    pwm_set_irq_enabled(slice, true);

    *slice_out = slice;
}

// ============ PWM 溢出中断 ============
void pwm_irq_handler()
{
    uint32_t mask = pwm_get_irq_status_mask();

    if (mask & (1u << slice_ref)) {
        pwm_clear_irq(slice_ref);
        ref_overflow++;
    }

    if (mask & (1u << slice_dut)) {
        pwm_clear_irq(slice_dut);
        dut_overflow++;
    }
}

// ============ 开始一次测量 ============
void start_measurement()
{
    ref_overflow = 0;
    dut_overflow = 0;
    pwm_set_counter(slice_ref, 0);
    pwm_set_counter(slice_dut, 0);
    measuring = true;
    pwm_set_enabled(slice_ref, true);
    pwm_set_enabled(slice_dut, true);
}

// ============ 停止测量 ============
void stop_measurement()
{
    pwm_set_enabled(slice_ref, false);
    pwm_set_enabled(slice_dut, false);
    measuring = false;
}

// ============ 更新滑动平均 ============
void update_average(uint64_t ref, uint64_t dut)
{
    ref_buf[avg_idx] = ref;
    dut_buf[avg_idx] = dut;
    avg_idx = (avg_idx + 1) % AVG_WINDOW;
    if (avg_count < AVG_WINDOW) avg_count++;
}

// ============ 打印单次结果 ============
void print_single_result(uint64_t ref_cnt, uint64_t dut_cnt)
{
    if (ref_cnt == 0) {
        Serial.println("ERROR: No reference clock!\n");
        return;
    }

    double gate = (double)ref_cnt / (double)REF_CLK_HZ;
    double freq = (double)dut_cnt / gate;

    Serial.println("------ Single Measurement ------");
    Serial.print("REF counts : ");
    Serial.println(ref_cnt);
    Serial.print("DUT counts : ");
    Serial.println(dut_cnt);
    Serial.print("Gate time  : ");
    Serial.print(gate, 6);
    Serial.println(" s");
    Serial.print("Frequency  : ");
    Serial.print(freq, 11);
    Serial.println(" Hz\n");
}

// ============ 打印滑动平均结果 ============
void print_averaged_result()
{
    if (avg_count == 0) return;

    uint64_t ref_sum = 0;
    uint64_t dut_sum = 0;

    for (uint8_t i = 0; i < avg_count; i++) {
        ref_sum += ref_buf[i];
        dut_sum += dut_buf[i];
    }

    double gate = (double)ref_sum / (double)REF_CLK_HZ;
    double freq = (double)dut_sum / gate;

    Serial.println("======= Sliding Average ========");
    Serial.print("Window     : ");
    Serial.print(avg_count);
    Serial.print(" x ");
    Serial.print(GATE_SECONDS);
    Serial.println(" s");

    Serial.print("Total REF  : ");
    Serial.println(ref_sum);
    Serial.print("Total DUT  : ");
    Serial.println(dut_sum);

    Serial.print("Avg Gate   : ");
    Serial.print(gate, 6);
    Serial.println(" s");

    Serial.print("Frequency  : ");
    Serial.print(freq, 11);
    Serial.println(" Hz");

    // 12 位有效数字（科学计数法）
    if (freq > 0) {
        int exp = 0;
        double f = freq;
        if (f >= 1.0) {
            while (f >= 10.0) { f /= 10.0; exp++; }
        } else {
            while (f < 1.0) { f *= 10.0; exp--; }
        }
        Serial.print("12sig      : ");
        Serial.print(f, 11);
        Serial.print("e");
        if (exp >= 0) Serial.print("+");
        Serial.print(exp);
        Serial.println(" Hz");
    }

    Serial.println("===============================\n");
}

// ============ Setup ============
void setup()
{
    Serial.begin(115200);
    delay(1000);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    init_pwm_counter(REF_GPIO, &slice_ref);
    init_pwm_counter(DUT_GPIO, &slice_dut);

    Serial.println("===============================");
    Serial.println(" RP2040 Freq Counter");
    Serial.printf(" REF=GPIO%d (Slice %d)\n", REF_GPIO, slice_ref);
    Serial.printf(" DUT=GPIO%d (Slice %d)\n", DUT_GPIO, slice_dut);
    Serial.printf(" %lu Hz, %d s gate, window=%d\n",
                  REF_CLK_HZ, GATE_SECONDS, AVG_WINDOW);
    Serial.println("===============================\n");
}

// ============ Loop ============
void loop()
{
    Serial.println("--- Start 10s measurement ---");

    start_measurement();
    delay(GATE_SECONDS * 1000);
    stop_measurement();

    // 拼接 64 位计数值
    uint64_t ref_total = (ref_overflow << 16) | pwm_get_counter(slice_ref);
    uint64_t dut_total = (dut_overflow << 16) | pwm_get_counter(slice_dut);

    if (ref_total == 0) {
        Serial.println("ERROR: No reference clock!\n");
        delay(2000);
        return;
    }

    print_single_result(ref_total, dut_total);
    update_average(ref_total, dut_total);
    print_averaged_result();

    delay(1000);
}
