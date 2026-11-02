/*
 * RP2040 Zero 12-Digit Precision Frequency Counter
 * ================================================
 * - 高频: 门控计数法 (10MHz参考, GPIO20=GPIN0)
 * - 低频: 周期倒数法 (自动切换)
 * - 双MAX7219独立CS显示
 * - 串口输出频率/单位/精度/模式
 *
 * 引脚:
 *   GPIO20 = 10MHz REF (GPIN0, 底垫)
 *   GPIO5  = 被测信号
 *   GP2    = SPI SCK
 *   GP3    = SPI MOSI
 *   GP4    = CS_LEFT  (左片8位)
 *   GP6    = CS_RIGHT (右片4位)
 */

#include <Arduino.h>
#include <SPI.h>
#include <hardware/pwm.h>
#include <hardware/gpio.h>
#include <hardware/clocks.h>
#include <hardware/irq.h>
#include <hardware/timer.h>
#include <math.h>

// ============ 兼容宏 ============
#ifndef GPIO_FUNC_CLK
#define GPIO_FUNC_CLK (gpio_function_t)8
#endif

// ============ 引脚 ============
#define SIG_PIN         5
#define EXT_CLK_GPIO    20
#define CS_LEFT         4
#define CS_RIGHT        6

// ============ 测量配置 ============
#define GATE_SEC        10      // 门控秒数(计数法)
#define GATE_DIV        160     // 10MHz/160 = 62500Hz
#define GATE_WRAP       62500   // 精确1秒
#define GATE_TICK_SEC   1.0f
#define AVG_N           4       // 滑动平均
#define MODE_THRESHOLD  50      // Hz, 低于此值切周期法

// 周期法参数
#define PERIOD_CYCLES   100     // 测100个周期取平均

// ============ 自动模式测量 ============
typedef enum {
  MODE_COUNTER,
  MODE_PERIOD
} measure_mode_t;


// ============ MAX7219 ============
#define R_DECODE  0x09
#define R_INTENS  0x0A
#define R_SCAN    0x0B
#define R_SHTDWN  0x0C
#define R_TEST    0x0F
#define D0        0x01

const uint8_t SEG[21] = {
  0b01111110, //0
  0b00110000, //1
  0b01101101, //2
  0b01111001, //3
  0b00110011, //4
  0b01011011, //5
  0b01011111, //6
  0b01110000, //7
  0b01111111, //8
  0b01111011, //9
  0b00000000, //10 blank
  0b10000000, //11 dp
  0b01110111, //12 A / M
  0b01000111, //13 L
  0b01001001, //14 C
  0b01001111, //15 E
  0b01000000, //16 -
  0b01100111, //17 P
  0b01101110, //18 H
  0b01001000, //19 k
  0b01011100, //20 o
};

// ============ 全局 ============
volatile uint32_t sig_ovf = 0;
volatile uint32_t gate_sec_counter = 0;
volatile bool gate_done = false;
uint32_t sig_sl, gate_sl;

static double freq_history[AVG_N] = {0};
static int hist_idx = 0;

// ============ MAX7219 驱动 ============
void mx_write(int cs, uint8_t reg, uint8_t dat) {
  gpio_put(cs, 0);
  SPI.transfer(reg);
  SPI.transfer(dat);
  gpio_put(cs, 1);
  asm volatile("nop\n nop");
}

void mx_init_one(int cs, uint8_t scanlim) {
  mx_write(cs, R_SHTDWN, 0x01);
  mx_write(cs, R_DECODE, 0x00);
  mx_write(cs, R_SCAN, scanlim);
  mx_write(cs, R_INTENS, 0x08);
  mx_write(cs, R_TEST, 0x00);
  for (int i = 0; i < 8; i++) mx_write(cs, D0 + i, 0x00);
}

void mx_init_all(void) {
  gpio_init(CS_LEFT);  gpio_set_dir(CS_LEFT,  GPIO_OUT); gpio_put(CS_LEFT, 1);
  gpio_init(CS_RIGHT); gpio_set_dir(CS_RIGHT, GPIO_OUT); gpio_put(CS_RIGHT, 1);
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  mx_init_one(CS_LEFT, 0x07);
  mx_init_one(CS_RIGHT, 0x03);
}

void disp_left(const uint8_t b[8]) {
  for (int i = 0; i < 8; i++) mx_write(CS_LEFT, D0 + i, b[i]);
}
void disp_right(const uint8_t b[4]) {
  for (int i = 0; i < 4; i++) mx_write(CS_RIGHT, D0 + i, b[i]);
  for (int i = 4; i < 8; i++) mx_write(CS_RIGHT, D0 + i, 0x00);
}

// ============ 显示格式化 ============
void show_freq(double f, bool period_mode) {
  uint8_t L[8], R[4];
  for (int i = 0; i < 8; i++) L[i] = 10;
  for (int i = 0; i < 4; i++) R[i] = 10;

  if (f <= 0 || f > 9.999999999e10) {
    L[7] = 15; // E
    disp_left(L); disp_right(R);
    return;
  }

  char buf[24];

  if (f >= 1e6) {
    dtostrf(f / 1e6, 12, 6, buf);
    int p = 0;
    for (int i = 0; buf[i] && p < 7; i++) {
      if (buf[i] >= '0' && buf[i] <= '9') { L[p++] = buf[i] - '0'; }
      else if (buf[i] == '.') { if (p > 0) L[p-1] |= 0x80; }
    }
    if (p < 8) L[p++] = 12; // M
    while (p < 8) L[p++] = 10;
    char* dot = strchr(buf, '.');
    if (dot) for (int i = 0; i < 4; i++) R[i] = (dot[1+i] ? dot[1+i]-'0' : 10);

  } else if (f >= 1e3) {
    dtostrf(f / 1e3, 12, 6, buf);
    int p = 0;
    for (int i = 0; buf[i] && p < 7; i++) {
      if (buf[i] >= '0' && buf[i] <= '9') { L[p++] = buf[i] - '0'; }
      else if (buf[i] == '.') { if (p > 0) L[p-1] |= 0x80; }
    }
    if (p < 8) L[p++] = 19; // k
    while (p < 8) L[p++] = 10;
    char* dot = strchr(buf, '.');
    if (dot) for (int i = 0; i < 4; i++) R[i] = (dot[1+i] ? dot[1+i]-'0' : 10);

  } else {
    // Hz模式: 低频显示更多小数
    if (f < 10) dtostrf(f, 11, 4, buf);
    else        dtostrf(f, 11, 2, buf);
    int p = 0;
    for (int i = 0; buf[i] && p < 8; i++) {
      if (buf[i] >= '0' && buf[i] <= '9') { L[p++] = buf[i] - '0'; }
      else if (buf[i] == '.') { if (p > 0) L[p-1] |= 0x80; }
    }
    while (p < 8) L[p++] = 10;
    char* dot = strchr(buf, '.');
    if (dot) for (int i = 0; i < 4; i++) R[i] = (dot[1+i] ? dot[1+i]-'0' : 10);
  }

  disp_left(L);
  disp_right(R);
}

// 显示模式标识 (右片)
void show_mode(bool period_mode) {
  uint8_t R[4];
  if (period_mode) {
    R[0] = 10; R[1] = 10; R[2] = 17; R[3] = 10; // " P " = Period
  } else {
    R[0] = 10; R[1] = 10; R[2] = 10; R[3] = 10; // blank
  }
  disp_right(R);
}

// ============ PWM 中断(合并) ============
void pwm_wrap_isr(void) {
  uint32_t m = pwm_get_irq_status_mask();
  if (m & (1u << sig_sl)) {
    pwm_clear_irq(sig_sl);
    sig_ovf++;
  }
  if (m & (1u << gate_sl)) {
    pwm_clear_irq(gate_sl);
    gate_sec_counter++;
    if (gate_sec_counter >= GATE_SEC) {
      gate_done = true;
      gate_sec_counter = 0;
    }
  }
}

// ============ 参考时钟 ============
void setup_ref_clock(void) {
  gpio_set_function(EXT_CLK_GPIO, GPIO_FUNC_CLK);
#if defined(CLOCKS_CLK_REF_CTRL_SRC_VALUE_CLKSRC_CLKIN)
  clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_CLKSRC_CLKIN, 0, 10000000, 10000000);
#elif defined(__has_include) && __has_include(<hardware/clocks.h>)
  clock_configure_gpin(clk_ref, EXT_CLK_GPIO, 10000000, 10000000);
#endif
}

// ============ 周期法: 测N个周期时间 ============
double measure_period_mode(uint32_t cycles) {
  // 用 PWM 计数高频时钟(10MHz) 测量信号周期
  // 等待上升沿 → 开始计数 → 数够cycles个边沿 → 停止
  // 用另一个PWM slice以10MHz计数时间

  uint32_t time_sl = gate_sl; // 复用gate slice做时间计数
  uint32_t start_cnt, end_cnt;
  uint32_t ovf = 0;

  // 等第一个上升沿
  while ((gpio_get(SIG_PIN)) != 0) tight_loop_contents();
  while ((gpio_get(SIG_PIN)) != 1) tight_loop_contents();

  // 开始计时
  pwm_set_counter(time_sl, 0);
  pwm_set_enabled(time_sl, true);

  // 数 cycles 个上升沿
  for (uint32_t i = 0; i < cycles; i++) {
    while ((gpio_get(SIG_PIN)) != 0) tight_loop_contents();
    while ((gpio_get(SIG_PIN)) != 1) tight_loop_contents();
  }

  end_cnt = pwm_get_counter(time_sl);
  pwm_set_enabled(time_sl, false);

  // 计算总时间 (10MHz = 100ns/计数)
  uint64_t total_ticks = end_cnt; // 单次不会溢出(<65536)
  double total_time_s = (double)total_ticks / 10000000.0; // 10MHz
  double period = total_time_s / (double)cycles;
  double freq = 1.0 / period;

  return freq;
}

// ============ 计数法 ============
double measure_counter_mode(void) {
  sig_ovf = 0;
  gate_sec_counter = 0;
  gate_done = false;

  pwm_set_counter(sig_sl, 0);
  pwm_set_counter(gate_sl, 0);
  pwm_set_enabled(sig_sl, true);
  pwm_set_enabled(gate_sl, true);

  uint32_t ws = millis();
  while (!gate_done) {
    tight_loop_contents();
    if (millis() - ws > (uint32_t)(GATE_SEC * 2000)) gate_done = true;
  }

  pwm_set_enabled(sig_sl, false);
  pwm_set_enabled(gate_sl, false);

  uint16_t cnt = pwm_get_counter(sig_sl);
  uint64_t total = ((uint64_t)sig_ovf << 16) + cnt;
  double actual_gate = (double)GATE_SEC * (double)GATE_TICK_SEC;
  double freq = (double)total / actual_gate;

  return freq;
}


measure_mode_t current_mode = MODE_COUNTER;

double measure_auto(void) {
  // 先用短门控判断信号频率量级
  // 快速采样: 1秒门控看有没有足够计数
  sig_ovf = 0;
  gate_sec_counter = 0;
  gate_done = false;

  pwm_set_counter(sig_sl, 0);
  pwm_set_counter(gate_sl, 0);
  pwm_set_enabled(sig_sl, true);
  pwm_set_enabled(gate_sl, true);

  // 等1个tick (1秒)
  uint32_t ws = millis();
  uint32_t quick_count = 0;
  while ((!gate_done) && millis() - ws < 1500) {
    tight_loop_contents();
  }

  // 如果1秒内门控没完成,强制看计数
  quick_count = pwm_get_counter(sig_sl) + ((uint64_t)sig_ovf << 16);

  // 判断模式
  if (quick_count < MODE_THRESHOLD) {
    // 低频: 切周期法
    current_mode = MODE_PERIOD;
    pwm_set_enabled(sig_sl, false);
    pwm_set_enabled(gate_sl, false);
    return measure_period_mode(PERIOD_CYCLES);
  } else {
    // 高频: 继续计数法 (等满GATE_SEC秒)
    current_mode = MODE_COUNTER;
    // 继续等剩余门控时间
    while (!gate_done) {
      tight_loop_contents();
      if (millis() - ws > (uint32_t)(GATE_SEC * 2000)) gate_done = true;
    }
    pwm_set_enabled(sig_sl, false);
    pwm_set_enabled(gate_sl, false);

    uint16_t cnt = pwm_get_counter(sig_sl);
    uint64_t total = ((uint64_t)sig_ovf << 16) + cnt;
    double actual_gate = (double)GATE_SEC * (double)GATE_TICK_SEC;
    return (double)total / actual_gate;
  }
}

// ============ 串口输出 ============
void serial_print_freq(double f, measure_mode_t mode) {
  char unit = 'H';
  double val = f;

  Serial.print("[");
  Serial.print(millis() / 1000);
  Serial.print("s] ");

  if (mode == MODE_PERIOD) {
    Serial.print("PERIOD  ");
    Serial.print(val, 6);
    Serial.print(" Hz");
  } else {
    Serial.print("COUNTER ");
    if (f >= 1e6) {
      Serial.print(f / 1e6, 6);
      Serial.print(" MHz");
    } else if (f >= 1e3) {
      Serial.print(f / 1e3, 4);
      Serial.print(" kHz");
    } else {
      Serial.print(f, 4);
      Serial.print(" Hz");
    }
  }

  // 分辨率估算
  double res_hz = (mode == MODE_COUNTER) ? (1.0 / (double)GATE_SEC) : (f * f / 10000000.0);
  Serial.print("  (res=");
  if (res_hz >= 1) Serial.print(res_hz, 1);
  else Serial.print(res_hz, 4);
  Serial.print("Hz)");

  Serial.println();
}

// ============ SETUP ============
void setup() {
  Serial.begin(115200);
  delay(300);

  mx_init_all();

  // 启动画面
  uint8_t boot_L[8] = {1, 2, 10, 11, 16, 16, 16, 16}; // "12bit"
  uint8_t boot_R[4] = {14, 12, 12, 19};                // "CLK"
  disp_left(boot_L);
  disp_right(boot_R);
  delay(1500);

  // 外部10MHz参考
  setup_ref_clock();

  // 信号PWM计数
  sig_sl = pwm_gpio_to_slice_num(SIG_PIN);
  assert(pwm_gpio_to_channel(SIG_PIN) == PWM_CHAN_B);
  gpio_set_function(SIG_PIN, GPIO_FUNC_PWM);

  pwm_config sc = pwm_get_default_config();
  pwm_config_set_clkdiv_mode(&sc, PWM_DIV_B_RISING);
  pwm_config_set_clkdiv(&sc, 1.0f);
  pwm_init(sig_sl, &sc, false);

  // 门控PWM
  gate_sl = (sig_sl == 0) ? 1 : 0;
  pwm_config gc = pwm_get_default_config();
  pwm_config_set_clkdiv(&gc, (float)GATE_DIV);
  pwm_config_set_wrap(&gc, GATE_WRAP);
  pwm_init(gate_sl, &gc, false);

  // 中断
  pwm_clear_irq(sig_sl);
  pwm_clear_irq(gate_sl);
  pwm_set_irq_enabled(sig_sl, true);
  pwm_set_irq_enabled(gate_sl, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_isr);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  Serial.println("=== RP2040 12-Digit Freq Counter ===");
  Serial.println("Modes: COUNTER(>=50Hz) | PERIOD(<50Hz)");
  Serial.println("Ref: 10MHz GPIO20 | Sig: GPIO5");
  Serial.println("-----------------------------------");
}

// ============ LOOP ============
void loop() {
  double freq = measure_auto();

  // 滑动平均
  freq_history[hist_idx] = freq;
  hist_idx = (hist_idx + 1) % AVG_N;
  double sum = 0;
  int valid = 0;
  for (int i = 0; i < AVG_N; i++) {
    if (freq_history[i] > 0) { sum += freq_history[i]; valid++; }
  }
  double favg = valid ? sum / valid : freq;

  // 串口输出
  serial_print_freq(favg, current_mode);

  // 显示
  show_freq(favg, current_mode);
  show_mode(current_mode == MODE_PERIOD);

  delay(300);
}
