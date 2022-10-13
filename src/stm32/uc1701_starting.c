#include "autoconf.h" // CONFIG_CLOCK_REF_FREQ

#if CONFIG_HURAKAN_CFG
#include "uc1701_starting.h" // uint8_t
#include "board/gpio.h" // struct gpio_out
#include "internal.h" // gpio_peripheral
#include <string.h> // memset
#include "sched.h" // sched_add_timer

// microsecond delay helper
static inline void
udelay(uint32_t usecs)
{
  uint32_t temp;
  uint32_t saveCtrl = SysTick->CTRL,
          saveLoad = SysTick->LOAD,
          saveVal = SysTick->VAL;

  uint32_t usCnt = (CONFIG_CLOCK_FREQ /  1000000);       // 1µs
  uint32_t Cnt  = usCnt * usecs;
  SysTick->CTRL &= ~(1 << 1);
  SysTick->CTRL |= 0x05;
  SysTick->LOAD = Cnt - 1;
  SysTick->VAL  = 0; // Set to start value
  do
  {
    temp = SysTick->CTRL;
  } while(temp & 0x01 && !(temp & (1 << 16)));

  SysTick->CTRL = saveCtrl;
  SysTick->LOAD = saveLoad;          // Restore SysTick rollover to 1 ms
  SysTick->VAL  = saveVal;           // Restore SysTick value
}

const uint8_t charset[][5] = {
  { 0x00, 0x00, 0x00, 0x00, 0x00 },  // 20 space
  { 0x00, 0x00, 0x5f, 0x00, 0x00 },  // 21 !
  { 0x00, 0x07, 0x00, 0x07, 0x00 },  // 22 "
  { 0x14, 0x7f, 0x14, 0x7f, 0x14 },  // 23 #
  { 0x24, 0x2a, 0x7f, 0x2a, 0x12 },  // 24 $
  { 0x23, 0x13, 0x08, 0x64, 0x62 },  // 25 %
  { 0x36, 0x49, 0x55, 0x22, 0x50 },  // 26 &
  { 0x00, 0x05, 0x03, 0x00, 0x00 },  // 27 '
  { 0x00, 0x1c, 0x22, 0x41, 0x00 },  // 28 (
  { 0x00, 0x41, 0x22, 0x1c, 0x00 },  // 29 )
  { 0x14, 0x08, 0x3e, 0x08, 0x14 },  // 2a *
  { 0x08, 0x08, 0x3e, 0x08, 0x08 },  // 2b +
  { 0x00, 0x50, 0x30, 0x00, 0x00 },  // 2c ,
  { 0x08, 0x08, 0x08, 0x08, 0x08 },  // 2d -
  { 0x00, 0x60, 0x60, 0x00, 0x00 },  // 2e .
  { 0x20, 0x10, 0x08, 0x04, 0x02 },  // 2f /
  { 0x3e, 0x51, 0x49, 0x45, 0x3e },  // 30 0
  { 0x00, 0x42, 0x7f, 0x40, 0x00 },  // 31 1
  { 0x42, 0x61, 0x51, 0x49, 0x46 },  // 32 2
  { 0x21, 0x41, 0x45, 0x4b, 0x31 },  // 33 3
  { 0x18, 0x14, 0x12, 0x7f, 0x10 },  // 34 4
  { 0x27, 0x45, 0x45, 0x45, 0x39 },  // 35 5
  { 0x3c, 0x4a, 0x49, 0x49, 0x30 },  // 36 6
  { 0x01, 0x71, 0x09, 0x05, 0x03 },  // 37 7
  { 0x36, 0x49, 0x49, 0x49, 0x36 },  // 38 8
  { 0x06, 0x49, 0x49, 0x29, 0x1e },  // 39 9
  { 0x00, 0x36, 0x36, 0x00, 0x00 },  // 3a :
  { 0x00, 0x56, 0x36, 0x00, 0x00 },  // 3b ;
  { 0x08, 0x14, 0x22, 0x41, 0x00 },  // 3c <
  { 0x14, 0x14, 0x14, 0x14, 0x14 },  // 3d =
  { 0x00, 0x41, 0x22, 0x14, 0x08 },  // 3e >
  { 0x02, 0x01, 0x51, 0x09, 0x06 },  // 3f ?
  { 0x32, 0x49, 0x79, 0x41, 0x3e },  // 40 @
  { 0x7e, 0x11, 0x11, 0x11, 0x7e },  // 41 A
  { 0x7f, 0x49, 0x49, 0x49, 0x36 },  // 42 B
  { 0x3e, 0x41, 0x41, 0x41, 0x22 },  // 43 C
  { 0x7f, 0x41, 0x41, 0x22, 0x1c },  // 44 D
  { 0x7f, 0x49, 0x49, 0x49, 0x41 },  // 45 E
  { 0x7f, 0x09, 0x09, 0x09, 0x01 },  // 46 F
  { 0x3e, 0x41, 0x49, 0x49, 0x7a },  // 47 G
  { 0x7f, 0x08, 0x08, 0x08, 0x7f },  // 48 H
  { 0x00, 0x41, 0x7f, 0x41, 0x00 },  // 49 I
  { 0x20, 0x40, 0x41, 0x3f, 0x01 },  // 4a J
  { 0x7f, 0x08, 0x14, 0x22, 0x41 },  // 4b K
  { 0x7f, 0x40, 0x40, 0x40, 0x40 },  // 4c L
  { 0x7f, 0x02, 0x0c, 0x02, 0x7f },  // 4d M
  { 0x7f, 0x04, 0x08, 0x10, 0x7f },  // 4e N
  { 0x3e, 0x41, 0x41, 0x41, 0x3e },  // 4f O
  { 0x7f, 0x09, 0x09, 0x09, 0x06 },  // 50 P
  { 0x3e, 0x41, 0x51, 0x21, 0x5e },  // 51 Q
  { 0x7f, 0x09, 0x19, 0x29, 0x46 },  // 52 R
  { 0x46, 0x49, 0x49, 0x49, 0x31 },  // 53 S
  { 0x01, 0x01, 0x7f, 0x01, 0x01 },  // 54 T
  { 0x3f, 0x40, 0x40, 0x40, 0x3f },  // 55 U
  { 0x1f, 0x20, 0x40, 0x20, 0x1f },  // 56 V
  { 0x3f, 0x40, 0x38, 0x40, 0x3f },  // 57 W
  { 0x63, 0x14, 0x08, 0x14, 0x63 },  // 58 X
  { 0x07, 0x08, 0x70, 0x08, 0x07 },  // 59 Y
  { 0x61, 0x51, 0x49, 0x45, 0x43 },  // 5a Z
  { 0x00, 0x7f, 0x41, 0x41, 0x00 },  // 5b [
  { 0x02, 0x04, 0x08, 0x10, 0x20 },  // 5c backslash
  { 0x00, 0x41, 0x41, 0x7f, 0x00 },  // 5d ]
  { 0x04, 0x02, 0x01, 0x02, 0x04 },  // 5e ^
  { 0x40, 0x40, 0x40, 0x40, 0x40 },  // 5f _
  { 0x00, 0x01, 0x02, 0x04, 0x00 },  // 60 `
  { 0x20, 0x54, 0x54, 0x54, 0x78 },  // 61 a
  { 0x7f, 0x48, 0x44, 0x44, 0x38 },  // 62 b
  { 0x38, 0x44, 0x44, 0x44, 0x20 },  // 63 c
  { 0x38, 0x44, 0x44, 0x48, 0x7f },  // 64 d
  { 0x38, 0x54, 0x54, 0x54, 0x18 },  // 65 e
  { 0x08, 0x7e, 0x09, 0x01, 0x02 },  // 66 f
  { 0x0c, 0x52, 0x52, 0x52, 0x3e },  // 67 g
  { 0x7f, 0x08, 0x04, 0x04, 0x78 },  // 68 h
  { 0x00, 0x44, 0x7d, 0x40, 0x00 },  // 69 i
  { 0x20, 0x40, 0x44, 0x3d, 0x00 },  // 6a j
  { 0x7f, 0x10, 0x28, 0x44, 0x00 },  // 6b k
  { 0x00, 0x41, 0x7f, 0x40, 0x00 },  // 6c l
  { 0x7c, 0x04, 0x18, 0x04, 0x78 },  // 6d m
  { 0x7c, 0x08, 0x04, 0x04, 0x78 },  // 6e n
  { 0x38, 0x44, 0x44, 0x44, 0x38 },  // 6f o
  { 0x7c, 0x14, 0x14, 0x14, 0x08 },  // 70 p
  { 0x08, 0x14, 0x14, 0x18, 0x7c },  // 71 q
  { 0x7c, 0x08, 0x04, 0x04, 0x08 },  // 72 r
  { 0x48, 0x54, 0x54, 0x54, 0x20 },  // 73 s
  { 0x04, 0x3f, 0x44, 0x40, 0x20 },  // 74 t
  { 0x3c, 0x40, 0x40, 0x20, 0x7c },  // 75 u
  { 0x1c, 0x20, 0x40, 0x20, 0x1c },  // 76 v
  { 0x3c, 0x40, 0x30, 0x40, 0x3c },  // 77 w
  { 0x44, 0x28, 0x10, 0x28, 0x44 },  // 78 x
  { 0x0c, 0x50, 0x50, 0x50, 0x3c },  // 79 y
  { 0x44, 0x64, 0x54, 0x4c, 0x44 },  // 7a z
  { 0x00, 0x08, 0x36, 0x41, 0x00 },  // 7b {
  { 0x00, 0x00, 0x7f, 0x00, 0x00 },  // 7c |
  { 0x00, 0x41, 0x36, 0x08, 0x00 },  // 7d }
  { 0x10, 0x08, 0x08, 0x10, 0x08 },  // 7e ~
  { 0x00, 0x00, 0x00, 0x00, 0x00 }   // 7f
};

#define HIGH 1
#define LOW  0

#define UC1701_RST_PIN   GPIO('A', 15) // PA15
#define UC1701_CS_PIN    GPIO('B', 9)  // PB9
#define UC1701_A0_PIN    GPIO('C', 3)  // PC3
#define UC1701_SCK_PIN   GPIO('B', 13) // PB13
#define UC1701_MOSI_PIN  GPIO('B', 15) // PB15
#define UC1701_RGB_PIN   GPIO('A', 10) // PA10

struct gpio_out uc1701_rst;
struct gpio_out uc1701_cs;
struct gpio_out uc1701_a0;
struct gpio_out uc1701_sck;
struct gpio_out uc1701_mosi;
struct gpio_out uc1701_neopixel;

#define UC1701_RST_INIT()  uc1701_rst = gpio_out_setup(UC1701_RST_PIN, 0)
#define UC1701_CS_INIT()   uc1701_cs = gpio_out_setup(UC1701_CS_PIN, 0)
#define UC1701_A0_INIT()   uc1701_a0 = gpio_out_setup(UC1701_A0_PIN, 0)
#define UC1701_SCLK_INIT() uc1701_sck = gpio_out_setup(UC1701_SCK_PIN, 0)
#define UC1701_SID_INIT()  uc1701_mosi = gpio_out_setup(UC1701_MOSI_PIN, 0)
#define UC1701_RGB_INIT()  uc1701_neopixel = gpio_out_setup(UC1701_RGB_PIN, 0)

#define UC1701_RST_SET(n)  gpio_out_write(uc1701_rst, n)
#define UC1701_CS_SET(n)   gpio_out_write(uc1701_cs, n)
#define UC1701_A0_SET(n)   gpio_out_write(uc1701_a0, n)
#define UC1701_SCLK_SET(n) gpio_out_write(uc1701_sck, n)
#define UC1701_SID_SET(n)  gpio_out_write(uc1701_mosi, n)
#define UC1701_RGB_SET(n)  gpio_out_write(uc1701_neopixel, n)

uint8_t uc1701_column = 0;
uint8_t uc1701_line = 0;
uint8_t uc1701_width = 128;
uint8_t uc1701_height = 64;

#define NEOPIXELS_NUM        3
uint32_t neopixel_rgb;

void UC1701_neopixelShow(void)
{
  __disable_irq(); // Need 100% focus on instruction timing

  uint8_t num = 0;
  uint32_t mask = 0x800000;
  uint32_t cyc;
  uint32_t saveCtrl = SysTick->CTRL,
           saveLoad = SysTick->LOAD,
           saveVal = SysTick->VAL;

  uint32_t top = (CONFIG_CLOCK_FREQ /  800000);       // 1.25µs
  uint32_t t0  = top - (CONFIG_CLOCK_FREQ / 2500000); // 0.4µs
  uint32_t t1  = top - (CONFIG_CLOCK_FREQ / 1250000); // 0.8µs
  SysTick->CTRL &= ~(1 << 1);
  SysTick->CTRL |= 0x05;
  SysTick->LOAD = top - 1; // Config SysTick for NeoPixel bit freq
  SysTick->VAL  = 0; // Set to start value
  for(;;) {
    UC1701_RGB_SET(1);
    cyc = (neopixel_rgb & mask) ? t1 : t0;
    while(SysTick->VAL > cyc);
    UC1701_RGB_SET(0);
    if(!(mask >>= 1)) {
      num++;
      if(num >= NEOPIXELS_NUM) break;
      mask = 0x800000;
    }
    while(SysTick->VAL <= cyc);
  }

  SysTick->CTRL = saveCtrl;
  SysTick->LOAD = saveLoad;          // Restore SysTick rollover to 1 ms
  SysTick->VAL  = saveVal;           // Restore SysTick value

  __enable_irq();
}

void UC1701_neopixelSetColor(uint8_t r, uint8_t g, uint8_t b)
{
  neopixel_rgb = r << 16 | g << 8 | b;
  UC1701_neopixelShow();
}

void UC1701_neopixelBegin(void)
{
  UC1701_RGB_INIT();
  UC1701_RGB_SET(0);
}

void UC1701_begin(void)
{
  // All pins are outputs
  UC1701_RST_INIT();
  UC1701_CS_INIT();
  UC1701_A0_INIT();
  UC1701_SCLK_INIT();
  UC1701_SID_INIT();
  udelay(100);
  // Reset pin
  UC1701_RST_SET(HIGH);
  udelay(100);

  // Reset the controller state...
  UC1701_CS_SET(LOW);
  UC1701_A0_SET(LOW);
  UC1701_CS_SET(HIGH);
  udelay(100);

  // Set the LCD parameters...
  UC1701_transfer(1, 0xE2);  //System Reset
  UC1701_transfer(1, 0x40); // Set display start line to 0
  UC1701_transfer(1, 0xA0); //Set SEG Direction
  UC1701_transfer(1, 0xC8); //Set COM Direction
  UC1701_transfer(1, 0xA2); //Set Bias = 1/9
  UC1701_transfer(1, 0x2C);  //Boost ON
  UC1701_transfer(1, 0x2E); //Voltage Regular On
  UC1701_transfer(1, 0x2F); //Voltage Follower On
  UC1701_transfer(1, 0xF8); //Set booster ratio to
  UC1701_transfer(1, 0x00); //4x
  UC1701_transfer(1, 0x23); //Set Resistor Ratio = 3
  UC1701_transfer(1, 0x81);
  UC1701_transfer(1, 63); //Set Electronic Volume = 40
  UC1701_transfer(1, 0xAC);//Set Static indicator off
  UC1701_transfer(1, 0x00);
  UC1701_transfer(1, 0XA6); // Disable inverse
  UC1701_transfer(1, 0xAF); //Set Display Enable
  udelay(200);
  UC1701_transfer(1, 0xA5); //display all points
  udelay(200);
  UC1701_transfer(1, 0xA4); //normal display
  UC1701_CS_SET(LOW);
  UC1701_clear();
}

void UC1701_clear(void)
{
  for (uint8_t j = 0; j < 8; j++)
  {
    UC1701_setCursor(0, j);
    for (uint8_t i = 0; i < 128 ; i++)
    {
      UC1701_transfer(0, 0x00);
    }
  }
  UC1701_setCursor(0, 0);
}

void UC1701_setCursor(uint8_t column, uint8_t line)
{
  int i, j;
  uc1701_column = column;
  uc1701_line = line;

  i = (column & 0xF0) >> 4;
  j = column & 0x0F;
  UC1701_CS_SET(LOW);
  UC1701_transfer(1, 0xb0 + line);
  UC1701_transfer(1, 0x10 + i);
  UC1701_transfer(1, j);
}

void UC1701_write(uint8_t chr)
{
    // ASCII 7-bit only...
    if (chr >= 0x80) {
        return;
    }

    if (chr == '\r') {
        UC1701_setCursor(0, uc1701_line);
        return;
    } else if (chr == '\n') {
        UC1701_setCursor(uc1701_column, uc1701_line + 1);
        return;
    }

    const unsigned char *glyph;
    unsigned char pgm_buffer[5];

    if (chr >= ' ') {
        // Regular ASCII characters are kept in flash to save RAM...
        memcpy(pgm_buffer, &charset[chr - ' '], sizeof(pgm_buffer));
        glyph = pgm_buffer;
    }

    // Output one column at a time...
    for (unsigned char i = 0; i < 5; i++) {
        UC1701_transfer(0, glyph[i]);
    }

    // One column between characters...
    UC1701_transfer(0,  0x00);

    // Update the cursor position...
    uc1701_column = uc1701_column + 6;

    if (uc1701_column > uc1701_width - 5) {
        uc1701_column = 0;
        uc1701_line = (uc1701_line + 1) % (uc1701_height/9 + 1);
        UC1701_setCursor(uc1701_column, uc1701_line);
    }

    return;
}

void UC1701_string(uint8_t *str)
{
  while(*str)
  {
    UC1701_write(*str++);
  }
}

void UC1701_transfer(uint8_t isCMD, uint8_t data)
{
  char i;
  UC1701_CS_SET(LOW);
  UC1701_A0_SET(isCMD ? LOW : HIGH);
  for (i=0; i<8; i++)
  {
    UC1701_SCLK_SET(LOW);
    udelay(5);
    if(data & 0x80)
      UC1701_SID_SET(HIGH);
    else
      UC1701_SID_SET(LOW);
    udelay(5);
    UC1701_SCLK_SET(HIGH);
    udelay(5);
    data = data << 1;
  }
}

void UC1701_Show(void)
{
  char *line0 = "BIQU-Hurakan";
  char *line1 = "Starting...";
  char *line3 = "OS Card must be ready";
  char *line4 = "before power on";
  char *line5 = "OS will start up";
  char *line6 = "45s after power on";
  char *line7 = "mcu: v0.11.0-20230216";

  uint8_t pixel;

  pixel = (128 - strlen(line0) * 6) / 2;
  UC1701_setCursor(pixel, 0);
  UC1701_string((uint8_t*)line0);
  pixel = (128 - strlen(line1) * 6) / 2;
  UC1701_setCursor(pixel, 1);
  UC1701_string((uint8_t*)line1);

  UC1701_setCursor(0, 3);
  UC1701_string((uint8_t*)line3);
  UC1701_setCursor(0, 4);
  UC1701_string((uint8_t*)line4);
  UC1701_setCursor(0, 5);
  UC1701_string((uint8_t*)line5);
  UC1701_setCursor(0, 6);
  UC1701_string((uint8_t*)line6);

  pixel = (128 - strlen(line7) * 6) / 2;
  UC1701_setCursor(pixel, 7);
  UC1701_string((uint8_t*)line7);
}

void BIQU_HurakanStarting(void)
{
  #define BRIGHTNESS  255
  UC1701_neopixelBegin();
  UC1701_neopixelSetColor(BRIGHTNESS, BRIGHTNESS, BRIGHTNESS);
  UC1701_begin();
  UC1701_Show();
}

extern uint8_t error_reason_hurakan;

const char *lookup_errormsg_string(uint8_t error)
{
    switch (error)
    {
    case 2:
        return "Shutdown cleared when not shutdown";
    case 3:
        return "Timer too close";
    case 4:
        return "sentinel timer called";
    case 5:
        return "Invalid command";
    case 6:
        return "Message encode error";
    case 7:
        return "Command parser error";
    case 8:
        return "Command request";
    case 9:
        return "config_reset only available when shutdown";
    case 10:
        return "oids already allocated";
    case 11:
        return "Can't assign oid";
    case 12:
        return "Invalid oid type";
    case 13:
        return "Already finalized";
    case 14:
        return "Invalid move request size";
    case 15:
        return "Move queue overflow";
    case 16:
        return "alloc_chunks failed";
    case 17:
        return "alloc_chunk failed";
    case 18:
        return "update_digital_out not valid with active queue";
    case 19:
        return "Scheduled digital out event will exceed max_duration";
    case 20:
        return "Can not set soft pwm cycle ticks while updates pending";
    case 21:
        return "Missed scheduling of next digital out event";
    case 22:
        return "Can't reset time when stepper active";
    case 23:
        return "Invalid count parameter";
    case 24:
        return "Stepper too far in past";
    case 25:
        return "Can't add signal that is already active";
    case 26:
        return "ADC out of range";
    case 27:
        return "Invalid spi config";
    case 28:
        return "Thermocouple reader fault";
    case 29:
        return "Thermocouple ADC out of range";
    case 30:
        return "Invalid thermocouple chip type";
    case 31:
        return "i2c_modify_bits: Odd number of bits!";
    case 32:
        return "Invalid buttons retransmit count";
    case 33:
        return "Set button past maximum button count";
    case 34:
        return "Max of 8 buttons";
    case 35:
        return "tmcuart data too large";
    case 36:
        return "Invalid neopixel update command";
    case 37:
        return "Invalid neopixel data_size";
    case 38:
        return "Not a valid input pin";
    case 39:
        return "Not an output pin";
    case 40:
        return "Rescheduled timer in the past";
    case 41:
        return "Not a valid ADC pin";
    case 42:
        return "i2c_read not supported on stm32";
    case 43:
        return "i2c timeout";
    case 44:
        return "Unsupported i2c bus";
    case 45:
        return "Invalid spi bus";

    default:
        return "Unknow error";
    }
}

void
uc1701_hurakan_shutdown(void)
{
  #define BRIGHTNESS  255
  UC1701_neopixelBegin();
  UC1701_neopixelSetColor(BRIGHTNESS, BRIGHTNESS, BRIGHTNESS);
  UC1701_begin();
  UC1701_setCursor(0, 0);
  UC1701_string((uint8_t*)"Error:");
  UC1701_setCursor(0, 1);
  UC1701_string((uint8_t*)lookup_errormsg_string(error_reason_hurakan));
}
DECL_SHUTDOWN(uc1701_hurakan_shutdown);

#endif
