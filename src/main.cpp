#include <Arduino.h>

#define PIN_YM_WR (2u)
#define PIN_YM_RD (3u)
#define PIN_YM_A0 (4u)
#define PIN_YM_A1 (5u)
#define PIN_YM_DATA_OFFSET (6u)
#define PIN_YM_DATA_LENGTH (8u)
#define PIN_YM_CSN_OFFSET (16u)
#define PIN_YM_CSN_LENGTH (5u)
#define PIN_YM_CS (14u)
#define PIN_YM_IC (15u)
#define PIN_YM_MODE_2612 (22u)
#define ym_data_mask(b) (((b)&0xff)<<PIN_YM_DATA_OFFSET)
#define ym_csn_mask(b) (((b)&0b11111)<<PIN_YM_CSN_OFFSET)
#define bit_mask(value, bit) ((value)<<(bit))

#define _n() __NOP()
#define delay20Ns() _n();_n();_n();
#define delay50Ns() _n();_n();_n();_n();_n();_n();_n();
#define delay100Ns() _n();_n();_n();_n();_n();_n();_n();_n();_n();_n();_n();_n();_n();_n();_n();
#define delay200Ns() delay100Ns();delay100Ns();
#define delay250Ns() delay200Ns();delay50Ns();
#define delay500Ns() delay200Ns();delay200Ns();delay100Ns();
#define delay600Ns() delay500Ns();delay100Ns();
#define delay700Ns() delay500Ns();delay200Ns();
#define delay800Ns() delay600Ns();delay200Ns();
#define delay12Clocks() delay500Ns();delay200Ns();delay200Ns();
#define ym_datamode_wait() delay500Ns();delay200Ns();delay50Ns();
#define A0_REG (false)
#define A0_DATA (true)

#define GPIO_IN_REG (SIO_BASE + 0x004)
#define gpio_read(pin) (((*(uint32_t*)GPIO_IN_REG) << (31-pin)) >> 31)

bool data_is_out = true;
bool mode_2612 = false;

uint32_t outPinsMask
  = ym_data_mask(0xff) | ym_csn_mask(0b11111)
  | bit(PIN_YM_WR) | bit(PIN_YM_RD)
  | bit(PIN_YM_A0) | bit(PIN_YM_A1)
  | bit(PIN_YM_CS) | bit(PIN_YM_IC)
  | bit(PIN_YM_MODE_2612) | bit(PIN_LED);

void ym_datamode_out() {
  if (!data_is_out) {
    ym_datamode_wait();
    gpio_set_dir_masked(ym_data_mask(0xff), ym_data_mask(0xff));
    data_is_out = true;
  }
}
void ym_datamode_in() {
  if (data_is_out) {
    ym_datamode_wait();
    gpio_set_dir_masked(ym_data_mask(0xff), 0);
    data_is_out = false;
  }
}

/*
AO low, A1 part
write reg
A0 high
write data
A0/A1 low

write:
CS low
DATA (1us)
WR low (1us)
WR high (1us)
CS high
*/

void ym_cleardata() {
  gpio_put_masked(ym_data_mask(0xff), 0);
}

void ym_write(bool a0, uint8_t a1, uint8_t data) {
  // A0 (select/write register)
  // A1 (0: ch1-3, 1: ch4-6)
  gpio_put_masked(bit(PIN_YM_A0) | bit(PIN_YM_A1),
    bit_mask(a0, PIN_YM_A0) | bit_mask(a1&1, PIN_YM_A1));

  // Tas (10ns)
  __NOP(); __NOP();

  // CS low, DATA write
  gpio_put_masked(ym_data_mask(0xff) | bit(PIN_YM_CS), ym_data_mask(data));
  __NOP(); __NOP();

  // WR low
  gpio_clr_mask(bit(PIN_YM_WR));

  // Tww (200ns)
  delay200Ns();

  // WR/CS high
  gpio_set_mask(bit(PIN_YM_WR) | bit(PIN_YM_CS));
}

void ym_write_jt(uint8_t chip, bool a0, uint8_t a1, uint8_t data) {
  // A0 (select/write register)
  // A1 (0: ch1-3, 1: ch4-6)
  // DATA write
  gpio_put_masked(bit(PIN_YM_A0) | bit(PIN_YM_A1) | ym_data_mask(0xff),
    bit_mask(a0, PIN_YM_A0) | bit_mask(a1&1, PIN_YM_A1) | ym_data_mask(data));

  // select chip
  gpio_put_masked(ym_csn_mask(0b11111), ym_csn_mask(chip&0b11111));
  // delay for CS propagation
  delay20Ns();

  // WR low
  gpio_clr_mask(bit(PIN_YM_WR));

  // Tww (200ns)
  delay20Ns(); _n();

  // WR/CS high
  gpio_set_mask(bit(PIN_YM_WR));
  
  // deselect chip
  gpio_put_masked(ym_csn_mask(0b11111), 0);
  // delay for CS propagation
  delay20Ns();
}

void ym_write_reg(uint8_t chip, uint8_t reg, uint8_t data, uint8_t part = 0) {
  //gpio_clr_mask(bit(PIN_LED));

  // Ready the data pins for output
  ym_datamode_out();

  // A0 low (select register)
	ym_write_jt(chip, A0_REG, part, reg);
  // Delay between selecting register and writing data
  delay12Clocks();
  // A0 high (write data to register)
	ym_write_jt(chip, A0_DATA, part, data);

  // After writing data, we need to wait a little before the next write
  // Wait time depends on which register we wrote to
  
  if (reg == 0x28) // wasn't keyon/off supposed to be slow...?
    {delay12Clocks();} //delayMicroseconds(2);
  else if (reg < 0x30) // GLOBAL registers
    delayMicroseconds(10); // needs testing
  else if (reg < 0xa0) // OPER registers
    delayMicroseconds(10);
  else // CHANNEL registers
    {delay12Clocks();} //delayMicroseconds(1); // frequency/etc are fast
    
  //gpio_set_mask(bit(PIN_LED));
}

void ym_reset() {
  gpio_clr_mask(bit(PIN_LED));
  
  gpio_clr_mask(bit(PIN_YM_CS) | bit(PIN_YM_A0) | bit(PIN_YM_A1));
  gpio_set_mask(bit(PIN_YM_WR) | bit(PIN_YM_RD) | bit(PIN_YM_IC));
  delay12Clocks();
  gpio_clr_mask(bit(PIN_YM_IC));
  gpio_set_mask(bit(PIN_YM_CS));

  gpio_set_mask(bit(PIN_LED));
}

void ym_prepare(uint8_t chip, uint8_t channel) {
  const bool part = channel > 2;
  const uint8_t chMod = channel % 3;
  if (part) channel += 1; // 0 1 2  4 5 6
  
  // Global registers
  ym_write_reg(chip, 0x22, 0x00); // GLOBAL: LFO off
  //ym_write_reg(chip, 0x24, 0x00); // GLOBAL: Timer A Freq (high)
  //ym_write_reg(chip, 0x25, 0x00); // GLOBAL: Timer A Freq (low)
  //ym_write_reg(chip, 0x26, 0x00); // GLOBAL: Timer B Freq
  ym_write_reg(chip, 0x27, 0x00); // GLOBAL: Ch3 mode normal + timer off
  ym_write_reg(chip, 0x28, channel); // GLOBAL: Note off
  //ym_write_reg(chip, 0x29, 0x00); // GLOBAL: Ch6 DAC output
  ym_write_reg(chip, 0x2B, 0x00); // GLOBAL: Ch6 DAC off

  // Operator registers
  ym_write_reg(chip, 0x30+chMod, 0x71, part); // OP1 DeTune / MULtiply (DT/MUL)
  ym_write_reg(chip, 0x34+chMod, 0x0D, part); // OP3
  ym_write_reg(chip, 0x38+chMod, 0x33, part); // OP2
  ym_write_reg(chip, 0x3C+chMod, 0x01, part); // OP4
  ym_write_reg(chip, 0x40+chMod, 0x23, part); // OP1 Total Level (TL)
  ym_write_reg(chip, 0x44+chMod, 0x2D, part); // OP3
  ym_write_reg(chip, 0x48+chMod, 0x26, part); // OP2
  ym_write_reg(chip, 0x4C+chMod, 0x00, part); // OP4
  ym_write_reg(chip, 0x50+chMod, 0x5F, part); // OP1 AttackRate/RateScale (AR/RS)
  ym_write_reg(chip, 0x54+chMod, 0x99, part); // OP3
  ym_write_reg(chip, 0x58+chMod, 0x5F, part); // OP2
  ym_write_reg(chip, 0x5C+chMod, 0x94, part); // OP4
  ym_write_reg(chip, 0x60+chMod, 0x05, part); // OP1 DecayRate / AmpMod Enable (DR[D1R]/AM)
  ym_write_reg(chip, 0x64+chMod, 0x05, part); // OP3
  ym_write_reg(chip, 0x68+chMod, 0x05, part); // OP2
  ym_write_reg(chip, 0x6C+chMod, 0x07, part); // OP4
  ym_write_reg(chip, 0x70+chMod, 0x02, part); // OP1 SustainRate (SR[D2R])
  ym_write_reg(chip, 0x74+chMod, 0x02, part); // OP3
  ym_write_reg(chip, 0x78+chMod, 0x02, part); // OP2
  ym_write_reg(chip, 0x7C+chMod, 0x02, part); // OP4
  ym_write_reg(chip, 0x80+chMod, 0x11, part); // OP1 ReleaseRate / SustainLevel (RR/SL)
  ym_write_reg(chip, 0x84+chMod, 0x11, part); // OP3
  ym_write_reg(chip, 0x88+chMod, 0x11, part); // OP2
  ym_write_reg(chip, 0x8C+chMod, 0xA6, part); // OP4
  ym_write_reg(chip, 0x90+chMod, 0x00, part); // OP1 SSG-EG
  ym_write_reg(chip, 0x94+chMod, 0x00, part); // OP3
  ym_write_reg(chip, 0x98+chMod, 0x00, part); // OP2
  ym_write_reg(chip, 0x9C+chMod, 0x00, part); // OP4

  // Channel registers
  ym_write_reg(chip, 0xA0+chMod, 0x69, part); // frequency (low)
  ym_write_reg(chip, 0xA4+chMod, 0x22, part); // frequency (high)
  ym_write_reg(chip, 0xB0+chMod, 0x32, part); // Feedback/algorithm
  ym_write_reg(chip, 0xB4+chMod, 0xC0, part); // Pan/PMS/AMS
}

void ym_play(uint8_t chip, uint8_t channel, uint16_t note) {
  ym_prepare(chip, channel);
  ym_write_reg(chip, 0xA4 + channel % 3, note >> 8, channel > 2); // frequency (high)
  ym_write_reg(chip, 0xA0 + channel % 3, note & 0xff, channel > 2); // frequency (low)
  ym_write_reg(chip, 0x28, 0xF0 + channel + (channel > 2 ? 1 : 0)); // Key on
}

uint16_t getNote(uint8_t octave, uint16_t fnum) {
  return ((octave & 7) << 11) | (fnum & 0x7ff);
}

void ym_test() {
  //mode_2612 = !mode_2612;
  //mode_2612 = true;
  //gpio_put(PIN_YM_MODE_2612, mode_2612);
  /* Program loop */
  ym_reset();
  const int noteDelay = 100;
  const uint16_t notes[6] = {
     getNote(4,1164), getNote(4,872), getNote(4,733), // 473.0437, 354.3764, 297.8875 hz
     getNote(3,1164), getNote(3,872), getNote(3,733)  // 236.5218, 177.1882, 148.9437
     };
  //const uint16_t notes[6] = {0x248C, 0x2368, 0x22DD, 0x1C8C, 0x1B68, 0x1ADD};
  for (int c=0; c<4; c++) {
    for(int i=0; i<6; i++) {
      //uint8_t chip = (i/4)%4+1;

      ym_reset();
      ym_play(c+1, i, notes[i]);
      delay(noteDelay);
    }
  }
}

void setup() {
  gpio_init_mask(outPinsMask);
  gpio_set_dir_masked(outPinsMask, outPinsMask);
  gpio_set_mask(bit(PIN_LED));
  ym_reset();

  Serial.begin(115200);
  Serial.println("initialized.");

  delay(1000);
  
}

void loop() {
  Serial.println("loop start");
  ym_test();
  //delay(100);
}