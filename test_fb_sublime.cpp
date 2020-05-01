
/// Out pin control *****************************
#define fb_set_low    0x00  ///
#define fb_set_hi     0x01  ///
#define fb_set_hiz    0x02  ///

#define fb_setf_5v    0x04  /// flag  0=3V/ 4=5V
#define fb_setf_opt   0x08  /// flag  opto for B10,B11,B12,B13

#define fb_led_off      0x00  /// also the same for: ERR; READ; WAIT; PWR1(V_ECU); PWR4(V_IGN), V_LED
#define fb_led_on     0x10
#define fb_led_flash1   0x20
#define fb_led_flash2   0x30
#define fb_led_flash3   0x40
#define fb_led_flashbeep  0x80  /// flag beep when on
#define fb_update_flag    0x8

#define    _fb_rtx_cmd_adc     0x05

//int baud_fb = 1000000;
//int dbg_baud = 9600;








enum status
{
  LED_OFF = 0x00,
  LED_ON = 0x10,
  ON_3V = 0x11,
  OFF_3V = 0x00,
  ON_5V = 0x15,
  OFF_5V = 0x00
};

enum fb_pin
{

  _B1 = 3,
  _B2,
  _B3,
  _B4,
  _B5,
  _B6,
  _B7,
  _B8,
  _B9,
  _B10,
  _B11,
  _B12,
  _B13,
  _B14,
  _A1,
  _A2 = _A1,
  _A3 = _A1,
  _A4,
  _A5,
  _A6 = _A5

};


typedef struct                          // cmd recived from Device
{

  uint8_t start_cmd;
  uint8_t len;                           // packet len for TX
  uint8_t cmd;                           // packet ID for TX
  uint8_t B1_can1h;                      // CAN1 H       ( 0xX_ - only led ctrl }
  uint8_t B2_can1l;                        // CAN1 L       ( 0xX_ - only led ctrl }
  uint8_t B3_k1;                             // K1         ( 0xX_ - only led ctrl }
  uint8_t B4_l;                                // L          ( 0xX_ - only led ctrl }
  uint8_t B5_can2h;                            // CAN2 H       ( 0xX_ - only led ctrl }
  uint8_t B6_can2l;                            // CAN2 L       ( 0xX_ - only led ctrl }
  uint8_t B7_io1;                            // io 1       ( 0xXX - pin/led ctrl }
  uint8_t B8_io2;                            // io 2       ( 0xXX - pin/led ctrl }
  uint8_t B9_io3;                          // io 3       ( 0xXX - pin/led ctrl }
  uint8_t B10_io4;                           // io 4       ( 0xXX - pin/led ctrl }
  uint8_t B11_io5;                             // io 5       ( 0xXX - pin/led ctrl }
  uint8_t B12_io6;                               // io 6       ( 0xXX - pin/led ctrl }
  uint8_t B13_io7;                               // io 7       ( 0xXX - pin/led ctrl }
  uint8_t B14_io8;                               // io 8       ( 0xXX - pin/led ctrl }
  uint8_t A1_vecu;                               // a1, a2, a3 V_ECU ( 0xXX - pin/led ctrl }
  uint8_t A4_vign;                               // A4 V_IGN     ( 0xXX - pin/led ctrl }
  uint8_t A6_gnd;                            // a5,a6 I_GND     ( 0xX0 - only led ctrl )
  uint8_t Led_Err;                               //            ( 0xX_ - only led ctrl }
  uint8_t Led_Wait;                            //            ( 0xX_ - only led ctrl }
  uint8_t Led_Ready;                         //            ( 0xX_ - only led ctrl }
  uint8_t btn_led;                             //            ( 0xX_ - only led ctrl }
  uint8_t vled;                              //          ( 0xX_ - only led ctrl }
  uint8_t ping;                                // sound/music type
  uint8_t A5_ignd_limit;                       //  a5,a6 I_GND     ( current limit mA/100, step 100mA, max value 25A, 0 = no limit )
  uint16_t overload_ignore_time;               // time to ignore overload error - time to try set power on
  uint16_t free;                               //
  uint32_t pwm0freq;                           // x flags  x kanal, xx xx xx freg
  uint32_t pwm1freq;                           // x flags  x kanal, xx xx xx freg
  uint32_t pwm2freq;                           // x flags  x kanal, xx xx xx freg
  uint32_t pwm0ctrl;                           // xx kanal, xx xx xx freg
  uint32_t pwm1ctrl;                           // xx kanal, xx xx xx freg
  uint32_t pwm2ctrl;                           // xx kanal, xx xx xx freg
  uint8_t adc_averge_size;                     // system adc averge count 0...0x10
  uint8_t crc;

} __attribute__((packed))CMD_TX;

CMD_TX var;

typedef struct                  /// cmd send to PC/Flex
{
    
    uint8_t len;                /// packet len for TX
    uint8_t id;                 /// packet ID for TX
    uint16_t B7_io1;          /// io 1
    uint16_t B8_io2;          /// io 2
    uint16_t B9_io3;          /// io 3
    uint16_t B10_io4;         /// io 4
    uint16_t B11_io5;         /// io 5
    uint16_t B12_io6;         /// io 6
    uint16_t B13_io7;         /// io 7
    uint16_t B14_io8;         /// io 8
    uint16_t A1_vecu;         /// a1, a2, a3 V_ECU
    uint16_t A3_vign;         /// A4 V_IGN
    uint16_t A5_ignd;         /// a5,a6 I_GND
    uint16_t vout;            ///
    uint16_t button;
    uint16_t err;
}__attribute__((packed)) CMD_RX;

CMD_RX var_rx;
/*************************************************************************************/


/********************************** DEFAULT_SETTINGS ****************************************************/

void setDefault() {

  var.start_cmd = 0x0F;
  var.len =              0x37;
  var.cmd =               0x08;           // packet ID for TX
  var.B1_can1h =           0x02;           // CAN1 H     ( 0xX_ - only led ctrl }
  var.B2_can1l =           0x02;           // CAN1 L     ( 0xX_ - only led ctrl }
  var.B3_k1 =            0x02;       // K.line     ( 0xX_ - only led ctrl }
  var.B4_l =               0x02;           // L          ( 0xX_ - only led ctrl }
  var.B5_can2h =             0x02;           // CAN2 H     ( 0xX_ - only led ctrl }
  var.B6_can2l =           0x02;           // CAN2 L     ( 0xX_ - only led ctrl }
  var.B7_io1 =           0x02;           // io 1       ( 0xXX - pin/led ctrl }
  var.B8_io2 =           0x02;           // io 2       ( 0xXX - pin/led ctrl }
  var.B9_io3 =           0x02;           // io 3       ( 0xXX - pin/led ctrl }
  var.B10_io4 =            0x02;           // io 4       ( 0xXX - pin/led ctrl }
  var.B11_io5 =            0x02;           // io 5       ( 0xXX - pin/led ctrl }
  var.B12_io6 =            0x02;           // io 6       ( 0xXX - pin/led ctrl }
  var.B13_io7 =            0x02;           // io 7       ( 0xXX - pin/led ctrl }
  var.B14_io8 =            0x02;           // io 8       ( 0xXX - pin/led ctrl }
  var.A1_vecu =            0x02;           // A1, A2, A3 V_ECU ( 0xXX - pin/led ctrl }
  var.A4_vign =            0x00;           // A4 V_IGN   ( 0xXX - pin/led ctrl }
  var.A6_gnd =             0x02;           // A5,A6 I_GND     ( 0xX0 - only led ctrl )
  var.Led_Err =            0x02;           //            ( 0xX_ - only led ctrl }
  var.Led_Wait =           0x02;           //            ( 0xX_ - only led ctrl }
  var.Led_Ready =          0x10;           //            ( 0xX_ - only led ctrl }
  var.btn_led =            0x02;           //            ( 0xX_ - only led ctrl }
  var.vled =             0x10;           //            ( 0xX_ - only led ctrl }
  var.ping =             0x80;           // sound/music type
  var.A5_ignd_limit =      0x41;           //  a5,a6 I_GND     ( current limit mA/100, step 100mA, max value 25A, 0 = no limit )
  var.overload_ignore_time = 0x0064;         // time to ignore overload error - time to try set power on
  var.free =               0x0202;         //
  var.pwm0freq =           0x00000000;     // x flags  x kanal, xx xx xx freg
  var.pwm1freq =           0x00000000;     // x flags  x kanal, xx xx xx freg
  var.pwm2freq =           0x00000000;     // x flags  x kanal, xx xx xx freg
  var.pwm0ctrl =           0x00000032;     // xx kanal, xx xx xx freg
  var.pwm1ctrl =           0x00000032;     // xx kanal, xx xx xx freg
  var.pwm2ctrl =           0x00000032;     // xx kanal, xx xx xx freg
  var.adc_averge_size =    0x10;
  var.crc = 0;
  //uint8_t arr[] = {0x0F,0x37,0x08,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x10,0x02,0x10,0x80,0x41,0x0202,0x00000000,0x00000000,0x00000000,0x00000032,0x00000032,0x00000032,0x10,0x64};
  //Serial.write((uint8_t*)&arr, sizeof(arr));
  cmd_tx(var);

}

/********************************** CRC8 ****************************************************/
uint8_t crc8(uint8_t* buf, int size) {
  uint8_t crc = 0;
  while (size--) {
    crc += *buf++;
  }
  return crc;
}


/********************************** SET_PIN ****************************************************/
void setPin(fb_pin pin, status s) {
  *((uint8_t*)&var + pin) = s;
  var.A6_gnd = s;
  
  cmd_tx(var);
}
/**************************************************************************************/


/**********************************SEND_CMD****************************************************/

void cmd_tx(CMD_TX var) {
  Serial.flush();
  var.crc = crc8((uint8_t*)&var + 1, var.len);
  Serial1.write((uint8_t*)&var, sizeof(var));
  
  cmd_rx();
}

/**************************************************************************************/

/**************************************************************************************/
void cmd_rx(){
  uint8_t temp[32];
  uint8_t expected_len=0;
  Serial.print("rx: ");
  while(Serial1.available() < 32 || Serial1.read()!=0x0F); //TO FIX LOOP
  for (int i = 0; i< 31 ; i++){
    temp[i]=Serial1.read();
    }
    memcpy(&var_rx,temp,sizeof(var_rx));
    Serial.print("memcpy: ");
     Serial.println(var_rx.B7_io1);
  }

/**************************************************************************************/

/******************************** LED-PORTB-SEQUENCE ******************************************************/
void LedPortB_sequence(uint8_t time_on, uint8_t time_off) {

  status j = 0x10;
  int k = 13;
  int m = 0;
  while (m < 3) {
    for (int i = 3; i < 29; i++) {
      if (i < 17) {
        k = i;
      }
      if (i > 15) {
        k--;
      }
      for (int n = 0; n < 2; n++) {
        setPin(k, j);
        if (j == 0x00) {
          delay(time_off);
          j = 0x10;
        } else {
          delay(time_on);
          j = 0x00;
        }
      }
    } m++;
  }

}


/**************************************************************************************/
void setup() {
  Serial1.begin(1000000);
  Serial.begin(9600);

  Serial.println("Done");
  
  setDefault();
  
  

  /* uint8_t *pippo= (uint8_t*)&;
    Serial.println("fb_pin");
    Serial.println(*pippo, HEX);
  */







}
int c;
void loop() {

  if (Serial.available() > 0) {

    c = Serial.read();

    if (c == 'q') {
      setDefault();
      Serial.println("Default settings");
    } else if (c == 'n') {
      setPin(_B7, ON_3V);
    } else if (c == 'f') {
      setPin(_B7, OFF_3V);
    
    } else if (c == 'h') {
      Serial.println("crc");
      Serial.println(crc8((uint8_t*)&var + 1, var.len), HEX);
    } else if (c == 'z') {
      LedPortB_sequence(50, 5);
    } else if (c == 'p') {
      cmd_rx();
      
    }
  }

  delay(10);

}