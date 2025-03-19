#include "Arduino.h"
#include <PubSubClient.h>
#include <Audio.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
#include <ESP32Encoder.h>
#include <PNGdec.h>
#include <ImprovWiFiLibrary.h>
#include "esp_wifi.h"
#include "qrcodeR.h"
#include <Arduino_ESP32_OTA.h>
//#include "root_ca.h"
//#include "USB.h"
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <FS.h>
#include <SD_MMC.h>
#include <driver/i2s.h>
#include "lwip/apps/sntp.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_system.h"


PNG png;
#define MAX_IMAGE_WDITH 240 // Adjust for your images

#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

#define version "V0.6"

#define I2S_DOUT        17
#define I2S_BCLK        5
#define I2S_LRC         16
#define I2S_DIN         4
#define I2SN (i2s_port_t)0
#define I2CN (i2c_port_t)0
#define SDA 18
#define SCL 11
#define JACK_DETECT     GPIO_NUM_10
#define USB_DETECT      GPIO_NUM_21
#define EN_4G           GPIO_NUM_38
#define IR              GPIO_NUM_47
#define BAT_GAUGE_PIN   13
#define ENC_A2          6
#define ENC_B2          7
#define ENC_A1          3
#define ENC_B1          42
//Digital buttons
#define CLICK2      GPIO_NUM_45
#define CLICK1      GPIO_NUM_48
#define PA GPIO_NUM_46
#define backLight   GPIO_NUM_41

#define MAX_WAIT_TIME 10000

//Analog Buttons
#define KEYs_ADC        1
#define Mute_key        1
#define OK_key          4
#define LEFT_key        5
#define VolP_key        2
#define VolM_key        0
#define RIGHT_key       3

#define sw0             0
#define sw1             1
#define sw2             2
#define sw3             3


int station = 0;
int previousStation;
int MS;
bool connected = true;
char* linkS;
char mes[200];
uint32_t sampleRate;
int iMes ;
int32_t V;
int32_t PV;
int32_t S;
int32_t VS;
int N;
bool CLICK1E    = false;
bool CLICK2E    = false;
bool Mute_keyE    = false;
bool muteB = false;
bool muteON = false;
#define Mute_keyB muteB
#define Mute_keyON muteON
#define CLICK1ON muteON
#define CLICK1B muteB
uint16_t REMOTE_KEY;
bool OK_keyE = false;
bool VALB = false;
#define CLICK2B VALB
#define OK_keyB VALB
bool jaugeB = false;

// remote keys
#define RET_rem       0x906F
#define MUTE_rem      0x807F
#define OK_rem        0x609F
#define UP_rem        0xA05F
#define DOWN_rem      0x20DF
#define LEFT_rem      0x40BF
#define RIGHT_rem     0x50AF
#define MAIN_rem      0xC03F
#define LIKE_rem      0xD02F
#define VOLP_rem      0x02FD
#define VOLM_rem      0x12ED
#define TRASH_rem     0x22DD


#define MAX_IMAGE_WDITH 240
#define TFT_GREY 0x5AEB
#define TFT_STATION 0x2BFF

TFT_eSPI tft = TFT_eSPI();

ESP32Encoder volEncoder;
ESP32Encoder staEncoder;

#define ES8388_ADDR 0x10
///////////////////////////////////////////////////////////////////////
// Write ES8388 register (using I2c)
///////////////////////////////////////////////////////////////////////
uint8_t ES8388_Write_Reg(uint8_t reg, uint8_t val)
{
  uint8_t buf[2], res;
  buf[0] = reg;
  buf[1] = val;
  Wire.beginTransmission(ES8388_ADDR);
  Wire.write(buf, 2);
  res = Wire.endTransmission();
  return res;
}

//////////////////////////////////////////////////////////////////
//
// init CODEC chip ES8388 (via I2C)
//
////////////////////////////////////////////////////////////////////
//
int ES8388_Init(void)
{
  // provides MCLK
  //    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
  //    WRITE_PERI_REG(PIN_CTRL, READ_PERI_REG(PIN_CTRL)& 0xFFFFFFF0);
  int st;
  st = 0;
  // reset
  st += ES8388_Write_Reg(0, 0x80);
  st += ES8388_Write_Reg(0, 0x00);
  // mute
  st += ES8388_Write_Reg(25, 0x04);
  st += ES8388_Write_Reg(1, 0x50);
  //powerup
  st += ES8388_Write_Reg(2, 0x00);
  // slave mode
  st += ES8388_Write_Reg(8, 0x00);
  // DAC powerdown
  st += ES8388_Write_Reg(4, 0xC0);
  // vmidsel/500k ADC/DAC idem
  st += ES8388_Write_Reg(0, 0x12);

  st += ES8388_Write_Reg(1, 0x00);
  // i2s 16 bits
  st += ES8388_Write_Reg(23, 0x18);
  // sample freq 256
  st += ES8388_Write_Reg(24, 0x02);
  // LIN2/RIN2 for mixer
  st += ES8388_Write_Reg(38, 0x09);
  // left DAC to left mixer
  st += ES8388_Write_Reg(39, 0x80);
  // right DAC to right mixer
  st += ES8388_Write_Reg(42, 0x80);
  // DACLRC ADCLRC idem
  st += ES8388_Write_Reg(43, 0x80);
  st += ES8388_Write_Reg(45, 0x00);
  // DAC volume max
  st += ES8388_Write_Reg(27, 0x00);
  st += ES8388_Write_Reg(26, 0x00);


  //mono (L+R)/2
  st += ES8388_Write_Reg(29, 0x20);

  // DAC power-up LOUT1/ROUT1 ET 2 enabled
  st += ES8388_Write_Reg(4, 0x3C);

  // DAC R phase inversion
  st += ES8388_Write_Reg(28, 0x10);


  // ADC poweroff
  ES8388_Write_Reg(3, 0xFF);


  // ADC amp 24dB
  ES8388_Write_Reg(9, 0x88);


  // differential input
  ES8388_Write_Reg(10, 0xFC);
  ES8388_Write_Reg(11, 0x02);


  //Select LIN2and RIN2 as differential input pairs
  //ES8388_Write_Reg(11,0x82);

  //i2S 16b
  ES8388_Write_Reg(12, 0x0C);
  //MCLK 256
  ES8388_Write_Reg(13, 0x02);
  // ADC high pass filter
  // ES8388_Write_Reg(14,0x30);

  // ADC Volume LADC volume = 0dB
  ES8388_Write_Reg(16, 0x00);

  // ADC Volume RADC volume = 0dB
  ES8388_Write_Reg(17, 0x00);

  // ALC
  ES8388_Write_Reg(0x12, 0xfd); // Reg 0x12 = 0xe2 (ALC enable, PGA Max. Gain=23.5dB, Min. Gain=0dB)
  //ES8388_Write_Reg(0x12, 0x22); // Reg 0x12 = 0xe2 (ALC enable, PGA Max. Gain=23.5dB, Min. Gain=0dB)
  ES8388_Write_Reg( 0x13, 0xF9); // Reg 0x13 = 0xc0 (ALC Target=-4.5dB, ALC Hold time =0 mS)
  ES8388_Write_Reg( 0x14, 0x02); // Reg 0x14 = 0x12(Decay time =820uS , Attack time = 416 uS)
  ES8388_Write_Reg( 0x15, 0x06); // Reg 0x15 = 0x06(ALC mode)
  ES8388_Write_Reg( 0x16, 0xc3); // Reg 0x16 = 0xc3(nose gate = -40.5dB, NGG = 0x01(mute ADC))
  ES8388_Write_Reg( 0x02, 0x55); // Reg 0x16 = 0x55 (Start up DLL, STM and Digital block for recording);

  // ES8388_Write_Reg(3, 0x09);
  ES8388_Write_Reg(3, 0x00);

  // reset power DAC and ADC
  st += ES8388_Write_Reg(2 , 0xF0);
  st += ES8388_Write_Reg(2 , 0x00);

  // unmute
  st += ES8388_Write_Reg(25, 0x00);
  // amp validation
  gpio_set_level(PA, 1);
  st += ES8388_Write_Reg(46, 15);
  st += ES8388_Write_Reg(47, 15);
  st += ES8388_Write_Reg(48, 33);
  st += ES8388_Write_Reg(49, 33);
  return st;

}

//////////////////////////////////////////////////////////////////////////
// Print the header for a display screen
//////////////////////////////////////////////////////////////////////////
void headerS(const char *string, uint16_t color)
{
  tft.fillScreen(color);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.fillRect(0, 0, 320, 36, TFT_BLUE);
  tft.setTextDatum(TC_DATUM);
  tft.drawString(string, 160, 10, 4);
}
void headerL(const char *string1, const char *string2, uint16_t color)
{
  tft.fillScreen(color);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE
                   , TFT_BLUE);
  tft.fillRect(0, 0, 320, 60, TFT_BLUE);
  tft.setTextDatum(TC_DATUM);
  tft.drawString(string1, 160, 10, 4);
  tft.drawString(string2, 160, 40, 2);
}
//////////////////////////////////////////////////////////////////////
//adc buttons
//////////////////////////////////////////////////////////////////////

int button_get_level(int nb)
{
#define THRESHOLD 200
#define maxB 3
  static int adcB[] = {1850, 2350, 450, 930};
  int adcValue, V;
  if ((nb > maxB) || (nb < 0))return -1;
  adcValue = analogRead(KEYs_ADC);
  V = adcB[nb];
  if (abs(V - adcValue) < THRESHOLD ) return 0; else return 1;
}

void setup(){

int FILESIZE;
#define bytesToRead 128
uint8_t b[bytesToRead];
//bool started = false;
// SD 1 wire
#define clk         14
#define cmd         15
#define d0          2

#define SDA         18
#define SCL         11


#define I2S_DOUT      17
#define I2S_BCLK      5
#define I2S_LRC       16
#define I2S_DIN       4
#define I2SW (i2s_port_t)0
#define I2SR (i2s_port_t)1

// SD 1 wire
#define clk         14
#define cmd         15
#define d0          2
#define EN_4G           GPIO_NUM_38
#define nominalVal 2700
#define alertVal  350
#define zeroVal   2000

int adcValue;
int res;
int vol = 20;
#define maxVol 31

IRrecv irrecv(IR);
decode_results results;


WiFiClient espClient;
PubSubClient client(espClient);
bool testON;
time_t now;
struct tm timeinfo;
char timeStr[60];

uint8_t header[] = {
  0x52, 0x49, 0x46, 0x46, //"RIFF"
  0x24, 0x7D, 0x00, 0x00, //taille fichier - 8 (little endian)
  0x57, 0x41, 0x56, 0x45, //"WAVE"
  0x66, 0x6d, 0x74, 0x20, //"fmt "
  0x10, 0x00, 0x00, 0x00, //nb d'octets du bloc
  0x01, 0x00,             //format PCM
  0x02, 0x00,             //nombre de canaux
  0x40, 0x1F, 0x00, 0x00, //frequence d'echantillonnage 8000
  0x00, 0x7D, 0x00, 0x00, //nombre d'octets a lire par seconde   32000
  0x02, 0x00,             //nombre d'octets par bloc d'échantillonnage
  0x10, 0x00,             //nb de bits par echantillon
  0x64, 0x61, 0x74, 0x61, //"data"
  0x00, 0x7D, 0x00, 0x00
};   //nombre d'octets de donnees

#define BLOCK_SIZE 128

// Configuration pour I2S0 (écriture)
i2s_config_t i2s_config_write = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 8000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 128/*,
    
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
    */
};

// Configuration pour I2S1 (lecture)
i2s_config_t i2s_config_read = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 8000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 128/*,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
    */
};

// Configuration des broches pour I2S0 (écriture)
i2s_pin_config_t pin_config_write = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
};

// Configuration des broches pour I2S1 (lecture)
i2s_pin_config_t pin_config_read = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DIN
};


  
#define RECB CLICK1B
#define bytesToRead 128
  //  uint8_t b[bytesToRead];
// printf("FreeHeap ===> %d\n", ESP.getFreeHeap()); 
  char buf[20];
  Serial.begin(115200);

  ///////////////////////////////////////////////////////
  // init gpios
  ///////////////////////////////////////////////////////
  //gpio_reset_pin
  gpio_reset_pin(CLICK1);
  gpio_reset_pin(CLICK2);
  gpio_reset_pin(EN_4G);
  gpio_reset_pin(JACK_DETECT);
  gpio_reset_pin(USB_DETECT);
  //gpio_set_direction
  gpio_set_direction(CLICK1, GPIO_MODE_INPUT);
  gpio_set_direction(CLICK2, GPIO_MODE_INPUT);
  gpio_set_direction(JACK_DETECT, GPIO_MODE_INPUT);
  gpio_set_direction(USB_DETECT, GPIO_MODE_INPUT);
  //gpio_set_pull_mode
  gpio_set_pull_mode(CLICK1, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(CLICK2, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(JACK_DETECT, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(USB_DETECT, GPIO_PULLUP_ONLY);
  // power enable
  gpio_reset_pin(PA);
  gpio_set_direction(PA, GPIO_MODE_OUTPUT);
  // 4G enable
  gpio_set_direction(EN_4G, GPIO_MODE_OUTPUT);

  //////////////////////////////////////////////////
  //Encoders init
  //////////////////////////////////////////////////
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  volEncoder.attachHalfQuad(ENC_A1, ENC_B1);
  staEncoder.attachHalfQuad(ENC_A2, ENC_B2);

////////////////////////////////////////////////////////////////
// restore OTA0 as boot partition
///////////////////////////////////////////////////////////////  
 const esp_partition_t* partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (partition != NULL) {
      esp_ota_set_boot_partition(partition);
    }
///////////////////////////////////////////////////////////////
// 4G modem validation
///////////////////////////////////////////////////////////////    
  gpio_set_level(EN_4G, 1);

  //////////////////////////////////////////////////////
  //Screen init
  //////////////////////////////////////////////////////
  printf("screen init...\n");
  tft.init();
  tft.setRotation(1);
  headerS("Ros&Co", TFT_NAVY);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(TC_DATUM);
  //  tft.fillScreen(TFT_NAVY);
  tft.drawString("FACTORY TEST", 160, 115, 4);


  ///////////////////////////////////////////////////////
  // test#1 buttons
  //////////////////////////////////////////////////////
 
  RECB = false;
  delay(2000);
  headerL("test#1 : Buttons", "try each, from left to right bottom to top", TFT_NAVY);
  tft.fillRect(130, 100, 70, 70, TFT_WHITE);
  tft.fillRect(132, 102, 66, 66, TFT_NAVY);
  tft.setTextColor(TFT_RED);
  tft.setTextDatum(TC_DATUM);
  for (int i = 0; i < 4; i++)
  {
    while (button_get_level(i) == 1) delay(100);
    sprintf(buf, "SW%d\n", i + 1);
    tft.fillRect(132, 102, 66, 66, TFT_NAVY);
    tft.drawString(buf, 170, 125, 4);

    delay(100);
  }
  RECB = false;
  delay(500);
  tft.fillRect(132, 102, 66, 66, TFT_NAVY);
  delay(500);
  tft.drawString("OK", 170, 125, 4);
  delay(1000);


  ///////////////////////////////////////////////////////
  //test#2 Jack detect
  ///////////////////////////////////////////////////////
  headerL("test#2 : Jack detect", "plug an audio cable", TFT_NAVY);
  tft.fillRect(130, 100, 70, 70, TFT_WHITE);
  tft.fillRect(132, 102, 66, 66, TFT_NAVY);
  int ndj = 0;
  while (gpio_get_level(JACK_DETECT) == 1) delay(50);
  delay(500);
  tft.fillRect(132, 102, 66, 66, TFT_NAVY);
  delay(500);
  tft.setTextColor(TFT_RED);
  tft.drawString("OK", 170, 125, 4);
  delay(2000);



  ///////////////////////////////////////////////////////
  //test#2 USB detect
  ///////////////////////////////////////////////////////
  headerL("test#2 : USB detect", "plug an usb cable", TFT_NAVY);
  tft.fillRect(130, 100, 70, 70, TFT_WHITE);
  tft.fillRect(132, 102, 66, 66, TFT_NAVY);
  while (gpio_get_level(USB_DETECT) == 0) delay(50);
  delay(500);
  tft.fillRect(132, 102, 66, 66, TFT_NAVY);
  delay(500);
  tft.setTextColor(TFT_RED);
  tft.drawString("OK", 170, 125, 4);
  delay(2000);


  ///////////////////////////////////////////////////////
  //test#3 volume encoder
  ///////////////////////////////////////////////////////
  headerL("test#3 : Volume encoder", "Turn right and left then click to terminate", TFT_NAVY);
  tft.fillRect(130, 100, 70, 70, TFT_WHITE);
  tft.fillRect(132, 102, 66, 66, TFT_NAVY);
#define maxC 10
  volEncoder.setCount(0);
  tft.setTextColor(TFT_RED);
  tft.setTextDatum(TC_DATUM);
  while (volEncoder.getCount() < maxC)
  {
    sprintf(buf, "%d\n", volEncoder.getCount());
    tft.fillRect(150, 110, 35, 35, TFT_NAVY);
    tft.drawString(buf, 170, 125, 4);
    delay(100);
  }
   volEncoder.setCount(maxC);
  while (volEncoder.getCount() > -maxC)
  {
    if (volEncoder.getCount() > maxC) continue;
    sprintf(buf, "%d\n", volEncoder.getCount());
    tft.fillRect(150, 110, 35, 35, TFT_NAVY);
    tft.drawString(buf, 170, 125, 4);
    delay(100);
  }
  sprintf(buf, "%d\n", -maxC);
  tft.fillRect(150, 110, 35, 35, TFT_NAVY);
  tft.drawString(buf, 170, 125, 4);

  while (gpio_get_level(CLICK1) == 1) delay(50);
  tft.fillRect(150, 110, 35, 35, TFT_NAVY);
  tft.drawString("OK", 170, 125, 4);
  delay(1000);
  tft.fillRect(132, 102, 66, 66, TFT_NAVY);


  ///////////////////////////////////////////////////////
  // test#4 stations encoder
  ///////////////////////////////////////////////////////
  headerL("test#4 : Stations encoder", "Turn right and left then click to terminate", TFT_NAVY);
  tft.fillRect(130, 100, 70, 70, TFT_WHITE);
  tft.fillRect(132, 102, 66, 66, TFT_NAVY);
  staEncoder.setCount(0);
  tft.setTextColor(TFT_RED);
  tft.setTextDatum(TC_DATUM);
  while (staEncoder.getCount() < maxC)
  {
    sprintf(buf, "%d\n", staEncoder.getCount());
    tft.fillRect(150, 110, 35, 35, TFT_NAVY);
    tft.drawString(buf, 170, 125, 4);
    delay(100);
  }
  staEncoder.setCount(maxC);
  while (staEncoder.getCount() > -maxC)
  {
    if (staEncoder.getCount() > maxC) continue;
    sprintf(buf, "%d\n", staEncoder.getCount());
    tft.fillRect(150, 110, 35, 35, TFT_NAVY);
    tft.drawString(buf, 170, 125, 4);
    delay(100);
  }

  sprintf(buf, "%d\n", -maxC);
  tft.fillRect(150, 110, 35, 35, TFT_NAVY);
  tft.drawString(buf, 170, 125, 4);
  while (gpio_get_level(CLICK2) == 1) delay(50);
  tft.fillRect(150, 110, 35, 35, TFT_NAVY);
  tft.drawString("OK", 170, 125, 4);
  delay(1000);


  ///////////////////////////////////////////////////////
  // test#5 Battery
  ///////////////////////////////////////////////////////
#define adcVMAX  1000
#define adcVMIN   200
  headerL("test#5 : Battery adc line", "Nothing to do...", TFT_NAVY);
  tft.fillRect(100, 100, 130, 70, TFT_WHITE);
  tft.fillRect(102, 102, 126, 66, TFT_NAVY);
  adcValue = analogRead(BAT_GAUGE_PIN) - zeroVal;
  sprintf((char*)b, "V = %d\n", adcValue);
  tft.setTextColor(TFT_RED);
  tft.drawString((const char*)b, 170, 125, 4);
  delay(2000);
  if ((adcValue < adcVMAX) && (adcValue > adcVMIN))
  {
    tft.fillRect(102, 102, 126, 66, TFT_NAVY);
    tft.drawString("OK", 170, 125, 4);
    delay(1000);
  }
  else return;


  ///////////////////////////////////////////////////////
  // test#6 Remote
  ///////////////////////////////////////////////////////
  headerL("test#6: Remote", "Press the OK button", TFT_NAVY);
  tft.fillRect(100, 100, 130, 70, TFT_WHITE);
  tft.fillRect(102, 102, 126, 66, TFT_NAVY);
  tft.setTextColor(TFT_RED, TFT_NAVY);
  tft.setTextDatum(TC_DATUM);
  irrecv.enableIRIn();
  uint32_t v;
  while (v != 0x609F)
  {
    if (irrecv.decode(&results)) {
      if (results.decode_type == NEC)
      {
        v = results.value;
        if (v != 0xffffffff) v = v & 0xFFFF;
      }
      irrecv.resume();  // Receive the next value

      delay(100);
    }
  }
  tft.fillRect(102, 102, 126, 66, TFT_NAVY);
  tft.drawString("OK", 170, 125, 4);
  delay(1000);

  ///////////////////////////////////////////////////////
  // test#7 SD
  ///////////////////////////////////////////////////////
  headerL("test#7: SD", "Nothing todo...", TFT_NAVY);
  tft.fillRect(50, 100, 220, 40, TFT_WHITE);
  tft.fillRect(52, 102, 216, 36, TFT_NAVY);
  tft.setTextColor(TFT_RED, TFT_NAVY);
  tft.setTextDatum(TC_DATUM);

  if (! SD_MMC.setPins(clk, cmd, d0)) {
    printf("Pin change failed!\n");
    tft.drawString("SD Pin change failed", 160, 110, 2);
    return;
  }
  if (!SD_MMC.begin("/sdcard", true)) {
    printf("Card Mount Failed\n");
    tft.drawString("Card mount failed", 160, 110, 2);
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    printf("No SD_MMC card attached\n");
    tft.drawString("No SD_MMC card attached", 160, 110, 2);
    return;
  }
  printf("SD init OK\n");
  tft.drawString("SD init OK", 160, 110, 4);
  File f = SD_MMC.open("/test", FILE_WRITE);
  f.write((const uint8_t*)"1234567890", 11);
  tft.fillRect(52, 102, 216, 36, TFT_NAVY);
  tft.drawString("Writing...", 160, 110, 4);
  f.close();
  delay(1000);
  f = SD_MMC.open("/test", FILE_READ);
  char bsd[15];
  f.read((uint8_t*)bsd, 15);
  tft.fillRect(52, 102, 216, 36, TFT_NAVY);
  tft.drawString("Reading...", 160, 110, 4);
  f.close();
  delay(1000);
  if (strcmp(bsd, "1234567890") != 0) return;
  tft.fillRect(52, 102, 216, 36, TFT_NAVY);
  tft.drawString("OK", 160, 110, 4);


  ///////////////////////////////////////////////////////
  // test#8 WiFi
  ///////////////////////////////////////////////////////
  headerL("test#8: WiFi-> xhkap", "Nothing todo...", TFT_NAVY);
  tft.fillRect(50, 100, 220, 40, TFT_WHITE);
  tft.fillRect(52, 102, 216, 36, TFT_NAVY);
  tft.setTextColor(TFT_RED, TFT_NAVY);
  tft.setTextDatum(TC_DATUM);

  tft.drawString("Connecting...", 160, 110, 4);
#define TOMax 22

  WiFi.mode(WIFI_STA);
  WiFi.begin("xhkap", "12345678");
  int to = 0;
  while ((!WiFi.isConnected()) && (to < TOMax))
  {
    to++;
    delay(1000);
  }
  printf("%d   %d\n", to, WiFi.isConnected());
  if (to < TOMax)
  {
    printf("Connected to xhkap(4G)\n");
    tft.fillRect(52, 102, 216, 36, TFT_NAVY);
    tft.drawString("Connected...", 160, 110, 4);
  }
  else
  {
    tft.fillRect(52, 102, 216, 36, TFT_NAVY);
    tft.drawString("Connection failed...", 160, 110, 4);
    for (;;);
  }
  delay(1000);
  sprintf((char*)b, "RSSI = %d dB\n", WiFi.RSSI());
  tft.setTextColor(TFT_RED, TFT_NAVY);
  tft.fillRect(52, 102, 216, 36, TFT_NAVY);
  tft.drawString((const char*)b, 160, 110, 4);
  delay(2000);

  ///////////////////////////////////////////////////////
  // test#9 Audio => speaker
  ///////////////////////////////////////////////////////
  headerL("test#9 Audio => Speaker", "nothing to do...", TFT_NAVY);
  tft.fillRect(50, 100, 220, 40, TFT_WHITE);
  tft.fillRect(52, 102, 216, 36, TFT_NAVY);
  tft.setTextColor(TFT_RED, TFT_NAVY);
  tft.setTextDatum(TC_DATUM);
  //I2S init
  i2s_driver_install(I2SW, &i2s_config_write, 0, NULL);
  i2s_set_pin(I2SW, &pin_config_write);
  i2s_set_clk(I2SW, 8000, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);
  i2s_zero_dma_buffer(I2SW);
  i2s_stop(I2SW);
  delay(1000);


  //ES8388 codec init
  Wire.setPins(SDA, SCL);
  Wire.begin();
  res = ES8388_Init();
  delay(500);
  tft.fillRect(52, 102, 216, 36, TFT_NAVY);
  tft.drawString("Playing...", 160, 110, 4);
  printf("mono\n");
  gpio_set_level(PA, 1);
  //mono (L+R)/2
  ES8388_Write_Reg(29, 0x20);
  // Right DAC phase inversion + click free power up/down
  ES8388_Write_Reg(28, 0x14);
  // DAC power-up LOUT1/ROUT1 enabled
  ES8388_Write_Reg(4, 0x30);
  i2s_start(I2SW);
  LittleFS.begin();
  int n;
  size_t t;
  f = LittleFS.open("/leftright.wav", FILE_READ);
  f.seek(44);
  i2s_zero_dma_buffer(I2SW);
  do
  {
    n = f.read(b, BLOCK_SIZE );
    i2s_write(I2SW, b, n, &t, portMAX_DELAY);
  } while (n > 0);
  i2s_zero_dma_buffer(I2SW);

  f.close();
  delay(500);
  tft.fillRect(52, 102, 216, 36, TFT_NAVY);
  tft.drawString("  OK", 160, 110, 4);
  delay(1000);

  ///////////////////////////////////////////////////////
  // test#10 Audio => headphones
  ///////////////////////////////////////////////////////
  headerL("test#10 Audio => HP", "plug in HP ", TFT_NAVY);
  tft.fillRect(50, 100, 220, 40, TFT_WHITE);
  tft.fillRect(52, 102, 216, 36, TFT_NAVY);
  tft.setTextColor(TFT_RED, TFT_NAVY);
  tft.drawString("Playing...", 160, 110, 4);
  tft.setTextDatum(TC_DATUM);
  while (gpio_get_level(JACK_DETECT) == 1) delay(50);
  delay(1000);
  gpio_set_level(PA, 0);
  //stereo
  ES8388_Write_Reg(29, 0x00);
  // no Right DAC phase inversion + click free power up/down
  ES8388_Write_Reg(28, 0x10);
  // DAC power-up LOUT2/ROUT2 enabled
  ES8388_Write_Reg(4, 0x0C);
  f = LittleFS.open("/leftright.wav", FILE_READ);
  f.seek(44);
  do
  {
    n = f.read(b, BLOCK_SIZE );
    i2s_write(I2SW, b, n, &t, portMAX_DELAY);
  } while (n > 0);
  i2s_zero_dma_buffer(I2SW);
  i2s_stop(I2SW);
  f.close();
  delay(500);
  tft.fillRect(52, 102, 216, 36, TFT_NAVY);
  tft.drawString("  OK", 160, 110, 4);
  delay(1000);


  ///////////////////////////////////////////////////////////////////
  // test #11 Microphones
  ///////////////////////////////////////////////////////////////////
  headerL("test#11 : Microphones", "Click1 and speak", TFT_NAVY);
  tft.fillRect(50, 100, 220, 40, TFT_WHITE);
  tft.fillRect(52, 102, 216, 36, TFT_NAVY);
  tft.setTextColor(TFT_RED, TFT_NAVY);
  tft.setTextDatum(TC_DATUM);
  i2s_driver_install(I2SR, &i2s_config_read, 0, NULL);
  i2s_set_pin(I2SR, &pin_config_read);
  i2s_set_clk(I2SR, 8000, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);  
  i2s_stop(I2SR);
  i2s_start(I2SR);
  i2s_zero_dma_buffer(I2SR);
  printf("mono\n");
  gpio_set_level(PA, 1);
  //mono (L+R)/2
  ES8388_Write_Reg(29, 0x20);
  // Right DAC phase inversion + click free power up/down
  ES8388_Write_Reg(28, 0x14);
  // DAC power-up LOUT1/ROUT1 enabled
  ES8388_Write_Reg(4, 0x30);
#define bytesToRead 128
 // uint8_t b[bytesToRead];
//  RECB = false;
//  while(RECB == false) delay(10);
    while (gpio_get_level(CLICK1) == 1) delay(50);  
    delay(200);
    i2s_start(I2SR);
    RECB = false;
    testON = true;
    f = SD_MMC.open("/record.wav", FILE_WRITE);
    f.seek(44);
//    int n;
    int i;
 //   size_t t;
    uint32_t s = 44;
    headerL("test#11 : Microphones", "Click1 to stop recording", TFT_NAVY); 
//    while (RECB == false)
    while (gpio_get_level(CLICK1) == 1) 
    {
      i = 0;
      n = 0;
      do
      {
        while (n == 0) i2s_read(I2SR, &b[i], BLOCK_SIZE, (size_t*)&n, portMAX_DELAY);
        i = i + n;
      } while ((i < bytesToRead) && (RECB == false));

      f.write(b, i);
      s += i;
    }
   
    printf("writing header...\n");
    f.seek(0);
    header[40] = s & 0xFF;
    header[41] = (s >> 8) & 0xFF;
    header[42] = (s >> 16) & 0xFF;
    header[43] = (s >> 24) & 0xFF;
    header[4] = (s - 8) & 0xFF;
    header[5] = ((s - 8) >> 8) & 0xFF;
    header[6] = ((s - 8) >> 16) & 0xFF;
    header[7] = ((s - 8) >> 24) & 0xFF;
    f.write(header, 44);
    printf("end\n");
    f.seek(s);
    f.close();
    delay(100);
    RECB = false;
 //   while (RECB == false) delay(10);   
    headerL("test#11 : Microphones", "Playing...", TFT_NAVY);
    i2s_stop(I2SR);
    i2s_set_pin(I2SW, &pin_config_write);
    i2s_set_clk(I2SW, 8000, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);
    i2s_start(I2SW);
    f = SD_MMC.open("/record.wav", FILE_READ);
    f.seek(44);
    do
    {
      n = f.read(b, BLOCK_SIZE);
      i2s_write(I2SW, b, n, &t, portMAX_DELAY);
    } while (n > 0);
    i2s_zero_dma_buffer(I2SW);
    i2s_stop(I2SW);
    f.close();
    RECB = false;
    testON = false;


  delay(500);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setTextDatum(TC_DATUM);
  tft.fillScreen(TFT_NAVY);
  
  tft.fillRect(50, 100, 220, 40, TFT_WHITE);
  tft.fillRect(52, 102, 216, 36, TFT_NAVY);
  tft.drawString("Wait...", 160, 110, 4);
  //////////////////////////////////////////////////////////////////
  // NTP time init
  //
  ////////////////////////////////////////////////////////////////////
  // time zone init
  setenv("TZ", "CEST-1", 1);
  tzset();
  //sntp init
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_init();
  int retry = 0;
  while ((timeinfo.tm_year < (2016 - 1900)) && (++retry < 20))
  {
    delay(500);
    time(&now);
    localtime_r(&now, &timeinfo);
  }
  time(&now);
  localtime_r(&now, &timeinfo);
  sprintf(timeStr, "ID: %s Ok on : %4d %02d %02d  %02d:%02d:%02d\n", WiFi.macAddress().c_str(), timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  printf("==>> %s\n", timeStr);



////////////////////////////////////////////////////////////
// connecting to Google sheets
////////////////////////////////////////////////////////////  
    delay(1000);
    printf("Google sheets\n");
//    String uniqueID = String((uint32_t)(ESP.getEfuseMac() >> 32), HEX) + String((uint32_t)ESP.getEfuseMac(), HEX) ;
    String uniqueID = String(timeStr);
    String data = "id=" + uniqueID;
    const char* googleScriptURL = "https://script.google.com/macros/s/AKfycbyNtmE7-G77xtEAAL8aoCWozyrNJcV2hoqTdlHBWbix1YImqDyb7lI8znLH3R11cq_s/exec";
    HTTPClient http;
    http.begin(googleScriptURL);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    int httpResponseCode = http.POST(data);
    delay(1000);
     if(httpResponseCode > 0) {
      String response = http.getString();
      printf("%x\n",httpResponseCode);
      Serial.println(response);
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }

    http.end();   


  delay(500);
  tft.setRotation(1);
  headerS("Ros&Co", TFT_NAVY);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("TEST OK", 160, 115, 4);
  delay(2000);
  ESP.restart();
}
void loop()
{
}
