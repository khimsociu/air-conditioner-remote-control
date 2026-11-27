#include <Arduino.h>
#include <Preferences.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRac.h>
#include <IRutils.h>
#include <dtg.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <driver/gpio.h>
#include <WiFi.h>
#include <secret.h>
//#include <secret_example.h>

// ===== Chân =====
const uint16_t PIN_LED_XANH = 7;  // nháy khi gửi
const uint16_t PIN_LED_DO   = 6;  // nháy khi nhận
const uint16_t PIN_IR_TX    = 5;  // -> điện trở -> base BJT
const uint16_t PIN_IR_RX    = 4;  // OUT của VS1838B
const uint16_t PIN_OLED_SDA = 8;  // OLED 0.96" I2C
const uint16_t PIN_OLED_SCL = 9;
const uint16_t PIN_ENC_A    = 2;  // encoder CLK
const uint16_t PIN_ENC_B    = 3;  // encoder DT
const uint16_t PIN_ENC_SW   = 10; // encoder SW (nút bấm)

DtgWiFi dtg(PROJECT_ID, GW_ALIAS, MQTT_USER, MQTT_PASS, CLIENT_ID);

DTG_CONNECTED()
{
}

// ===== Cấu hình nhận =====
const uint16_t kCaptureBufferSize = 1024;
const uint8_t  kTimeout = 50;

// ===== Bảng nhóm: {danh sách protocol (thử lần lượt), danh sách tên hãng dùng nhóm này} =====
// Thêm hãng mới: ghi tên (chữ thường, không dấu) vào cột thứ hai.
const char* GROUPS[][2] = {
  {"MIRAGE",       "mirage,sumikura"},
  {"PANASONIC_AC", "panasonic"},
  // Daikin có nhiều biến thể remote: chọn "daikin 1", "daikin 2"... lần lượt cho đến khi máy phản ứng
  {"DAIKIN,DAIKIN2,DAIKIN216,DAIKIN176,DAIKIN160,DAIKIN152,DAIKIN128,DAIKIN64", "daikin"},
  {"LG2,LG",       "lg"},
  {"SAMSUNG_AC",   "samsung"},
  {"TOSHIBA_AC",   "toshiba"},
  {"MITSUBISHI_AC,MITSUBISHI112", "mitsubishi"},
  {"GREE",         "gree"},
  {"MIDEA,COOLIX", "midea"},
  {"CARRIER_AC",   "carrier"},
  {"SHARP_AC",     "sharp"},
  {"HITACHI_AC",   "hitachi"},
  {"HAIER_AC",     "haier,aqua"},
  {"FUJITSU_AC",   "fujitsu,general"},
  {"TCL112AC",     "tcl"},
  {"COOLIX",       "coolix"},
  // Nhóm đoán (chưa kiểm chứng), thử lần lượt bằng: ten <hang> 1 / 2 / 3
  {"GREE,COOLIX,MIDEA", "casper,reetech"},
  {"COOLIX,MIDEA,GREE", "funiki"},
  {"COOLIX,GREE,MIDEA", "nagakawa"},
};

// ===== Hồ sơ điều hòa (lưu cả trạng thái điều khiển) =====
const uint8_t MAX_PROFILES = 2;   // chỉ lưu 2 thiết bị
// Bit trong flags (F_LIGHT chỉ giữ nội bộ để đồng bộ với remote gốc, không điều khiển từ web)
const uint8_t F_TURBO = 1, F_QUIET = 2, F_ECONO = 4, F_LIGHT = 8,
              F_FILTER = 16, F_CLEAN = 32, F_BEEP = 64, F_SLEEP = 128;

struct Profile {
  char    name[16];
  int16_t proto;
  int16_t model;
  bool    power;
  int8_t  mode;
  int8_t  fan;
  int8_t  swingv;
  int8_t  swingh;
  uint8_t flags;
  float   degrees;
};
Profile profiles[MAX_PROFILES];
uint8_t nProfiles = 0;
int8_t  activeIdx = -1;

// ===== Echo: VS1838B nghe lại chính tín hiệu mình gửi (để debug) =====

IRrecv irrecv(PIN_IR_RX, kCaptureBufferSize, kTimeout, true);
decode_results results;
IRac ac(PIN_IR_TX);
Preferences prefs;

// ===== Hẹn giờ đếm ngược (millis, không cần giờ thật; mất khi reset) =====
const int HEN_MAX_PHUT = 1440;   // 24 giờ
struct Hen {
  bool     on  = false;
  uint32_t den = 0;              // thời điểm (millis) hẹn giờ chạy
};
Hen henTat, henBat;

int phutConLai(const Hen &h) {
  if (!h.on) return 0;
  int32_t ms = (int32_t)(h.den - millis());
  if (ms <= 0) return 0;
  return (ms + 59999) / 60000;   // làm tròn lên
}

bool henDenGio(const Hen &h) {
  return h.on && (int32_t)(millis() - h.den) >= 0;
}

void datHen(Hen &h, int phut) {
  if (phut <= 0) { h.on = false; return; }
  h.on  = true;
  h.den = millis() + (uint32_t)phut * 60000UL;
}

// ===== Lệnh nhận từ Hub (gom lại rồi gửi IR trong loop) =====
struct HubCmd {
  bool     has    = false;
  uint32_t last   = 0;
  int      power  = -1;     // -1 = không đổi
  float    temp   = NAN;    // NAN = không đổi
  int      mode   = -1;
  int      fan    = -1;
  int      swingv = -1;
  int      turbo  = -1;
  int      quiet  = -1;
  int      econo  = -1;
  int      sleep  = -1;
  int      tOff   = -1;     // hẹn tắt (phút), 0 = hủy
  int      tOn    = -1;     // hẹn bật (phút), 0 = hủy
  int      tCancel = -1;    // 1 = hủy cả hai hẹn giờ
};
HubCmd hubCmd;

inline void hubDanhDau() { hubCmd.has = true; hubCmd.last = millis(); }

// Tên trong DTG_LISTEN phải trùng Stream ID trên nền tảng (topic down/c/_/<stream>).
DTG_LISTEN("power")     { hubCmd.power  = param.getBool() ? 1 : 0;  hubDanhDau(); }
DTG_LISTEN("temp")      { hubCmd.temp   = param.getFloat();         hubDanhDau(); }
DTG_LISTEN("mode")      { hubCmd.mode   = (int)param.getInt();      hubDanhDau(); }
DTG_LISTEN("fan")       { hubCmd.fan    = (int)param.getInt();      hubDanhDau(); }
DTG_LISTEN("swingv")    { hubCmd.swingv = (int)param.getInt();      hubDanhDau(); }
DTG_LISTEN("turbo")     { hubCmd.turbo  = param.getBool() ? 1 : 0;  hubDanhDau(); }
DTG_LISTEN("quiet")     { hubCmd.quiet  = param.getBool() ? 1 : 0;  hubDanhDau(); }
DTG_LISTEN("econo")     { hubCmd.econo  = param.getBool() ? 1 : 0;  hubDanhDau(); }
DTG_LISTEN("sleep")     { hubCmd.sleep  = param.getBool() ? 1 : 0;  hubDanhDau(); }
DTG_LISTEN("timer_off") { hubCmd.tOff   = (int)param.getInt();      hubDanhDau(); }
DTG_LISTEN("timer_on")  { hubCmd.tOn    = (int)param.getInt();      hubDanhDau(); }
DTG_LISTEN("timer_cancel") { hubCmd.tCancel = param.getBool() ? 1 : 0; hubDanhDau(); }

// ---------- Chuỗi hiển thị ----------
const char* tenMode(stdAc::opmode_t m) {
  switch (m) {
    case stdAc::opmode_t::kAuto: return "Auto";
    case stdAc::opmode_t::kCool: return "Cool";
    case stdAc::opmode_t::kHeat: return "Heat";
    case stdAc::opmode_t::kDry:  return "Dry";
    case stdAc::opmode_t::kFan:  return "Fan";
    default: return "?";
  }
}

const char* tenFan(stdAc::fanspeed_t f) {
  switch (f) {
    case stdAc::fanspeed_t::kAuto:   return "Auto";
    case stdAc::fanspeed_t::kMin:    return "Min";
    case stdAc::fanspeed_t::kLow:    return "Low";
    case stdAc::fanspeed_t::kMedium: return "Med";
    case stdAc::fanspeed_t::kHigh:   return "High";
    case stdAc::fanspeed_t::kMax:    return "Max";
    default: return "?";
  }
}

// ---------- Mã số <-> enum (dùng chung cho uplink và downlink) ----------
int maMode(stdAc::opmode_t m) {
  switch (m) {
    case stdAc::opmode_t::kAuto: return 0;
    case stdAc::opmode_t::kCool: return 1;
    case stdAc::opmode_t::kHeat: return 2;
    case stdAc::opmode_t::kDry:  return 3;
    case stdAc::opmode_t::kFan:  return 4;
    default: return -1;
  }
}

int maFan(stdAc::fanspeed_t f) {
  switch (f) {
    case stdAc::fanspeed_t::kAuto:   return 0;
    case stdAc::fanspeed_t::kMin:    return 1;
    case stdAc::fanspeed_t::kLow:    return 2;
    case stdAc::fanspeed_t::kMedium: return 3;
    case stdAc::fanspeed_t::kHigh:   return 4;
    case stdAc::fanspeed_t::kMax:    return 5;
    default: return -1;
  }
}

// Swing dọc rút gọn còn 5 mã: 0 Off, 1 Auto, 2 Low, 3 Med, 4 Max
// Chiều lên Hub: vị trí gần nhất thì gộp (Lowest/Low -> 2, High/Highest -> 4)
int maSwingV(stdAc::swingv_t v) {
  switch (v) {
    case stdAc::swingv_t::kOff:     return 0;
    case stdAc::swingv_t::kAuto:    return 1;
    case stdAc::swingv_t::kLowest:
    case stdAc::swingv_t::kLow:     return 2;
    case stdAc::swingv_t::kMiddle:  return 3;
    case stdAc::swingv_t::kHigh:
    case stdAc::swingv_t::kHighest: return 4;
    default: return -1;
  }
}

bool modeTuMa(int c, stdAc::opmode_t &o) {
  switch (c) {
    case 0: o = stdAc::opmode_t::kAuto; return true;
    case 1: o = stdAc::opmode_t::kCool; return true;
    case 2: o = stdAc::opmode_t::kHeat; return true;
    case 3: o = stdAc::opmode_t::kDry;  return true;
    case 4: o = stdAc::opmode_t::kFan;  return true;
    default: return false;
  }
}

bool fanTuMa(int c, stdAc::fanspeed_t &o) {
  switch (c) {
    case 0: o = stdAc::fanspeed_t::kAuto;   return true;
    case 1: o = stdAc::fanspeed_t::kMin;    return true;
    case 2: o = stdAc::fanspeed_t::kLow;    return true;
    case 3: o = stdAc::fanspeed_t::kMedium; return true;
    case 4: o = stdAc::fanspeed_t::kHigh;   return true;
    case 5: o = stdAc::fanspeed_t::kMax;    return true;
    default: return false;
  }
}

// Chiều xuống từ Hub (cách A: hai đầu và giữa)
bool swingvTuMa(int c, stdAc::swingv_t &o) {
  switch (c) {
    case 0: o = stdAc::swingv_t::kOff;     return true;
    case 1: o = stdAc::swingv_t::kAuto;    return true;
    case 2: o = stdAc::swingv_t::kLowest;  return true;
    case 3: o = stdAc::swingv_t::kMiddle;  return true;
    case 4: o = stdAc::swingv_t::kHighest; return true;
    default: return false;
  }
}

// ---------- Đẩy trạng thái lên DTG IoT Hub ----------
void dayLenIoT() {
  if (!dtg.connected()) return;
  if (ac.next.protocol == decode_type_t::UNKNOWN) return;   // chưa chọn máy
  dtg.publish("power",     ac.next.power ? 1 : 0);
  dtg.publish("temp",      (float)ac.next.degrees);
  dtg.publish("mode",      maMode(ac.next.mode));
  dtg.publish("fan",       maFan(ac.next.fanspeed));
  dtg.publish("swingv",    maSwingV(ac.next.swingv));
  dtg.publish("turbo",     ac.next.turbo ? 1 : 0);
  dtg.publish("quiet",     ac.next.quiet ? 1 : 0);
  dtg.publish("econo",     ac.next.econo ? 1 : 0);
  dtg.publish("sleep",     ac.next.sleep >= 0 ? 1 : 0);
  dtg.publish("timer_off", phutConLai(henTat));
  dtg.publish("timer_on",  phutConLai(henBat));

  // Tên hãng + protocol đang dùng (dạng chuỗi). Trên Hub cần tạo 2 datastream kiểu text: "brand" và "protocol"
  char js[64];
  const char* ten = (activeIdx >= 0) ? profiles[activeIdx].name : "";
  snprintf(js, sizeof(js), "{\"v\":\"%s\"}", ten);
  dtg.publishJson("brand", js);
  snprintf(js, sizeof(js), "{\"v\":\"%s\"}", typeToString(ac.next.protocol).c_str());
  dtg.publishJson("protocol", js);
}

// ---------- Lưu / nạp ----------
void saveProfiles() {
  prefs.begin("ac", false);
  prefs.putUChar("n", nProfiles);
  prefs.putChar("act", activeIdx);
  prefs.putBytes("p3", profiles, sizeof(profiles));
  prefs.end();
}

void loadProfiles() {
  prefs.begin("ac", true);
  nProfiles = prefs.getUChar("n", 0);
  activeIdx = prefs.getChar("act", -1);
  if (prefs.isKey("p3") && prefs.getBytesLength("p3") == sizeof(profiles)) {
    prefs.getBytes("p3", profiles, sizeof(profiles));
  } else {
    nProfiles = 0;      // dữ liệu cũ hoặc chưa có: bắt đầu lại
    activeIdx = -1;
  }
  prefs.end();
  if (nProfiles > MAX_PROFILES) nProfiles = 0;
  if (activeIdx >= nProfiles) activeIdx = -1;
}

int findProfile(const String &name) {
  for (uint8_t i = 0; i < nProfiles; i++)
    if (name.equals(profiles[i].name)) return i;
  return -1;
}

uint8_t packFlags() {
  uint8_t f = 0;
  if (ac.next.turbo)  f |= F_TURBO;
  if (ac.next.quiet)  f |= F_QUIET;
  if (ac.next.econo)  f |= F_ECONO;
  if (ac.next.light)  f |= F_LIGHT;
  if (ac.next.filter) f |= F_FILTER;
  if (ac.next.clean)  f |= F_CLEAN;
  if (ac.next.beep)   f |= F_BEEP;
  if (ac.next.sleep >= 0) f |= F_SLEEP;
  return f;
}

void unpackFlags(uint8_t f) {
  ac.next.turbo  = f & F_TURBO;
  ac.next.quiet  = f & F_QUIET;
  ac.next.econo  = f & F_ECONO;
  ac.next.light  = f & F_LIGHT;
  ac.next.filter = f & F_FILTER;
  ac.next.clean  = f & F_CLEAN;
  ac.next.beep   = f & F_BEEP;
  ac.next.sleep  = (f & F_SLEEP) ? 0 : -1;
}

// Chép trạng thái điều khiển từ ac.next vào hồ sơ idx (không đụng tên/protocol/model)
void ghiVaoProfile(int idx) {
  profiles[idx].power   = ac.next.power;
  profiles[idx].mode    = (int8_t)ac.next.mode;
  profiles[idx].fan     = (int8_t)ac.next.fanspeed;
  profiles[idx].swingv  = (int8_t)ac.next.swingv;
  profiles[idx].swingh  = (int8_t)ac.next.swingh;
  profiles[idx].flags   = packFlags();
  profiles[idx].degrees = ac.next.degrees;
}

// Ghi trạng thái hiện tại vào hồ sơ đang dùng
void luuTrangThai() {
  if (activeIdx < 0) return;
  if ((int16_t)ac.next.protocol != profiles[activeIdx].proto) return;  // đang thử protocol khác
  ghiVaoProfile(activeIdx);
  saveProfiles();
}

// ---------- Trạng thái mặc định ----------
void setDefaults() {
  ac.next.protocol = decode_type_t::UNKNOWN;
  ac.next.model    = -1;
  ac.next.mode     = stdAc::opmode_t::kCool;
  ac.next.celsius  = true;
  ac.next.degrees  = 26;
  ac.next.fanspeed = stdAc::fanspeed_t::kAuto;
  ac.next.swingv   = stdAc::swingv_t::kOff;
  ac.next.swingh   = stdAc::swingh_t::kOff;
  ac.next.quiet = ac.next.turbo = ac.next.econo = false;
  ac.next.light = false;
  ac.next.filter = ac.next.clean = ac.next.beep = false;
  ac.next.sleep = -1;
  ac.next.clock = -1;
  ac.next.power = false;
}

// Protocol hỗ trợ nhiệt độ lẻ 0,5 độ (theo thư viện IRremoteESP8266)
bool hoTroNuaDo(decode_type_t p) {
  return p == decode_type_t::DAIKIN || p == decode_type_t::FUJITSU_AC ||
         p == decode_type_t::MITSUBISHI_AC || p == decode_type_t::TCL112AC;
}

// Máy có 0,5 độ thì làm tròn đến 0,5; máy khác làm tròn đến độ nguyên
float lamTronNhiet(float t, decode_type_t p) {
  return hoTroNuaDo(p) ? roundf(t * 2) / 2 : roundf(t);
}

void useProfile(int idx) {
  activeIdx = idx;
  ac.next.protocol = (decode_type_t)profiles[idx].proto;
  ac.next.model    = profiles[idx].model;
  ac.next.power    = profiles[idx].power;
  ac.next.mode     = (stdAc::opmode_t)profiles[idx].mode;
  ac.next.fanspeed = (stdAc::fanspeed_t)profiles[idx].fan;
  ac.next.swingv   = (stdAc::swingv_t)profiles[idx].swingv;
  ac.next.swingh   = (stdAc::swingh_t)profiles[idx].swingh;
  unpackFlags(profiles[idx].flags);
  float d = profiles[idx].degrees;
  ac.next.degrees  = (d >= 16 && d <= 32) ? d : 26;
  saveProfiles();

  dayLenIoT();
}

// Máy vừa BẬT thì hẹn bật không còn ý nghĩa (và ngược lại): hủy hẹn giờ cùng chiều
void huyHenThua() {
  if (ac.next.power) henBat.on = false;
  else               henTat.on = false;
}

// ---------- Gửi ----------
void guiLenh() {
  if (ac.next.protocol == decode_type_t::UNKNOWN) {
    return;
  }
  irrecv.disableIRIn();   // tránh tự thu tín hiệu của chính mình
  digitalWrite(PIN_LED_XANH, HIGH);
  bool ok = ac.sendAc();
  digitalWrite(PIN_LED_XANH, LOW);
  irrecv.enableIRIn();
  if (ok) {
    huyHenThua();
    luuTrangThai();
    dayLenIoT();
  }
}

// Các lệnh chỉnh thông số: nếu đang coi máy là TẮT thì gửi kèm BẬT,
// để không vô tình gửi frame "Power Off"
void batNeuTat() {
  if (!ac.next.power) {
    ac.next.power = true;
  }
}

// Đến giờ hẹn thì bật/tắt máy (gọi mỗi vòng loop)
void kiemTraHen() {
  bool tat = henDenGio(henTat);
  bool bat = henDenGio(henBat);
  if (!tat && !bat) return;
  if (tat) henTat.on = false;
  if (bat) henBat.on = false;
  ac.next.power = (bat && !tat);     // trùng giờ thì ưu tiên TẮT

  guiLenh();
}

void xoaProfile(int idx) {
  for (uint8_t i = idx; i + 1 < nProfiles; i++) profiles[i] = profiles[i + 1];
  nProfiles--;
  if (activeIdx == idx) activeIdx = -1;
  else if (activeIdx > idx) activeIdx--;
  saveProfiles();
}

// Lưu protocol/model hiện tại của ac.next thành hồ sơ.
// Hồ sơ đã tồn tại thì giữ nguyên trạng thái đã lưu.
void luuProfile(const String &name) {
  if (name.length() == 0 || name.length() >= 16) {
    return;
  }
  if (ac.next.protocol == decode_type_t::UNKNOWN) {
    return;
  }
  int idx = findProfile(name);
  bool moi = (idx < 0);
  if (moi) {
    if (nProfiles >= MAX_PROFILES) {
      return;
    }
    idx = nProfiles++;
  }
  strlcpy(profiles[idx].name, name.c_str(), sizeof(profiles[idx].name));
  profiles[idx].proto = (int16_t)ac.next.protocol;
  profiles[idx].model = (int16_t)ac.next.model;
  if (moi) ghiVaoProfile(idx);
  useProfile(idx);
}

// ---------- Nhập tên hãng ----------
bool coTrongDs(const char* ds, const String &tok) {
  String s = String(",") + ds + ",";
  return s.indexOf("," + tok + ",") >= 0;
}

// Cú pháp: ten <hang> [n]   (n = lựa chọn thứ n nếu nhóm có nhiều protocol)
void datTen(String arg) {
  arg.trim();
  int n = 1;
  int sp = arg.lastIndexOf(' ');
  if (sp > 0 && arg.substring(sp + 1).toInt() > 0) {
    n = arg.substring(sp + 1).toInt();
    arg = arg.substring(0, sp);
    arg.trim();
  }
  if (arg.length() == 0) return;

  String list = "";
  for (auto &g : GROUPS) {
    if (coTrongDs(g[1], arg)) { list = g[0]; break; }
  }
  if (list.length() == 0) {   // không có trong bảng: thử coi như tên protocol
    list = arg;
    list.toUpperCase();
  }

  int count = 1;
  for (unsigned i = 0; i < list.length(); i++) if (list[i] == ',') count++;
  if (n > count) return;

  int start = 0;
  for (int i = 1; i < n; i++) start = list.indexOf(',', start) + 1;
  int end = list.indexOf(',', start);
  if (end < 0) end = list.length();
  String p = list.substring(start, end);

  decode_type_t t = strToDecodeType(p.c_str());
  if (t == decode_type_t::UNKNOWN || !IRac::isProtocolSupported(t)) {
    return;
  }

  setDefaults();                         // trạng thái khởi đầu sạch cho hồ sơ mới
  ac.next.protocol = t;
  if (t == decode_type_t::MIRAGE) {
    ac.next.model  = 1;                  // KKG9AC1
    ac.next.swingv = stdAc::swingv_t::kMiddle;   // giống frame remote gốc
  }

  String name = arg;
  if (name.length() > 15) name = name.substring(0, 15);
  luuProfile(name);
}

// ---------- Hẹn giờ ----------
// Đặt hẹn giờ (phút). phut = 0: hủy. Trả về false nếu không hợp lệ.
bool henHopLe(int phut) {
  if (ac.next.protocol == decode_type_t::UNKNOWN) {
    return false;
  }
  if (phut < 0 || phut > HEN_MAX_PHUT) {
    return false;
  }
  return true;
}

void datHenTat(int phut) {
  if (!henHopLe(phut)) return;
  datHen(henTat, phut);
  dayLenIoT();
}

void datHenBat(int phut) {
  if (!henHopLe(phut)) return;
  datHen(henBat, phut);
  dayLenIoT();
}

// ---------- Xử lý lệnh nhận từ Hub ----------
// Máy nào biết chắc là KHÔNG có tính năng thì bỏ qua (khỏi gửi frame vô ích).
// MIRAGE KKG9AC1 (model 1): thư viện không hỗ trợ quiet và econo.
// Hãng khác: cứ gửi, thư viện tự bỏ qua phần không hỗ trợ.
bool hoTroTinhNang(const char* ten) {
  if (ac.next.protocol == decode_type_t::MIRAGE && ac.next.model == 1) {
    if (!strcmp(ten, "quiet") || !strcmp(ten, "econo")) return false;
  }
  return true;
}

// Đợi 300 ms không có lệnh mới rồi gửi MỘT frame IR gộp tất cả thay đổi.
void xuLyHubCmd() {
  if (!hubCmd.has || millis() - hubCmd.last < 300) return;
  HubCmd c = hubCmd;
  hubCmd = HubCmd{};

  // Hẹn giờ: chỉ đặt bộ đếm, không gửi IR
  if (c.tCancel > 0) { datHenTat(0); datHenBat(0); }
  if (c.tOff >= 0) datHenTat(c.tOff);
  if (c.tOn  >= 0) datHenBat(c.tOn);

  bool doi = false;        // có thay đổi thông số hợp lệ (nhiệt độ, mode, quạt, swing)
  bool tinhNang = false;   // có thay đổi tính năng (turbo, quiet, econo, sleep)

  if (!isnan(c.temp)) {
    if (c.temp >= 16 && c.temp <= 32) { ac.next.degrees = lamTronNhiet(c.temp, ac.next.protocol); doi = true; }
  }
  if (c.mode >= 0) {
    stdAc::opmode_t x;
    if (modeTuMa(c.mode, x)) { ac.next.mode = x; doi = true; }
  }
  if (c.fan >= 0) {
    stdAc::fanspeed_t x;
    if (fanTuMa(c.fan, x)) { ac.next.fanspeed = x; doi = true; }
  }
  if (c.swingv >= 0) {
    stdAc::swingv_t x;
    if (swingvTuMa(c.swingv, x)) { ac.next.swingv = x; doi = true; }
  }

  if (c.turbo >= 0) {
    ac.next.turbo = (c.turbo != 0);
    tinhNang = true;
  }
  if (c.quiet >= 0) {
    if (hoTroTinhNang("quiet")) { ac.next.quiet = (c.quiet != 0); tinhNang = true; }
  }
  if (c.econo >= 0) {
    if (hoTroTinhNang("econo")) { ac.next.econo = (c.econo != 0); tinhNang = true; }
  }
  if (c.sleep >= 0) { ac.next.sleep = (c.sleep != 0) ? 0 : -1; tinhNang = true; }

  if (c.power < 0 && !doi && !tinhNang) return;    // không có gì hợp lệ để gửi

  if (c.power >= 0) {
    ac.next.power = (c.power != 0);
  } else if (doi) {
    batNeuTat();                                   // chỉnh thông số khi máy đang tắt -> bật kèm
  } else if (!ac.next.power) {
    // Chỉ đổi tính năng mà máy đang tắt: lưu lại, không tự bật máy
    luuTrangThai();
    dayLenIoT();
    return;
  }

  guiLenh();
}

// ============================================================
// ===== Màn hình OLED 0.96" (I2C) + núm xoay encoder =========
// ============================================================
// Chỉ dùng để CHỌN HÃNG điều hòa. Nhiệt độ, bật/tắt, mode... điều khiển qua IoT.
// Màn hình chính chỉ hiển thị trạng thái.
//   Xoay hoặc bấm : mở danh sách hãng
//   Trong danh sách: xoay = duyệt | bấm = chọn | giữ lâu = quay lại
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, PIN_OLED_SCL, PIN_OLED_SDA);

const bool ENC_DAO_CHIEU = false;   // xoay phải mà thấy đi lên thì đổi thành true

volatile int16_t encDelta = 0;
volatile int8_t  encAcc   = 0;
volatile uint8_t encState = 0;

// Giải mã encoder bằng bảng trạng thái (4 bước = 1 nấc)
void IRAM_ATTR encISR() {
  static DRAM_ATTR const int8_t bang[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  encState = ((encState << 2) |
              (gpio_get_level((gpio_num_t)PIN_ENC_A) << 1) |
               gpio_get_level((gpio_num_t)PIN_ENC_B)) & 0x0F;
  encAcc = encAcc + bang[encState];
  if (encAcc >= 4)       { encDelta = encDelta + 1; encAcc = 0; }
  else if (encAcc <= -4) { encDelta = encDelta - 1; encAcc = 0; }
}

int16_t layDelta() {
  noInterrupts();
  int16_t d = encDelta;
  encDelta = 0;
  interrupts();
  return ENC_DAO_CHIEU ? -d : d;
}

// Dùng hằng số thay cho enum: Arduino IDE tự sinh khai báo hàm ở đầu file nên không thấy kiểu enum
const uint8_t NUT_KHONG = 0, NUT_NGAN = 1, NUT_DAI = 2;

// Nút bấm của encoder: bấm ngắn trả về khi nhả ra, bấm giữ >= 800 ms trả về ngay khi đủ giờ
uint8_t docNut() {
  static bool xuong = false, daDai = false;
  static uint32_t tBam = 0, tNha = 0;
  bool bam = (digitalRead(PIN_ENC_SW) == LOW);
  uint32_t now = millis();
  if (bam && !xuong && now - tNha > 40) {
    xuong = true; daDai = false; tBam = now;
  } else if (bam && xuong && !daDai && now - tBam >= 800) {
    daDai = true;
    return NUT_DAI;
  } else if (!bam && xuong) {
    xuong = false; tNha = now;
    if (!daDai && now - tBam >= 30) return NUT_NGAN;
  }
  return NUT_KHONG;
}

// ---------- Danh sách hãng cho màn hình chọn (lấy từ bảng GROUPS) ----------
struct Hang { char label[20]; char arg[20]; };
const int MAX_HANG = 48;
Hang brands[MAX_HANG];
int nBrands = 0;

void themHang(const char* label, const char* arg) {
  if (nBrands >= MAX_HANG) return;
  strlcpy(brands[nBrands].label, label, sizeof(brands[nBrands].label));
  strlcpy(brands[nBrands].arg,   arg,   sizeof(brands[nBrands].arg));
  nBrands++;
}

// Danh sách protocol "A,B,C" của nhóm chứa hãng (nullptr nếu không có trong bảng)
const char* nhomProto(const String &hang) {
  for (auto &g : GROUPS) if (coTrongDs(g[1], hang)) return g[0];
  return nullptr;
}

int soProto(const char* ds) {
  int n = 1;
  for (const char* p = ds; *p; p++) if (*p == ',') n++;
  return n;
}

// Vị trí (từ 0) của protocol t trong danh sách; -1 nếu không có
int viTriProto(const char* ds, decode_type_t t) {
  String s = ds;
  int idx = 0, start = 0;
  while (start <= (int)s.length()) {
    int end = s.indexOf(',', start);
    if (end < 0) end = s.length();
    if (strToDecodeType(s.substring(start, end).c_str()) == t) return idx;
    idx++;
    start = end + 1;
  }
  return -1;
}

// Mỗi hãng một dòng trong danh sách (các biến thể protocol của hãng được xử lý tự động)
void buildBrandList() {
  nBrands = 0;
  themHang("[Do tu remote]", "#do");   // mục đặc biệt: đo hãng từ remote gốc
  for (auto &g : GROUPS) {
    String names = g[1];
    int start = 0;
    while (start <= (int)names.length()) {
      int end = names.indexOf(',', start);
      if (end < 0) end = names.length();
      String nm = names.substring(start, end);
      nm.trim();
      if (nm.length()) themHang(nm.c_str(), nm.c_str());
      start = end + 1;
    }
  }
}

// Vị trí trong danh sách của hãng đang dùng (không thấy thì 0)
int viTriHangHienTai() {
  if (activeIdx < 0) return 0;
  for (int i = 0; i < nBrands; i++) {
    String b = brands[i].arg;
    int sp = b.lastIndexOf(' ');
    if (sp > 0) b = b.substring(0, sp);
    if (b.equals(profiles[activeIdx].name)) return i;
  }
  return 0;
}

// Chọn hãng từ màn hình. Chỉ lưu 2 thiết bị: đầy thì thay hồ sơ đang dùng.
// Hãng có nhiều biến thể protocol: lần đầu dùng biến thể đầu tiên; bấm remote gốc một lần
// thì tự nhận đúng biến thể. Chọn lại đúng hãng đang dùng = thử biến thể kế tiếp.
void chonHang(int i) {
  String base = brands[i].arg;
  if (base.length() > 15) base = base.substring(0, 15);
  String arg = base;
  const char* ds = nhomProto(base);
  if (ds && activeIdx >= 0 && base.equals(profiles[activeIdx].name)) {
    int n = soProto(ds);
    if (n > 1) {
      int cur = viTriProto(ds, (decode_type_t)profiles[activeIdx].proto);
      arg = base + " " + String(((cur < 0 ? 0 : cur) + 1) % n + 1);
    }
  }
  if (findProfile(base) < 0 && nProfiles >= MAX_PROFILES) {
    int victim = (activeIdx >= 0) ? activeIdx : 0;
    xoaProfile(victim);
  }
  datTen(arg);
}

// ---------- Trạng thái giao diện ----------
enum Man { UI_HOME, UI_HANG, UI_DO };
Man man = UI_HOME;
int      hangIdx = 0;
// ----- Chế độ đo hãng từ remote -----
uint32_t doStart = 0;            // thời điểm vào chế độ đo
bool     doCo = false;           // đã nhận được 1 tín hiệu
char     doProto[16] = "";       // tên protocol đo được
char     doHangTxt[2][22] = {"", ""};   // tối đa 2 dòng tên hãng
int      doIdx = -1;             // vị trí hãng trong danh sách để chọn nhanh
uint16_t doBits = 0;
int16_t  doProtoT = -1, doModel = -1;   // protocol/model đo được (để chọn đúng biến thể)
const uint32_t DO_TIMEOUT_MS = 30000;
uint32_t lastInput = 0, lastDraw = 0;

// Thứ tự trùng mã swingv trên Hub: 0 Off, 1 Auto, 2 Low, 3 Mid, 4 High
const char* SW_TEN[5] = {"Off", "Auto", "Low", "Mid", "High"};

// ---------- Vẽ ----------
void veHang() {
  u8g2.setFont(u8g2_font_6x12_tf);
  int tong = nBrands;
  int top = hangIdx - 2;
  if (top > tong - 5) top = tong - 5;
  if (top < 0) top = 0;
  for (int r = 0; r < 5 && top + r < tong; r++) {
    int y = r * 12;
    if (top + r == hangIdx) {
      u8g2.setDrawColor(1);
      u8g2.drawBox(0, y, 128, 12);
      u8g2.setDrawColor(0);
      u8g2.drawStr(2, y + 10, brands[top + r].label);
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(2, y + 10, brands[top + r].label);
    }
  }
}

// Màn hình chính: chỉ hiển thị
void veHome() {
  u8g2.setFont(u8g2_font_6x12_tf);
  if (ac.next.protocol == decode_type_t::UNKNOWN) {
    u8g2.drawStr(0, 24, "Chua chon may lanh");
    u8g2.drawStr(0, 40, "Bam nut de chon hang");
    return;
  }
  // Máy đang TẮT: chỉ hiện chữ OFF to ở giữa màn hình, không hiện gì khác
  if (!ac.next.power) {
    u8g2.setFont(u8g2_font_logisoso28_tr);
    int w = u8g2.getStrWidth("OFF");
    u8g2.drawStr((128 - w) / 2, 46, "OFF");
    return;
  }

  const char* ten = (activeIdx >= 0) ? profiles[activeIdx].name : "(chua luu)";
  u8g2.drawStr(0, 10, ten);
  u8g2.drawStr(ac.next.power ? 110 : 104, 10, ac.next.power ? "ON" : "OFF");
  if (dtg.connected()) u8g2.drawDisc(98, 6, 2);      // chấm đặc = đã nối IoT
  else                 u8g2.drawCircle(98, 6, 2);    // chấm rỗng = chưa nối

  char b[24];

  u8g2.setFont(u8g2_font_logisoso28_tn);
  int nguyen = (int)ac.next.degrees;
  bool nua = (ac.next.degrees - nguyen) >= 0.25f;   // có phần lẻ 0,5
  snprintf(b, sizeof(b), "%d", nguyen);
  u8g2.drawStr(0, 50, b);
  int w = u8g2.getStrWidth(b);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(w + 3, 26, nua ? ".5C" : "C");

  u8g2.drawStr(66, 28, tenMode(ac.next.mode));
  snprintf(b, sizeof(b), "Quat %s", tenFan(ac.next.fanspeed));
  u8g2.drawStr(66, 40, b);
  int sc = maSwingV(ac.next.swingv);
  snprintf(b, sizeof(b), "Swing %s", SW_TEN[sc < 0 ? 0 : sc]);
  u8g2.drawStr(66, 52, b);

  String fl = "";
  if (ac.next.turbo) fl += "Turbo ";
  if (ac.next.sleep >= 0) fl += "Sleep";
  u8g2.drawStr(0, 63, fl.c_str());
  if (henTat.on)      snprintf(b, sizeof(b), "Tat %dp", phutConLai(henTat));
  else if (henBat.on) snprintf(b, sizeof(b), "Bat %dp", phutConLai(henBat));
  else                b[0] = 0;
  u8g2.drawStr(76, 63, b);
}

void veDo() {
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, "DO HANG REMOTE");
  char b[28];
  int giay = (int)((DO_TIMEOUT_MS - (millis() - doStart)) / 1000);
  if (giay < 0) giay = 0;
  snprintf(b, sizeof(b), "%ds", giay);
  u8g2.drawStr(110, 10, b);
  if (!doCo) {
    u8g2.drawStr(0, 28, "Huong remote vao mat");
    u8g2.drawStr(0, 40, "thu IR (GPIO4) va");
    u8g2.drawStr(0, 52, "bam 1 nut bat ky");
    u8g2.drawStr(0, 63, "Giu nut: thoat");
    return;
  }
  snprintf(b, sizeof(b), "Proto: %s", doProto);
  u8g2.drawStr(0, 24, b);
  if (doHangTxt[0][0]) {
    u8g2.drawStr(0, 36, doHangTxt[0]);
    if (doHangTxt[1][0]) u8g2.drawStr(0, 48, doHangTxt[1]);
    u8g2.drawStr(0, 63, doIdx >= 0 ? "Bam: chon | Giu: thoat" : "Giu nut: thoat");
  } else {
    u8g2.drawStr(0, 36, "Khong co trong ds");
    u8g2.drawStr(0, 63, "Giu nut: thoat");
  }
}

void veMan() {
  u8g2.clearBuffer();
  if (man == UI_HANG)    veHang();
  else if (man == UI_DO) veDo();
  else                   veHome();
  u8g2.sendBuffer();
}

void uiSetup() {
  buildBrandList();
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  encState = ((digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B)) & 0x03;
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encISR, CHANGE);
  delay(250);   // đợi OLED lên nguồn ổn định khi vừa cắm điện
  u8g2.begin();
  u8g2.setBusClock(400000);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "Dieu hoa IR");
  u8g2.drawStr(0, 28, "Dang khoi dong...");
  u8g2.sendBuffer();
  lastInput = millis();
}

uint32_t lastSig = 0;

// "Chữ ký" nội dung màn hình: đổi thì mới cần vẽ lại
uint32_t chuKy() {
  uint32_t h = (uint32_t)man;
  h = h * 31 + (ac.next.power ? 1 : 0);
  h = h * 31 + (uint32_t)(ac.next.degrees * 2);
  h = h * 31 + (uint32_t)ac.next.mode;
  h = h * 31 + (uint32_t)ac.next.fanspeed;
  h = h * 31 + (uint32_t)ac.next.swingv;
  h = h * 31 + (ac.next.turbo ? 1 : 0);
  h = h * 31 + (ac.next.sleep >= 0 ? 1 : 0);
  h = h * 31 + (uint32_t)phutConLai(henTat);
  h = h * 31 + (uint32_t)phutConLai(henBat);
  h = h * 31 + (dtg.connected() ? 1 : 0);
  h = h * 31 + (uint32_t)(activeIdx + 1);
  h = h * 31 + (uint32_t)ac.next.protocol;
  h = h * 31 + (uint32_t)hangIdx;
  if (man == UI_DO) {
    h = h * 31 + (uint32_t)((millis() - doStart) / 1000);
    h = h * 31 + (doCo ? 1 : 0) + (uint32_t)doBits * 7 + (uint32_t)(doIdx + 2);
  }
  return h;
}

void uiLoop() {
  uint32_t now = millis();
  int16_t d = layDelta();
  uint8_t nut = docNut();
  if (d != 0 || nut != NUT_KHONG) lastInput = now;

  // Lâu không thao tác thì về màn hình chính
  if (man == UI_HANG && now - lastInput > 20000) man = UI_HOME;
  if (man == UI_DO && (now - doStart > DO_TIMEOUT_MS)) man = UI_HOME;

  if (man == UI_HOME) {
    // Xoay hoặc bấm: mở danh sách hãng
    if ((d != 0 || nut == NUT_NGAN) && nBrands > 0) {
      man = UI_HANG;
      hangIdx = viTriHangHienTai();
      if (d != 0) hangIdx = ((hangIdx + d) % nBrands + nBrands) % nBrands;
    }
  } else if (man == UI_DO) {
    if (nut == NUT_DAI) man = UI_HOME;
    else if (nut == NUT_NGAN && doCo && doIdx >= 0) { chonHang(doIdx);
      if (activeIdx >= 0 && doProtoT > 0) {   // ép đúng biến thể protocol/model đo được, không dùng biến thể mặc định của hãng
        profiles[activeIdx].proto = doProtoT;
        profiles[activeIdx].model = doModel;
        ac.next.protocol = (decode_type_t)doProtoT;
        ac.next.model    = doModel;
        luuTrangThai();
        dayLenIoT();
      }
      man = UI_HOME; }
    else if (d != 0 || nut == NUT_NGAN) doStart = now;   // thao tác thì gia hạn thời gian chờ
  } else {   // UI_HANG
    if (d != 0 && nBrands > 0) hangIdx = ((hangIdx + d) % nBrands + nBrands) % nBrands;
    if (nut == NUT_DAI) {
      man = UI_HOME;
    } else if (nut == NUT_NGAN && nBrands > 0) {
      if (brands[hangIdx].arg[0] == '#') {   // "[Do tu remote]"
        man = UI_DO; doStart = now; doCo = false; doProto[0] = 0;
        doHangTxt[0][0] = doHangTxt[1][0] = 0; doIdx = -1; doBits = 0; doProtoT = -1; doModel = -1;
      } else {
        chonHang(hangIdx);
        man = UI_HOME;
      }
    }
  }

  // Chỉ vẽ lại khi có thay đổi (hoặc mỗi 2 giây để tự phục hồi nếu OLED bị nhiễu): ít ghi I2C hơn
  uint32_t ky = chuKy();
  if (d != 0 || nut != NUT_KHONG || ky != lastSig || now - lastDraw > 2000) {
    lastDraw = now;
    lastSig = ky;
    veMan();
  }
}

// ============================================================
// ===== Kết nối Wi-Fi TRƯỚC, xong mới làm việc khác ==========
// ============================================================
const wifi_power_t WIFI_TX_POWER = WIFI_POWER_8_5dBm;   // Super Mini hay lỗi ở công suất mặc định; 11 hoặc 15 dBm nếu sóng yếu
const uint32_t WIFI_CHO_TOI_DA_MS = 40000;              // chờ tối đa 40 giây, không được thì chạy offline và tự thử lại
volatile int wifiLyDo = 0;                              // mã lý do ngắt Wi-Fi gần nhất

// Màn hình thông báo 4 dòng
void veThongBao(const char* a, const char* b, const char* c, const char* d) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, a);
  u8g2.drawStr(0, 26, b);
  u8g2.drawStr(0, 40, c);
  u8g2.drawStr(0, 54, d);
  u8g2.sendBuffer();
}

void ketNoiWifiTruoc() {
  WiFi.onEvent([](arduino_event_id_t ev, arduino_event_info_t info) {
    if (ev == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) wifiLyDo = info.wifi_sta_disconnected.reason;
    else if (ev == ARDUINO_EVENT_WIFI_STA_GOT_IP)  wifiLyDo = 0;
  });

  dtg.begin(WIFI_SSID, WIFI_PASS);
  dtg.enableHubCmd();                 // bật nhận lệnh xuống (DTG_LISTEN)
  WiFi.setSleep(false);               // thư viện bật modem sleep, tắt đi cho ổn định
  WiFi.setTxPower(WIFI_TX_POWER);

  uint32_t t0 = millis(), tThu = t0;
  int lan = 1;
  char c[32], d[32];
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CHO_TOI_DA_MS) {
    dtg.run();
    if (millis() - tThu > 12000) {    // 12 giây chưa vào được thì thử lại từ đầu
      tThu = millis();
      lan++;
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    snprintf(c, sizeof(c), "Lan thu %d  (%us)", lan, (unsigned)((millis() - t0) / 1000));
    if (wifiLyDo) snprintf(d, sizeof(d), "Loi: %d", (int)wifiLyDo);
    else          d[0] = 0;
    veThongBao("Dang noi WiFi...", WIFI_SSID, c, d);
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    snprintf(c, sizeof(c), "Tin hieu %d dBm", WiFi.RSSI());
    veThongBao("WiFi da ket noi", WiFi.localIP().toString().c_str(), c, "");
    delay(1200);
  } else {
    snprintf(d, sizeof(d), "Loi: %d", (int)wifiLyDo);
    veThongBao("Khong co WiFi", "Chay offline,", "se tu thu lai", d);
    delay(2000);
  }
}

// Tra bảng GROUPS: protocol đo được thuộc những hãng nào (tối đa 2 dòng hiển thị)
void hangTuRemote(decode_type_t t) {
  doHangTxt[0][0] = doHangTxt[1][0] = 0;
  doIdx = -1;
  String dong[2] = {"", ""};
  for (auto &g : GROUPS) {
    if (viTriProto(g[0], t) < 0) continue;
    String first = g[1];
    int c = first.indexOf(',');
    if (c > 0) first = first.substring(0, c);
    if (doIdx < 0) {
      for (int i = 0; i < nBrands; i++) if (first.equals(brands[i].arg)) { doIdx = i; break; }
    }
    // Gom tên vào 2 dòng, mỗi dòng tối đa 20 ký tự
    for (int k = 0; k < 2; k++) {
      if (dong[k].length() + first.length() + (dong[k].length() ? 1 : 0) <= 20) {
        if (dong[k].length()) dong[k] += ",";
        dong[k] += first;
        break;
      }
    }
  }
  strlcpy(doHangTxt[0], dong[0].c_str(), sizeof(doHangTxt[0]));
  strlcpy(doHangTxt[1], dong[1].c_str(), sizeof(doHangTxt[1]));
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_LED_XANH, OUTPUT);
  pinMode(PIN_LED_DO, OUTPUT);
  digitalWrite(PIN_LED_XANH, LOW);
  digitalWrite(PIN_LED_DO, LOW);

  uiSetup();            // OLED + núm xoay
  ketNoiWifiTruoc();    // Wi-Fi xong rồi mới làm các việc còn lại

  setDefaults();
  loadProfiles();
  if (activeIdx >= 0) useProfile(activeIdx);

  irrecv.setUnknownThreshold(12);
  irrecv.setTolerance(35);   // mặc định 25%, nới ra để chịu được xung lệch do Wi-Fi
  irrecv.enableIRIn();
}

void loop() {
  dtg.run();   // gọi mỗi vòng loop để giữ kết nối Wi-Fi + MQTT

  // Vừa kết nối được IoT thì đẩy trạng thái hiện tại lên một lần
  static bool daKetNoi = false;
  bool ketNoi = dtg.connected();
  if (ketNoi && !daKetNoi) dayLenIoT();
  daKetNoi = ketNoi;

  // Lệnh từ Hub và hẹn giờ
  xuLyHubCmd();
  kiemTraHen();

  // Cập nhật thời gian hẹn giờ còn lại lên Hub mỗi khi đổi phút (mỗi 1 phút)
  {
    static int lastOff = -1, lastOn = -1;
    int po = phutConLai(henTat), pn = phutConLai(henBat);
    if (ketNoi && ac.next.protocol != decode_type_t::UNKNOWN) {
      if (po != lastOff) dtg.publish("timer_off", po);
      if (pn != lastOn)  dtg.publish("timer_on",  pn);
    }
    if (ketNoi || (po == 0 && pn == 0)) { lastOff = po; lastOn = pn; }
  }

  uiLoop();   // núm xoay + màn hình OLED

  // Nhận tín hiệu từ remote gốc và đồng bộ trạng thái
  // max_skip = 1: bỏ qua tối đa 1 cặp xung đầu. Remote Daikin này gửi 6 bit mở đầu thay vì 5 nên phải bỏ 1 cặp mới giải mã được
  if (irrecv.decode(&results, NULL, 1)) {
    digitalWrite(PIN_LED_DO, HIGH);
    Serial.println("---- Nhan duoc ----");
    Serial.print(resultToHumanReadableBasic(&results));
    String acStr = IRAcUtils::resultAcToString(&results);
    if (acStr.length()) Serial.println("AC: " + acStr);
    // Không nhận dạng được thì in toàn bộ xung thô để phân tích
    if (results.decode_type == decode_type_t::UNKNOWN) Serial.print(resultToSourceCode(&results));

    stdAc::state_t st;
    bool giaiMa = IRAcUtils::decodeToState(&results, &st);

    if (man == UI_DO) {
      // Chế độ đo hãng: chỉ nhận dạng, KHÔNG đồng bộ trạng thái
      if (results.decode_type != decode_type_t::UNKNOWN) {
        strlcpy(doProto, typeToString(results.decode_type).c_str(), sizeof(doProto));
        doBits = results.bits;
        doProtoT = (int16_t)results.decode_type;
        doModel = giaiMa ? (int16_t)st.model : -1;
        hangTuRemote(results.decode_type);
        Serial.printf("(do hang) protocol=%s, %u bit\n", doProto, (unsigned)results.bits);
      } else {
        strlcpy(doProto, "UNKNOWN", sizeof(doProto));
        doProtoT = -1;
        doBits = results.bits;
        doHangTxt[0][0] = doHangTxt[1][0] = 0;
        doIdx = -1;
      }
      doCo = true;
      doStart = millis();   // nhận được thì gia hạn thời gian chờ
    } else if (activeIdx >= 0 && giaiMa) {
      const char* ds = nhomProto(String(profiles[activeIdx].name));
      bool cungHang = (st.protocol == ac.next.protocol) ||
                      (ds != nullptr && viTriProto(ds, st.protocol) >= 0);
      if (cungHang) {
        if (st.protocol != ac.next.protocol) {   // remote cho biết đúng biến thể: tự chuyển
          profiles[activeIdx].proto = (int16_t)st.protocol;
          profiles[activeIdx].model = (int16_t)st.model;
          Serial.println("(tu nhan dang bien the protocol cua hang)");
        }
        ac.next = st;
        huyHenThua();
        luuTrangThai();
        dayLenIoT();
        Serial.println("(da dong bo trang thai tu remote goc)");
      }
    }
    digitalWrite(PIN_LED_DO, LOW);
    irrecv.resume();
  }
}