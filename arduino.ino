#include <zb_znp.h>
#include <zb_zcl.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
// #include "DHT.h"            
#include <AM2302-Sensor.h>


#define runEvery(t) for (static uint16_t _lasttime; \
                         (uint16_t)((uint16_t)millis() - _lasttime) >= (t); \
                         _lasttime += (t))
#define TIME_OUT_FOR_KEY_PRESSED 200
#define TIMER_CYCLE 100
#define WAITING_TIME 1000

#define PRESSED_STATE 0
#define RELEASED_STATE 1

#define BUTTON_IS_PRESSED 11
#define BUTTON_IS_RELEASED 12
#define BUTTON_IS_LONG_PRESSED 13

#define DHTTYPE DHT11   

const int DHTPIN = 4;
const int ledPin1 = 5;
const int ledPin2 = 6;  
const int buzzerPin = 7; 
const int buttonPin = 8;

static bool buzzerState = 0;
static bool ledState = 0;
static bool alarmState = 0;

bool door = 0;
int thief = 2;
float h = 0;
float temph=0;
float t = 0;
float tempt=0;

static int buttonDebounce[4]; // variable for reading the pushbutton status
static int buttonState = BUTTON_IS_RELEASED;
static int buttonFlag;
static int counter_for_button_pressed;
static String s1,s2="";

AM2302::AM2302_Sensor am2302{DHTPIN};

// DHT dht(DHTPIN, DHTTYPE);

// SoftwareSerial espSerial(4,5);
SoftwareSerial znp_serial(2, 3);
zb_znp zigbee_network(&znp_serial);

/* Biến xử lý điều khiển switch */
uint8_t control_switch_cmd_seq = 0;
uint16_t control_switch_address = 0;



int zb_znp::zigbee_message_handler(zigbee_msg_t &zigbee_msg) {
  /* zigbee start debug message */
  afIncomingMSGPacket_t *st_af_incoming_msg = (afIncomingMSGPacket_t *)zigbee_msg.data;


  // if (s2=="")
  // {
  //   if(zigbee_msg.data[20] == 1) {door=1;}
  //   else if(zigbee_msg.data[20] == 0) {door=0;}

  //   Serial.print("!D");
  //   Serial.print(zigbee_msg.data[20], HEX);
  //   s1=zigbee_msg.data[20];
  //   s1 = "!D" + s1 + "#";
  //   Serial.print("#");
  //   Serial.println("");
  // }
  // while(1)
  // {
  //   s2=zigbee_msg.data[20];
  //   s2 = "!D" + s2 + "#";
  //   if (s1==s2) {continue;}
  //   else {
  //     s1=s2;
  //     Serial.println(s1);
  //     break;}
  // }
  if(zigbee_msg.data[20] == 1) {door=1;}
  else if(zigbee_msg.data[20] == 0) {door=0;}

 
  Serial.print("!D");
  Serial.print(zigbee_msg.data[20], HEX);
  Serial.print("#");
  Serial.println("");


  // for (int i = 0; i < zigbee_msg.len; i++)
  // {
  //     Serial.print(zigbee_msg.data[i], HEX);
  //     Serial.print(" ");
  // }

  //Serial.print(zigbee_msg.data[21], HEX);
  //Serial.print(zigbee_msg.data[22], HEX);
 
  /* zigbee stop debug message */

  uint16_t zigbee_cmd = BUILD_UINT16(zigbee_msg.cmd1, zigbee_msg.cmd0);

  switch (zigbee_cmd) {
    case ZDO_MGMT_LEAVE_REQ:
      {
        Serial.println("ZDO_MGMT_LEAVE_REQ");
        break;
      }

    case ZB_RECEIVE_DATA_INDICATION:
      {
        Serial.println("ZB_RECEIVE_DATA_INDICATION");
        break;
      }

    case ZDO_MGMT_PERMIT_JOIN_RSP:
      {
        Serial.println("ZDO_MGMT_PERMIT_JOIN_RSP");
        break;
      }

    case ZDO_TC_DEV_IND:
      {
        Serial.println("ZDO_TC_DEV_IND");
        break;
      }

    case AF_DATA_REQUEST_IND:
      {
        Serial.println("AF_DATA_REQUEST_IND");
        break;
      }

    case AF_DATA_CONFIRM:
      {
        Serial.println("AF_DATA_CONFIRM");
        afDataConfirm_t *afDataConfirm = (afDataConfirm_t *)zigbee_msg.data;
        Serial.print("\tstatus: ");
        Serial.println(afDataConfirm->status);
        Serial.print("\tendpoint: ");
        Serial.println(afDataConfirm->endpoint);
        Serial.print("\ttransID: ");
        Serial.println(afDataConfirm->transID);
        break;
      }

    case AF_INCOMING_MSG:
      {
        afIncomingMSGPacket_t *st_af_incoming_msg = (afIncomingMSGPacket_t *)zigbee_msg.data;
        // Serial.println("AF_INCOMING_MSG");

#if defined(DBG_ZB_FRAME)
        Serial.print("group_id: ");
        Serial.println(st_af_incoming_msg->group_id, HEX);
        Serial.print("cluster_id: ");
        Serial.println(st_af_incoming_msg->cluster_id, HEX);
        Serial.print("src_addr: ");
        Serial.println(st_af_incoming_msg->src_addr, HEX);
        Serial.print("src_endpoint: ");
        Serial.println(st_af_incoming_msg->src_endpoint, HEX);
        Serial.print("dst_endpoint: ");
        Serial.println(st_af_incoming_msg->dst_endpoint, HEX);
        Serial.print("was_broadcast: ");
        Serial.println(st_af_incoming_msg->was_broadcast, HEX);
        Serial.print("link_quality: ");
        Serial.println(st_af_incoming_msg->link_quality, HEX);
        Serial.print("security_use: ");
        Serial.println(st_af_incoming_msg->security_use, HEX);
        Serial.print("time_stamp: ");
        Serial.println(st_af_incoming_msg->time_stamp, HEX);
        Serial.print("trans_seq_num: ");
        Serial.println(st_af_incoming_msg->trans_seq_num, HEX);
        Serial.print("len: ");
        Serial.println(st_af_incoming_msg->len);
        Serial.print("data: ");
        for (int i = 0; i < st_af_incoming_msg->len; i++) {
          Serial.print(st_af_incoming_msg->payload[i], HEX);
          Serial.print(" ");
        }
        Serial.println("");
#endif

        switch (st_af_incoming_msg->cluster_id) {
          case ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY:
            {
              Serial.println("ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY");
              uint16_t retHum = (uint16_t)((st_af_incoming_msg->payload[st_af_incoming_msg->len - 1] * 256) + st_af_incoming_msg->payload[st_af_incoming_msg->len - 2]);

              // Ví dụ: retHum = 6789, thì giá trị trả về là 67,89 %
              Serial.print(retHum / 100);  // Lấy Trước dấu phẩy -> 67
              Serial.print(",");
              Serial.println(retHum % 100);  // Lấy sau dấu phẩy -> 89
              break;
            }

          case ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT:
            {
              Serial.println("ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT");
              uint16_t retTemp = (uint16_t)((st_af_incoming_msg->payload[st_af_incoming_msg->len - 1] * 256) + st_af_incoming_msg->payload[st_af_incoming_msg->len - 2]);

              // Ví dụ: retTemp = 2723, thì giá trị trả về là 27,23 *C
              Serial.print(retTemp / 100);  // Lấy Trước dấu phẩy -> 27
              Serial.print(",");
              Serial.println(retTemp % 100);  // Lấy sau dấu phẩy -> 23
              break;
            }

          case ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING:
            {
              Serial.println("ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING");
              uint8_t retOccu = st_af_incoming_msg->payload[st_af_incoming_msg->len - 1];
              Serial.println(retOccu);
              break;
            }

          case ZCL_CLUSTER_ID_GEN_ON_OFF:
            Serial.println("ZCL_CLUSTER_ID_GEN_ON_OFF");
            uint8_t retGenOnOff;
            if (st_af_incoming_msg->len > 9) {
              control_switch_address = st_af_incoming_msg->src_addr;
              retGenOnOff = st_af_incoming_msg->payload[st_af_incoming_msg->len - 8];
              Serial.println(retGenOnOff);
            } else {
              retGenOnOff = st_af_incoming_msg->payload[st_af_incoming_msg->len - 1];
              Serial.println(retGenOnOff);
            }
            break;

          default:
            break;
        }
        break;
      }

    case ZDO_MGMT_LEAVE_RSP:
      {
        Serial.println("ZDO_MGMT_LEAVE_RSP");
        break;
      }

    case ZDO_END_DEVICE_ANNCE_IND:
      {
        Serial.println("ZDO_END_DEVICE_ANNCE_IND");
        ZDO_DeviceAnnce_t *ZDO_DeviceAnnce = (ZDO_DeviceAnnce_t *)zigbee_msg.data;
        Serial.print("\tSrcAddr: ");
        Serial.println(ZDO_DeviceAnnce->SrcAddr, HEX);
        Serial.print("\tnwkAddr: ");
        Serial.println(ZDO_DeviceAnnce->nwkAddr, HEX);
        Serial.print("\textAddr: ");
        for (int i = 0; i < Z_EXTADDR_LEN; i++) {
          Serial.print(ZDO_DeviceAnnce->extAddr[i], HEX);
        }
        Serial.print("\n");
        /***
         * Specifies the MAC capabilities of the device.
         * Bit: 0 – Alternate PAN Coordinator
         * 1 – Device type: 1- ZigBee Router; 0 – End Device
         * 2 – Power Source: 1 Main powered
         * 3 – Receiver on when idle
         * 4 – Reserved
         * 5 – Reserved
         * 6 – Security capability
         * 7 – Reserved
         */
        Serial.print("\tcapabilities: ");
        Serial.println(ZDO_DeviceAnnce->capabilities);
        break;
      }
  }
}

void setLed(int state)
{
  if (state==0)
  {
    digitalWrite(ledPin1, 0);
    digitalWrite(ledPin2, 0);
    ledState=0;
  }
  else if (state==1)
  {
    digitalWrite(ledPin1, 1);
    digitalWrite(ledPin2, 0);
    ledState=1;
  }
}

void setBuzzer(int state)
{
  if (state==1)
  {
    digitalWrite(buzzerPin, 1);
    buzzerState = 1;
  }
  else if(state ==0)
  {
    digitalWrite(buzzerPin, 0);
    buzzerState = 0;
  }
}

void alarm() {
    if (buzzerState == 1) {setBuzzer(0);}
    else if (buzzerState == 0) {setBuzzer(1);}

    if (ledState == 0) {setLed(1);}
    else if (ledState == 1) {setLed(0);}
}

void stopAlarm() {
  setBuzzer(0);
  setLed(0);
  Serial.print("!L");
  Serial.print(ledState);
  Serial.print("#");
  Serial.println("");
  alarmState=0;
  
  return;
}

int isButtonPressed()
{
    if (buttonFlag == 1)
    {
        buttonFlag = 0;
        return 1;
    }
    return 0;
}

void button_process()
{
    buttonFlag = 1;
}

void button_reading()
{
    buttonDebounce[3] = buttonDebounce[2];
    buttonDebounce[2] = buttonDebounce[1];

    buttonDebounce[1] = digitalRead(buttonPin);

    // process after debouncing
    if ((buttonDebounce[3] == buttonDebounce[2]) && buttonDebounce[2] == buttonDebounce[1])
    {
        buttonDebounce[0] = buttonDebounce[3];
        // fsm for processing button
        switch (buttonState)
        {
        case BUTTON_IS_PRESSED:
            // waiting for a period if the button is pressed in a duration
            // counter_for_button_pressed++;
            // if (counter_for_button_pressed == (WAITING_TIME / TIMER_CYCLE))
            // {
            //     buttonState = BUTTON_IS_LONG_PRESSED;
            //     counter_for_button_pressed = 0;
            //     button_process();
            // }
            // button_process();
            if (buttonDebounce[0] == RELEASED_STATE)
            {
                buttonState = BUTTON_IS_RELEASED;
                // counter_for_button_pressed = 0;
            }
            break;
        case BUTTON_IS_RELEASED:
            if (buttonDebounce[0] == PRESSED_STATE)
            {
                buttonState = BUTTON_IS_PRESSED;
                button_process();
            }
            break;
        // case BUTTON_IS_LONG_PRESSED:
        //     // if the button continues being pressed in duration, the button only triggered in a period defined previous.
        //     counter_for_button_pressed++;
        //     if (counter_for_button_pressed == (TIME_OUT_FOR_KEY_PRESSED / TIMER_CYCLE))
        //     {
        //         button_process();
        //         counter_for_button_pressed = 0;
        //     }
        //     if (buttonDebounce[0] == RELEASED_STATE)
        //     {
        //         buttonDebounce[0] = BUTTON_IS_RELEASED;
        //         counter_for_button_pressed = 0;
        //     }
        //     break;
        default:
            break;
        }
    }
}

void setup() {
  Serial.begin(9600);
  znp_serial.begin(115200);
  znp_serial.setTimeout(100);
  // espSerial.begin(9600);
  // dht.begin();       
  am2302.begin();

  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(buttonPin, INPUT);

  // pinMode(10, INPUT);
  // pinMode(11, OUTPUT);

  /* Khởi động coodinatior */
  Serial.println("\nstart_coordinator(1)");
  if (zigbee_network.start_coordinator(1) == 0) {
    Serial.println("OK");
  } 
  else {
    Serial.println("NG");
  }

  /* Cho phép thiết bị tham gia vào mạng */
  Serial.println("set_permit_joining_req");
          /* ALL_ROUTER_AND_COORDINATOR -> cho phép thiết bị tham gia mạng từ Coodinator (ZigBee Shield)
             * hoặc qua router (router thường là các thiết bị đc cấp điện, như ổ cắm, công tắc, bóng đèn ...
             * 60, sau 60s nếu không có thiết bị tham gia mạng, coodinator sẽ trở về mode hoạt động bình thường
             * người dùng muốn thêm thiết bị mới phải yêu cấu thêm lần nữa
             * 1 , đợi thiết bị join thành công, mới thoát khỏi hàm, nếu 0, sự kiện có thiết bị mới tham gia mạng
             * sẽ được nhận ở hàm callback int zb_znp::zigbee_message_handler(zigbee_msg_t& zigbee_msg)
             */
  zigbee_network.set_permit_joining_req(ALL_ROUTER_AND_COORDINATOR, 300, 1);
}

/* ký tự tạm để xử lý yêu cầu từ terminal */
char serial_cmd;

void loop() {
  /* hàm update() phải được gọi trong vòng lặp để xử lý các gói tin nhận được từ ZigBee Shield */
  zigbee_network.update();

  /* Cấu hình lại coodinator, đưa các cáu hình về mặc định.
             * Chú ý: list thiết bị đã tham gia vào mạng trước đó sẽ bị mất */

  /* Kiểm tra / thực hiện các lệnh từ terminal */
  // if (Serial.available()) {
  //   serial_cmd = Serial.read();

  //   switch (serial_cmd) {
  //       /* Cấu hình lại coodinator, đưa các cáu hình về mặc định.
  //            * Chú ý: list thiết bị đã tham gia vào mạng trước đó sẽ bị mất */
  //     case '0':
  //       {
  //         Serial.println("\nstart_coordinator(1)");
  //         if (zigbee_network.start_coordinator(1) == 0) {
  //           Serial.println("OK");
  //         } else {
  //           Serial.println("NG");
  //         }
  //         break;
  //       }

  //       /* Cho phép thiết bị tham gia vào mạng */
  //     case '1':
  //       {
  //         Serial.println("set_permit_joining_req");
  //         /* ALL_ROUTER_AND_COORDINATOR -> cho phép thiết bị tham gia mạng từ Coodinator (ZigBee Shield)
  //            * hoặc qua router (router thường là các thiết bị đc cấp điện, như ổ cắm, công tắc, bóng đèn ...
  //            * 60, sau 60s nếu không có thiết bị tham gia mạng, coodinator sẽ trở về mode hoạt động bình thường
  //            * người dùng muốn thêm thiết bị mới phải yêu cấu thêm lần nữa
  //            * 1 , đợi thiết bị join thành công, mới thoát khỏi hàm, nếu 0, sự kiện có thiết bị mới tham gia mạng
  //            * sẽ được nhận ở hàm callback int zb_znp::zigbee_message_handler(zigbee_msg_t& zigbee_msg)
  //            */
  //         zigbee_network.set_permit_joining_req(ALL_ROUTER_AND_COORDINATOR, 60, 1);
  //         break;
  //       }

  //       /* yêu cầu Toggle công tắc */
  //     case '3':
  //       {
  //         Serial.println("TOOGLE Switch Req !\n");
  //         /*
  //            * Frame Control, Transaction Sequence Number, Value control
  //            * Value control -> 0x00: off, 0x01: on, 0x02: toogle
  //            */
  //         if (control_switch_address) {
  //           uint8_t st_buffer[3] = { /* Frame control */ 0x01,
  //                                    /* Transaction Sequence Number */ 0x00, /* control_switch_cmd_seq++ */
  //                                    /* Value Control */ 0x02 };             /* Value Control [ 0x00:OFF , 0x01:ON , 0x02:TOOGLE ] */
  //           st_buffer[1] = control_switch_cmd_seq++;

  //           af_data_request_t st_af_data_request;
  //           st_af_data_request.cluster_id = ZCL_CLUSTER_ID_PI_GENERIC_TUNNEL;
  //           st_af_data_request.dst_address = control_switch_address;
  //           st_af_data_request.dst_endpoint = 0x01;
  //           st_af_data_request.src_endpoint = 0x01;
  //           st_af_data_request.trans_id = 0x00;
  //           st_af_data_request.options = 0x10;
  //           st_af_data_request.radius = 0x0F;
  //           st_af_data_request.len = sizeof(st_buffer);
  //           st_af_data_request.data = st_buffer;

  //           zigbee_network.send_af_data_req(st_af_data_request);
  //         } else {
  //           Serial.println("Please join Switch !\n");
  //         }
  //         break;
  //       }

  //       /******************************************************************
  //            *  Ví dụ:
  //            * gửi data từ Gateway(coodinator) đến các thiết bị / cảm biến
  //            * các thông số cần thiết cho quá trình này bao gồm
  //            * 1. short address, là địa chỉ đc coodinator cấp khi thiết bị / cảm biến join vào mạng
  //            * 2. độ dài của mảng data cần truyền
  //            * 3. data

  //         case 's': {
  //           uint8_t st_buffer[10];
  //           af_data_request_t st_af_data_request;
  //           st_af_data_request.cluster_id    = ZCL_CLUSTER_ID_PI_GENERIC_TUNNEL;
  //           st_af_data_request.dst_address   = [ Địa chỉ đích của thiết bị / sensor ] ví du: control_switch_address
  //           st_af_data_request.dst_endpoint  = 0x01;
  //           st_af_data_request.src_endpoint  = 0x01;
  //           st_af_data_request.trans_id      = 0x00;
  //           st_af_data_request.options       = 0x10;
  //           st_af_data_request.radius        = 0x0F;
  //           st_af_data_request.len           = [ Độ dài data cần gửi đi ] ví dụ: sizeof(st_buffer)
  //           st_af_data_request.data          = [ data ] ví dụ: st_buffer
  //           zigbee_network.send_af_data_req(st_af_data_request);
  //         }
  //           break;
  //           ********************************************************************/

  //     default:
  //       break;
  //   }
  //   espSerial.write(serial_cmd);
  // }

  if (Serial.available() > 0) {
    // Serial.println(1);
    String string_data = "";
    char data = Serial.read();
    if (data == '!') {
      // Serial.println(2);
      string_data += data;
      while (data != '#') {
        // Serial.println(3);
        if (Serial.available() > 0) {
          data = Serial.read();
          string_data += data;
        }
      }
    }

    char feed_data = string_data[1]; 
    string_data = string_data.substring(2, string_data.length() - 1);
    if (feed_data == 'L')
    {
      if(string_data == "0") {setLed(0);}
      else if(string_data=="1") {setLed(1);}
    }
    else if (feed_data == 'F')
    {
      if (string_data=="0") {thief = 0;}
      else if (string_data=="1") {thief=1;}
    }
  }
  
  if (thief == 1) {
    if (door == 1) {
      alarmState=1;
      thief=2;
    }
  } else if (thief == 0) {
    stopAlarm();
    thief = 2;
  }

  runEvery(10) 
  {
    button_reading();
    if (isButtonPressed() == 1) 
    {
      setLed(1-ledState);
      Serial.print("!L");
      Serial.print(ledState);
      Serial.print("#");
      Serial.println("");
    }

  }

  runEvery(500) 
  {
    if(alarmState==1)
    {
      alarm();
    }
  }

  runEvery(30000) 
  {
    // temph=h; tempt=t;
    // h = dht.readHumidity();    
    // t = dht.readTemperature();
    // float f = dht.readTemperature(true);
    am2302.read();

    Serial.print("!T");
    Serial.print(am2302.get_Temperature());
    Serial.print("#");
    Serial.println("");

    Serial.print("!H");
    Serial.print(am2302.get_Hunidity());
    Serial.print("#");
    Serial.println("");
  }

  
}
