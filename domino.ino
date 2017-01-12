SYSTEM_MODE(SEMI_AUTOMATIC);



//Commun parameters
#define MIN_CONN_INTERVAL          0x0028 // 50ms.
#define MAX_CONN_INTERVAL          0x0190 // 500ms.
#define SLAVE_LATENCY              0x0000 // No slave latency.
#define CONN_SUPERVISION_TIMEOUT   0x03E8 // 10s.
#define BLE_PERIPHERAL_APPEARANCE  BLE_APPEARANCE_UNKNOWN

#define BLE_DEVICE_NAME            "Domino"

//Commun parameters
#define CHARACTERISTIC1_MAX_LEN    15


//Commun parameters
static uint8_t  appearance[2] = {
  LOW_BYTE(BLE_PERIPHERAL_APPEARANCE),
  HIGH_BYTE(BLE_PERIPHERAL_APPEARANCE)
};
static uint8_t  change[4] = {
  0x00, 0x00, 0xFF, 0xFF
};
static uint8_t  conn_param[8] = {
  LOW_BYTE(MIN_CONN_INTERVAL), HIGH_BYTE(MIN_CONN_INTERVAL),
  LOW_BYTE(MAX_CONN_INTERVAL), HIGH_BYTE(MAX_CONN_INTERVAL),
  LOW_BYTE(SLAVE_LATENCY), HIGH_BYTE(SLAVE_LATENCY),
  LOW_BYTE(CONN_SUPERVISION_TIMEOUT), HIGH_BYTE(CONN_SUPERVISION_TIMEOUT)
};
static advParams_t adv_params = {
  .adv_int_min   = 0x0030,
  .adv_int_max   = 0x0030,
  .adv_type      = BLE_GAP_ADV_TYPE_ADV_IND,
  .dir_addr_type = BLE_GAP_ADDR_TYPE_PUBLIC,
  .dir_addr      = {0,0,0,0,0,0},
  .channel_map   = BLE_GAP_ADV_CHANNEL_MAP_ALL,
  .filter_policy = BLE_GAP_ADV_FP_ANY
};
static uint8_t adv_data[] = {
  0x02,
  BLE_GAP_AD_TYPE_FLAGS,
  BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,

  0x11,
  BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE,
  0x1e,0x94,0x8d,0xf1,0x48,0x31,0x94,0xba,0x75,0x4c,0x3e,0x50,0x00,0x00,0x3d,0x71
};



//Service + characteristique definitions
static uint8_t service1_uuid[16]    = { 0x71,0x3d,0x00,0x00,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t char1_uuid[16] = { 0x71,0x3d,0x00,0x02,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };

static uint16_t character1_handle = 0x0000;

static uint8_t characteristic1_data[CHARACTERISTIC1_MAX_LEN] = { 0x00 };




//Hardware var
int LEDS = D0;
int buttIn = D6;
int butOut = D7;



void deviceConnectedCallback(BLEStatus_t status, uint16_t handle) {
  switch (status) {
    case BLE_STATUS_OK:
      Serial.println("Device connected!");
      break;
    default: break;
  }
}

void deviceDisconnectedCallback(uint16_t handle){
  Serial.println("Device disconnected!");

}

void setup() {

  delay(1000);
  Serial.begin(115200);

  //Init pins
  pinMode(LEDS, OUTPUT);
  pinMode(buttIn, INPUT);
  pinMode(butOut, OUTPUT);

  digitalWrite(butOut, HIGH);
  digitalWrite(LEDS, LOW);





//Init BLE
  ble.init();


  // Register BLE callback functions
  ble.onConnectedCallback(deviceConnectedCallback);
  ble.onDisconnectedCallback(deviceDisconnectedCallback);


/*
 * Commun service + characteristic
 */
  // Add GAP service and characteristics
  ble.addService(BLE_UUID_GAP);
  ble.addCharacteristic(BLE_UUID_GAP_CHARACTERISTIC_DEVICE_NAME, ATT_PROPERTY_READ|ATT_PROPERTY_WRITE, (uint8_t*)BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME));
  ble.addCharacteristic(BLE_UUID_GAP_CHARACTERISTIC_APPEARANCE, ATT_PROPERTY_READ, appearance, sizeof(appearance));
  ble.addCharacteristic(BLE_UUID_GAP_CHARACTERISTIC_PPCP, ATT_PROPERTY_READ, conn_param, sizeof(conn_param));
  // Add GATT service and characteristics
  ble.addService(BLE_UUID_GATT);
  ble.addCharacteristic(BLE_UUID_GATT_CHARACTERISTIC_SERVICE_CHANGED, ATT_PROPERTY_INDICATE, change, sizeof(change));



  /*
   * User define service + characteristic
   */
  ble.addService(service1_uuid);
  //notify characteristic
  character1_handle = ble.addCharacteristicDynamic(char1_uuid, ATT_PROPERTY_NOTIFY, characteristic1_data, CHARACTERISTIC1_MAX_LEN);

  // Set BLE advertising parameters
  ble.setAdvertisementParams(&adv_params);

  // Set BLE advertising data
  ble.setAdvertisementData(sizeof(adv_data), adv_data);

  // BLE peripheral starts advertising now.
  ble.startAdvertising();
  Serial.println("BLE start advertising.");


}


bool prec = false;
void loop() {
    if(digitalRead(buttIn)!=HIGH && prec == false){
        prec = true;
        digitalWrite(LEDS, HIGH);
        ble.sendNotify(character1_handle, (uint8_t*)"Up", CHARACTERISTIC1_MAX_LEN);
    }
    else if (digitalRead(buttIn)!=LOW && prec == true){
        digitalWrite(LEDS, LOW);
        ble.sendNotify(character1_handle, (uint8_t*)"Down", CHARACTERISTIC1_MAX_LEN);
        prec = false;
    }
    delay(100);
}
