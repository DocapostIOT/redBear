#define SOUND_LEVEL 400
#define SOUND_INTERVAL 5000 //10s
#define NB_SOUND_DETECT 10
#define COLOR_DURATION 10000

/*
**
**Humidity + T°
**
*/

#include "math.h"

double Rref = 10000.0; //Résistance de référence à 25°C
double V_IN = 5.0; //Alimentation électrique

//Information de la thermistance
double A_1 = 3.354016E-3;
double B_1 = 2.569850E-4;
double C_1 = 2.620131E-6;
double D_1 = 6.383091E-8;

double SteinhartHart(double R)
{
  //Division de l'équation en 4 parties. La premiere est
  //uniquement A1
  double equationB1 = B_1 * log(R/Rref);
  double equationC1 = C_1 * pow(log(R/Rref), 2);
  double equationD1 = D_1 * pow(log(R/Rref), 3);
  double equation = A_1 + equationB1 + equationC1 + equationD1;
  return pow(equation, -1);

}



#define MAXTIMINGS 85

#define cli noInterrupts
#define sei interrupts

#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21

#define NAN 999999


#define DHTPIN D2    // Digital pin D2

// IMPORTANT !! Make sure you set this to your
// sensor type.  Options: [DHT11, DHT22, DHT21, AM2301]
#define DHTTYPE DHT11



float h;      // humidity
float t;      // temperature
char h1[10];  // humidity string
char t1[10];  // temperature string
int f = 0;    // failed?




//Hardwar var
const int inputPin = A0;
const int LEDc = D7;
const int LEDa = D6;
const int LEDp = D5;




#define BLE_SCAN_TYPE        0x00  // Passive scanning
#define BLE_SCAN_INTERVAL    0x0010 // 60 ms
#define BLE_SCAN_WINDOW      0x0010 // 30 ms



static uint16_t conn_handle = 0xFFFF;

static uint16_t tab[50];
static int tab_count = 0;

static uint16_t handle_second = 0xFFFF;


static gatt_client_service_t serv[10];
static int serv_count = 0;
static gatt_client_service_t discovered_service;

static gatt_client_characteristic_t discovered_char;
static uint16_t power = NULL;
static uint16_t color = NULL;
static uint16_t brightness = NULL;
static int val = 0;
static uint16_t tab_char[100];
static String char_name[100];
static int char_count = 0;
static uint16_t handle = 0;


bool prec = false;


// size of the window
const int inputWindow = 25;
// placeholder for a single measurement
unsigned int inputSample;

static int value = 0;







/*
**
**
**BLE PERIPHERAL
**
*/
//Commun parameters
#define MIN_CONN_INTERVAL          0x0028 // 50ms.
#define MAX_CONN_INTERVAL          0x0190 // 500ms.
#define SLAVE_LATENCY              0x0000 // No slave latency.
#define CONN_SUPERVISION_TIMEOUT   0x03E8 // 10s.
#define BLE_PERIPHERAL_APPEARANCE  BLE_APPEARANCE_UNKNOWN

#define BLE_DEVICE_NAME            "Domino"

//Commun parameters
#define CHARACTERISTIC1_MAX_LEN    15
#define CHARACTERISTIC2_MAX_LEN    15
#define TXRX_BUF_LEN               15


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
static uint8_t service1_tx_uuid[16] = { 0x71,0x3d,0x00,0x03,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t service1_rx_uuid[16] = { 0x71,0x3d,0x00,0x02,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };

static uint16_t character1_handle = 0x0000;
static uint16_t character2_handle = 0x0000;
static uint16_t character3_handle = 0x0000;

static uint8_t characteristic1_data[CHARACTERISTIC1_MAX_LEN] = { 0x01 };
static uint8_t characteristic2_data[CHARACTERISTIC2_MAX_LEN] = { 0x00 };


static uint16_t service2 = 0x1234;
static uint16_t char2 = 0x1253;


static btstack_timer_source_t characteristic2;

char rx_buf[TXRX_BUF_LEN];
static uint8_t rx_buf_num;
static uint8_t rx_state = 0;



int gattWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {
  Serial.print("Write value handler: ");
  Serial.println(value_handle, HEX);
                      //ble.sendNotify(character_handle, (uint8_t*)"buf", CHARACTERISTIC1_MAX_LEN);


  if (character1_handle == value_handle) {
    memcpy(characteristic1_data, buffer, min(size,CHARACTERISTIC1_MAX_LEN));
    Serial.print("Characteristic1 write value: ");
    String str = "";
    for (uint8_t index = 0; index < min(size,CHARACTERISTIC1_MAX_LEN); index++) {
      str.concat(String(char(characteristic1_data[index])));
    }
    Serial.println( str);
    if (str.compareTo("c") == 0){
        digitalWrite(LEDc, HIGH);
        delay(5000);
        digitalWrite(LEDc, LOW);
    }
    else if (str.compareTo("a") == 0){
        digitalWrite(LEDa, HIGH);
        delay(5000);
        digitalWrite(LEDa, LOW);
    }
    else if (str.compareTo("p") == 0){
        digitalWrite(LEDp, HIGH);
        delay(5000);
        digitalWrite(LEDp, LOW);
    }
    else if (str.compareTo("s") == 0){
        uint8_t data[]= {0xff,0x00,0x00};
        ble.writeValue(conn_handle, color, sizeof(data), data);
        prec = true;
    }

  }
  return 0;
}






//humidity + tmp



class DHT {
    private:
        uint8_t data[6];
        uint8_t _pin, _type, _count;
        bool read(void);
        unsigned long _lastreadtime;
        bool firstreading;

    public:
        DHT(uint8_t pin, uint8_t type, uint8_t count=6);
        void begin(void);
        float readTemperature(bool S=false);
        float convertCtoF(float);
        float readHumidity(void);

};


DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
    _pin = pin;
    _type = type;
    _count = count;
    firstreading = true;
}


void DHT::begin(void) {
    // set up the pins!
    pinMode(_pin, INPUT);
    digitalWrite(_pin, HIGH);
    _lastreadtime = 0;
}


//boolean S == Scale.  True == Farenheit; False == Celcius
float DHT::readTemperature(bool S) {
    float _f;

    if (read()) {
        switch (_type) {
            case DHT11:
                _f = data[0];

                if(S)
                    _f = convertCtoF(_f);

                return _f;


            case DHT22:
            case DHT21:
                _f = data[2] & 0x7F;
                _f *= 256;
                _f += data[3];
                _f /= 10;

                if (data[2] & 0x80)
                    _f *= -1;

                if(S)
                    _f = convertCtoF(_f);

                return _f;
        }
    }

    return NAN;
}


float DHT::convertCtoF(float c) {
    return c * 9 / 5 + 32;
}


float DHT::readHumidity(void) {
    float _f;
    if (read()) {
        switch (_type) {
            case DHT11:
                _f = data[2];
                return _f;


            case DHT22:
            case DHT21:
                _f = data[0];
                _f *= 256;
                _f += data[1];
                _f /= 10;
                return _f;
        }
    }

    return NAN;
}


bool DHT::read(void) {
    uint8_t laststate = HIGH;
    uint8_t counter = 0;
    uint8_t j = 0, i;
    unsigned long currenttime;

    // pull the pin high and wait 250 milliseconds
    digitalWrite(_pin, HIGH);

    currenttime = millis();
    if (currenttime < _lastreadtime) {
        // ie there was a rollover
        _lastreadtime = 0;
    }

    if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
        //delay(2000 - (currenttime - _lastreadtime));
        return true; // return last correct measurement
    }

    firstreading = false;
    Serial.print("Currtime: "); Serial.print(currenttime);
    Serial.print(" Lasttime: "); Serial.print(_lastreadtime);
    _lastreadtime = millis();

    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    // now pull it low for ~20 milliseconds
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    delay(20);
    cli();
    digitalWrite(_pin, HIGH);
    delayMicroseconds(40);
    pinMode(_pin, INPUT);

    // read in timings
    for ( i=0; i< MAXTIMINGS; i++) {
        counter = 0;

        while (digitalRead(_pin) == laststate) {
            counter++;
            delayMicroseconds(1);

            if (counter == 255)
                break;
        }

        laststate = digitalRead(_pin);

        if (counter == 255)
            break;

        // ignore first 3 transitions
        if ((i >= 4) && (i%2 == 0)) {
            // shove each bit into the storage bytes
            data[j/8] <<= 1;

            if (counter > _count)
                data[j/8] |= 1;

            j++;
        }
    }

    sei();


    // check we read 40 bits and that the checksum matches
    if ((j >= 40) &&  (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)))
        return true;


    return false;
}



/*
**
**
**BLE CENTRAL
**
*/

static void bleScanCallback(advertisementReport_t *report) {
  uint8_t index;
  Serial.println("BLE scan callback: ");

  Serial.print("Advertising event type: ");
  Serial.println(report->advEventType, HEX);

  Serial.print("Peer device address type: ");
  Serial.println(report->peerAddrType, HEX);

  Serial.print("Peer device address: ");
  for (index = 0; index < 6; index++) {
    Serial.print(report->peerAddr[index], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
  Serial.print("Advertising/Scan response data packet: ");
  for (index = 0; index < report->advDataLen; index++) {
    Serial.print(report->advData[index], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
  Serial.println(" ");

     if (conn_handle != 0xFFFF){
         ble.stopScanning();
     }
    if(report->peerAddr[0] == 0xc4 &&
    report->peerAddr[1] == 0xbe &&
    report->peerAddr[2] == 0x84 &&
    report->peerAddr[3] == 0xc7 &&
    report->peerAddr[4] == 0x88 &&
    report->peerAddr[5] == 0x4b){

             ble.connect(report->peerAddr, (bd_addr_type_t)report->peerAddrType);


    }
}


static void deviceConnectedCallback(BLEStatus_t status, uint16_t handle) {
  switch (status) {
    case BLE_STATUS_OK:
      Serial.print("BLE device connection established! Connection handle: ");
      Serial.println(handle, HEX);
      if (conn_handle ==0xFFFF ){
         conn_handle = handle;
         ble.discoverPrimaryServices(conn_handle);
      }
      break;
    default:
      Serial.println("Failed to establish connection with peer device!");
      break;
  }
}



static void serviceDiscoveredCallback(BLEStatus_t status, uint16_t conn_handle, gatt_client_service_t *service) {
  if (status == BLE_STATUS_OK) {
      // Found a servi
      Serial.print("serv : ");
    String str = "";
    for (int i = 0; i< 16; i++){
        str.concat(String(service->uuid128[i], HEX));
    }
    Serial.println(str);
    Serial.println("Service");
    serv[serv_count] = *service;
    serv_count++;
  }
  else if (status == BLE_STATUS_DONE) {
    Serial.println("Discovers service completed");
         ble.discoverCharacteristics(conn_handle, &serv[0]);
  }
}



static void charsDiscoveredCallback(BLEStatus_t status, uint16_t conn_handle2, gatt_client_characteristic_t *characteristic) {
  if (status == BLE_STATUS_OK) {   // Found a characteristic.
    discovered_char = *characteristic;

    Serial.print("charac : ");
   String str = "";
    for (int i = 0; i< 16; i++){
        str.concat(String(characteristic->uuid128[i], HEX));
    }
    Serial.println(str);
    if (str.compareTo("217887f8af24029c524c9ecf7160")==0){
        power = characteristic->value_handle;
    }
    else if (str.compareTo("74532143fff146d8e8a37f934d40be")==0){
       color = characteristic->value_handle;
    }
    else if (str.compareTo("1c537ba4eaa4e19b98ceaaa5bcd9bc9")==0){
        brightness = characteristic->value_handle;
    }

    Serial.println("");
  }
  else if (status == BLE_STATUS_DONE) {
    Serial.println("Discovers characteristic completed.");
    if (val <= serv_count){
        ble.discoverCharacteristics(conn_handle, &serv[val]);
        val++;
    }
  }
}


static void deviceDisconnectedCallback(uint16_t handle) {
  Serial.print("Disconnected from peer BLE device. Connection handle: ");

  Serial.println(handle, HEX);
  if (handle == conn_handle){
        color = 0;
        conn_handle = 0xFFFF;
  }
Serial.print("restart scanning ");
  ble.startScanning();


}


static void gattWrittenCallback(BLEStatus_t status, uint16_t conn_handle) {
  if (status == BLE_STATUS_DONE) {
  //  Serial.println(" ");
//    Serial.println("Writes characteristic value successfully.");

  }
  else if (status == BLE_STATUS_OK){
      Serial.println("1");
  }
  else if (status == BLE_STATUS_CONNECTION_TIMEOUT){
      Serial.println("2");
  }
  else if (status == BLE_STATUS_CONNECTION_ERROR){
      Serial.println("3");
  }
  else if (status == BLE_STATUS_OTHER_ERROR){
      Serial.println("4");
  }
}

DHT dht(DHTPIN, DHTTYPE);


static bool disp = true;

void tim(){
    if (color != 0){

        if(prec == true){
            uint8_t data[]= {0xff,0x00,0x00};
            ble.writeValue(conn_handle, color, sizeof(data), data);
        }
        else{
            disp = true;
        }
        prec = false;
        value = 0;
    }
}

Timer timer(SOUND_INTERVAL, tim);

void setup() {


    pinMode(D0,OUTPUT);
    digitalWrite(D0,HIGH);
    pinMode(D1,OUTPUT);
    digitalWrite(D1,HIGH);
    pinMode(D3,OUTPUT);
    digitalWrite(D3,HIGH);
    pinMode(A1, INPUT);
    pinMode(inputPin, INPUT);

    pinMode(LEDa, OUTPUT);
    pinMode(LEDc, OUTPUT);
    pinMode(LEDp, OUTPUT);

    digitalWrite(LEDa, LOW);
    digitalWrite(LEDp, LOW);
    digitalWrite(LEDc, LOW);



    //BLE CENTRAL
    ble.init();



    // initializing the serial communication
    ble.onCharacteristicDiscoveredCallback(charsDiscoveredCallback);
    ble.onScanReportCallback(bleScanCallback);
    ble.onConnectedCallback(deviceConnectedCallback);
    ble.onDisconnectedCallback(deviceDisconnectedCallback);
    ble.onServiceDiscoveredCallback(serviceDiscoveredCallback);
    ble.onGattCharacteristicWrittenCallback(gattWrittenCallback);


    ble.setScanParams(BLE_SCAN_TYPE, BLE_SCAN_INTERVAL, BLE_SCAN_WINDOW);
    ble.startScanning();
    dht.begin();
    timer.start();

    //BLE PERIPHERAL
    ble.onDataWriteCallback(gattWriteCallback);

    // Add GAP service and characteristics
    ble.addService(BLE_UUID_GAP);
    ble.addCharacteristic(BLE_UUID_GAP_CHARACTERISTIC_DEVICE_NAME, ATT_PROPERTY_READ|ATT_PROPERTY_WRITE, (uint8_t*)BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME));
    ble.addCharacteristic(BLE_UUID_GAP_CHARACTERISTIC_APPEARANCE, ATT_PROPERTY_READ, appearance, sizeof(appearance));
    ble.addCharacteristic(BLE_UUID_GAP_CHARACTERISTIC_PPCP, ATT_PROPERTY_READ, conn_param, sizeof(conn_param));

    // Add GATT service and characteristics
    ble.addService(BLE_UUID_GATT);
    ble.addCharacteristic(BLE_UUID_GATT_CHARACTERISTIC_SERVICE_CHANGED, ATT_PROPERTY_INDICATE, change, sizeof(change));

    // Add user defined service and characteristics
    ble.addService(service1_uuid);
    character1_handle = ble.addCharacteristicDynamic(service1_tx_uuid, ATT_PROPERTY_NOTIFY|ATT_PROPERTY_WRITE|ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, characteristic1_data, CHARACTERISTIC1_MAX_LEN);
    character2_handle = ble.addCharacteristicDynamic(service1_rx_uuid, ATT_PROPERTY_NOTIFY, characteristic2_data, CHARACTERISTIC2_MAX_LEN);

    // Set BLE advertising parameters
    ble.setAdvertisementParams(&adv_params);

    // Set BLE advertising data
    ble.setAdvertisementData(sizeof(adv_data), adv_data);
    // BLE peripheral starts advertising now.

}

static bool once = true;
static int i = 0;
static int moyenne = 0;
static int nb_moy = 0;
static uint8_t c1[] = {0xff, 0x00, 0x00};
static uint8_t c2[] = {0x00, 0xff, 0x00};
static uint8_t c3[] = {0x00, 0x00, 0xff};
static uint8_t cd[] = {0xff, 0x00, 0x00};
static uint8_t ca[] = {0x00, 0xff, 0x00};
static int it = 0;
static int col = 0;

TCPClient client;
TCPClient client1;
TCPClient client2;
void loop() {
    unsigned int inputMax = 0;
    unsigned int inputMin = 4096;
    if (color != 0){
        uint8_t R = cd[0] + (((ca[0] - cd[0])*it)/COLOR_DURATION);
        uint8_t G = cd[1] + (((ca[1] - cd[1])*it)/COLOR_DURATION);
        uint8_t B = cd[2] + (((ca[2] - cd[2])*it)/COLOR_DURATION);
        it++;
        if (it == COLOR_DURATION){
            it = 0;
            col++;
            if (col > 1){
                col = 0;

            }
            if(c1[0]==cd[0] && c1[1]==cd[1] && c1[2]==cd[2]){
                cd[0] = c2[0];
                cd[1] = c2[1];
                cd[2] = c2[2];

                ca[0] = c3[0];
                ca[1] = c3[1];
                ca[2] = c3[2];
            }
            else if(c2[0]==cd[0] && c2[1]==cd[1] && c2[2]==cd[2]){
                cd[0] = c3[0];
                cd[1] = c3[1];
                cd[2] = c3[2];

                ca[0] = c1[0];
                ca[1] = c1[1];
                ca[2] = c1[2];
            }
            else if(c3[0]==cd[0] && c3[1]==cd[1] && c3[2]==cd[2]){
                cd[0] = c1[0];
                cd[1] = c1[1];
                cd[2] = c1[2];

                ca[0] = c2[0];
                ca[1] = c2[1];
                ca[2] = c2[2];
            }
        }
        if (disp){
                uint8_t data[]= {R, G, B};
                ble.writeValue(conn_handle, color, sizeof(data), data);
            }

        if (i > 100){

            t = dht.readTemperature();
            Serial.print("h:");Serial.println(t,DEC);
            Serial.println("==========");
            String str = "h";
            str.concat(t);
            byte buf[str.length()];
            str.getBytes(buf, str.length());
            ble.sendNotify(character2_handle, (uint8_t*)buf, CHARACTERISTIC1_MAX_LEN);
            delay(50);
            double valeurAnalog = analogRead(A2);
            Serial.println(valeurAnalog);
            double V =  valeurAnalog / 4095 * V_IN;
            Serial.println(V);
            //Calcul de la résistance de la thermistance
            double Rth = (Rref * V ) / (V_IN - V);
            Serial.print("Rth = ");
            Serial.print(Rth);
            //Calcul de la température en kelvin( Steinhart and Hart)
            double kelvin = SteinhartHart(Rth);
            double celsius = kelvin - 273.15; //Conversion en celsius
            Serial.print("Ohm  -  T = ");
            Serial.print(celsius);
            Serial.println("C\n");
            str = "t";
            str.concat(celsius);
            byte buf2[str.length()];
            str.getBytes(buf2, str.length());
            ble.sendNotify(character2_handle, (uint8_t*)buf2, CHARACTERISTIC1_MAX_LEN);

            delay(50);
            str = "s";
            str.concat(moyenne);
            byte buf3[str.length()];
            str.getBytes(buf3, str.length());
            ble.sendNotify(character2_handle, (uint8_t*)buf3, CHARACTERISTIC1_MAX_LEN);
            str = String(celsius);
            if (col == 1){
                col = 0;
                Particle.publish("temp", str, PRIVATE);
                str = String(t);
                Particle.publish("humi", str, PRIVATE);
                str = String(moyenne);
                Particle.publish("son", str, PRIVATE);
            }
/*
           if (col == 1){
               col = 0;
               if(client2.connect("docapost-iot.cspvdi.fr", 80)){
                        str = "POST /api/teamIOapp/humi?";
                        str.concat(t);
                        str.concat(" HTTP/1.0");
                                                Serial.println(str);


                        client2.println(str);
                        client2.println("Host: docapost-iot.cspvdi.fr");
                        client2.println("Content-Length: 0");
                        client2.println();
                        if (client2.available())
                          {
                            char c = client2.read();
                            Serial.print(c);
                            c = client2.read();
                            Serial.print(c);
                          }
                        client2.stop();
            }


               if(client.connect("docapost-iot.cspvdi.fr", 80)){
                        str = "POST /api/teamIOapp/son?";
                        str.concat(moyenne);
                        str.concat(" HTTP/1.0");
                        Serial.println(str);
                        client.println(str);
                        client.println("Host: docapost-iot.cspvdi.fr");
                        client.println("Content-Length: 0");
                        client.println();
                        client.stop();


            }


            if(client1.connect("docapost-iot.cspvdi.fr", 80)){
                        str = "POST /api/teamIOapp/temp?";
                        str.concat(celsius);
                        str.concat(" HTTP/1.0");
                         Serial.println(str);


                        client1.println(str);
                        client1.println("Host: docapost-iot.cspvdi.fr");
                        client1.println("Content-Length: 0");
                        client1.println();
                        client1.stop();


            }

           }

           */

            moyenne = 0;
            nb_moy = 0;
            i = 0;

        }
        i++;
        if (once == true){
            once =false;
            ble.startAdvertising();
            delay(500);
            Serial.println("BLE start advertising.");
        }
        for (unsigned int i = 0; i < inputWindow; i++) {
            inputSample = analogRead(inputPin);
            inputMin = min(inputMin, inputSample);
            inputMax = max(inputMax, inputSample);
        }
        moyenne = moyenne + (inputMax - inputMin);
        nb_moy ++;
        if ((inputMax - inputMin) > SOUND_LEVEL){
            value ++;
            Serial.println("sound");
            if (value > NB_SOUND_DETECT){
                Serial.println("write");
                uint8_t data[]= {0xff,0x00,0x00};
                ble.writeValue(conn_handle, color, sizeof(data), data);
                prec = true;
                disp = false;




            }
        }


    }
}
