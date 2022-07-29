// agregar sonidos para para error de conexión.
#include <SPI.h>
#include <Ethernet.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// determina si estoy en modo debugger (serial) o no
bool debugger;
byte blink_status = 0;
byte blink_step = 0;
unsigned long last_blink;
#define blink_T   250   // tiempo de cada parpadeo del led de status

// tiempo de timeout para esperar respuesta del server
#define TIMEOUT_LEN 60000

// configuracion del EHTERNET

// MOLINETE 1
static String AGENTE = "User-Agent: 9 Julio - Molinete 1";
static byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xE0};     // molinete 1
IPAddress ip(192, 168, 49, 31);                                 // molinete 1
#define PWM2      120                  // PWM para sostener la solenoide
#if 0
// MOLINETE 2
static String AGENTE = "User-Agent: 9 Julio - Molinete 2";
static byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xE1};       // molinete 2
IPAddress ip(192, 168, 49, 32);                                   // molinete 2
// #define PWM2      80               // PWM para sostener la solenoide
#define PWM2      120               // PWM para sostener la solenoide
#endif

// IP y DPs por si falla el DHCP, para conectar de manera autónoma
IPAddress myDns(192, 168, 49, 1);
static byte server[] = {192, 168, 49, 30};
static String HOST = "Host: 192.168.49.30";
#define PUERTO_SRV 5050
#define intervalo_srv 1000      // tiempo entre revisiones al servidor, en milisegundos.
unsigned long lastcon_srv;

// initialize the library instance:
EthernetClient client;
EthernetClient srv_client;
EthernetServer servidor = EthernetServer(PUERTO_SRV);

// varibles y parámetro de la conexión ethernet
#define ING_OK      '1'               // Puede ingresar
#define ING_NO_EX   '0'               // no existe el usuario
#define ING_NO_PAG  '2'               // falta de pago
#define ING_ERR     'E'               // error en la conexión con el servidor
#define MOL_AP      'A'               // Abrir purtas, dejar abiertas
#define MOL_CER     'C'               // cerrar puertas
#define MOL_TEM     'T'               // Abrir por un tiempo definido
#define MOL_IN_HAB  'I'               // habilito ingreso libre
#define MOL_IN_DES  'i'               // deshabilito ingreso libre
#define MOL_OUT_HAB 'S'               // habilito salida libre
#define MOL_OUT_DES 's'               // deshabilito salida libre

#define T_apertura 5000    // tiempo en que permanecen los molinetes abiertos (en milisegundos)
#define T_max     10        // 500ms * T_max
#define PWM1      255       // PWM al cerrar la solenoide
                            //  #define PWM2      80 //80    120    // PWM para sostener la solenoide
#define dem_PWM    1200      // demora del PWM

// PINES
#define L0DT    4     // data lector 0 Entrada
#define L0CK    2     // clock lector 0 (en interrup)
#define L1DT    A0     // data lector 1 Salida   // CAMBIADO
#define L1CK    3     // clock lector 1 (en interrup)
#define OPTO0   A3    // opto in
#define OPTO1   A2    // opto out
#define OUT0    5    // solenoide in     // cambiado
#define OUT1    6    // solenoide out    // cambiado
#define LED_NP  8     // no puede pasar
#define LED_IN  A1     // puede ingresar     // CAMBIADO
#define LED_OUT 7     // puede salir
#define W0BUZ   9     // buzzer de Lector0
#define W1BUZ   A4    // buzzer de lector1
#define STA_LED A5    // led que indica status del equipo

bool accessIN = false;      // variable con la confirmacion del acceso ENTRADA
bool accessOUT = false;      // variable con la confirmacion del acceso SALIDA
bool manual = false;        // determina si estoy en acceso manual o automático
bool accessIN_BLOQ = true;    // determina si controlo el ingreso
bool accessOUT_BLOQ = true;   // determina si controlo el egreso

// Variables y parámetros de RFID
unsigned long lastCon0 = 0;           // Tiempo en ms desde el último bit recibido por RX0
unsigned long lastCon1 = 0;           // Tiempo en ms desde el último bit recibido por RX1
#define intervalo 100                 // tiempo de espera máximo entr dos bits com Wiegand (en ms)

// Variables golbales para lectores RFID
unsigned long RFID0 = 0;            // ID leido por el lector Wiegand0
unsigned long RFID1 = 0;            // ID Leido por el lector Wiegand1

// Variables Lector Wiegand 0
byte bit_count0=26;
unsigned long  rec_data0 = 0;
boolean error0 =0;

// Variables Lector Wiegand 1
byte bit_count1=26;
unsigned long  rec_data1 = 0;
boolean error1 =0;

char resp;        // respuesta de la conexión al servidor
String srfid;     // variable para pasar el código RFID temoralmente a string

// Configuración
void setup() {
  //deshabilito el WDT
  wdt_disable();

  // blink
  pinMode(STA_LED, OUTPUT);
  digitalWrite(STA_LED,HIGH);

  // configuro los pines
  pinMode(OPTO0, INPUT_PULLUP);
  pinMode(OPTO1, INPUT_PULLUP);

  // espero a que se estabilicen tensiones
  delay(2000);

  // reviso a ver si estoy en modo Debug
  bool auxRead0 = digitalRead(OPTO0);
  bool auxRead1 = digitalRead(OPTO1);

  if (auxRead0 == true || auxRead1 == true) {
    debugger = true;

    // start serial port:
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("Debbugger ON");
    Serial.println("");

  } else{
    debugger = false;
  }

  // conexion a la red ethernet
  EthernetConect();

  // Config del Lector WG0
  pinMode(L0DT, INPUT_PULLUP);
  pinMode(L0CK, INPUT_PULLUP);
  pinMode(W0BUZ, OUTPUT);
  digitalWrite(W0BUZ,LOW);
  attachInterrupt(digitalPinToInterrupt(L0CK), Lector0, FALLING);

  // Config del lector WG1
  pinMode(L1DT, INPUT_PULLUP);
  pinMode(L1CK, INPUT_PULLUP);
  pinMode(W1BUZ, OUTPUT);
  digitalWrite(W1BUZ,LOW);
  attachInterrupt(digitalPinToInterrupt(L1CK), Lector1, FALLING);

  // pines de salida para solenoides
  pinMode(OUT0, OUTPUT);
  pinMode(OUT1, OUTPUT);
  digitalWrite(OUT0,LOW);
  digitalWrite(OUT1,LOW);

  // cofiguración optoacopladores para molinete
  pinMode(OPTO0, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(OPTO0), isr_opto0, CHANGE);

  pinMode(OPTO1, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(OPTO1), isr_opto1, CHANGE);

  // configuracion de leds de salida
  pinMode(LED_NP, OUTPUT);
  pinMode(LED_IN, OUTPUT);
  pinMode(LED_OUT, OUTPUT);
  digitalWrite(LED_IN,LOW);
  digitalWrite(LED_OUT,LOW);
  digitalWrite(LED_NP,HIGH);

  if ((char)EEPROM.read(0) == MOL_IN_HAB) {accessIN_BLOQ = false;} else {accessIN_BLOQ = true;}   // leo el estado del control de acceso
  if ((char)EEPROM.read(1) == MOL_OUT_HAB) {accessOUT_BLOQ = false;} else {accessOUT_BLOQ = true;}   // false libero molinetes, true traba molinete
  if (debugger == true) {
    if (accessIN_BLOQ == true) { Serial.println("Molinete ingreso trabado"); } else { Serial.println("Molinete ingreso liberado"); }
    if (accessOUT_BLOQ == true) { Serial.println("Molinete egreso trabado"); } else { Serial.println("Molinete egreso liberado"); }
  }

  // Habilito el WDT
  wdt_enable(WDTO_8S);
  digitalWrite(STA_LED,LOW);
  last_blink = millis();
}

// LOOP
void loop() {
  int i;
  wdt_reset();

  if (millis() - last_blink > blink_T) {     // si pasaron 250ms cambio el parpadeo
    blink_led();
  }
  wdt_reset();

  if (millis() - lastcon_srv > intervalo_srv) {     // si pasó mas tiempo que el intervalo desde el último bit recibido, descarto todo
    lastcon_srv = millis();
    resp = http_servidor();

    // recibí algun dato?
    if (resp!=0)
    {
      if (debugger == true) { Serial.println(resp);}

      if ( resp == MOL_AP){

        accessIN = true;
        accessOUT = true;
        manual = true;
        digitalWrite(LED_IN,HIGH);
        digitalWrite(LED_OUT,HIGH);
        digitalWrite(LED_NP,LOW);
        digitalWrite(W0BUZ,LOW);
        digitalWrite(W1BUZ,LOW);

        wdt_reset();

      } else if (resp == MOL_CER){

        accessIN = false;
        accessOUT = false;
        manual = false;
        digitalWrite(LED_IN,LOW);
        digitalWrite(LED_OUT,LOW);
        digitalWrite(LED_NP,HIGH);
        digitalWrite(W0BUZ,LOW);
        digitalWrite(W1BUZ,LOW);
        RFID0=0;
        RFID1=0;
        wdt_reset();

      } else if (resp == MOL_TEM){

        accessIN = true;
        accessOUT = true;
        manual = true;
        digitalWrite(LED_IN,HIGH);
        digitalWrite(LED_OUT,HIGH);
        digitalWrite(LED_NP,LOW);
        digitalWrite(W0BUZ,HIGH);
        digitalWrite(W1BUZ,HIGH);

        wdt_reset();
        delay(T_apertura);
        wdt_reset();

        accessIN = false;
        accessOUT = false;
        manual = false;
        digitalWrite(LED_IN,LOW);
        digitalWrite(LED_OUT,LOW);
        digitalWrite(LED_NP,HIGH);
        digitalWrite(W0BUZ,LOW);
        digitalWrite(W1BUZ,LOW);
        RFID0=0;
        RFID1=0;
        wdt_reset();

      } else if (resp == MOL_IN_HAB) {  // ingreso libre
        accessIN_BLOQ = false;
        EEPROM.update(0,MOL_IN_HAB);
        if (debugger == true) { Serial.println("Molinete ingreso liberado"); }

      } else if (resp == MOL_IN_DES) {  // ingreso controlado
        accessIN_BLOQ = true;
        EEPROM.update(0,MOL_IN_DES);
        if (debugger == true) { Serial.println("Molinete ingreso controlado"); }

      } else if (resp == MOL_OUT_HAB) {  // egreso libre
        accessOUT_BLOQ = false;
        EEPROM.update(1,MOL_OUT_HAB);
        if (debugger == true) { Serial.println("Molinete egreso liberado"); }

      } else if (resp == MOL_OUT_DES) {  // egreso controlado
        accessOUT_BLOQ = true;
        EEPROM.update(1,MOL_OUT_DES);
        if (debugger == true) { Serial.println("Molinete egreso controlado"); }

      }else if (resp == 'E') {
        blink_status = 1;
      }
    }
    lastcon_srv = millis();
  }  // fin recepción datos modo servidor

  wdt_reset();

  // reviso si hay algo leido en el lector 0
  if (RFID0 != 0)
  {
    srfid = String(RFID0,DEC);
    resp =  httpRequest(srfid,'1');
    if ( resp==ING_OK)
    {
      blink_status = 0;

      accessIN = true;
      digitalWrite(LED_IN,HIGH);
      digitalWrite(LED_NP,LOW);
      digitalWrite(W0BUZ,HIGH);

      // espera del molinete
      wdt_reset();
      for (i=0; i<T_max; i++)
      {
        delay(500);
        if (accessIN==false) break;
      }
      // delay(T_apertura);
      wdt_reset();

      accessIN=false;
      digitalWrite(LED_IN,LOW);
      digitalWrite(LED_NP,HIGH);
      digitalWrite(W0BUZ,LOW);

    } else if (resp== ING_ERR)
    {
      blink_status = 1;

      accessIN = true;
      digitalWrite(LED_IN,HIGH);
      digitalWrite(LED_NP,LOW);
      digitalWrite(W0BUZ,HIGH);

      // espera del molinete
      wdt_reset();
      for (i=0; i<T_max; i++)
      {
        delay(500);
        digitalWrite(W0BUZ,!digitalRead(W0BUZ));
        if (accessIN==false) break;
      }
      // delay(T_apertura);
      wdt_reset();

      accessIN=false;
      digitalWrite(LED_IN,LOW);
      digitalWrite(LED_NP,HIGH);
      digitalWrite(W0BUZ,LOW);

    } else{

      accessIN=false;
      digitalWrite(W0BUZ,HIGH);
      delay(300);
      digitalWrite(W0BUZ,LOW);
      wdt_reset();
    }
    // borro la tarjeta leida
    RFID0 = 0;
  }

  wdt_reset();

  // reviso si hay algo leido en el lector 1
  if (RFID1 != 0)
  {
    srfid = String(RFID1,DEC);
    resp =  httpRequest(srfid,'0');
    if ( resp==ING_OK)
    {
      blink_status = 0;

      accessOUT = true;
      digitalWrite(LED_OUT,HIGH);
      digitalWrite(LED_NP,LOW);
      digitalWrite(W1BUZ,HIGH);

      wdt_reset();
      // espera del molinete
      for (i=0; i<T_max; i++)
      {
        delay(500);
        if (accessOUT==false) break;
      }
      // delay(T_apertura);
      wdt_reset();

      accessOUT=false;
      digitalWrite(LED_OUT,LOW);
      digitalWrite(LED_NP,HIGH);
      digitalWrite(W1BUZ,LOW);

      wdt_reset();

    } else if (resp == ING_ERR)
    {
      blink_status = 0;

      accessOUT = true;
      digitalWrite(LED_OUT,HIGH);
      digitalWrite(LED_NP,LOW);
      digitalWrite(W1BUZ,HIGH);

      wdt_reset();
      // espera del molinete
      for (i=0; i<T_max; i++)
      {
        delay(500);
        digitalWrite(W1BUZ,!digitalRead(W1BUZ));
        if (accessOUT==false) break;
      }
      // delay(T_apertura);
      wdt_reset();

      accessOUT=false;
      digitalWrite(LED_OUT,LOW);
      digitalWrite(LED_NP,HIGH);
      digitalWrite(W1BUZ,LOW);

      wdt_reset();

    } else{
      accessOUT=false;
      digitalWrite(W1BUZ,HIGH);
      delay(300);
      digitalWrite(W1BUZ,LOW);
      wdt_reset();
    }
    // borro la tarjeta Leida
    RFID1 = 0;
  }
  wdt_reset();
}

// para recibir información desde el socket (La raspberry manda información para abrir o cerrar el molinete/puerta)

char http_servidor()
{
  bool recibe_numeral = false;
  char status_ingreso = 0;

  EthernetClient srv_client = servidor.available();
  if (srv_client) {

    // proceso lo recibido en el puerto
    while (srv_client.connected()) {
      while (srv_client.available()) {
        char c = srv_client.read();
        if (c == '#') {
          recibe_numeral = true;

        } else if (recibe_numeral == true) {
          status_ingreso = c;
          srv_client.stop();

          while (srv_client.available()) {}
          srv_client.stop();
          return status_ingreso;
        }
      }
      srv_client.stop();
      return 'E';
    }
  }
  return 0;
}


/* Control Ethernet*/
// char httpRequest(String ID) {
//  Recibe como parámetro el ID para enviar en formato string
//  devuelve como parámetro el permiso de la conexión:
//    0: NO puede ingresar (No existe usr)
//    1: puede ingresar
//    2: NO puede ingresar (Falta de pago)
//    E: Error en la conexión.

char httpRequest(String ID,char acceso) { //string in o out
#define intentos 3            // cantidad de intentos que quiero hacer para intentar conectar.
  byte i;                        //  auxiliar para el for
  bool recibe_numeral=false;    // flag para saber si recibió el numeral
  char status_ingreso;          // devuelve el estado para ingresar o no.
  unsigned long startedWaiting;

  for (i=0; i<intentos;i++)
  {
    // close any connection before send a new request.
    // This will free the socket on the WiFi shield
    while (client.available()){
      char c_flush = client.read();
    }
    client.stop();

    // if there's a successful connection:
    if (client.connect(server, 80)) {

      // send the HTTP GET request:
      String GET = String("GET /general/?nrTarjeta=" + ID + "&direccion=" + acceso + " HTTP/1.1");
      client.println(GET);
      client.println(HOST);
      client.println(AGENTE);
      client.println("Connection: close");
      client.println();

      startedWaiting = millis();
      while (client.available() == 0 && millis() - startedWaiting <= TIMEOUT_LEN);

      // proceso el resultado
      while (client.available())
      {
        char c = client.read();

        if (c=='#')
        { recibe_numeral = true; }
        else if (recibe_numeral == true) {
          status_ingreso = c;

          return status_ingreso;
        }
      }
      // note the time that the connection was made:
    }  else {
      // if you couldn't make a connection:
      delay(200);
    }
  }

  return 'E';
}


/* Función para conectar la placa de RED*/
void EthernetConect() {
  // You can use Ethernet.init(pin) to configure the CS pin
  Ethernet.init(10);  // Most Arduino shields

  // start the Ethernet connection:
  if (debugger == true) {
    Serial.println("Conectando por DHCP:");
    Serial.println("Conectando a la red:");
    Serial.print("  MAC: ");
    for (byte i = 0; i < 6; i++) {
      Serial.print(mac[i],HEX);
      if (i!=5) Serial.print(":");
    }
    Serial.println("");
    Serial.println("  " + AGENTE);
    Serial.println("");
  }
  if (Ethernet.begin(mac) == 0) {
    if (debugger == true) { Serial.println("Falló la conexión por DHCP"); }
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      if (debugger == true) { Serial.println("No se detecta la placa de Red.  no se puede seguir. :("); }
      while (true) {
        delay(1); // do nothing, no point running without Ethernet hardware
      }
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      if (debugger == true) { Serial.println("Cable ethernet desconectado."); }
    }

    // Conectando directo
    if (debugger == true)  {
      Serial.println("Conectando con IP estática:");
    }

    Ethernet.begin(mac, ip, myDns);

  }
  if (debugger == true) {
    Serial.print("  Mi dirección IP: ");
    Serial.println(Ethernet.localIP());
  }

  // give the Ethernet shield a second to initialize:
  delay(1000);

  // abro para que quede también en modo servidor
  servidor.begin();
  if (debugger == true){
    Serial.print("  Socket abierto en: ");
    Serial.print(Ethernet.localIP());
    Serial.print(" : ");
    Serial.println(PUERTO_SRV);
    Serial.println("");
  }
  delay(1000);
}

// LECTOR WIEGAND 0 (INGRESO)
void Lector0()
{
  byte i;      // auxiliar para el chequeo de paridad
  byte suma0;   // auxiliar para el chequeo de paridad (suma)
  if (millis() - lastCon0 > intervalo) {     // si pasó mas tiempo que el intervalo desde el último bit recibido, descarto todo
    bit_count0=26;
    rec_data0=0;
  }

  bit_count0--;
  if (digitalRead(L0DT)==1)
  { bitSet(rec_data0,bit_count0);
  } else {
    bitClear(rec_data0,bit_count0);
  }
  lastCon0 = millis();

  if (bit_count0 == 0)
  {
    // Si llegué hasta acá es porque recibí los 26 bits
    // control de errores por bit de paridad
    error0 = false;

    // reviso primer bit de paridad
    for (i=24,suma0=0;i>12;i--)
    {
      if (bitRead(rec_data0,i)==1) { suma0++; }
    }
    suma0 = suma0%2;
    if (suma0!=bitRead(rec_data0,25)) {error0 = true; }

    // reviso segundo bit de paridad
    for (i=12,suma0=0;i>0;i--)
    {
      if (bitRead(rec_data0,i)==1){ suma0++; }
    }
    suma0 = suma0%2;
    if (suma0==bitRead(rec_data0,0)) {error0 = true;}

    if (error0==false)
    {
      bitClear(rec_data0,25);
      RFID0 = rec_data0>>1;

    } else {
      // error de datos en primer bit de paridad
      RFID0=0;
    }
  }
}

// LECTOR WIEGAND 1 (SALIDA)
void Lector1()
{
  byte i;      // auxiliar para el chequeo de paridad
  byte suma1;   // auxiliar para el chequeo de paridad (suma)
  if (millis() - lastCon1 > intervalo) {     // si pasó mas tiempo que el intervalo desde el último bit recibido, descarto todo
    bit_count1=26;
    rec_data1=0;
  }

  bit_count1--;
  if (digitalRead(L1DT)==1)
  { bitSet(rec_data1,bit_count1);
  } else {
    bitClear(rec_data1,bit_count1);
  }

  lastCon1 = millis();

  if (bit_count1==0)
  {
    // Si llegué hasta acá es porque recibí los 26 bits
    // control de errores por bit de paridad
    error1 = false;

    // reviso primer bit de paridad
    for (i=24,suma1=0;i>12;i--)
    {
      if (bitRead(rec_data1,i)==1) { suma1++; }
    }
    suma1 = suma1%2;
    if (suma1!=bitRead(rec_data1,25)) {error1 = true; }

    // reviso segundo bit de paridad
    for (i=12,suma1=0;i>0;i--)
    {
      if (bitRead(rec_data1,i)==1){ suma1++; }
    }
    suma1 = suma1%2;
    if (suma1==bitRead(rec_data1,0)) {error1 = true;}

    if (error1==false)
    {
      bitClear(rec_data1,25);
      RFID1 = rec_data1>>1;

    } else {
      // error de datos en primer bit de paridad
      RFID1=0;
    }
  }
}

void isr_opto0()//
{
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(OPTO0));
  if (trigger == RISING)
  {

    if ((accessIN == true) || ((accessOUT == true) && (digitalRead(OPTO1)==1)))
    {
      digitalWrite(OUT0,LOW);   // me aseguro que esté apagado
    } else {
      if (accessOUT_BLOQ == true)
      {
        digitalWrite(OUT0,HIGH);    // prende el solenoide
        delay(dem_PWM);
        analogWrite(OUT0,PWM2);    // prende el solenoide
      } else {
        digitalWrite(OUT0,LOW);   // me aseguro que esté apagado
      }
    }
    /*
       if (accessIN == false)    // NO PUEDE ENTRAR
       {
       digitalWrite(OUT0,HIGH);    // prende el solenoide
       if (debugger == true) { Serial.println(" Bloqueo Molinete (OPTO0)"); }
       delay(dem_PWM);
       analogWrite(OUT0,PWM2);    // prende el solenoide
       } else {
       digitalWrite(OUT0,LOW);   // me aseguro que esté apagado
       }
     */
  } else if (trigger == FALLING) {
    if (manual==false)
    {
      accessIN = false;       // ya pasó, bloqueo el molinete.
      digitalWrite(OUT0,LOW); //se apaga el sole
      digitalWrite(LED_IN,LOW);
      digitalWrite(LED_NP,HIGH);
    }
  }
}

void isr_opto1()//EXIT
{
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(OPTO1));
  if (trigger == RISING)
  {
    if ((accessOUT == true) || ((accessIN == true) && (digitalRead(OPTO0)==1)))
    {
      digitalWrite(OUT1,LOW);   // me aseguro que esté apagado
    } else {
      if (accessIN_BLOQ == true)
      {
        digitalWrite(OUT1,HIGH);    // prende el solenoide
        delay(dem_PWM);
        analogWrite(OUT1,PWM2);    // prende el solenoide
      } else {
        digitalWrite(OUT1,LOW);   // me aseguro que esté apagado
      }
    }
    /*
       if (accessOUT == false)    // NO PUEDE SALIR
       {
       digitalWrite(OUT1,HIGH);    // prende el solenoide
       if (debugger == true) { Serial.println(" Bloqueo Molinete (OPTO1)"); }
       delay(dem_PWM);
       analogWrite(OUT1,PWM2);    // prende el solenoide

       } else {
       digitalWrite(OUT1,LOW);   // me aseguro que esté apagado
       }*/

  } else if (trigger == FALLING) {
    if (manual==false)
    {
      accessOUT = false;       // ya pasó, bloqueo el molinete.
      digitalWrite(OUT1,LOW); //se apaga el sole
      digitalWrite(LED_OUT,LOW);
      digitalWrite(LED_NP,HIGH);
    }
  }
}

void blink_led()
{
  byte seq[3][8] = {
    {1,1,1,1,0,0,0,0},    // todo OK
    {1,0,1,0,1,0,1,0},    // error recibiendo desde el servidor.
    {1,1,0,0,0,0,0,0}
  };

  if (seq[blink_status][blink_step] == 1)
  {
    digitalWrite(STA_LED,HIGH);
  } else {
    digitalWrite(STA_LED,LOW);
  }

  blink_step++;
  if (blink_step== 8) blink_step = 0;

  last_blink = millis();
}
