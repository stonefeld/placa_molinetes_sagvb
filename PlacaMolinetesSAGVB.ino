/* ========================================================================= *
 * File: PlacaMolinetesSAGVB.ino                                             *
 * Authors: stonefeld, LucasAffre                                            *
 * Git: https://github.com/stonefeld/placa_molinetes_sagvb.git               *
 * Last Updated: 2022/12/23                                                  *
 * ========================================================================= */

#include <EEPROM.h>
#include <Ethernet.h>
#include <PinChangeInterrupt.h>
#include <SPI.h>
#include <avr/wdt.h>

// comentar para compilar para el molinete 2
#define MOLINETE_1

// defino los pines
// DT: data, CK: clock, BUZ: buzzer
// OPTO: optoacoplador, MOL: solenoides que activan los molinetes
#define WG_IN_DT     4
#define WG_IN_CK     2
#define WG_IN_BUZ    9
#define WG_OUT_DT    A0
#define WG_OUT_CK    3
#define WG_OUT_BUZ   A4
#define OPTO_IN      A3
#define OPTO_OUT     A2
#define MOL_IN       5
#define MOL_OUT      6
#define LED_IN       A1    // puede ingresar
#define LED_OUT      7     // puede egresar
#define LED_NP       8     // no puede pasar
#define LED_STAT     A5    // status del equipo

// respuestas del server
#define SRV_OK       '1'   // puede ingresar
#define SRV_NO_EX    '0'   // no existe el usuario
#define SRV_NO_PAG   '2'   // socio moroso
#define SRV_ERR      'E'   // error en la comunicacion con el servidor

// respuestas del socket
#define SOC_AP       'A'   // abrir molinete
#define SOC_CER      'C'   // cerrar molinete
#define SOC_TEM      'T'   // abrir durante `TEM_AP_MAN`ms
#define SOC_IN_LIB   'I'   // entrada libre
#define SOC_IN_CON   'i'   // entrada controlada
#define SOC_OUT_LIB  'S'   // salida libre
#define SOC_OUT_CON  's'   // salida controlada
#define SOC_ERR      'E'   // error en la comunicacion con el cliente del socket
#define SOC_NO_RES   0     // no habia cliente en el socket

// tiempos de timeout
#define SRV_TRIES    3     // veces que se reintenta hacer el request
#define TIMES_TEM    10    // veces que se realiza el delay para timeout
#define TEM_AP_MAN   5000  // tiempo de apertura manual
#define INTER_RFID   100   // intervalo entre lecturas de los wiegand
#define INTER_SOC    1000  // intervalo entre comunicaciones en el socket
#define INTER_BLINK  250   // intervalo entre blink

// manejo del PWM
#define TEM_PWM      1200  // demora del PWM
#define PWM          120

// estados de blinking del `LED_STAT`
#define BLINK_OK     0     // todo esta ok
#define BLINK_ERR    1     // ocurrio un error

// variables de ethernet
IPAddress dns(192, 168, 49, 1);
#ifdef MOLINETE_1
String agent = "User-Agent: 9 Julio - Molinete 1";
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xE0 };
IPAddress ip(192, 168, 49, 31);
#else
String agent = "User-Agent: 9 Julio - Molinete 2";
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xE1 };
IPAddress ip(192, 168. 49, 32);
#endif

// variables del server
byte srvIp[] = { 192, 168, 49, 30 };
String srvHost = "Host: 192.168.49.30";
int srvPort = 80;
EthernetClient srvClient;

// variables del socket
int socPort = 5050;
unsigned long socLastCon = 0;
EthernetServer socket = EthernetServer(socPort);

// respuesta del servidor o del socket
char response;

// permite imprimir mensajes de debugueo
bool debugger = false;

// variables de manejo de acceso
bool accessIn = false;
bool accessOut = false;
bool accessInLib = true;   // determina si la entrada esta controlado
bool accessOutLib = true;  // determina si la salida esta controlado
bool manual = false;       // determina si el acceso manual esta siendo usado

// variables del lector de entrada
boolean errorIn = 0;
byte bitCountIn = 26;
unsigned long recDataIn = 0;
unsigned long lastReadIn = 0;
unsigned long tagIn = 0;

// variables del lector de salida
boolean errorOut = 0;
byte bitCountOut = 26;
unsigned long recDataOut = 0;
unsigned long lastReadOut = 0;
unsigned long tagOut = 0;

// variables para el blink del `LED_STAT`
byte blinkStatus = BLINK_OK;
byte blinkStep = 0;
unsigned long lastBlink = 0;

void setup() {
  // deshabilito el watch-dog-timer
  wdt_disable();

  // led de estado
  pinMode(LED_STAT, OUTPUT);
  digitalWrite(LED_STAT, HIGH);

  // pines de optoacopladores
  pinMode(OPTO_IN, INPUT_PULLUP);
  pinMode(OPTO_OUT, INPUT_PULLUP);

  // espero que se estabilice la conexion
  delay(2000);

  // para activar el modo debug hay que activar algun optoacoplador al
  // reiniciar el arduino
  if (digitalRead(OPTO_IN) || digitalRead(OPTO_OUT)) {
    debugger = true;

    // inicializamos el serial port
    Serial.begin(9600);
    while (!Serial);

    Serial.println("==== DEBUGGER ON ====");
  }

  // me conecto a la red
  ethernetConnect();

  // configuro los pines del lector de entrada
  pinMode(WG_IN_DT, INPUT_PULLUP);
  pinMode(WG_IN_CK, INPUT_PULLUP);
  pinMode(WG_IN_BUZ, OUTPUT);

  // configuro los pines del lector de salida
  pinMode(WG_OUT_DT, INPUT_PULLUP);
  pinMode(WG_OUT_CK, INPUT_PULLUP);
  pinMode(WG_OUT_BUZ, OUTPUT);

  // apago el buzzer por si acaso
  pinMode(WG_IN_BUZ, LOW);
  pinMode(WG_OUT_BUZ, LOW);

  // agrego una interrupcion para los lectores
  attachInterrupt(digitalPinToInterrupt(WG_IN_CK), readRfidIn, FALLING);
  attachInterrupt(digitalPinToInterrupt(WG_OUT_CK), readRfidOut, FALLING);

  // configuro los pines de los solenoides
  pinMode(MOL_IN, OUTPUT);
  pinMode(MOL_OUT, OUTPUT);

  // apago los solenoides por si acaso
  digitalWrite(MOL_IN, LOW);
  digitalWrite(MOL_OUT, LOW);

  // agrego una interrupcion para los optoacopladores
  attachPCINT(digitalPinToPCINT(OPTO_IN), optoIn, FALLING);
  attachPCINT(digitalPinToPCINT(OPTO_OUT), optoOut, FALLING);

  // configuro los pines de los leds
  pinMode(LED_IN, OUTPUT);
  pinMode(LED_OUT, OUTPUT);
  pinMode(LED_NP, OUTPUT);

  // pongo la configuracion de leds
  digitalWrite(LED_IN, LOW);
  digitalWrite(LED_OUT, LOW);
  digitalWrite(LED_NP, HIGH);

  // verificamos el control de la entrada y la salida
  accessInLib = (char)EEPROM.read(0) == SOC_IN_LIB ? true : false;
  accessOutLib = (char)EEPROM.read(1) == SOC_OUT_LIB ? true : false;
  if (debugger) {
    Serial.println("Entrada esta " + accessInLib ? "controlada" : "liberada");
    Serial.println("Salida esta " + accessOutLib ? "controlada" : "liberada");
  }

  // habilito el watch-dog-timer
  wdt_enable(WDTO_8S);
  digitalWrite(LED_STAT, LOW);

  // cargo la ultima vez que se realizo el blink
  lastBlink = millis();
}

void loop() {
  // reseteo el watch-dog-timer
  wdt_reset();

  // TODO: revisar si es necesario lo del blinkLed
  if (millis() - lastBlink > INTER_BLINK)
    blinkLed();

  // si se supero el intervalo desde el ultimo bit recibido reseteo todo
  if (millis() - socLastCon > INTER_SOC) {
    socLastCon = millis();
    response = readSocket();

    if (response != SOC_NO_RES) {
      if (debugger)
        Serial.println("Mensaje recibido por socket");

      switch (response) {
        case SOC_AP: {
          if (debugger)
            Serial.println("> Entrada y la salida abiertas");

          accessIn = true;
          accessOut = true;
          manual = true;

          digitalWrite(LED_IN, HIGH);
          digitalWrite(LED_OUT, HIGH);
          digitalWrite(LED_NP, LOW);

          digitalWrite(WG_IN_BUZ, LOW);
          digitalWrite(WG_OUT_BUZ, LOW);
        } break;

        case SOC_CER: {
          if (debugger)
            Serial.println("> Entrada y la salida cerradas");

          accessIn = false;
          accessOut = false;
          manual = false;

          digitalWrite(LED_IN, LOW);
          digitalWrite(LED_OUT, LOW);
          digitalWrite(LED_NP, HIGH);

          digitalWrite(WG_IN_BUZ, LOW);
          digitalWrite(WG_OUT_BUZ, LOW);

          tagIn = 0;
          tagOut = 0;
        } break;

        case SOC_TEM: {
          if (debugger)
            Serial.println("> Entrada y la salida abiertas temporalmente");

          accessIn = true;
          accessOut = true;
          manual = true;

          digitalWrite(LED_IN, HIGH);
          digitalWrite(LED_OUT, HIGH);
          digitalWrite(LED_NP, LOW);

          digitalWrite(WG_IN_BUZ, HIGH);
          digitalWrite(WG_OUT_BUZ, HIGH);

          wdt_reset();
          delay(TEM_AP_MAN);
          wdt_reset();

          accessIn = false;
          accessOut = false;
          manual = false;

          digitalWrite(LED_IN, LOW);
          digitalWrite(LED_OUT, LOW);
          digitalWrite(LED_NP, HIGH);

          digitalWrite(WG_IN_BUZ, LOW);
          digitalWrite(WG_OUT_BUZ, LOW);

          tagIn = 0;
          tagOut = 0;
        } break;

        case SOC_IN_LIB: {
          if (debugger)
            Serial.println("> Entrada liberada");

          accessInLib = true;
          EEPROM.update(0, SOC_IN_LIB);
        } break;

        case SOC_IN_CON: {
          if (debugger)
            Serial.println("> Entrada controlada");

          accessInLib = false;
          EEPROM.update(0, SOC_IN_CON);
        } break;

        case SOC_OUT_LIB: {
          if (debugger)
            Serial.println("> Salida liberada");

          accessOutLib = true;
          EEPROM.update(1, SOC_OUT_LIB);
        } break;

        case SOC_OUT_CON: {
          if (debugger)
            Serial.println("> Salida controlada");

          accessOutLib = false;
          EEPROM.update(1, SOC_OUT_CON);
        } break;

        case SOC_ERR: {
          if (debugger)
            Serial.println("> Ocurrio error en comunicacion");

          blinkStatus = BLINK_ERR;
        };
      }
    }
    socLastCon = millis();
  }

  // reseteo el watch-dog-timer
  wdt_reset();

  // verifico si alguna tarjeta fue registrada en la entrada
  if (tagIn != 0) {
    response = serverRequest(String(tagIn, DEC), '1');
    if (debugger)
      Serial.println("Realizado request al servidor");

    switch (response) {
      case SRV_OK: {
        if (debugger)
          Serial.println("> Socio puede entrar");

        blinkStatus = BLINK_OK;
        accessIn = true;

        digitalWrite(LED_IN, HIGH);
        digitalWrite(LED_NP, LOW);

        digitalWrite(WG_IN_BUZ, HIGH);

        wdt_reset();

        // mientras todavia tenga acceso y el tiempo de timeout no se haya
        // cumplido
        for (byte i = 0; i < TIMES_TEM && accessIn; i++)
          delay(TEM_AP_MAN / TIMES_TEM);

        wdt_reset();

        accessIn = false;

        digitalWrite(LED_IN, LOW);
        digitalWrite(LED_NP, HIGH);

        digitalWrite(WG_IN_BUZ, LOW);
      } break;

      case SRV_ERR: {
        if (debugger)
          Serial.println("> Ocurro un error. Abriendo entrada por tiemout");

        blinkStatus = BLINK_ERR;
        accessIn = true;

        digitalWrite(LED_IN, HIGH);
        digitalWrite(LED_NP, LOW);

        digitalWrite(WG_IN_BUZ, HIGH);

        wdt_reset();

        // mientras todavia tenga acceso y el tiempo de timeout no se haya
        // cumplido
        for (byte i = 0; i < TIMES_TEM && accessIn; i++) {
          delay(500);
          digitalWrite(WG_IN_BUZ, !digitalRead(WG_IN_BUZ));
        }

        wdt_reset();

        accessIn = false;

        digitalWrite(LED_IN, LOW);
        digitalWrite(LED_NP, HIGH);

        digitalWrite(WG_IN_BUZ, LOW);
      } break;

      default: {
        if (debugger)
          Serial.println("> Socio no existe o tiene deuda. No puede entrar");

        accessIn = false;

        digitalWrite(WG_IN_BUZ, HIGH);
        delay(300);
        digitalWrite(WG_IN_BUZ, LOW);
      } break;
    }

    // como ya la procese borro la tarjeta leida
    tagIn = 0;
  }

  // reseteo el watch-dog-timer
  wdt_reset();

  // verifico si alguna tarjeta fue registrada en la salida
  if (tagOut != 0) {
    response = serverRequest(String(tagOut, DEC), '0');
    if (debugger)
      Serial.println("Realizado request al servidor");

    switch (response) {
      case SRV_OK: {
        if (debugger)
          Serial.println("> Socio puede salir");

        blinkStatus = BLINK_OK;
        accessOut = true;

        digitalWrite(LED_OUT, HIGH);
        digitalWrite(LED_NP, LOW);

        digitalWrite(WG_OUT_BUZ, HIGH);

        wdt_reset();

        // mientras todavia tenga acceso y el tiempo de timeout no se haya
        // cumplido
        for (byte i = 0; i < TIMES_TEM && accessOut; i++)
          delay(500);

        wdt_reset();

        accessOut = false;

        digitalWrite(LED_OUT, LOW);
        digitalWrite(LED_NP, HIGH);

        digitalWrite(WG_OUT_BUZ, LOW);
      } break;

      case SRV_ERR: {
        if (debugger)
          Serial.println("> Ocurro un error. Abriendo salida por tiemout");

        blinkStatus = BLINK_ERR;
        accessOut = true;

        digitalWrite(LED_OUT, HIGH);
        digitalWrite(LED_NP, LOW);

        digitalWrite(WG_OUT_BUZ, HIGH);

        wdt_reset();

        // mientras todavia tenga acceso y el tiempo de timeout no se haya
        // cumplido
        for (byte i = 0; i < TIMES_TEM && accessOut; i++) {
          delay(500);
          digitalWrite(WG_OUT_BUZ, !digitalRead(WG_OUT_BUZ));
        }

        wdt_reset();

        accessOut = false;

        digitalWrite(LED_OUT, LOW);
        digitalWrite(LED_NP, HIGH);

        digitalWrite(WG_OUT_BUZ, LOW);
      } break;

      default: {
        if (debugger)
          Serial.println("> Socio no existe o tiene deuda. No puede salir");

        accessOut = false;

        digitalWrite(WG_OUT_BUZ, HIGH);
        delay(300);
        digitalWrite(WG_OUT_BUZ, LOW);
      } break;
    }

    // como ya la procese borro la tarjeta leida
    tagOut = 0;
  }
}

void ethernetConnect() {
  Ethernet.init(10);

  if (debugger) {
    Serial.println("Inicializando la red");
    Serial.println("> Conectando DHCP...");
    Serial.print("> MAC: ");
    for (byte i = 0; i < 6; i++) {
      Serial.print(mac[i], HEX);
      if (i != 5)
        Serial.print(":");
    }
    Serial.println(agent);
  }

  if (Ethernet.begin(mac) == 0) {
    if (debugger)
      Serial.println("> Fallo conexion por DHCP");

    // verifico la existencia de una placa de red
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      if (debugger)
        Serial.println("> No se detecta placa de red. Programa no continuara");

      // sin placa de red no tiene sentido que el programa continue
      while (true)
        delay(1);
    }

    // verifico que el cable ethernet este conectado
    if (debugger && Ethernet.linkStatus() == LinkOFF)
      Serial.println("> Cable ethernet no conectado");

    // conecto por IP estatica
    if (debugger)
      Serial.println("> Conectando con IP estatica...");

    Ethernet.begin(mac, ip, dns);
  }

  if (debugger) {
    Serial.print("> IP: ");
    Serial.println(Ethernet.localIP());
  }

  // le doy tiempo a la placa de red para inicializar
  delay(1000);

  // inicializo tambien el socket
  socket.begin();
  if (debugger) {
    Serial.print("> Socket abierto en: ");
    Serial.print(Ethernet.localIP());
    Serial.print(":");
    Serial.println(socPort);
  }

  // le doy tiempo al socket para inicializar
  delay(1000);
}

char readSocket() {
  bool numeralFound = false;
  char ret = SOC_NO_RES, c;

  // buscamos un cliente conectado al socket
  EthernetClient socClient = socket.available();

  // si el cliente no esta conectado no hubo respuesta
  if (socClient) {
    while (socClient.connected()) {
      // ante la duda reseteo el watch-dog-timer
      wdt_reset();

      if (socClient.available()) {
        // leo el siguiente caracter del mensaje
        c = socClient.read();

        // el mensaje se compone de #<c> donde `c` es el comando para manejar
        // los molinetes
        if (c == '#') {
          numeralFound = true;
        } else if (numeralFound) {
          ret = c;
          socClient.stop();
        }
      }
      socClient.stop();

      // si se llego hasta este punto y `ret` no cambio, ocurrio algun error
      if (ret == SOC_NO_RES)
        ret = SOC_ERR;
    }
  }
  return ret;
}

char serverRequest(String tagId, char direccion) {
  bool numeralFound = false;
  char ret = SRV_ERR, c;

  // intento enviar el request `SRV_TRIES` veces
  for (byte i = 0; i < SRV_TRIES; i++) {
    // TODO: verificar si el flush es necesario
#if 0
    while (client.available())
      client.read();
#endif

    // cerramos cualquier conexion previa
    srvClient.stop();

    // me conecto al servidor
    if (srvClient.connect(srvIp, srvPort)) {
      // envio el GET request con la informacion solicitada
      String reqMessage = String("GET /general/?nrTarjeta=" + tagId + "&direccion=" + direccion + " HTTP/1.1");
      srvClient.println(reqMessage);
      srvClient.println(srvHost);
      srvClient.println(agent);
      srvClient.println("Connection: close");
      srvClient.println();

      // TODO: verificar si este while es necesario
#if 0
      while (client.available() == 0) {
        wdt_reset();
      }
#endif

      // proceso el resultado de la response
      while (srvClient.available()) {
        // ante la duda reseteo el watch-dog-timer
        wdt_reset();

        // leo el siguiente caracter del mensaje
        c = srvClient.read();

        // el mensaje se compone de #<c> donde `c` es el estado del socio:
        // 0: el usuario no existe, no debe ingresar
        // 1: puede ingresar, todo ok
        // 2: es un socio moroso, no debe ingresar
        if (c == '#') {
          numeralFound = true;
        } else if (numeralFound) {
          ret = c;
          srvClient.stop();
        }
      }
    } else {
      // si no se pudo establecer la conexion
      delay(200);
    }
  }
  return ret;
}

void readRfidIn() {
  byte sumaIn = 0;

  // si paso mas tiempo que el intervalo desde el ultimo bit, descarto todo
  if (millis() - lastReadIn > INTER_RFID) {
    bitCountIn = 26;
    recDataIn = 0;
  }

  bitCountIn--;
  if (digitalRead(WG_IN_DT) == 1)
    bitSet(recDataIn, bitCountIn);
  else
    bitClear(recDataIn, bitCountIn);

  lastReadIn = millis();

  if (bitCountIn == 0) {
    // si llego hasta aca es porque recibi los 26 bits
    errorIn = false;

    // reviso primer bit de paridad
    for (byte i = 24, sumaIn = 0; i > 12; i--)
      if (bitRead(recDataIn, i) == 1)
        sumaIn++;

    sumaIn = sumaIn % 2;
    if (sumaIn != bitRead(recDataIn, 25))
      errorIn = true;

    // reviso segundo bit de paridad
    for (byte i = 12, sumaIn = 0; i > 0; i--)
      if (bitRead(recDataIn, i) == 1)
        sumaIn++;

    sumaIn = sumaIn % 2;
    if (sumaIn != bitRead(recDataIn, 0))
      errorIn = true;

    if (!errorIn) {
      bitClear(recDataIn, 25);
      tagIn = recDataIn >> 1;
    } else {
      // ocurrio algun error en los bits de paridad
      tagIn = 0;
    }
  }
}

void readRfidOut() {
  byte sumaOut = 0;

  // si paso mas tiempo que el intervalo desde el ultimo bit, descarto todo
  if (millis() - lastReadOut > INTER_RFID) {
    bitCountOut = 26;
    recDataOut = 0;
  }

  bitCountOut--;
  if (digitalRead(WG_OUT_DT) == 1)
    bitSet(recDataOut, bitCountOut);
  else
    bitClear(recDataOut, bitCountOut);

  lastReadOut = millis();

  if (bitCountOut == 0) {
    // si llego hasta aca es porque recibi los 26 bits
    errorOut = false;

    // reviso primer bit de paridad
    for (byte i = 24, sumaOut = 0; i > 12; i--)
      if (bitRead(recDataOut, i) == 1)
        sumaOut++;

    sumaOut = sumaOut % 2;
    if (sumaOut != bitRead(recDataOut, 25))
      errorOut = true;

    // reviso segundo bit de paridad
    for (byte i = 12, sumaOut = 0; i > 0; i--)
      if (bitRead(recDataOut, i) == 1)
        sumaOut++;

    sumaOut = sumaOut % 2;
    if (sumaOut != bitRead(recDataOut, 0))
      errorOut = true;

    if (!errorOut) {
      bitClear(recDataOut, 25);
      tagOut = recDataOut >> 1;
    } else {
      // ocurrio algun error en los bits de paridad
      tagOut = 0;
    }
  }
}

void optoIn() {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(OPTO_IN));
  if (trigger == RISING) {
    if (accessIn || (accessOut && digitalRead(OPTO_OUT) == 1)) {
      // me aseguro de que este abierto
      digitalWrite(MOL_IN, LOW);
    } else {
      if (!accessOutLib) {
        // cierro el molinete
        digitalWrite(MOL_IN, HIGH);
        delay(TEM_PWM);
        analogWrite(MOL_IN, PWM);
      } else {
        // me aseguro de que este abierto
        digitalWrite(MOL_IN, LOW);
      }
    }
  } else if (trigger == FALLING && !manual) {
    // al ya haber pasado vuelvo a bloquear el molinete
    accessIn = false;

    // libero el molinete
    digitalWrite(MOL_IN, LOW);

    digitalWrite(LED_IN, LOW);
    digitalWrite(LED_NP, HIGH);
  }
}

void optoOut() {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(OPTO_OUT));
  if (trigger == RISING) {
    if (accessOut || (accessIn && digitalRead(OPTO_IN) == 1)) {
      // me aseguro de que este abierto
      digitalWrite(MOL_OUT, LOW);
    } else {
      if (!accessInLib) {
        // cierro el molinete
        digitalWrite(MOL_OUT, HIGH);
        delay(TEM_PWM);
        analogWrite(MOL_OUT, PWM);
      } else {
        // me aseguro de que este abierto
        digitalWrite(MOL_OUT, LOW);
      }
    }
  } else if (trigger == FALLING && !manual) {
    // al ya haber pasado vuelvo a bloquear el molinete
    accessOut = false;

    // libero el molinete
    digitalWrite(MOL_OUT, LOW);

    digitalWrite(LED_OUT, LOW);
    digitalWrite(LED_NP, HIGH);
  }
}

void blinkLed() {
  byte seq[][8] = {
    { 1, 1, 1, 1, 0, 0, 0, 0 },  // esta todo ok
    { 1, 0, 1, 0, 1, 0, 1, 0 }   // error al recibir desde el server
  };

  digitalWrite(LED_STAT, seq[blinkStatus][blinkStep++] ? HIGH : LOW);
  if (blinkStep >= 8)
    blinkStep = 0;

  lastBlink = millis();
}
