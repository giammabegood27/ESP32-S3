#include <NimBLEDevice.h>
#include "RTC.h"
#include <EEPROM.h>

// Fallback minimale per ambienti (es. copia temporanea dell'IDE) che non
// includono il file RTC.h: viene definito solo se l'header non è stato
// fornito dal progetto (guardato tramite il define usato in RTC.h).
#ifndef SIGNOIRRIGA_RTC_H
enum class SaveLight { SAVING_TIME_INACTIVE = 0, SAVING_TIME_ACTIVE = 1 };
enum class Month { JANUARY = 1 };
enum DayOfWeek { SUNDAY = 1, MONDAY, TUESDAY, WEDNESDAY, THURSDAY, FRIDAY, SATURDAY };

class RTCTime {
public:
  RTCTime() : dayOfMonth(1), month(Month::JANUARY), year(2026), hour(0), minute(0), second(0), dayOfWeek(MONDAY), saveLight(SaveLight::SAVING_TIME_INACTIVE) {}
  RTCTime(uint8_t d, Month m, int y, uint8_t h, uint8_t min, uint8_t s, DayOfWeek dow, SaveLight sl)
    : dayOfMonth(d), month(m), year(y), hour(h), minute(min), second(s), dayOfWeek(dow), saveLight(sl) {}
  int getDayOfWeek() const { return (int)dayOfWeek; }
  int getHour() const { return hour; }
  int getMinutes() const { return minute; }

  uint8_t dayOfMonth;
  Month month;
  int year;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  DayOfWeek dayOfWeek;
  SaveLight saveLight;
};

class RTCClass {
public:
  RTCClass() : currentTime(), lastMillis(0) {}

  void begin() {
    lastMillis = millis();
  }

  void getTime(RTCTime &t) {
    updateTime();
    t = currentTime;
  }

  void setTime(const RTCTime &t) {
    currentTime = t;
    lastMillis = millis();
  }

private:
  RTCTime currentTime;
  unsigned long lastMillis;

  void updateTime() {
    unsigned long now = millis();
    unsigned long deltaMs = now - lastMillis;
    if (deltaMs < 1000) return;

    unsigned long secondsToAdvance = deltaMs / 1000;
    lastMillis += secondsToAdvance * 1000;

    while (secondsToAdvance > 0) {
      secondsToAdvance--;
      currentTime = advanceOneSecond(currentTime);
    }
  }

  RTCTime advanceOneSecond(const RTCTime &t) {
    RTCTime next = t;
    next.second += 1;
    if (next.second >= 60) {
      next.second = 0;
      next.minute += 1;
      if (next.minute >= 60) {
        next.minute = 0;
        next.hour += 1;
        if (next.hour >= 24) {
          next.hour = 0;
          next.dayOfWeek = (DayOfWeek)(next.dayOfWeek == SATURDAY ? SUNDAY : (next.dayOfWeek + 1));
          next.dayOfMonth = (next.dayOfMonth % 31) + 1;
        }
      }
    }
    return next;
  }
};

RTCClass RTC;
#endif

// Definizione Servizio e Caratteristiche BLE
static const char* SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214";
static const char* CHAR_RX_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214";
static const char* CHAR_TX_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214";

NimBLEServer* pServer = nullptr;
NimBLEService* pService = nullptr;
NimBLECharacteristic* pRxCharacteristic = nullptr;
NimBLECharacteristic* pTxCharacteristic = nullptr;

volatile bool rxWritten = false;
String rxValue = "";

// Callback BLE: memorizza il comando ricevuto dal client per l'elaborazione nel loop principale.
class RXCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string v = pChar->getValue();
    rxValue = String(v.c_str());
    rxWritten = true;
  }
};

void startBLEAdvertising();
void spegniZoneSeNonInCiclo();

// Callback BLE: gestisce connessione/disconnessione del client, riavviando l'advertising alla disconnessione.
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    (void)pServer;
    (void)connInfo;
    Serial.println("BLE client connesso");
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    (void)pServer;
    (void)connInfo;
    (void)reason;
    Serial.println("BLE client disconnesso, riavvio advertising...");
    spegniZoneSeNonInCiclo();
    startBLEAdvertising();
  }
};

// Avvia l'advertising BLE con il nome del dispositivo, rendendolo visibile ai client (es. app HMI).
void startBLEAdvertising() {
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setName("SIGNORETTI_Garden_esp");
  pAdvertising->addServiceUUID(SERVICE_UUID);

  if (!pAdvertising->start()) {
    Serial.println("Errore avvio advertising BLE");
  } else {
    Serial.println("BLE advertising avviato");
    Serial.println("Nome BLE pubblicizzato: SIGNORETTI_Garden_esp");
  }
}

// Mappatura Indirizzi EEPROM
// Le durate ora occupano 2 byte l'una (uint16_t) invece di 1, per non troncare
// valori superiori a 255 secondi (fino a 1800s supportati lato HMI).
const int ADDR_FIRMA = 0;         // 2 byte (0-1)
const int ADDR_ORA = 2;           // 1 byte
const int ADDR_MIN = 3;           // 1 byte
const int ADDR_DURATE = 4;        // 12 byte (4-15): 6 zone x 2 byte - Start 1
const int ADDR_GIORNI = 16;       // 1 byte - Giorni Start 1
const int ADDR_ORA_2 = 17;        // 1 byte
const int ADDR_MIN_2 = 18;        // 1 byte
const int ADDR_ABILITA_1 = 19;    // 1 byte
const int ADDR_ABILITA_2 = 20;    // 1 byte
const int ADDR_GIORNI_2 = 21;     // 1 byte - Giorni Start 2
const int ADDR_DURATE_2 = 22;     // 12 byte (22-33): 6 zone x 2 byte - Start 2

// Firma cambiata per forzare una riscrittura pulita: il layout EEPROM è cambiato.
const uint16_t FIRMA_EEPROM = 0xABD3;

const int pinZone[] = {1, 2, 4, 5, 6, 7};
const int numeroZone = 6;
const int DURATA_ZONA_TEST = 10; // secondi fissi per zona in modalità test
const unsigned long STOP_TRA_ZONE_MS = 3000; // pausa tra una zona e la successiva

int oraStart = 6;
int minutoStart = 0;
bool abilitaStart1 = true;      

int oraStart2 = 20;
int minutoStart2 = 0;
bool abilitaStart2 = false;     

int durataZone[6] = {10, 10, 10, 10, 10, 10}; 
int durataZone2[6] = {5, 5, 5, 5, 5, 5};        

bool giorniSettimana[7] = {false, false, false, false, false, false, false}; 
bool giorniSettimana2[7] = {false, false, false, false, false, false, false};

bool cicloAutomaticoAttivo = false;
bool inCicloTest = false;
int startCorrenteAttivo = 1;                    

unsigned long timeInizioZonaCorrente = 0;
int zonaAttivaCorrente = -1; 
int giornoUltimoCiclo = -1;  
int startUltimoCiclo = -1;
unsigned long ultimoInvioStato = 0;
bool inStopTraZone = false;
unsigned long timeInizioStopTraZone = 0;
int prossimaZonaDaAvviare = -1;

// Accumulo dei secondi già trascorsi nelle zone completate del ciclo in corso.
// Usato per calcolare il progresso complessivo del ciclo (non solo della singola zona).
unsigned long secondiAccumulatiCiclo = 0;

// Spegne tutte le valvole, ma solo se non è in corso un ciclo automatico o di test
// (evita di interrompere un'irrigazione già avviata, es. alla disconnessione BLE).
void spegniZoneSeNonInCiclo() {
  if (!cicloAutomaticoAttivo && !inCicloTest) {
    for (int i = 0; i < numeroZone; i++) digitalWrite(pinZone[i], LOW);
  }
}

// Inizializzazione: seriale, RTC, pin delle zone, EEPROM (con caricamento impostazioni salvate)
// e avvio del server BLE (servizio + caratteristiche RX/TX + advertising).
void setup() {
  Serial.begin(9600);
  RTC.begin();
  EEPROM.begin(64);
  
  for (int i = 0; i < numeroZone; i++) {
    pinMode(pinZone[i], OUTPUT);
    digitalWrite(pinZone[i], LOW);
  }

  caricaImpostazioniEEPROM();
  // Init NimBLE for ESP32
  Serial.println("Inizializzazione BLE (NimBLE)...");
  NimBLEDevice::init("SIGNORETTI_Garden_esp");
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);

  pRxCharacteristic = pService->createCharacteristic(CHAR_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
  pRxCharacteristic->setCallbacks(new RXCallbacks());

  pTxCharacteristic = pService->createCharacteristic(CHAR_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  pTxCharacteristic->setValue("");

  pService->start();
  startBLEAdvertising();
}

// Ciclo principale: controlla l'avvio automatico dei cicli programmati, avanza la sequenza
// di irrigazione in corso, elabora eventuali comandi BLE ricevuti e invia periodicamente
// (ogni 2s, o subito dopo un comando) lo stato aggiornato al client via BLE.
void loop() {
  controlloAvvioAutomatico();
  gestisciAvanzamentoSequenza();

  if (rxWritten) {
    elaboraStringaComando(rxValue);
    rxWritten = false;
    rxValue = "";
  }

  if (millis() - ultimoInvioStato > 2000) {
    inviaStatoBLE();
    ultimoInvioStato = millis();
  }

  delay(10);
}

// Verifica se è il momento di avviare uno dei due cicli automatici programmati (Start1/Start2),
// confrontando ora/minuto correnti dell'RTC con quelli impostati e il giorno della settimana.
// Il controllo giornoUltimoCiclo/startUltimoCiclo impedisce che lo stesso start riparta più
// volte nello stesso giorno.
void controlloAvvioAutomatico() {
  if (cicloAutomaticoAttivo || inCicloTest) return;

  RTCTime oraAttuale;
  RTC.getTime(oraAttuale);
  int giornoRTC = (int)oraAttuale.getDayOfWeek();
  int giornoOggi = (giornoRTC >= 1 && giornoRTC <= 7) ? (giornoRTC - 1) : giornoRTC;
  
  if (giornoOggi < 0 || giornoOggi > 6) return;

  bool start1_attivo = abilitaStart1 && (oraAttuale.getHour() == oraStart && oraAttuale.getMinutes() == minutoStart) && giorniSettimana[giornoOggi];
  bool start2_attivo = abilitaStart2 && (oraAttuale.getHour() == oraStart2 && oraAttuale.getMinutes() == minutoStart2) && giorniSettimana2[giornoOggi];

  if (start1_attivo && (giornoUltimoCiclo != giornoOggi || startUltimoCiclo != 1)) {
    giornoUltimoCiclo = giornoOggi;
    startUltimoCiclo = 1;
    cicloAutomaticoAttivo = true;
    startCorrenteAttivo = 1;
    secondiAccumulatiCiclo = 0;
    iniziaProssimaZona(0); 
  } 
  else if (start2_attivo && (giornoUltimoCiclo != giornoOggi || startUltimoCiclo != 2)) {
    giornoUltimoCiclo = giornoOggi;
    startUltimoCiclo = 2;
    cicloAutomaticoAttivo = true;
    startCorrenteAttivo = 2;
    secondiAccumulatiCiclo = 0;
    iniziaProssimaZona(0); 
  }
}

// Avvia (apre la valvola della) prima zona con durata > 0 a partire dall'indice indicato.
// Se nessuna zona successiva ha una durata valida, il ciclo corrente (automatico o test) termina.
void iniziaProssimaZona(int daIndice) {
  zonaAttivaCorrente = -1;
  for (int i = daIndice; i < numeroZone; i++) {
    int durataValida = (startCorrenteAttivo == 2) ? durataZone2[i] : durataZone[i];
    if (durataValida > 0) {
      zonaAttivaCorrente = i;
      timeInizioZonaCorrente = millis();
      for (int z = 0; z < numeroZone; z++) digitalWrite(pinZone[z], LOW);
      digitalWrite(pinZone[zonaAttivaCorrente], HIGH);
      break;
    }
  }
  if (zonaAttivaCorrente == -1) {
    cicloAutomaticoAttivo = false;
    inCicloTest = false;
    secondiAccumulatiCiclo = 0;
    for (int i = 0; i < numeroZone; i++) digitalWrite(pinZone[i], LOW);
  }
}

// Gestisce l'avanzamento tra le zone del ciclo attivo: rileva quando la zona corrente ha
// esaurito la sua durata, la spegne, applica una pausa (STOP_TRA_ZONE_MS) e poi avvia la zona
// successiva tramite iniziaProssimaZona().
void gestisciAvanzamentoSequenza() {
  if (inStopTraZone) {
    if (millis() - timeInizioStopTraZone >= STOP_TRA_ZONE_MS) {
      inStopTraZone = false;
      if (prossimaZonaDaAvviare >= 0) {
        iniziaProssimaZona(prossimaZonaDaAvviare);
        prossimaZonaDaAvviare = -1;
      }
    }
    return;
  }

  if (zonaAttivaCorrente == -1) return;
  int dSec = inCicloTest ? DURATA_ZONA_TEST : ((startCorrenteAttivo == 2) ? durataZone2[zonaAttivaCorrente] : durataZone[zonaAttivaCorrente]);
  
  unsigned long durataAttesa = (unsigned long)dSec * 1000;

  if (millis() - timeInizioZonaCorrente >= durataAttesa) {
    // La zona corrente è terminata: la sua durata va sommata all'accumulo del ciclo
    secondiAccumulatiCiclo += dSec;

    // Spegni tutte le valvole e fai una sosta di 3 secondi prima della prossima apertura
    for (int i = 0; i < numeroZone; i++) digitalWrite(pinZone[i], LOW);

    int zonaSuccessiva = zonaAttivaCorrente + 1;
    zonaAttivaCorrente = -1;
    prossimaZonaDaAvviare = zonaSuccessiva;
    inStopTraZone = true;
    timeInizioStopTraZone = millis();
  }
}

// Somma delle durate delle 6 zone di uno dei due cicli automatici (1 o 2).
long calcolaTotaleCiclo(int numeroCiclo) {
  long totale = 0;
  for (int i = 0; i < numeroZone; i++) {
    totale += (numeroCiclo == 2) ? durataZone2[i] : durataZone[i];
  }
  return totale;
}

// Totale del ciclo di test: solo le zone con durata (Start1) > 0 vengono attivate,
// ciascuna per un tempo fisso di DURATA_ZONA_TEST secondi.
long calcolaTotaleTest() {
  long totale = 0;
  for (int i = 0; i < numeroZone; i++) {
    if (durataZone[i] > 0) totale += DURATA_ZONA_TEST;
  }
  return totale;
}

// 0 = nessun ciclo attivo (fermo o manuale), 1/2 = ciclo automatico Start1/Start2, 3 = test
int tipoCicloCorrente() {
  if (inCicloTest) return 3;
  if (cicloAutomaticoAttivo) return startCorrenteAttivo;
  return 0;
}

// Interpreta i comandi testuali ricevuti via BLE (STOP, TEST, MANUAL:<zona>, SAVE:<...>,
// RESETGUARD, TIME:<ora,min,giorno>) e aggiorna di conseguenza stato e impostazioni,
// notificando poi lo stato aggiornato al client.
void elaboraStringaComando(String comando) {
  if (comando.length() == 0) return;

  if (comando.startsWith("STOP")) {
    zonaAttivaCorrente = -1; cicloAutomaticoAttivo = false; inCicloTest = false;
    secondiAccumulatiCiclo = 0;
    inStopTraZone = false;
    prossimaZonaDaAvviare = -1;
    for (int i = 0; i < numeroZone; i++) digitalWrite(pinZone[i], LOW);
  } 
  else if (comando.startsWith("TEST")) {
    cicloAutomaticoAttivo = false; inCicloTest = true; startCorrenteAttivo = 1;
    secondiAccumulatiCiclo = 0;
    iniziaProssimaZona(0);
  } 
  else if (comando.startsWith("MANUAL:")) {
    int z = comando.substring(7).toInt();
    cicloAutomaticoAttivo = false; inCicloTest = false;
    zonaAttivaCorrente = -1;
    secondiAccumulatiCiclo = 0;
    
    // Spegni tutto prima di attivare la zona manuale
    for (int i = 0; i < numeroZone; i++) digitalWrite(pinZone[i], LOW);
    
    if (z > 0 && z <= numeroZone) {
      int indiceZona = z - 1;
      digitalWrite(pinZone[indiceZona], HIGH);
    }
  } 
  else if (comando.startsWith("SAVE:")) {
    int h1 = 0, m1 = 0, e1 = 0, h2 = 0, m2 = 0, e2 = 0; 
    int dur1[6] = {0}, dur2[6] = {0};
    int g1[7] = {0}, g2[7] = {0};
    
    int argomenti = sscanf(comando.c_str(), "SAVE:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                           &h1, &m1, &e1, &h2, &m2, &e2,
                           &dur1[0], &dur1[1], &dur1[2], &dur1[3], &dur1[4], &dur1[5],
                           &dur2[0], &dur2[1], &dur2[2], &dur2[3], &dur2[4], &dur2[5],
                           &g1[0], &g1[1], &g1[2], &g1[3], &g1[4], &g1[5], &g1[6],
                           &g2[0], &g2[1], &g2[2], &g2[3], &g2[4], &g2[5], &g2[6]);

    if (argomenti >= 32) { 
      oraStart = h1; minutoStart = m1; abilitaStart1 = (e1 == 1); 
      oraStart2 = h2; minutoStart2 = m2; abilitaStart2 = (e2 == 1);
      // Durate limitate a 0-1800 secondi, coerente con l'interfaccia web
      for (int i = 0; i < 6; i++) {
        durataZone[i] = constrain(dur1[i], 0, 1800);
        durataZone2[i] = constrain(dur2[i], 0, 1800);
      }
      for (int i = 0; i < 7; i++) { giorniSettimana[i] = (g1[i] == 1); giorniSettimana2[i] = (g2[i] == 1); }
      
      EEPROM.put(ADDR_FIRMA, FIRMA_EEPROM);
      EEPROM.write(ADDR_ORA, (byte)oraStart); EEPROM.write(ADDR_MIN, (byte)minutoStart);
      EEPROM.write(ADDR_ORA_2, (byte)oraStart2); EEPROM.write(ADDR_MIN_2, (byte)minutoStart2);
      EEPROM.write(ADDR_ABILITA_1, (byte)(abilitaStart1 ? 1 : 0)); EEPROM.write(ADDR_ABILITA_2, (byte)(abilitaStart2 ? 1 : 0));

      // Durate salvate come uint16_t (2 byte) invece di byte singolo, per non troncare valori > 255
      for (int i = 0; i < 6; i++) {
        uint16_t v1 = (uint16_t)durataZone[i];
        uint16_t v2 = (uint16_t)durataZone2[i];
        EEPROM.put(ADDR_DURATE + (i * 2), v1);
        EEPROM.put(ADDR_DURATE_2 + (i * 2), v2);
      }

      byte gb1 = 0, gb2 = 0;
      for(int g = 0; g < 7; g++) { if(giorniSettimana[g]) gb1 |= (1 << g); if(giorniSettimana2[g]) gb2 |= (1 << g); }
      EEPROM.write(ADDR_GIORNI, gb1); EEPROM.write(ADDR_GIORNI_2, gb2);
      EEPROM.commit();
      
      Serial.println("Configurazione EEPROM salvata correttamente.");
    }
  }
  else if (comando.startsWith("RESETGUARD")) {
    // Azzera il blocco "una volta al giorno" per Start 1 e Start 2, utile in fase di test
    // per far ripartire lo stesso start più volte nello stesso giorno senza riavviare l'Arduino.
    giornoUltimoCiclo = -1;
    startUltimoCiclo = -1;
  }
  else if (comando.startsWith("TIME:")) {
    int p1 = comando.indexOf(','), p2 = comando.indexOf(',', p1 + 1);
    if (p1 != -1 && p2 != -1) {
      int h = comando.substring(5, p1).toInt(), m = comando.substring(p1 + 1, p2).toInt(), g = comando.substring(p2 + 1).toInt(); 
      RTCTime tempoSincro(1, Month::JANUARY, 2026, h, m, 0, (DayOfWeek)((g == 0) ? 1 : (g + 1)), SaveLight::SAVING_TIME_INACTIVE);
      RTC.setTime(tempoSincro);
    }
  }

  // Invia subito lo stato aggiornato invece di aspettare il prossimo ciclo di 2s:
  // rende il popup di progresso (comparsa/scomparsa, cambio zona) quasi istantaneo lato HMI.
  inviaStatoBLE();
  ultimoInvioStato = millis();
}

// Costruisce un JSON con lo stato corrente (orari/giorni/durate configurati, ora RTC, zona
// attiva e progresso di zona/ciclo) e lo invia via notifica BLE al client (app HMI).
void inviaStatoBLE() {
  RTCTime oraAttuale; RTC.getTime(oraAttuale);
  String rtcTime = String(oraAttuale.getHour()) + ":" + (oraAttuale.getMinutes() < 10 ? "0" : "") + String(oraAttuale.getMinutes());

  int cicloTipo = tipoCicloCorrente();

  // Durata e secondi trascorsi nella zona corrente (per la barra di progresso "zona")
  long durZonaCorr = 0;
  long elapsedZonaCorr = 0;
  if (zonaAttivaCorrente >= 0) {
    durZonaCorr = inCicloTest ? DURATA_ZONA_TEST : ((startCorrenteAttivo == 2) ? durataZone2[zonaAttivaCorrente] : durataZone[zonaAttivaCorrente]);
    elapsedZonaCorr = (millis() - timeInizioZonaCorrente) / 1000;
    if (elapsedZonaCorr > durZonaCorr) elapsedZonaCorr = durZonaCorr;
  }

  // Secondi trascorsi nell'intero ciclo (il totale lo calcola il client dagli array d/d2 già inviati,
  // per tenere il pacchetto BLE più corto e sotto al limite del buffer)
  long elapsedCiclo = 0;
  if (cicloTipo > 0) {
    elapsedCiclo = secondiAccumulatiCiclo + elapsedZonaCorr;
  }

  String json = "{\"h\":" + String(oraStart) + ",\"m\":" + String(minutoStart) + ",\"en1\":" + String(abilitaStart1 ? "1" : "0") + 
                ",\"h2\":" + String(oraStart2) + ",\"m2\":" + String(minutoStart2) + ",\"en2\":" + String(abilitaStart2 ? "1" : "0") + ",\"d\":[";
  for(int i=0; i<6; i++) json += String(durataZone[i]) + (i < 5 ? "," : "");
  json += "],\"d2\":[";
  for(int i=0; i<6; i++) json += String(durataZone2[i]) + (i < 5 ? "," : "");
  json += "],\"g\":[";
  for(int g=0; g<7; g++) json += String(giorniSettimana[g] ? "1" : "0") + (g < 6 ? "," : "");
  json += "],\"g2\":[";
  for(int g=0; g<7; g++) json += String(giorniSettimana2[g] ? "1" : "0") + (g < 6 ? "," : "");
  json += "],\"rtc\":\"" + rtcTime + "\",\"z\":" + String(zonaAttivaCorrente + 1);
  json += ",\"ct\":" + String(cicloTipo);
  json += ",\"dz\":" + String(durZonaCorr);
  json += ",\"ez\":" + String(elapsedZonaCorr);
  json += ",\"ec\":" + String(elapsedCiclo);
  json += "}";
  
  if (pTxCharacteristic) {
    pTxCharacteristic->setValue(json.c_str());
    pTxCharacteristic->notify();
  }
}

// Carica le impostazioni salvate in EEPROM, solo se la firma letta corrisponde a FIRMA_EEPROM
// (altrimenti la EEPROM è vuota/di un layout diverso e restano i valori di default).
void caricaImpostazioniEEPROM() {
  uint16_t firmaLetta; EEPROM.get(ADDR_FIRMA, firmaLetta);
  if(firmaLetta == FIRMA_EEPROM) {
    oraStart = EEPROM.read(ADDR_ORA); minutoStart = EEPROM.read(ADDR_MIN);
    oraStart2 = EEPROM.read(ADDR_ORA_2); minutoStart2 = EEPROM.read(ADDR_MIN_2);
    abilitaStart1 = (EEPROM.read(ADDR_ABILITA_1) == 1); abilitaStart2 = (EEPROM.read(ADDR_ABILITA_2) == 1);

    // Durate lette come uint16_t (2 byte), coerente con il salvataggio
    for(int i = 0; i < 6; i++) {
      uint16_t v1, v2;
      EEPROM.get(ADDR_DURATE + (i * 2), v1);
      EEPROM.get(ADDR_DURATE_2 + (i * 2), v2);
      durataZone[i] = v1;
      durataZone2[i] = v2;
    }

    byte gb1 = EEPROM.read(ADDR_GIORNI), gb2 = EEPROM.read(ADDR_GIORNI_2);
    for(int g = 0; g < 7; g++) { giorniSettimana[g] = (gb1 & (1 << g)) ? true : false; giorniSettimana2[g] = (gb2 & (1 << g)) ? true : false; }
  }
}
