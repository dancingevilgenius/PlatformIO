#include <Arduino.h>

// --- Forward Declarations ---
void SERCOM0_Handler();
void camminata1(int richiedente);
void camminata (int richiedente);
void riabbasso6();
void alzo6();
void riassetto();
void girogambe(int numerogiri);
void shiftorizz();
void varialarghezza(int vdlar, int alzata);
void vibraon();
void vibraoff();
void precaricaposfine();
void variaaltezza(int valtezza);
void setaltezza(int altezza);
void flessioni(int numerof, int passif, int altfless);
void movimentop(int passi,int ritardo);
void movimentog (int passi,int ritardo);
void stampaposizionixyz();
void debugcomandoP();
void debugcalcolaabc();
void mettili();
void calcolaxyz();
void calcolaabc ();
void ricezioneterminale ();
void gestionecomandi();
void ricezionetelecomando();
void caricaatan();
int calcolaatan(double argomento);
int calcolagamma(double argomento);
void chelp();

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "wiring_private.h"
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();       
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
#define SERVO_FREQ 50 // I servo lavorano a  ~50 Hz 

Uart mySerial(&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
// Attach the interrupt handler to the SERCOM

int incomingByte2=0;

void SERCOM0_Handler()
{
    mySerial.IrqHandler();
}

// geometric definitions of the hexapod... origin in center
/*
 *        + (pgx[6],pgy[6])  Gamba6                  + (pgx[1],pgy[1])  Gamba1
 *            org[6]                                     org[1]
 * 
 *   + (pgx[5],pgy[5])  Gamba5   +   (0,0)  origine      + (pgx[2],pgy[2])  Gamba2
 *        org[5]                                              org[2]
 * 
 *        + (pgx[4],pgy[4])  Gamba6                  + (pgx[3],pgy[3])  Gamba3
 *            org[4]                                     org[3]    
 *            
 *  the orientation is counterclockwise 0 to the right, 90 up 180 to the left...          
 *  all values ​​are in mm
 *  
 *    osso       artic    min   cen   max         
 *    TIBIA
 *     200 mm  ltibia
 *               [3]     210    90     60         basso       dritto   alto
 *  
 *    FEMORE
 *     86 mm   lfemore
 *               [2]      20    90    180         basso       dritto  alto
 *  
 *    SPALLA   lscapola
 *     65 mm
 *               [1]      20    90    160      rot sinistra   dritto  rot destra   
 *    
 *    SCAPOLA
 *    
 *    ELECTRICAL WIRING
 *    
 *    schrda giunzioni
 *    
 *    1   (nero)     GND            (14) GND da arduino
 *    2   (bianco)   OUTPUT ENABLE       abilita i servo, disabilita semposta a VCC   
 *    3   (rosso)    SCL            (9) SCL da arduino
 *    4   (giallo)   SDA            (8) SDA da arduino
 *    5   (verde)    VCC            (2) 3,3V da arduino
 *    6   (azzurro)  V+                 5V per i servo
 *    
 *    microswitch:  1  disabilita servo
 *                  2
 *                  3
 *                  4
 */
int videbug=0;  // if set to 0 it hides the debug display on the terminal
int debshow=0;  // debug view


int viposizioni=0;
     // DEFINIZIONI DELLE MATRICI CHE CONTERRANNO I DATI
int angolominimo[7][4];   // matrice finecorsa miinimo servi  (gamba, snodo)  (gradi)
int vangolominimo[7][4];   // valore output servo per ottenere quell'angolo
int angolomassimo[7][4];  // matrice finecorsa massimo servi (gamba, snodo)   (gradi)
int vangolomassimo[7][4];  // valore output servo per ottenere quell'angolo
int scheda[7][4];         // scheda di collegamento (gamba, snodo)
int collegamento[7][4];   // matrice collegamenti (gamba, snodo)
     // punto di origine delle 6 gambe
int pgx[7];               // posizione X gambe
int pgy[7];               // posizione y gambe
int org[7];               // orientamento gamba
     // parametri dimensionali (lunghezza arti)
const int ltibia=220;           // lunghezza tibia    perno-punta
const int lfemore=86;           // lunghezza femore   perno-perno
const int lspalla=65;           // lungezza spalla    perno-perno
const double ragra=57.29577;         // 180/PI 
const double grara=0.01745329;       // PI/180
     // matrice di folle, se il valore di una gamba è a 0 questa è in folle,se a 1 è operativa
int folle[7][2];           // gamba, articolazione

double posizioni[7][4];   // posizione dei servi  (notazione double perchè in gradi con virgola)
double posiniz[7][4];     // matrice posizioni iniziali durante un movimento
double posfine[7][4];     // matrice fine corsa finale durante un  movimento
  //  posizioni  spaziali in mm punta gamba 
int posinizx[7];         // coordinata x
int posinizy[7];        //  coordinata y
int posinizz[7];        //  coordinata z
int posfinex[7];         // coordinata x
int posfiney[7];        //  coordinata y
int posfinez[7];        //  coordinata z
int posgambax[7];         // coordinata x
int posgambay[7];        //  coordinata y
int posgambaz[7];        //  coordinata z
byte datotel[6];


double provaatan[181];  // ricerca veloce
double provaacos[126][3];
int testatan;


int posx[7];         // coordinata x
int posy[7];        //  coordinata y
int posz[7];        //  coordinata z
int velocita=1;           // parametro velocita (1 veloce 100 lenta)
int velposa=20;

// parametri per il controllo da terminale
int sequenza=0;   
int ricangolo=-100;  
int ricsnodo=0;   // Sg1s3a096  numero gamba, snodo e angolo, (angolo con 3 cifre)
int ricgamba=0;
int ricx;         // Pg2x-010y+001z-020  posiziona una gamba in un punto definito
int ricy;
int ricz;

int incomingByte = 0;
int segno=1;
int comando=0;

int ricnumerof=0;//parametri per flessioni
int ricpassif=0;
int ricaltezza=0;
int dlarghezza=30;  // parametri per divarico gambe
int daltezza=30;       
int ricnumerog;  // parametri girogambe
int ricvariax;
int ricvariay;
int ricrotazione;


int altezza=-100;
int larghezza=0;
int shiftorizzx=0;
int shiftorizzy=0;
int alzpasso=70;
int continuo=0;
int passo6=0;

int direzioneweb=128;
int rotazioneweb=128;
int distanzaweb=127;
int velocitaweb=128;
void setup() {
  // riassegno i pin 5 e 6 a SERCOM ALT
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);
  mySerial.begin(9600);

  
  int conta=0;
    // posizioni gambe rispetto all origine in centro
  pgx[1]=114;     pgy[1]=-129;    org[1]=315;    // alto destra            ^
  pgx[2]=156;     pgy[2]=0;      org[2]=0;     // destra             4      3
  pgx[3]=114;     pgy[3]=+129;   org[3]=45;   // basso destra     5          2
  pgx[4]=-114;    pgy[4]=+129;   org[4]=135;   // basso sinistra     6      1
  pgx[5]=-156;    pgy[5]=0;      org[5]=180;   // sinistra
  pgx[6]=-114;    pgy[6]=-129;    org[6]=225;   // altosinistra

     //scheda a cui sono collegati i servi 
  scheda[1][1]=1;  scheda[1][2]=1;  scheda[1][3]=1;   // GAMBA1
  scheda[2][1]=1;  scheda[2][2]=1;  scheda[2][3]=1;   // GAMBA2
  scheda[3][1]=1;  scheda[3][2]=1;  scheda[3][3]=1;   // GAMBA3 
  scheda[4][1]=2;  scheda[4][2]=2;  scheda[4][3]=2;   // GAMBA4
  scheda[5][1]=2;  scheda[5][2]=2;  scheda[5][3]=2;   // GAMBA5
  scheda[6][1]=2;  scheda[6][2]=2;  scheda[6][3]=2;   // GAMBA6

     // posizionamento del collegamento sulla scheda del servo
  collegamento[1][1]=0;  collegamento[1][2]=1;  collegamento[1][3]=2;  // GAMBA1
  collegamento[2][1]=3;  collegamento[2][2]=4;  collegamento[2][3]=5;  // GAMBA2
  collegamento[3][1]=6;  collegamento[3][2]=7;  collegamento[3][3]=8;  // GAMBA3
  collegamento[4][1]=0;  collegamento[4][2]=1;  collegamento[4][3]=2;  // GAMBA4
  collegamento[5][1]=3;  collegamento[5][2]=4;  collegamento[5][3]=5;  // GAMBA5
  collegamento[6][1]=6;  collegamento[6][2]=7;  collegamento[6][3]=8;  // GAMBA6

    // all'avvio tutte le gambe devono essere molli per evitare rotture
  folle[1][1]=0; folle[2][1]=0; folle[3][1]=0; folle[4][1]=0; folle[5][1]=0; folle[6][1]=0;
  folle[1][2]=0; folle[2][2]=0; folle[3][2]=0; folle[4][2]=0; folle[5][2]=0; folle[6][2]=0;
  folle[1][3]=0; folle[2][3]=0; folle[3][3]=0; folle[4][3]=0; folle[5][3]=0; folle[6][3]=0;

   // dati sperimentali di taratura servo, 
 angolominimo[1][1]=-45; vangolominimo[1][1]=412; angolomassimo[1][1]=45; vangolomassimo[1][1]=210;
 angolominimo[1][2]=-60; vangolominimo[1][2]=200; angolomassimo[1][2]=80; vangolomassimo[1][2]=409;
 angolominimo[1][3]=-125; vangolominimo[1][3]=396; angolomassimo[1][3]=25; vangolomassimo[1][3]=64;
 angolominimo[2][1]=-45; vangolominimo[2][1]=405; angolomassimo[2][1]=45; vangolomassimo[2][1]=195;
 angolominimo[2][2]=-60; vangolominimo[2][2]=250; angolomassimo[2][2]=80; vangolomassimo[2][2]=445;
 angolominimo[2][3]=-125; vangolominimo[2][3]=430; angolomassimo[2][3]=25; vangolomassimo[2][3]=87;
 angolominimo[3][1]=-45; vangolominimo[3][1]=392; angolomassimo[3][1]=45; vangolomassimo[3][1]=184;
 angolominimo[3][2]=-60; vangolominimo[3][2]=231; angolomassimo[3][2]=80; vangolomassimo[3][2]=435;
 angolominimo[3][3]=-125; vangolominimo[3][3]=410; angolomassimo[3][3]=25; vangolomassimo[3][3]=71;
 angolominimo[4][1]=-45; vangolominimo[4][1]=376; angolomassimo[4][1]=45; vangolomassimo[4][1]=191;
 angolominimo[4][2]=-60; vangolominimo[4][2]=360; angolomassimo[4][2]=80; vangolomassimo[4][2]=159;
 angolominimo[4][3]=-125; vangolominimo[4][3]=184; angolomassimo[4][3]=25; vangolomassimo[4][3]=507;
 angolominimo[5][1]=-45; vangolominimo[5][1]=381; angolomassimo[5][1]=45; vangolomassimo[5][1]=195;
 angolominimo[5][2]=-60; vangolominimo[5][2]=405; angolomassimo[5][2]=80; vangolomassimo[5][2]=222;
 angolominimo[5][3]=-125; vangolominimo[5][3]=168; angolomassimo[5][3]=25; vangolomassimo[5][3]=485;
 angolominimo[6][1]=-45; vangolominimo[6][1]=379; angolomassimo[6][1]=45; vangolomassimo[6][1]=193;
 angolominimo[6][2]=-60; vangolominimo[6][2]=365; angolomassimo[6][2]=80; vangolomassimo[6][2]=161;
 angolominimo[6][3]=-125; vangolominimo[6][3]=170; angolomassimo[6][3]=25; vangolomassimo[6][3]=496;

// setto posizione iniziale con spalle centrate e gambe in alto e dritta
 for(conta=1;conta<7;conta++){
   posizioni[conta][1]=0;  posizioni[conta][2]=90;  posizioni[conta][3]=10; 
   posiniz[conta][1]=0;  posiniz[conta][2]=90;  posiniz[conta][3]=10;  
   posfine[conta][1]=0;  posfine[conta][2]=90;  posfine[conta][3]=10;
 }
  // prima era 0 60 110
  Serial.begin(57600);
  delay(4000);
  Serial.println ("Seriale inizializzata");   
  // attivazione scheda servo 1

  caricaatan();

  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Settaggio frequenza servi ~50 Hz 
  // attivazione secheda servo 2
  pwm2.begin();
  pwm2.setOscillatorFrequency(27000000);
  pwm2.setPWMFreq(SERVO_FREQ);  // Settaggio frequenza servi ~50 Hz 
  delay(1000);

  calcolaxyz ();   //  dal valore di posizioni iniziali calcola le coordinate di tutte le zampe
  Serial.println ("Attivazione, gambe in folle");
  for(conta=1;conta<7;conta++){
      folle[conta][2]=1;         //sblocco il folle per il secondo servo di tutte le gambe in modo da sollevarle
   }
   Serial.println ("Attivazione, sollevamento gambe"); 
  mettili();                     //imposto posizione
  delay (1000);                  // pausa di un secondo
  for(conta=1;conta<7;conta++){
      folle[conta][3]=1;         //sblocco il folle per il terzo servo di tutte le gambe in modo da sollevarle
   } 
   Serial.println ("Attivazione, raddrizzamento gambe");
  mettili();                     //imposto posizione
  delay (1000);                  // pausa di un secondo
  for(conta=1;conta<7;conta++){
      folle[conta][1]=1;         //sblocco il folle per il primo servo di tutte le gambe in modo da girale
   }
   Serial.println ("Attivazione, allineamento gambe"); 
  mettili();                     //imposto posizione
  
  delay (1000);                  // pausa di un secondo
  // a questo punto il robot è con tutte le gambe sollevate e centrate


// setto posizione con gambe in appoggio
 for(conta=1;conta<7;conta++){
   posfine[conta][1]=0;  posfine[conta][2]=60;  posfine[conta][3]=-110;
 }
 Serial.println("Abbasso le gambe");
 movimentog(400,1);
  
   Serial.println ("READY");

  calcolaxyz();
  stampaposizionixyz(); 
  
  
}// setup


//ciclo principale
void loop() {
  ricezioneterminale();    // controllo immissione da terminale
  ricezionetelecomando();  // controllo comandi da telecomando
  gestionecomandi();       // gestione comandi (da ogni fonte)
}  //loop



void camminata1(int richiedente){    // con una gamba per volta  0=telec  1=web
  double variax=0;
  double variay=0;
  double rotazione=0;
  double variaxr[7];
  double variayr[7];
  int spostx=0;
  int sposty=0; 
  if(richiedente==0){  // richiesto da telecomando, leggo gli stick
    variax=datotel[1]-128;
    variay=-(datotel[2]-127);
    rotazione=-(datotel[3]-128);
  }
  if(richiedente==1){  // richiesto da web, leggo i cursori nella pagina
    variax=-distanzaweb*sin(direzioneweb*3.14159/128)/2;
    variay=-distanzaweb*cos(direzioneweb*3.14159/128)/2;
    rotazione=-(rotazioneweb-128);
  }
   if(richiedente==2){  // richiesto da seriale, leggo i dati ricevuti
    variax=-ricvariax;
    variay=-ricvariay; 
    rotazione=-ricrotazione;
  }
  Serial.print("variax= ");
  Serial.print(variax);
  Serial.print("  variay= ");
  Serial.print(variay);
  Serial.print("  rotazione=");
  Serial.println(rotazione);
  for(int gamba=1;gamba<=6;gamba++){
    variaxr[gamba]=sin(rotazione*3.14159/180/6/6)*sin(-org[gamba]*3.1469/180)*150;
    variayr[gamba]=sin(rotazione*3.14159/180/6/6)*cos(-org[gamba]*3.1469/180)*150;
  }
  
  spostx=variax/9;
  sposty=variay/9;
  precaricaposfine();
    posfinez[1]=posgambaz[1]+ alzpasso;   // alzo gamba 1
    movimentop(velposa,1);
    posfinex[1]=posgambax[1]+ spostx*5 + variaxr[1] *5;    // setto in avanti gamba 1
    posfiney[1]=posgambay[1]+ sposty*5 + variayr[1] *5;
    posfinex[2]=posgambax[2]- spostx - variaxr[2];    // setto indietro le altre gambe
    posfiney[2]=posgambay[2]- sposty - variayr[2];
    posfinex[3]=posgambax[3]- spostx - variaxr[3]; 
    posfiney[3]=posgambay[3]- sposty - variayr[3];   
    posfinex[4]=posgambax[4]- spostx - variaxr[4];
    posfiney[4]=posgambay[4]- sposty - variayr[4];
    posfinex[5]=posgambax[5]- spostx - variaxr[5];
    posfiney[5]=posgambay[5]- sposty - variayr[5];
    posfinex[6]=posgambax[6]- spostx - variaxr[6];
    posfiney[6]=posgambay[6]- sposty - variayr[6];
    movimentop(velposa,1);
    posfinez[1]=posgambaz[1]- alzpasso;   // abbasso gamba 1
    movimentop(velposa,5);
    
    posfinez[2]=posgambaz[2]+ alzpasso;   // alzo gamba 2
    movimentop(velposa,1);
    posfinex[2]=posgambax[2]+ spostx*5 + variaxr[2] *5;    // setto in avanti gamba 2
    posfiney[2]=posgambay[2]+ sposty*5 + variayr[2] *5;
    posfinex[1]=posgambax[1]- spostx - variaxr[1];    // setto indietro le altre gambe
    posfiney[1]=posgambay[1]- sposty - variayr[1];
    posfinex[3]=posgambax[3]- spostx - variaxr[3]; 
    posfiney[3]=posgambay[3]- sposty - variayr[3];   
    posfinex[4]=posgambax[4]- spostx - variaxr[4];
    posfiney[4]=posgambay[4]- sposty - variayr[4];
    posfinex[5]=posgambax[5]- spostx - variaxr[5];
    posfiney[5]=posgambay[5]- sposty - variayr[5];
    posfinex[6]=posgambax[6]- spostx - variaxr[6];
    posfiney[6]=posgambay[6]- sposty - variayr[6];
    movimentop(velposa,1);
    posfinez[2]=posgambaz[2]- alzpasso;   // abbasso gamba 2
    movimentop(velposa,5);

    posfinez[3]=posgambaz[3]+ alzpasso;   // alzo gamba 3
    movimentop(velposa,1);
    posfinex[3]=posgambax[3]+ spostx*5 + variaxr[3]*5;    // setto in avanti gamba 3
    posfiney[3]=posgambay[3]+ sposty*5 + variayr[3]*5;
    posfinex[1]=posgambax[1]- spostx - variaxr[1];    // setto indietro le altre gambe
    posfiney[1]=posgambay[1]- sposty - variayr[1];
    posfinex[2]=posgambax[2]- spostx - variaxr[2]; 
    posfiney[2]=posgambay[2]- sposty - variayr[2];   
    posfinex[4]=posgambax[4]- spostx - variaxr[4];
    posfiney[4]=posgambay[4]- sposty - variayr[4];
    posfinex[5]=posgambax[5]- spostx - variaxr[5];
    posfiney[5]=posgambay[5]- sposty - variayr[5];
    posfinex[6]=posgambax[6]- spostx - variaxr[6];
    posfiney[6]=posgambay[6]- sposty - variayr[6];
    movimentop(velposa,1);
    posfinez[3]=posgambaz[3]- alzpasso;   // abbasso gamba 3
    movimentop(velposa,5);

    posfinez[4]=posgambaz[4]+ alzpasso;   // alzo gamba 4
    movimentop(velposa,1);
    posfinex[4]=posgambax[4]+ spostx*5 + variaxr[4]*5;    // setto in avanti gamba 4
    posfiney[4]=posgambay[4]+ sposty*5 + variayr[4]*5;
    posfinex[1]=posgambax[1]- spostx - variaxr[1];    // setto indietro le altre gambe
    posfiney[1]=posgambay[1]- sposty - variayr[1];
    posfinex[2]=posgambax[2]- spostx - variaxr[2]; 
    posfiney[2]=posgambay[2]- sposty - variayr[2];   
    posfinex[3]=posgambax[3]- spostx - variaxr[3];
    posfiney[3]=posgambay[3]- sposty - variayr[3];
    posfinex[5]=posgambax[5]- spostx - variaxr[5];
    posfiney[5]=posgambay[5]- sposty - variayr[5];
    posfinex[6]=posgambax[6]- spostx - variaxr[6];
    posfiney[6]=posgambay[6]- sposty - variayr[6];
    movimentop(velposa,1);
    posfinez[4]=posgambaz[4]- alzpasso;   // abbasso gamba 4
    movimentop(velposa,5);
  
    posfinez[5]=posgambaz[5]+ alzpasso;   // alzo gamba 5
    movimentop(velposa,1);
    posfinex[5]=posgambax[5]+ spostx*5 + variaxr[5]*5;    // setto in avanti gamba 5
    posfiney[5]=posgambay[5]+ sposty*5 + variayr[5]*5;
    posfinex[1]=posgambax[1]- spostx - variaxr[1];    // setto indietro le altre gambe
    posfiney[1]=posgambay[1]- sposty - variayr[1];
    posfinex[2]=posgambax[2]- spostx - variaxr[2]; 
    posfiney[2]=posgambay[2]- sposty - variayr[2];   
    posfinex[3]=posgambax[3]- spostx - variaxr[3];
    posfiney[3]=posgambay[3]- sposty - variayr[3];
    posfinex[4]=posgambax[4]- spostx - variaxr[4];
    posfiney[4]=posgambay[4]- sposty - variayr[4];
    posfinex[6]=posgambax[6]- spostx - variaxr[6];
    posfiney[6]=posgambay[6]- sposty - variayr[6];
    movimentop(velposa,1);
    posfinez[5]=posgambaz[5]- alzpasso;   // abbasso gamba 5
    movimentop(velposa,5);

    posfinez[6]=posgambaz[6]+ alzpasso;   // alzo gamba 6
    movimentop(velposa,1);
    posfinex[6]=posgambax[6]+ spostx*5 + variaxr[6] *5;    // setto in avanti gamba 6
    posfiney[6]=posgambay[6]+ sposty*5 + variayr[6] *5;
    posfinex[1]=posgambax[1]- spostx - variaxr[1];    // setto indietro le altre gambe
    posfiney[1]=posgambay[1]- sposty - variayr[1];
    posfinex[2]=posgambax[2]- spostx - variaxr[2]; 
    posfiney[2]=posgambay[2]- sposty - variayr[2];   
    posfinex[3]=posgambax[3]- spostx - variaxr[3];
    posfiney[3]=posgambay[3]- sposty - variayr[3];
    posfinex[4]=posgambax[4]- spostx - variaxr[4];
    posfiney[4]=posgambay[4]- sposty - variayr[4];
    posfinex[5]=posgambax[5]- spostx - variaxr[5];
    posfiney[5]=posgambay[5]- sposty - variayr[5];
    movimentop(velposa,1);
    posfinez[6]=posgambaz[6]- alzpasso;   // abbasso gamba 6
    movimentop(velposa,5);   
} // camminata1

void camminata (int richiedente){    // con 3 gambe per volta  se richiedente=0 tel =1 web =2 seriale
  double direzione=0;
  double distanza=0;
  double rotazione=0;
  double variax=0;
  double variay=0;
  double variaxr[7];
  double variayr[7];
  double passomax=5;
  int spostx=0;
  int sposty=0;
  if(richiedente==0){  // richiesto da telecomando, leggo gli stick
    variax=datotel[1]-128;
    variay=-(datotel[2]-127);
    rotazione=-(datotel[3]-128);
  }
   if(richiedente==1){  // richiesto da web, leggo i cursori nella pagina
    variax=-distanzaweb*sin(direzioneweb*3.14159/128)/2;
    variay=-distanzaweb*cos(direzioneweb*3.14159/128)/2;
    rotazione=-(rotazioneweb-128);
  }  
   if(richiedente==2){  // richiesto da seriale, leggo i dati ricevuti
    variax=-ricvariax;
    variay=-ricvariay; 
    rotazione=-ricrotazione;
  }  
  Serial.print("variax= ");
  Serial.print(variax);
  Serial.print("  variay= ");
  Serial.print(variay);
  Serial.print("  rotazione=");
  Serial.println(rotazione);
//  direzione=1* atan(variay/(variax+1))*180/3.14;
//  if (variax<0) direzione=direzione + 180;
//  distanza=sqrt(variax*variax+variay*variay)*passomax/128;
  spostx=variax/3;
  sposty=variay/3;
  for(int gamba=1;gamba<=6;gamba++){
    variaxr[gamba]=sin(rotazione*3.14159/180/6)*sin(-org[gamba]*3.1469/180)*150;
    variayr[gamba]=sin(rotazione*3.14159/180/6)*cos(-org[gamba]*3.1469/180)*150;
  }
  
  precaricaposfine();

   if (passo6==0) alzo6();
    posfinex[2]=posgambax[2]+ spostx + variaxr[2];    // setto in avanti gambe pari
    posfinex[4]=posgambax[4]+ spostx + variaxr[4];
    posfinex[6]=posgambax[6]+ spostx + variaxr[6];      
    posfiney[2]=posgambay[2]+ sposty + variayr[2];    
    posfiney[4]=posgambay[4]+ sposty + variayr[4];
    posfiney[6]=posgambay[6]+ sposty + variayr[6];      
    posfinex[1]=posgambax[1]- spostx - variaxr[1];    // setto indietro gambe dispari
    posfinex[3]=posgambax[3]- spostx - variaxr[3];
    posfinex[5]=posgambax[5]- spostx - variaxr[5];      
    posfiney[1]=posgambay[1]- sposty - variayr[1];    
    posfiney[3]=posgambay[3]- sposty - variayr[3];
    posfiney[5]=posgambay[5]- sposty - variayr[5];      
    movimentop(velposa,1);   
    posfinez[2]=posgambaz[2]- alzpasso;  //abbasso gambe pari
    posfinez[4]=posgambaz[4]- alzpasso;
    posfinez[6]=posgambaz[6]- alzpasso;
    movimentop(velposa,5);
    posfinez[1]=posgambaz[1]+ alzpasso;  //alzo gambe dispari
    posfinez[3]=posgambaz[3]+ alzpasso;
    posfinez[5]=posgambaz[5]+ alzpasso;
    movimentop(velposa,1);
    posfinex[2]=posgambax[2]- 2*spostx - 2*variaxr[2];    // setto in indietro gambe pari
    posfinex[4]=posgambax[4]- 2*spostx - 2*variaxr[4];
    posfinex[6]=posgambax[6]- 2*spostx - 2*variaxr[6];      
    posfiney[2]=posgambay[2]- 2*sposty - 2*variayr[2];    
    posfiney[4]=posgambay[4]- 2*sposty - 2*variayr[4];
    posfiney[6]=posgambay[6]- 2*sposty - 2*variayr[6];      
    posfinex[1]=posgambax[1]+ 2*spostx + 2*variaxr[1];    // setto avanti gambe dispari
    posfinex[3]=posgambax[3]+ 2*spostx + 2*variaxr[3];
    posfinex[5]=posgambax[5]+ 2*spostx + 2*variaxr[5];      
    posfiney[1]=posgambay[1]+ 2*sposty + 2*variayr[1];    
    posfiney[3]=posgambay[3]+ 2*sposty + 2*variayr[3];
    posfiney[5]=posgambay[5]+ 2*sposty + 2*variayr[5];
    movimentop(velposa*2,1); 
    posfinez[1]=posgambaz[1]- alzpasso;  //abbasso gambe dispari
    posfinez[3]=posgambaz[3]- alzpasso;
    posfinez[5]=posgambaz[5]- alzpasso;
    movimentop(velposa,5);    
    posfinez[2]=posgambaz[2]+ alzpasso;  //alzo gambe pari
    posfinez[4]=posgambaz[4]+ alzpasso;
    posfinez[6]=posgambaz[6]+ alzpasso;
    movimentop(velposa,1);
    posfinex[2]=posgambax[2]+ spostx + variaxr[2];    // setto in avanti gambe pari
    posfinex[4]=posgambax[4]+ spostx + variaxr[4];
    posfinex[6]=posgambax[6]+ spostx + variaxr[6];      
    posfiney[2]=posgambay[2]+ sposty + variayr[2];    
    posfiney[4]=posgambay[4]+ sposty + variayr[4];
    posfiney[6]=posgambay[6]+ sposty + variayr[6];      
    posfinex[1]=posgambax[1]- spostx - variaxr[1];    // setto indietro gambe dispari
    posfinex[3]=posgambax[3]- spostx - variaxr[3];
    posfinex[5]=posgambax[5]- spostx - variaxr[5];      
    posfiney[1]=posgambay[1]- sposty - variayr[1];    
    posfiney[3]=posgambay[3]- sposty - variayr[3];
    posfiney[5]=posgambay[5]- sposty - variayr[5];      
    movimentop(velposa,1);   
    passo6=1; 
}

void riabbasso6(){ 
    posfinez[2]=posgambaz[2]- alzpasso;  //abbasso gambe pari
    posfinez[4]=posgambaz[4]- alzpasso;
    posfinez[6]=posgambaz[6]- alzpasso;
    movimentop(velposa,5);
   // continuo=0;
    passo6=0;
}

void alzo6(){
    posfinez[2]=posgambaz[2]+ alzpasso;  //alzo gambe pari
    posfinez[4]=posgambaz[4]+ alzpasso;
    posfinez[6]=posgambaz[6]+ alzpasso;
    movimentop(2,1);
}

void riassetto(){
calcolaxyz();
    int zalto=posgambaz[1] +50;
    int zbasso=posgambaz[1];  
    posinizx[1]=posgambax[1];
    posinizy[1]=posgambay[1];
    posinizz[1]=zbasso; 
    posfinex[1]=posgambax[1];
    posfiney[1]=posgambay[1];
    posfinez[1]=zalto;
    movimentop(2,1);     
               
     for(int gamba=2;gamba<=6;gamba++){  //per tutte le gambe
                 posinizx[gamba]=posgambax[gamba];
                 posinizy[gamba]=posgambay[gamba];
                 posinizz[gamba]=zbasso; 
                 posfinex[gamba]=posgambax[gamba];
                 posfiney[gamba]=posgambay[gamba];
                 posfinez[gamba]=zalto;
                 movimentop(2,1);     
                 posizioni[gamba][1]=posizioni[1][1];
                 posizioni[gamba][2]=posizioni[1][2];
                 posizioni[gamba][3]=posizioni[1][3];                 
                 movimentog(3,1); 
                 larghezza=posfinex[2];
              }
}

void girogambe(int numerogiri){
   //calcolaxyz(); 
    Serial.print("*");
   Serial.println(posgambaz[1]); 
    int zalto=posgambaz[1] +50;
    int zbasso=posgambaz[1];
      for(int giro=1;giro<=numerogiri;giro++){ // per tutti i giri           
              for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
                 posinizx[gamba]=posgambax[gamba];
                 posinizy[gamba]=posgambay[gamba];
                 posinizz[gamba]=zbasso; 
                 posfinex[gamba]=posgambax[gamba];
                 posfiney[gamba]=posgambay[gamba];
                 posfinez[gamba]=zalto;
                 movimentop(2,1);     
                 posinizx[gamba]=posgambax[gamba];
                 posinizy[gamba]=posgambay[gamba];
                 posinizz[gamba]=zalto; 
                 posfinex[gamba]=posgambax[gamba];
                 posfiney[gamba]=posgambay[gamba];
                 posfinez[gamba]=zbasso;
                 movimentop(velposa,5);
                 movimentop(2,1); 
                 larghezza=posfinex[2];
              }               
           
      }
   
}

void shiftorizz(){
  int alzata=25;
  precaricaposfine();
   for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
    posfinex[gamba]=posgambax[gamba]+shiftorizzx;
    posfiney[gamba]=posgambay[gamba]+shiftorizzy ; 
    posfinez[gamba]=posgambaz[gamba];   
   }
   movimentop(20,1);
   stampaposizionixyz();  
}
    


void varialarghezza(int vdlar, int alzata){
  precaricaposfine();
   for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
         // ALZO
      posfinex[gamba]=posgambax[gamba];
      posfiney[gamba]=posgambay[gamba] ; 
      posfinez[gamba]=posgambaz[gamba]+alzata;
      movimentop(25,1);
         // DIVARICO
      posfinex[gamba]=posgambax[gamba]+vdlar*cos(org[gamba]*grara);
      posfiney[gamba]=posgambay[gamba]+vdlar*sin(org[gamba]*grara) ; 
      posfinez[gamba]=posgambaz[gamba];
      movimentop(25,1); 
         // ABBASSO
      posfinex[gamba]=posgambax[gamba];
      posfiney[gamba]=posgambay[gamba] ;   
      posfinez[gamba]=posgambaz[gamba]-alzata;
      movimentop(40,1); 
   }
 stampaposizionixyz();  
}







void vibraon(){
  byte segnale=0xff;
  Serial.println("Vibrazione on");  
  Wire.beginTransmission(7);
  Wire.write(segnale);
  Wire.endTransmission();
}


void vibraoff(){  
byte segnale=0;
  Serial.println("Vibrazione off"); 
  Wire.beginTransmission(7);
  Wire.write(segnale);
  Wire.endTransmission();
}

void precaricaposfine(){
     for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
      posfinex[gamba]=posgambax[gamba];
      posfiney[gamba]=posgambay[gamba];
      posfinez[gamba]=posgambaz[gamba];
     }
}

void variaaltezza(int valtezza){  
  precaricaposfine();
  for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
      posfinez[gamba]=posfinez[gamba] - valtezza;
  }
  movimentop(20,1);
  stampaposizionixyz(); 
}

void setaltezza(int altezza){
  precaricaposfine(); 
  for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
    posfinez[gamba]=-94-altezza;  // calcolo Z alta
  } 
   movimentop(80,1);     // movimento lineare verticale  
}



void flessioni(int numerof, int passif, int altfless){ 
  for(int fle=1;fle<=numerof;fle++){  //numerof flessioni
    
     precaricaposfine(); 
    // calcolaxyz(); 
     for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
       posfinez[gamba]=posfinez[gamba] - altfless;  // calcolo Z alta
     }
     movimentop(passif,10);     // movimento lineare verso l'alto
     
     precaricaposfine();
     //calcolaxyz();
     for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
      posfinez[gamba]=posfinez[gamba] + altfless;  // calcolo Z bassa
     }
     movimentop(passif,10);     // movimento lineare verso il basso
  }  // fine ciclo flessioni
}


void movimentop(int passi,int ritardo){
  double incrx=0;
  double incry=0;
  double incrz=0;
  double stepx=0;
  double stepy=0;
  double stepz=0;
   for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
      posinizx[gamba]=posgambax[gamba];
      posinizy[gamba]=posgambay[gamba];
      posinizz[gamba]=posgambaz[gamba];      
   }         
   for(int contapassi=1;contapassi<=passi;contapassi++){
       for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
           incrx=(posfinex[gamba]-posinizx[gamba])*100;
           stepx=incrx/passi/100;
           incry=(posfiney[gamba]-posinizy[gamba])*100;
           stepy=incry/passi/100;
           incrz=(posfinez[gamba]-posinizz[gamba])*100;
           stepz=incrz/passi/100;
           posx[gamba]=posinizx[gamba]+ stepx*contapassi;
           posy[gamba]=posinizy[gamba]+ stepy*contapassi;
           posz[gamba]=posinizz[gamba]+ stepz*contapassi;
       }
           calcolaabc();
   //        Serial.print(".");
       for(int gamba=1;gamba<=6;gamba++){
           for(int snodo=1;snodo<=3;snodo++){ // per tutti gli snodi 
              posizioni[gamba][snodo]=posfine[gamba][snodo];
             // Serial.print (posizioni[gamba][snodo]);
           }   
       }  
       mettili();    
       delay (ritardo);   
   }
  // Serial.println("");   
       for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
        posgambax[gamba]=posfinex[gamba];
        posgambay[gamba]=posfiney[gamba];
        posgambaz[gamba]=posfinez[gamba];
       }
   //if(debshow==1) 
   //debugcomandoP();
   
   // delay(5000);
} //movimentop



void movimentog (int passi,int ritardo){   //input posiniz[],posfine[]  muove posizioni[]
  double step=0;
    for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
       for(int snodo=1;snodo<=3;snodo++){ // per tutti gli snodi 
         posiniz[gamba][snodo]=posizioni[gamba][snodo];  
       }
    } 
    for(int contapassi=1;contapassi<=passi;contapassi++){
       for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
          for(int snodo=1;snodo<=3;snodo++){ // per tutti gli snodi 
             step= (posfine[gamba][snodo]-posiniz[gamba][snodo])/passi;
             posizioni[gamba][snodo]=posiniz[gamba][snodo]+contapassi*step;               
          }
       }
   //    Serial.print (".");
       mettili(); 
       delay (ritardo);
    }
    Serial.println(" ");
} // movimentog



void stampaposizionixyz(){    // stampa posizioni xyz da matrice posgamba_[] dove _=(x,y,z)
 // Serial.println("");
  Serial.print("Pos ");
  for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
          Serial.print("  G");
          Serial.print (gamba);
          Serial.print(" x=");
          Serial.print (posgambax[gamba]);   
          Serial.print(" y=");
          Serial.print (posgambay[gamba]);
          Serial.print(" z=");
          Serial.print (posgambaz[gamba]);              
  }  
 Serial.println("");
}


void debugcomandoP(){
   Serial.print ("P g");
    Serial.print (ricgamba);
    Serial.print (" x=");
    Serial.print (ricx);
    Serial.print (" y=");
    Serial.print (ricy);
    Serial.print (" z=");
    Serial.print (ricz);
     Serial.print (" a=");
    Serial.print (posfine[ricgamba][1]);
    Serial.print (" b=");
    Serial.print (posfine[ricgamba][2]);
    Serial.print (" c=");
    Serial.println (posfine[ricgamba][3]);  
}

void debugcalcolaabc(){   // stampa posfine[]
      Serial.println(">>> ");
    for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
       posx[gamba]=posgambax[gamba];    // routine di controllo per verificare il
       posy[gamba]=posgambay[gamba];    // funzionamento della funzione calcolaabc()
       posz[gamba]=posgambaz[gamba];
    }  
    calcolaabc();
    for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
       Serial.print ("check  S1=");
       Serial.print (posfine[gamba][1]);
       Serial.print (" S2=");
       Serial.print (posfine[gamba][2]);
       Serial.print (" S3=");
       Serial.print (posfine[gamba][3]);
       Serial.println (" ");
    }
    Serial.println("<<< ");
  
}





void mettili(){     // mette tutti i servo nella posizione indicata dalla matrice
                    // posizioni, per spostare tutti i servo impostare i valori
                    // in posizioni e poi invocare mettili();
double posizione=0;
/*  if (videbug==1){
      Serial.println(" ");
      Serial.println("GAMBA\t1\t\tGAMBA\t2\t\tGAMBA\t3\t\tGAMBA\t4\t\tGAMBA\t5\t\tGAMBA\t6\t");
      Serial.println("S1\tS1\tS3\tS1\tS1\tS3\tS1\tS1\tS3\tS1\tS1\tS3\tS1\tS1\tS3\tS1\tS1\tS3");                  
  }*/
  for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
   for(int snodo=1;snodo<=3;snodo++){ // per tutti gli snodi 
     if (folle[gamba][snodo]==0){  //il servo è in folle
          if (scheda[gamba][snodo]==1) pwm.setPWM(collegamento[gamba][snodo], 0, 0);
          if (scheda[gamba][snodo]==2) pwm2.setPWM(collegamento[gamba][snodo], 0, 0);                       
        }
        else
        {          
                   // il servo non è in folle, va quindi impostato all'angolo in posizioni[]
             if (posizioni[gamba][snodo]>=angolominimo[gamba][snodo]){
             if (posizioni[gamba][snodo]<=angolomassimo[gamba][snodo]){
                   // posizione nel range, muovo su posizione
                posizione=(vangolomassimo[gamba][snodo]-vangolominimo[gamba][snodo])*(posizioni[gamba][snodo]-angolominimo[gamba][snodo])/(angolomassimo[gamba][snodo]-angolominimo[gamba][snodo])+vangolominimo[gamba][snodo];   // inserire qui il calcolo della posizione del servo
             }
             else
             {
                   // posizione fuori range, muovo fine corsa superiore
                posizione=(vangolomassimo[gamba][snodo]-vangolominimo[gamba][snodo])*( angolomassimo[gamba][snodo]-angolominimo[gamba][snodo])/(angolomassimo[gamba][snodo]-angolominimo[gamba][snodo])+vangolominimo[gamba][snodo];    //servo posizionato all'angolo massimo       
             }
          }
          else
          {
                  // posizione fuori range, muovo fine corsa inferiore 
             posizione=(vangolomassimo[gamba][snodo]-vangolominimo[gamba][snodo])*( angolominimo[gamba][snodo]-angolominimo[gamba][snodo])/(angolomassimo[gamba][snodo]-angolominimo[gamba][snodo])+vangolominimo[gamba][snodo];    //servo posizionato all'angolo minimo
                
          }          
          if (scheda[gamba][snodo]==1) pwm.setPWM(collegamento[gamba][snodo], 0, posizione);
          if (scheda[gamba][snodo]==2) pwm2.setPWM(collegamento[gamba][snodo], 0, posizione);
        }  
     }
   }
 /*  if(videbug==1){    //debug
      Serial.println("");
      for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
         for(int snodo=1;snodo<=3;snodo++){ // per tutti gli snodi 
            Serial.print(posizioni[gamba][snodo]);
            Serial.print("°\t");
         }
      }   
      Serial.println("");
      Serial.println("X \tY \tZ \tX \tY \tZ \tX \tY \tZ \tX \tY \tZ \tX \tY \tZ \tX \tY \tZ \t");
          for(int gamba=1;gamba<=6;gamba++){  //per tutte le gambe
            Serial.print(posgambax[gamba]);
            Serial.print("\t");
            Serial.print(posgambay[gamba]);
            Serial.print("\t");
            Serial.print(posgambaz[gamba]);
            Serial.print("\t");
          }
      Serial.println("");
   }  // fine debug  
*/
}  //mettili
   


void calcolaxyz(){   //calcola le coordinate delle 6 zampe partendo da posizioni[(angoli)] => posgambaxé(mm)] 
  for(int gamba=1;gamba<=6;gamba++){
     int scapolax=pgx[gamba];   // calcolo posizione assoluta attacco gamba
     int scapolay=pgy[gamba];
     int scapolaz=0;
     double spallax=scapolax+cos((org[gamba]+posizioni[gamba][1])/180*3.1415926)*lspalla;   //  /180*3.141
     double spallay=scapolay+sin((org[gamba]+posizioni[gamba][1])/180*3.1415926)*lspalla;
     double spallaz=scapolaz;
     double femorex=spallax+cos((org[gamba]+posizioni[gamba][1])/180*3.1415926)*cos((posizioni[gamba][2])/180*3.1415926)*lfemore;    
     double femorey=spallay+sin((org[gamba]+posizioni[gamba][1])/180*3.1415926)*cos((posizioni[gamba][2])/180*3.1415926)*lfemore;
     double femorez=spallaz+sin((posizioni[gamba][2])/180*3.1415926)*lfemore;
     double tibiax=femorex+cos((org[gamba]+posizioni[gamba][1])/180*3.1415926)*cos((posizioni[gamba][2]+posizioni[gamba][3])/180*3.1415926)*ltibia;
     double tibiay=femorey+sin((org[gamba]+posizioni[gamba][1])/180*3.1415926)*cos((posizioni[gamba][2]+posizioni[gamba][3])/180*3.1415926)*ltibia;
     double tibiaz=femorez+sin((posizioni[gamba][2]+posizioni[gamba][3])/180*3.1415926)*ltibia;
     posgambax[gamba]= tibiax*1;
     posgambay[gamba]= tibiay*1;
     posgambaz[gamba]= tibiaz*1;
     
     if(debshow==1){//attivazione debug multigamba
         Serial.print("GAMBA");
         Serial.print(gamba);
         Serial.print("   alpha=");
         Serial.print(posizioni[gamba][1]);
         Serial.print("   betha=");
         Serial.print(posizioni[gamba][2]);
         Serial.print("   gamma=");
         Serial.println(posizioni[gamba][3]);
         Serial.print (" SCAPOLA   x=\t");
         Serial.print (scapolax);
         Serial.print ("\t y=\t");
         Serial.print (scapolay);
         Serial.print ("\t z=\t");
         Serial.println (scapolaz);

         Serial.print (" SPALLA   x=\t");
         Serial.print (spallax);
         Serial.print ("\t y=\t");
         Serial.print (spallay);
         Serial.print ("\t z=\t");
         Serial.println (spallaz);

         Serial.print (" FEMORE   x=\t");
         Serial.print (femorex);
         Serial.print ("\t y=\t");
         Serial.print (femorey);
         Serial.print ("\t z=\t");
         Serial.println (femorez);

         Serial.print (" TIBIA   x=\t");
         Serial.print (tibiax);
         Serial.print ("\t y=\t");
         Serial.print (tibiay);
         Serial.print ("\t z=\t");
         Serial.println (tibiaz);       
     }
  }
}




void calcolaabc (){    // dato posx[]  posy[]  posz[] calcola gli angoli e li mette in posfine[]
double argomento=0;
double numeratore;
double denominatore;
double angolo=0;
double angolo2=0;
double angolo1=0;
double lpg=0;        // lunghezza proiezione verticale gamba  
double ltf=0;        // lunghezza proiezione verticale tibia e femore 
double lcat2=0;      //  lunghezza tibia+femore al quadrato;
double lcat=0;       //  lunghezza tibia+femore al quadrato;

  for(int gamba=1;gamba<=6;gamba++){
     numeratore=posy[gamba]*100-pgy[gamba]*100;
     denominatore=posx[gamba]*100-pgx[gamba]*100;
     argomento = (numeratore/denominatore);
         //     angolo = atan (argomento) ;                       // calcolo classico
         //     posfine[gamba][1]=angolo*180/3.14159- org[gamba]; // calcolo classico
       posfine[gamba][1]=calcolaatan(argomento)-org[gamba];       // calcolo convettore testatan     
     if (posfine[gamba][1]<-90) posfine[gamba][1]=posfine[gamba][1]+180;    
     if (posfine[gamba][1]<-90) posfine[gamba][1]=posfine[gamba][1]+180;    
     if (posfine[gamba][1]<-90) posfine[gamba][1]=posfine[gamba][1]+180;    
     if (posfine[gamba][1]>90) posfine[gamba][1]=posfine[gamba][1]-180;   
     if (posfine[gamba][1]>90) posfine[gamba][1]=posfine[gamba][1]-180;   
     if (posfine[gamba][1]>90) posfine[gamba][1]=posfine[gamba][1]-180;
 
     lpg=sqrt ((denominatore/100)*(denominatore/100)+(numeratore/100)*(numeratore/100));
     ltf=lpg-lspalla;
     lcat2=(ltf*ltf)+((posz[gamba])*(posz[gamba]));
     
         //     angolo= -acos ((lcat2-lfemore*lfemore-ltibia*ltibia)/(2*ltibia*lfemore));
     posfine[gamba][3]=calcolagamma(lcat2);

   
         //     posfine[gamba][3]=angolo*180/3.14159;                                 //calcolo classico
         //     if (posfine[gamba][3]<-180) posfine[gamba][3]=posfine[gamba][3]+360;  // calcolo classico  
         //     if (posfine[gamba][3]>180) posfine[gamba][3]=posfine[gamba][3]-360;   // calcolo classico
     lcat=sqrt(lcat2);
         //     angolo1= acos ((lcat2+lfemore*lfemore-ltibia*ltibia)/(2*lfemore*lcat))*180/3.14159;  //calcolo classico
     int provvi=-posfine[gamba][3];
     angolo1=provaacos[provvi][2];       // calcolo rapido
         //     angolo2=atan(posz[gamba]/ltf)*180/3.14159;    // calcolo classico            
     angolo2=(calcolaatan(posz[gamba]/ltf));  // calcolo rapido    
     angolo=angolo2+angolo1;     
     posfine[gamba][2]=angolo;    
         //     if (posfine[gamba][2]<-180) posfine[gamba][2]=posfine[gamba][2]+360;    
         //     if (posfine[gamba][2]>180) posfine[gamba][2]=posfine[gamba][2]-360;        
  }
}






void ricezioneterminale (){
   Serial.print ("");
   while (mySerial.available() > 0) { 
   int rotazprov=0;  
   int direzprov=0;
   int distaprov=0;
   int velocprov=0;
  
  //   Serial.write(mySerial.read());
    
       incomingByte2 = mySerial.read();
       Serial.write(incomingByte2);

       if (incomingByte2==84){  // ricevuto T
               Serial.write ("T");
               sequenza=1;
               while (mySerial.available() > 0) {
                    incomingByte2 = mySerial.read();
                    delay (1);
                    if (incomingByte2==76 && sequenza==1){ // ricevuto TL
                       Serial.println ("L");
                       shiftorizzx=10;
                       shiftorizzy=0;
                       shiftorizz();
                       sequenza=2;
                       incomingByte2=0;
                       Serial.println ("IPT Shift orizzontale left");
                    }
                    
                    if (incomingByte2==82 && sequenza==1){ // ricevuto TR
                       Serial.println ("R");
                       shiftorizzx=-10;
                       shiftorizzy=0;
                       shiftorizz();
                       sequenza=2;
                       incomingByte2=0;
                       Serial.println ("IPT Shift orizzontale Right");
                    }
                    
                    if (incomingByte2==70 && sequenza==1){ // ricevuto TF
                       Serial.println ("F");
                       shiftorizzx=0;
                       shiftorizzy=-10;
                       shiftorizz();
                       sequenza=2;
                       incomingByte2=0;
                       Serial.println ("IPT Shift orizzontale Front");
                    }

                    if (incomingByte2==66 && sequenza==1){ // ricevuto TB
                       Serial.println ("B");
                       shiftorizzx=0;
                       shiftorizzy=10;
                       shiftorizz();
                       sequenza=2;
                       incomingByte2=0;
                       Serial.println ("IPT Shift orizzontale back");
                    }

               }
               
               if (sequenza==1) {
                 Serial.write (incomingByte2);
                 Serial.println (" IPT Wrong!");
               }
       }

       if (incomingByte2==86){  // ricevuto V
               Serial.write ("V");
               sequenza=1;
               while (mySerial.available() > 0) {
                    incomingByte2 = mySerial.read();
                    delay (1);
                    if (incomingByte2==65 && sequenza==1){ // ricevuto VA
                       Serial.println ("A");
                       dlarghezza=30;
                       daltezza=50;
                       varialarghezza(dlarghezza,daltezza); 
                       sequenza=2;
                       incomingByte2=0;
                       Serial.print ("IPT Allarga di ");
                       Serial.println(dlarghezza);
                    }

                    if (incomingByte2==83 && sequenza==1){ // ricevuto VS
                       Serial.println ("S");
                       dlarghezza=30;
                       daltezza=50;
                       varialarghezza(-dlarghezza,daltezza); 
                       sequenza=2;
                       incomingByte2=0;
                       Serial.print ("IPT Stringi di ");
                       Serial.println(dlarghezza);
                    }

                    if (incomingByte2==85 && sequenza==1){ // ricevuto VU
                       Serial.println ("U");   
                       variaaltezza(10);
                       sequenza=2;
                       incomingByte2=0;
                       Serial.print ("IPT Solleva ");
                    }

                    if (incomingByte2==68 && sequenza==1){ // ricevuto VD
                       Serial.println ("D");
                       variaaltezza(-10);
                       sequenza=2;
                       incomingByte2=0;
                       Serial.print ("IPT Abbassa ");
                    }                    
               }
               
               if (sequenza==1) {
                 Serial.write (incomingByte2);
                 Serial.println (" IPT Wrong!");
               }
       }
 
       if (incomingByte2==77){  // ricevuto M
               Serial.write ("M");
               sequenza=1;
               while (mySerial.available() > 0) {
                    incomingByte2 = mySerial.read();
                    delay (1);
                    if (incomingByte2==49 && sequenza==1){ // ricevuto M1
                       Serial.println ("1");
                       camminata1(1);
                       incomingByte2=0; 
                       sequenza=2;
                       Serial.print ("IPT Passo singolo direzione");
                       Serial.println(larghezza);
                       
                    }

                    if (incomingByte2==54 && sequenza==1){ // ricevuto M6
                       Serial.println ("6");
                       camminata(1);
                       incomingByte2=0;  
                       sequenza=2;
                       Serial.print ("IPT Passo 3 gambe direzione ");
                       Serial.println(larghezza);
                    }
               }     
               if (sequenza==1) {
                 Serial.println (" IPT Wrong!");
               }
       }
    
       if (incomingByte2==114){  // ricevuto r
               Serial.write ("r");
               sequenza=1;
               while (mySerial.available() > 0) {
                    incomingByte2 = mySerial.read();
                    delay (1);
                    if (incomingByte2==111 && sequenza==1){ // ricevuto ro
                       Serial.print ("o"); 
                       incomingByte2=0;                       
                       sequenza=2;   
                    }
                    if (incomingByte2==61 && sequenza==2){ // ricevuto ro=
                       Serial.print ("=");
                       incomingByte2=0;                        
                       sequenza=3;   
                    }
                     if (incomingByte2>=48 && incomingByte2<=57 && sequenza==3){
                       rotazprov=incomingByte2-48;
                       incomingByte2=0;
                       sequenza=4;            
                    }
                    if (incomingByte2>=48 && incomingByte2<=57 && sequenza==4){
                       rotazprov=rotazprov*10+incomingByte2-48;
                       incomingByte2=0;
                       sequenza=5;            
                     } 
                    if (incomingByte2>=48 && incomingByte2<=57 && sequenza==5){
                       rotazprov=rotazprov*10+incomingByte2-48;
                       incomingByte2=0;                       
                       sequenza=6;            
                     }  
               }  
               Serial.println(rotazprov);   
               if (sequenza<4) {
                 Serial.println (" IPr Wrong!");
               }
               else
               {
                rotazioneweb=rotazprov;
               }
       }

       if (incomingByte2==100){  // ricevuto d
               Serial.write ("d");
               sequenza=1;
               while (mySerial.available() > 0) {
                    incomingByte2 = mySerial.read();
                    delay (1);
                    if (incomingByte2==105 && sequenza==1){ // ricevuto di
                       Serial.print ("i"); 
                       incomingByte2=0;                       
                       sequenza=2;   
                    }
                    if (incomingByte2==115 && sequenza==1){ // ricevuto ds
                       Serial.print ("s"); 
                       incomingByte2=0;                       
                       sequenza=22;   
                    }
                    if (incomingByte2==61 && sequenza==2){ // ricevuto di=
                       Serial.print ("=");
                       incomingByte2=0;                        
                       sequenza=3;   
                    }
                    if (incomingByte2==61 && sequenza==22){ // ricevuto ds=
                       Serial.print ("=");
                       incomingByte2=0;                        
                       sequenza=23;   
                    }
                     if (incomingByte2>=48 && incomingByte2<=57 && sequenza==3){
                       direzprov=incomingByte2-48;
                       incomingByte2=0;
                       sequenza=4;            
                    }
                    if (incomingByte2>=48 && incomingByte2<=57 && sequenza==4){
                       direzprov=direzprov*10+incomingByte2-48;
                       incomingByte2=0;
                       sequenza=5;            
                     } 
                    if (incomingByte2>=48 && incomingByte2<=57 && sequenza==5){
                       direzprov=direzprov*10+incomingByte2-48;
                       incomingByte2=0;
                       sequenza=6;            
                     }  
                     
                     if (incomingByte2>=48 && incomingByte2<=57 && sequenza==23){
                       distaprov=incomingByte2-48;
                       incomingByte2=0;
                       sequenza=24;            
                    }
                    if (incomingByte2>=48 && incomingByte2<=57 && sequenza==24){
                       distaprov=distaprov*10+incomingByte2-48;
                       incomingByte2=0;
                       sequenza=25;            
                     } 
                    if (incomingByte2>=48 && incomingByte2<=57 && sequenza==25){
                       distaprov=distaprov*10+incomingByte2-48;
                       incomingByte2=0;
                       sequenza=26;            
                     }  
               }  
                                       
               if (sequenza>=4 and sequenza<=6) {
                  direzioneweb=direzprov;
                  Serial.println(direzprov);
               }
               else
               {               
                 if (sequenza>=24 and sequenza<=26){              
                    distanzaweb=distaprov;
                    Serial.println(distaprov);
                 }
                 else
                 {
                    Serial.println (" IPr Wrong!");
                 }
               }  
       }

       if (incomingByte2==115){  // ricevuto s
               Serial.write ("s");
               sequenza=1;
               while (mySerial.available() > 0) {
                    incomingByte2 = mySerial.read();
                    delay (1);
                    if (incomingByte2==112 && sequenza==1){ // ricevuto sp
                       Serial.print ("p"); 
                       incomingByte2=0;                       
                       sequenza=2;   
                    }
                    if (incomingByte2==61 && sequenza==2){ // ricevuto sp=
                       Serial.print ("=");
                       incomingByte2=0;                        
                       sequenza=3;   
                    }
                     if (incomingByte2>=48 && incomingByte2<=57 && sequenza==3){
                       velocprov=incomingByte2-48;
                       incomingByte2=0;
                       sequenza=4;            
                    }
                    if (incomingByte2>=48 && incomingByte2<=57 && sequenza==4){
                       velocprov=velocprov*10+incomingByte2-48;
                       incomingByte2=0;
                       sequenza=5;            
                     } 
                    if (incomingByte2>=48 && incomingByte2<=57 && sequenza==5){
                       velocprov=velocprov*10+incomingByte2-48;
                       incomingByte2=0;                       
                       sequenza=6;            
                     }  
               }  
               Serial.println(velocprov);   
               if (sequenza<4) {
                 Serial.println (" IPr Wrong!");
               }
               else
               {
                velocitaweb=velocprov;
               }
       }
    
   }
    
   if (Serial.available() > 0) {    // se è vero ho ricevuto un dato      
      incomingByte = Serial.read();  // leggo il byte ricevuto:
      if (incomingByte==83){  //comando S (posiziona per angoli)
         Serial.println();
         Serial.print ("Comando S Richiesta Servo per angoli: ");
         sequenza=1;       
         while (Serial.available() > 0) {
           incomingByte = Serial.read();           
           delay (1);
           
           if (incomingByte==103 && sequenza==1){
             Serial.print ("Gamba= ");
             sequenza=2;            
           }
           if (incomingByte>=49 && incomingByte<=54 && sequenza==2){
             ricgamba=incomingByte-48;
             Serial.print (ricgamba);
             Serial.print (" ");
             sequenza=3;            
           }
                    
           if (incomingByte==115 && sequenza==3){
             Serial.print ("Snodo= ");
             sequenza=4;            
           }           
           if (incomingByte>=49 && incomingByte<=54 && sequenza==4){
             ricsnodo=incomingByte-48;
             Serial.print (ricsnodo);
             Serial.print (" ");
             sequenza=5;            
           }        

            if (incomingByte==97 && sequenza==5){
             Serial.print ("Angolo= ");
             sequenza=6;            
           }      
           if (incomingByte==45 && sequenza==6){
             Serial.print ("-");
             segno=-1;
             sequenza=7;            
           }
           if (incomingByte==43 && sequenza==6){
             Serial.print ("+");
             segno=+1;
             sequenza=7;
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==7){
             ricangolo=(incomingByte-48);
             incomingByte=0;
             sequenza=8;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==8){
             ricangolo=ricangolo*10 +incomingByte-48;
             incomingByte=0;
             sequenza=9;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==9){
             ricangolo=(ricangolo*10 +incomingByte-48)*segno;
             Serial.print (ricangolo);
             Serial.println(" OK!");
             sequenza=10;
             comando=83;            
           }                   
         }
       if (sequenza!=10) Serial.println ("WRONG!!!"); 
        
       }// fine comando S


     if (incomingByte==72){  //comando H (Help)
         Serial.println();
         Serial.println ("Comando H help: ");
         chelp();
     }// fine comando S

 if (incomingByte==86){  //comando V (Altezza)
         Serial.println();
         Serial.print ("Comando V varia altezza: ");
         sequenza=1;       
         while (Serial.available() > 0) {
           incomingByte = Serial.read();           
           delay (1);
            if (incomingByte==105 && sequenza==1){  // Ai
             Serial.print (" Incremento altezza= ");
             sequenza=2;            
           } 
          if (incomingByte==45 && sequenza==2){    // Ai-
             //Serial.print ("-");
             segno=-1;
             sequenza=3;            
           }
           if (incomingByte==43 && sequenza==2){    //Ai+
             //Serial.print ("+");
             segno=+1;
             sequenza=3;
           }
            if (incomingByte>=48 && incomingByte<=57 && sequenza==3){
             ricz=(incomingByte-48);                   //Ai+1
             incomingByte=0;
             sequenza=4;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==4){
             ricz=ricz*10 +incomingByte-48;            //Ai+11
             incomingByte=0;
             sequenza=5;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==5){
             ricz=(ricz*10 +incomingByte-48)*segno;    //Ai+111 
             Serial.print (ricz);
             sequenza=6; 
             Serial.println (" OK!"); 
             comando=86;          
           }
         }
         if (sequenza!=6) Serial.println ("WRONG!!!"); 
       }// fine comando V   




     if (incomingByte==65){  //comando A (Altezza)
         Serial.println();
         Serial.print ("Comando A altezza: ");
         sequenza=1;       
         while (Serial.available() > 0) {
           incomingByte = Serial.read();           
           delay (1);
            if (incomingByte==122 && sequenza==1){  // Az
             Serial.print (" Altezza= ");
             sequenza=2;            
           } 
          if (incomingByte==45 && sequenza==2){    // Az-
             //Serial.print ("-");
             segno=-1;
             sequenza=3;            
           }
           if (incomingByte==43 && sequenza==2){    //Az+
             //Serial.print ("+");
             segno=+1;
             sequenza=3;
           }
            if (incomingByte>=48 && incomingByte<=57 && sequenza==3){
             ricz=(incomingByte-48);                   //Az+1
             incomingByte=0;
             sequenza=4;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==4){
             ricz=ricz*10 +incomingByte-48;            //Az+11
             incomingByte=0;
             sequenza=5;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==5){
             ricz=(ricz*10 +incomingByte-48)*segno;    //Az+111 
             Serial.print (ricz);
             sequenza=6; 
             Serial.println (" OK!"); 
             comando=65;          
           }
         }
         if (sequenza!=6) Serial.println ("WRONG!!!"); 
       }// fine comando A   

     if (incomingByte==68){  //comando D (Divarica gambe)
         Serial.println();
         Serial.print ("Comando D divarica gambe: ");
         sequenza=1;       
         while (Serial.available() > 0) {
           incomingByte = Serial.read();           
           delay (1);
   
           if (incomingByte==105 && sequenza==1){   // Di
             Serial.print (" Incremento= ");
             sequenza=2;            
           }
           if (incomingByte==45 && sequenza==2){    // Di-
             //Serial.print ("-");
             segno=-1;
             sequenza=3;            
           }
           if (incomingByte==43 && sequenza==2){    //Di+
             //Serial.print ("+");
             segno=+1;
             sequenza=3;
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==3){  //Di+1
             dlarghezza=(incomingByte-48);                   
             incomingByte=0;
             sequenza=4;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==4){  //Di+11
             dlarghezza=dlarghezza*10 +incomingByte-48;          
             incomingByte=0;
             sequenza=5;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==5){  //Di+111
             dlarghezza=(dlarghezza*10 +incomingByte-48)*segno;  
             Serial.print (ricz);
             sequenza=6; 
           }
           if (incomingByte==104 && sequenza==6){      //  Di+111h
             Serial.print (" Sollevamento= ");
             sequenza=7;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==7){ //  Di+111h2
             daltezza=(incomingByte-48);                  
             incomingByte=0;
             sequenza=8;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==8){ //  Di+111h22
             daltezza=daltezza*10 +incomingByte-48;           
             incomingByte=0;
             sequenza=9;  
             Serial.println (" OK!"); 
             comando=68;          
           }
         }
         if (sequenza!=9) Serial.println ("WRONG!!!"); 
       }// fine comando D   




      if (incomingByte==70){  //comando F (Flessioni)
         Serial.println();
         Serial.print ("Comando F flessioni: ");
         sequenza=1;       
         while (Serial.available() > 0) {
           incomingByte = Serial.read();           
           delay (1);
           
           if (incomingByte==110 && sequenza==1){
             Serial.print (" Numero= ");
             sequenza=2;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==2){
             ricnumerof=(incomingByte-48);
             incomingByte=0;
             sequenza=3;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==3){
             ricnumerof=ricnumerof*10 +incomingByte-48;
             incomingByte=0;
             Serial.print (ricnumerof);
             sequenza=4;            
           }
           if (incomingByte==112 && sequenza==4){
             Serial.print (" Passi= ");
             sequenza=5;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==5){
             ricpassif=(incomingByte-48);
             incomingByte=0;
             sequenza=6;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==6){
             ricpassif=ricpassif*10 +incomingByte-48;
             incomingByte=4;
             sequenza=7; 
             Serial.print (ricpassif); 
           }  
           if (incomingByte==97 && sequenza==7){
             Serial.print (" Altezza= ");
             sequenza=8;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==8){
             ricaltezza=(incomingByte-48);
             incomingByte=0;
             sequenza=9;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==9){
             ricaltezza=ricaltezza*10 +incomingByte-48;
             incomingByte=0;
             sequenza=10; 
             Serial.print (ricaltezza);           
             comando=70;             
             Serial.println(" OK!");         
           } 
         }
      if (sequenza!=10) Serial.println (" WRONG!!!"); 
          
      }// fine comando 70 

                  
       if (incomingByte==71){  //comando G (girogambe)
            // sintassi: Gn1   dove 1 è il numero di giri          
         Serial.println();
         Serial.print ("Comando G Girogambe ");
         sequenza=1;       
         while (Serial.available() > 0) {
           incomingByte = Serial.read();           
           delay (1);
           
           if (incomingByte==110 && sequenza==1){
            Serial.print ("Numero giri= ");
             sequenza=2;            
           }
           if (incomingByte>=49 && incomingByte<=54 && sequenza==2){
             ricnumerog=incomingByte-48;
             Serial.print (ricnumerog);
             Serial.print (" ");
             sequenza=3;          
             comando=71;  
           }
         }  
       if (sequenza!=3) Serial.println ("WRONG!!!"); 
       
       }// fine comando 71




       if (incomingByte==80){  //comando P (posiziona in mm)
            // sintassi: Pg1x+222y+333z+444   dove 1 è il numero di gamba  + è il segno 
         
         Serial.println();
         Serial.print ("Comando P Richiesta posizionamento gamba: ");
         sequenza=1;       
         while (Serial.available() > 0) {
           incomingByte = Serial.read();           
           delay (1);
           
           if (incomingByte==103 && sequenza==1){
            Serial.print ("Gamba= ");
             sequenza=2;            
           }
           if (incomingByte>=49 && incomingByte<=54 && sequenza==2){
             ricgamba=incomingByte-48;
             Serial.print (ricgamba);
             Serial.print (" ");
             sequenza=3;            
           }


            if (incomingByte==120 && sequenza==3){
             Serial.print (" x=");
             sequenza=4;            
           }      
           if (incomingByte==45 && sequenza==4){
             //Serial.print ("-");
             segno=-1;
             sequenza=5;            
           }
           if (incomingByte==43 && sequenza==4){
             //Serial.print ("+");
             segno=+1;
             sequenza=5;
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==5){
             ricx=(incomingByte-48);
             incomingByte=0;
             sequenza=6;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==6){
             ricx=ricx*10 +incomingByte-48;
             incomingByte=0;
             sequenza=7;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==7){
             ricx=(ricx*10 +incomingByte-48)*segno;
             Serial.print (ricx);
             sequenza=8;            
           }

            if (incomingByte==121 && sequenza==8){
             Serial.print (" y=");
             sequenza=9;            
           }      
           if (incomingByte==45 && sequenza==9){
             //Serial.print ("-");
             segno=-1;
             sequenza=10;            
           }
           if (incomingByte==43 && sequenza==9){
             //Serial.print ("+");
             segno=+1;
             sequenza=10;
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==10){
             ricy=(incomingByte-48);
             incomingByte=0;
             sequenza=11;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==11){
             ricy=ricy*10 +incomingByte-48;
             incomingByte=0;
             sequenza=12;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==12){
             ricy=(ricy*10 +incomingByte-48)*segno;
             Serial.print (ricy);
             sequenza=13;            
           }


            if (incomingByte==122 && sequenza==13){
             Serial.print (" z=");
             sequenza=14;            
           }      
           if (incomingByte==45 && sequenza==14){
             //Serial.print ("-");
             segno=-1;
             sequenza=15;            
           }
           if (incomingByte==43 && sequenza==14){
             //Serial.print ("+");
             segno=+1;
             sequenza=15;
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==15){
             ricz=(incomingByte-48);
             incomingByte=0;
             sequenza=16;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==16){
             ricz=ricz*10 +incomingByte-48;
             incomingByte=0;
             sequenza=17;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==17){
             ricz=(ricz*10 +incomingByte-48)*segno;
             Serial.print (ricz);
             sequenza=18; 
             Serial.println (" OK!"); 
             comando=80;          
           }
           
           
         }
         if (sequenza!=18) Serial.println ("WRONG!!!"); 
       }// fine comando P   



     if (incomingByte==84){  //comando T (Trasla)
         Serial.println();
         Serial.print ("Comando T trasla: ");
         sequenza=1;       
         while (Serial.available() > 0) {
           incomingByte = Serial.read();           
           delay (1);
   
           if (incomingByte==120 && sequenza==1){   // Tx
             Serial.print (" IncrX= ");
             sequenza=2;            
           }
           if (incomingByte==45 && sequenza==2){    // Tx-
             //Serial.print ("-");
             segno=-1;
             sequenza=3;            
           }
           if (incomingByte==43 && sequenza==2){    //Tx+
             //Serial.print ("+");
             segno=+1;
             sequenza=3;
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==3){  //Tx+1
             shiftorizzx=(incomingByte-48);                   
             incomingByte=0;
             sequenza=4;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==4){  //Tx+11
             shiftorizzx=shiftorizzx*10 +incomingByte-48;
             shiftorizzx=shiftorizzx*segno;
             Serial.print(shiftorizzx);
             shiftorizzx=shiftorizzx * (-1);            
             incomingByte=0;
             sequenza=5;            
           }
           if (incomingByte==121 && sequenza==5){   // Tx+11y
             Serial.print (" IncrY= ");
             sequenza=6;            
           }
           if (incomingByte==45 && sequenza==6){    // Tx+11y-
             //Serial.print ("-");
             segno=-1;
             sequenza=7;            
           }
           if (incomingByte==43 && sequenza==6){    //Tx+11y+
             //Serial.print ("+");
             segno=+1;
             sequenza=7;
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==7){  //Tx+11y+2
             shiftorizzy=(incomingByte-48);                   
             incomingByte=0;
             sequenza=8;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==8){  //Tx+11y+22
             shiftorizzy=shiftorizzy*10 +incomingByte-48;
             shiftorizzy=shiftorizzy*segno;
             Serial.print(shiftorizzy);
             shiftorizzy=shiftorizzy * (-1);                      
             incomingByte=0;
             sequenza=9; 
             comando=84;
             Serial.println (" OK!");           
           }
         }
         if (sequenza!=9) Serial.println ("WRONG!!!"); 
       }// fine comando T   



     if (incomingByte==77){  //comando M (Marcia)
         Serial.println();
         Serial.print ("Comando M marcia: ");
         sequenza=1;       
         while (Serial.available() > 0) {
           incomingByte = Serial.read();           
           delay (1);
   
           if (incomingByte==120 && sequenza==1){   // Mx
             Serial.print (" IncrX= ");
             sequenza=2;            
           }
           if (incomingByte==45 && sequenza==2){    // Mx-
             //Serial.print ("-");
             segno=-1;
             sequenza=3;            
           }
           if (incomingByte==43 && sequenza==2){    //Mx+
             //Serial.print ("+");
             segno=+1;
             sequenza=3;
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==3){  //Mx+1
             ricvariax=(incomingByte-48);                   
             incomingByte=0;
             sequenza=4;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==4){  //Mx+11
             ricvariax=ricvariax*10 +incomingByte-48;
             ricvariax=ricvariax*segno;
             Serial.print(ricvariax);
             ricvariax=ricvariax * (-1);            
             incomingByte=0;
             sequenza=5;            
           }
           if (incomingByte==121 && sequenza==5){   // Mx+11y
             Serial.print (" IncrY= ");
             sequenza=6;            
           }
           if (incomingByte==45 && sequenza==6){    // Mx+11y-
             //Serial.print ("-");
             segno=-1;
             sequenza=7;            
           }
           if (incomingByte==43 && sequenza==6){    //Mx+11y+
             //Serial.print ("+");
             segno=+1;
             sequenza=7;
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==7){  //Mx+11y+2
             ricvariay=(incomingByte-48);                   
             incomingByte=0;
             sequenza=8;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==8){  //Mx+11y+22
             ricvariay=ricvariay*10 +incomingByte-48;
             ricvariay=ricvariay*segno;
             Serial.print(ricvariay);
             ricvariay=ricvariay * (-1);                      
             incomingByte=0;
             sequenza=9;           
           }


           
           if (incomingByte==114 && sequenza==9){   // Mx+11y+22r
             Serial.print (" Rotaz= ");
             sequenza=10;            
           }
           if (incomingByte==45 && sequenza==10){    // Mx+11y+22r-
             //Serial.print ("-");
             segno=-1;
             sequenza=11;            
           }
           if (incomingByte==43 && sequenza==10){    //Mx+11y+22r+
             //Serial.print ("+");
             segno=+1;
             sequenza=11;
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==11){  //Mx+11y+22r+3
             ricrotazione=(incomingByte-48);                   
             incomingByte=0;
             sequenza=12;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==12){  //Mx+11y+22r+33
             ricrotazione=ricrotazione*10 +incomingByte-48;
             ricrotazione=ricrotazione*segno;
             Serial.print(ricrotazione);
             ricrotazione=ricrotazione * (-1);                      
             incomingByte=0;
             sequenza=13; 
             comando=77;
             Serial.println (" OK!");           
           }
         }
         if (sequenza!=13) Serial.println ("WRONG!!!"); 
       }// fine comando M   



     if (incomingByte==67){  //comando C (Camminata)
         Serial.println();
         Serial.print ("Comando C camminata: ");
         sequenza=1;       
         while (Serial.available() > 0) {
           incomingByte = Serial.read();           
           delay (1);
   
           if (incomingByte==120 && sequenza==1){   // Cx
             Serial.print (" IncrX= ");
             sequenza=2;            
           }
           if (incomingByte==45 && sequenza==2){    // Cx-
             //Serial.print ("-");
             segno=-1;
             sequenza=3;            
           }
           if (incomingByte==43 && sequenza==2){    //Cx+
             //Serial.print ("+");
             segno=+1;
             sequenza=3;
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==3){  //Cx+1
             ricvariax=(incomingByte-48);                   
             incomingByte=0;
             sequenza=4;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==4){  //Cx+11
             ricvariax=ricvariax*10 +incomingByte-48;
             ricvariax=ricvariax*segno;
             Serial.print(ricvariax);
             ricvariax=ricvariax * (-1);            
             incomingByte=0;
             sequenza=5;            
           }
           if (incomingByte==121 && sequenza==5){   // Cx+11y
             Serial.print (" IncrY= ");
             sequenza=6;            
           }
           if (incomingByte==45 && sequenza==6){    // Cx+11y-
             //Serial.print ("-");
             segno=-1;
             sequenza=7;            
           }
           if (incomingByte==43 && sequenza==6){    //Cx+11y+
             //Serial.print ("+");
             segno=+1;
             sequenza=7;
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==7){  //Cx+11y+2
             ricvariay=(incomingByte-48);                   
             incomingByte=0;
             sequenza=8;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==8){  //Cx+11y+22
             ricvariay=ricvariay*10 +incomingByte-48;
             ricvariay=ricvariay*segno;
             Serial.print(ricvariay);
             ricvariay=ricvariay * (-1);                      
             incomingByte=0;
             sequenza=9;           
           }


           
           if (incomingByte==114 && sequenza==9){   // Cx+11y+22r
             Serial.print (" Rotaz= ");
             sequenza=10;            
           }
           if (incomingByte==45 && sequenza==10){    // Cx+11y+22r-
             //Serial.print ("-");
             segno=-1;
             sequenza=11;            
           }
           if (incomingByte==43 && sequenza==10){    //Cx+11y+22r+
             //Serial.print ("+");
             segno=+1;
             sequenza=11;
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==11){  //Cx+11y+22r+3
             ricrotazione=(incomingByte-48);                   
             incomingByte=0;
             sequenza=12;            
           }
           if (incomingByte>=48 && incomingByte<=57 && sequenza==12){  //Cx+11y+22r+33
             ricrotazione=ricrotazione*10 +incomingByte-48;
             ricrotazione=ricrotazione*segno;
             Serial.print(ricrotazione);
             ricrotazione=ricrotazione * (-1);                      
             incomingByte=0;
             sequenza=13; 
             comando=67;
             Serial.println (" OK!");           
           }
         }
         if (sequenza!=13) Serial.println ("WRONG!!!"); 
       }// fine comando C   




     
    }// fine ricezione dati
}


void gestionecomandi() {
  if (comando==71){  //inviato Gn[numero(1)]
      girogambe(ricnumerog);
      comando=0;
  }
   
  if (comando==83){  //inviato Sg[gamba(1)]s[snodo(1)]a[-/+(1)][angolo(3)]
    posizioni[ricgamba][ricsnodo]=ricangolo;
    calcolaxyz ();   //  dal valore di posizioni iniziali calcola le coordinate di tutte le zampe
    mettili();
    comando=0;
    //debugcalcolaabc();
  }  // comando83   S

  if (comando==80){  // inviato Pg[gamba(1)]x[-/+(1)][x(3)]y[-/+(1)][y(3)]z[-/+(1)][z(3)]   
    posx[ricgamba]=ricx;
    posy[ricgamba]=ricy;
    posz[ricgamba]=ricz;
    calcolaabc();    
      for(int snodo=1;snodo<=3;snodo++){ // per tutti gli snodi 
        posizioni[ricgamba][snodo]=posfine[ricgamba][snodo];
      }
    mettili();
    comando=0;
    calcolaabc();
  }  //comando 80  P

  if (comando==70){  // inviato Fn[numero(2)]p[passi(2)]a[altezza(2)]
    flessioni(ricnumerof,ricpassif,ricaltezza);
    comando=0;
    calcolaabc();   
  }  // comando 70 F

  if (comando==65){  // inviato Az+[altezza(3)]
    setaltezza(ricz);
    comando=0;
    calcolaabc();   
  }  // comando 65 A


  if (comando==68){  // inviato Di+[incremento(3)]h[altezza(2)]
    varialarghezza(dlarghezza,daltezza);
    comando=0;
    calcolaabc();   
  }  // comando 68 D

  
  if (comando==86){  // inviato Vi+[altezza(3)]
    Serial.println("variaalt");
    variaaltezza(ricz);
    comando=0;
    calcolaabc();   
  }  // comando 86 V

 if (comando==84){  // inviato Tx+[incrX(2)]y+[incrY(2)]
    Serial.println("trasla");
    shiftorizz();
    comando=0;
    calcolaabc();   
  }  // comando 86 V

 
 if (comando==77){  // inviato Mx+[incrX(2)]y+[incrY(2)]r+[rotaz(2)]
    Serial.println("marcia");
    camminata(2);
    comando=0;
    calcolaabc();   
  }  // comando 77 M 

 
 if (comando==67){  // inviato Cx+[incrX(2)]y+[incrY(2)]r+[rotaz(2)]
    Serial.println("camminata");
    camminata1(2);
    comando=0;
    calcolaabc();   
  }  // comando 67 M 
 
}// fine gestionecomandi


void ricezionetelecomando(){
  delay (20);
  Wire.requestFrom (7,7);          //era (7,6)
  int contatel=0;
  while(Wire.available()){
    char ctel=Wire.read();
 //   Serial.print (ctel,DEC);
    datotel[contatel]=ctel;
       contatel++;
    
    // qui il master riceverà i 6 byte e li gestirà
  }
  delay (20);

   if(bitRead(datotel[5], 2)==1 and bitRead(datotel[5], 3)==1){
   //errore ricezione
   Serial.println("errore");
   datotel[1]=128;
   datotel[2]=127;
   datotel[3]=128;
   datotel[4]=127;
   datotel[5]=0;
   datotel[6]=0;    
  }
 /* 
  Serial.print (datotel[0],DEC);
  Serial.print ("\t");
  Serial.print (datotel[1],DEC);
  Serial.print ("\t");
  Serial.print (datotel[2],DEC);
  Serial.print ("\t");
  Serial.print (datotel[3],DEC);
  Serial.print ("\t");
  Serial.print (datotel[4],DEC);
  Serial.print ("\t");
  Serial.print (datotel[5],DEC);
  Serial.print ("\t");
  Serial.print (datotel[6],DEC);
  Serial.print ("\t");  
  Serial.println("OK");
*/
   if (bitRead(datotel[6], 3)==0 and passo6==1){  // non continua il passo
     riabbasso6();
   }
 
  if (bitRead(datotel[5], 2)==1){    // tasto sali
    bitClear(datotel[5], 2);
    delay (1);  
    Serial.print ("TLC Varia altezza (SALI)");
    variaaltezza(10);
    delay(1);   
  }
  
  if (bitRead(datotel[5], 3)==1){  // tasto scendi
    bitClear(datotel[5], 3);
    delay(1);  
    Serial.print ("TLC Varia altezza (SCENDI)");
    variaaltezza(-10);   
    delay(1); 
  }


if (bitRead(datotel[5], 0)==1){  // tasto stringi
    bitClear(datotel[5], 0);
    delay(1);  
    Serial.print ("TLC stringi");
    dlarghezza=30;
    daltezza=50;
    varialarghezza(-dlarghezza,daltezza);   
    delay(1); 
  }

if (bitRead(datotel[5], 1)==1){  // tasto allarga
    bitClear(datotel[5], 1);
    delay(1);  
    Serial.print ("TLC allarga");
    dlarghezza=30;
    daltezza=50;
    varialarghezza(dlarghezza,daltezza);   
    delay(1); 
  }

if (bitRead(datotel[5], 4)==1){  // tasto shiftorizz destra
    shiftorizzx=+10;
    shiftorizzy=0; 
    bitClear(datotel[5], 4);
    delay(1);  
    Serial.print ("TLC shiftorizzx=");
    Serial.print (shiftorizzx);
    Serial.print ("  shiftorizzy=");
    Serial.println (shiftorizzy);
    shiftorizz();   
    delay(1); 
  }

if (bitRead(datotel[5], 5)==1){  // tasto shiftorizz sinistra
    shiftorizzx=-10;
    shiftorizzy=0; 
    bitClear(datotel[5], 5);
    delay(1);  
    Serial.print ("TLC shiftorizzx=");
    Serial.print (shiftorizzx);
    Serial.print ("  shiftorizzy=");
    Serial.println (shiftorizzy);
    shiftorizz();   
    delay(1); 
  }  

if (bitRead(datotel[5], 6)==1){  // tasto shiftorizz indietro
    shiftorizzx=0;
    shiftorizzy=-10; 
    bitClear(datotel[5], 6);
    delay(1);  
    Serial.print ("TLC shiftorizzx=");
    Serial.print (shiftorizzx);
    Serial.print ("  shiftorizzy=");
    Serial.println (shiftorizzy);
    shiftorizz();   
    delay(1); 
  } 

if (bitRead(datotel[5], 7)==1){  // tasto shiftorizz avanti
    shiftorizzx=0;
    shiftorizzy=+10; 
    bitClear(datotel[5], 7);
    delay(1);  
    Serial.print ("TLC shiftorizzx=");
    Serial.print (shiftorizzx);
    Serial.print ("  shiftorizzy=");
    Serial.println (shiftorizzy);
    shiftorizz();   
    delay(1); 
  } 
  
  if (bitRead(datotel[6], 1)==1){  // tasto L2 (alto)
    bitClear(datotel[6], 1);      
    Serial.println ("TLC girogambe");    
    girogambe(1);
    delay(200);
  }

 if (bitRead(datotel[6], 0)==1){  // tasto L2 (basso)
    bitClear(datotel[6], 1);      
    Serial.println ("TLC riassetto ");    
    riassetto();
    delay(200);
  }

  if (bitRead(datotel[6], 3)==1){  // tasto R1 (alto)
    bitClear(datotel[6], 3);      
    Serial.println ("camminata 6 g");    
    camminata(0);
    delay(200);
  } 
  if (bitRead(datotel[6], 4)==1){  // tasto R2 (basso)
    bitClear(datotel[6], 4);      
    Serial.println ("camminata 1 g");    
    camminata1(0);
    delay(200);
  }  
} // fine ricezione telecomando


// ottimizzazioni per aumentare la velocità
void caricaatan(){
    for(int ga=0;ga<=180;ga++){  
       provaatan[ga]=tan((ga-90)*3.14159/180);
       Serial.print(ga);
       Serial.print(" => ");
       Serial.println (provaatan[ga]);
    }
    for(int ga=0;ga<=125;ga++){
       provaacos[ga][0]=2*cos(ga*3.14159/180)*lfemore*ltibia+lfemore*lfemore+ltibia*ltibia;
       provaacos[ga][1]=sqrt(provaacos[ga][0]);
       provaacos[ga][2]=acos((provaacos[ga][0]+lfemore*lfemore-ltibia*ltibia)/(2*lfemore*provaacos[ga][1]))*180/3.14159;
       
       Serial.print(ga);
       Serial.print(" lc2= ");
       Serial.print (provaacos[ga][0]);
       Serial.print(" lc= ");
       Serial.print (provaacos[ga][1]);
       Serial.print(" iota= ");
       Serial.println (provaacos[ga][2]);
    }
    
}

int calcolaatan(double argomento){
  int medio=0;
  int minimo=0;
  int massimo=180;
  while (massimo>(minimo+1)){
    medio=int((massimo+minimo+1)/2);
    if (argomento>=provaatan[medio]){
       minimo=medio;
    }
    else
    {
       massimo=medio;
    }
  }
  massimo=massimo-90;
  return massimo;
}

int calcolagamma(double argomento){
  int medio=0;
  int minimo=0;
  int massimo=125;
   while (massimo>(minimo+1)){
    medio=int((massimo+minimo+1)/2);
    if (argomento<=provaacos[medio][0]){
       minimo=medio;      
    }
    else
    {
       massimo=medio;
    }
   }
   massimo=-massimo;
   return massimo;
}


void chelp(){
  Serial.println("COMANDO            SIGNIFICATO");
  Serial.println("Gn1                GIROGAMBE n=numero giri           ");
  Serial.println("Sg1s2a+333         SET SERVO g=gamba s=servo a=angolo");
  Serial.println("Pg1x+111y+222z+333 POSIZIONA GAMBA g=gamba x=cordx y=cordy z=cordz");
  Serial.println("Fn11p22            FLESSIONI n=numero p=passi        ");
  Serial.println("Az+111             ALTEZZA z=altezza                 ");
  Serial.println("Vi+111             VARIA ALTEZZA i=incremento        ");
  Serial.println("Di+111h22          DIVARICA GAMBE i=incremento h=altezza");
  Serial.println("Cx+11y+22r+33      CAMMINATA x=direz.X y=direz.Y r=rotaz");
  Serial.println("Mx+11y+22r+33      MARCIA x=direz.X y=direz.Y r=rotaz");
  Serial.println("Tx+11y+11          TRASLA x=direz.X y=direz.Y");
  Serial.println("H                  HELP");
  Serial.println("");
}

