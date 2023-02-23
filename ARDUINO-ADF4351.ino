//   ADF4251 and Arduino
//   By Alain Fort F1CJN feb 2,2016
//   update march 7, 2016 (ROBOT V1.1 and V1.0)
//
//
//  ****************************************************** FRANCAIS ****************************************************
//   Ce programme utilise un Arduino Uno muni d'un "LCD button shield" de marque ROBOT, avec boutons permettant de commander
//   un ADF4351 qui generer une frequence comprise entre 34,5 et 4400MHz.
//   Vingt fréquences peuvent être memorisees dans le memoire EEPROM de l'Arduino.
//   Si une ou plusieurs fréquence sont mémorisées, alors la fréquence en memoire zero sera affichee à la mise sous tension
//   
//   Le curseur se deplace avec les touches LEFT and RIGHT. Les digits placés sous le curseur peuvent être modifiées avec
//   les touches UP et DOWN, ceci pour la fréquence, la memoire et la frequence de reference:
//   - pour la fréquence, il suffit de placer le curseur sur le chiffre à modifier,
//   - pour la mémoire , il suffit de placer le curseur sur le numero de memoire,
//   - pour la fréqence de référence, il suffit de placer le curseur sur 10 ou 25,
//   - pour la lecture ou écriture de la frequence en memoire, placer le curseur en bas à gauche (passage de REE(lecture
//    EEPROM) à WEE(Ecriture EEPROM).
//    Le curseur disparait apres quelques secondes et est re active lors de l'appui sur une touche.
//
//   MEMORISATION 
//   - pour la frequennce, mettre à WEE, puis selectionner le numero de memoire, puis appuyer sur la touche SELECT pendant 
//    une seconde. Le mot MEMORISATION apparait alors sur l'ecran. Ceci fonctionne quelquesoit le placement du curseur excepte sur 
//    l'emplacement de la fréquence de réference 10 ou 25.
//   - Pour la frequence de reference, placer le curseur sur 10 ou 25, puis appuyer pendant 1s sur la touche SELECT.
//
//  ********************************************* HARDWARE IMPORTANT *******************************************************
//  Avec un Arduino UN0 : utilise un pont de résistances pour réduire la tension, MOSI (pin 11) vers
//  ADF DATA, SCK (pin13) vers CLK ADF, Select (PIN 3) vers LE
//  Resistances de 560 Ohm avec 1000 Ohm à la masse sur les pins 11, 13 et 3 de l'Arduino UNO pour
//  que les signaux envoyés DATA, CLK et LE vers l'ADF4351 ne depassent pas 3,3 Volt.
//  Pin 2 de l'Arduino (pour la detection de lock) connectee directement à la sortie MUXOUT de la carte ADF4351
//  La carte ADF est alimentée en 5V par la carte Arduino (les pins +5V et GND sont proches de la LED Arduino).
//  ***********************************************************************************************************************
//  Attention : si vous utiliser un afficheur ROBOT Version 1.1 il faut modifier la routine de lecture des boutons
//  en enlevant les commantaires de la version 1.1 et en mettant en commentaires la version 1.0
//
//  *************************************************** ENGLISH ***********************************************************
//
//  This sketch uses and Arduino Uno (5€), a standard "LCD buttons shield" from ROBOT (5€), with buttons and an ADF4351 chineese
//  card found at EBAY (40€). The frequency can be programmed between 34.5 and 4400 MHz.
//  Twenty frequencies can be memorized into the Arduino EEPROM.
//  If one or more frequencies are memorized, then at power on, the memory zero is always selected.
//
//   The cursor can move with le LEFT and RIGHT buttons. Then the underlined digit can be modified with the UP and DOWN buttons, 
//    for the frequency, the memories and the frequency reference (10 or 25 MHz):
//   - to change the frequency, move the cursor to the digit to be modified, then use the UP and DOWN buttons,
//   - to modify the memory number,move the cursor to the number to be modified, then use the UP and DOWN buttons,
//   - to select the refrence frequence,move the cursor on 10 or 25 and select with UP and DOWN.
//   - to read or write the frequency in memory, place the cursor on the more left/more down position and select REE (for Reading EEprom)
//    or WEE (for Writing EEprom).
//    The cursor dissapears after few seconds and is re activated if a button is pressed.
//
//   MEMORIZATION 
//    - For the frequency, select WEE, then select the memory number, then push the SELECT button for a second. The word MEMORISATION 
//    appears on the screen. This memorization works then the cursor is anywhere except on the reference 10 or 25 position.
//    - For the reference frequency, move the cursor to 10 or 25, the press the SELECT for one second. 

//  ******************************************** HARDWARE IMPORTANT********************************************************
//  With an Arduino UN0 : uses a resistive divider to reduce the voltage, MOSI (pin 11) to
//  ADF DATA, SCK (pin13) to ADF CLK, Select (PIN 3) to ADF LE
//  Resistive divider 560 Ohm with 1000 Ohm to ground on Arduino pins 11, 13 et 3 to adapt from 5V
//  to 3.3V the digital signals DATA, CLK and LE send by the Arduino.
//  Arduino pin 2 (for lock detection) directly connected to ADF4351 card MUXOUT.
//  The ADF card is 5V powered by the ARDUINO (PINs +5V and GND are closed to the Arduino LED).

//************************************************* MANUEL*****************************************************************
//Touche LEFT    curseur à gauche, cursor to the left
//Touche RIGHT   curseur à droite, cursor to the right
//Touche UP      incremente frequence ou memoire, increase frequency
//Touche DOWN    decremente frequence ou memoire, decrease frequency
//Touche SELECT  long push = frequency memorization into the EE number EEPROM / or reference memorization
//*************************************************************************************************************************
// Warning : if you are using a ROBOT Shied version 1.1, it is necessary to modify the read_lcd_buttons sub routine 
// you need not to comment the 1.1 version and to comment the 1.0 version. See below

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <SPI.h>



LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

byte poscursor = 0; //position curseur courante 0 à 15
byte line = 0; // ligne afficheur LCD en cours 0 ou 1
byte memoire,RWtemp; // numero de la memoire EEPROM

// Register 0
// 0x4580A8
// 3|3222222222211111|111110000000|000
// 1|0987654321098765|432109876543|210
//          010001011|000000010101|000
//                                 CCC
//
// Reserved
// 16-Bit INT value
// 16-Bit FRAC value
// Control

// Register 1
// 0x80080C9
// 332|2|2|222222211111|111110000000|000
// 109|8|7|654321098765|432109876543|210
//    | |1|000000000001|000000011001|001
// RRR                               CCC
//
// Reserve
// Phase Adjust
// Prescaler
// 12-Bit PHASE value
// 12-Bit MOD value
// Control

// Register 2
// 0x4E42
// 3|32|222|2|2|2222111111|1|1110|0|0|0|0|0|0|000
// 1|09|876|5|4|3210987654|3|2109|8|7|6|5|4|3|210
//  |  |   | | |        01|0|0111|0|0|1|0|0|0|010
// R                                          CCC
//
// Reserved
// Low Noise and Low Spur Modes
// MUXOUT
// Reference Doubler
// RDIV2
// 16-Bit R Counter
// Double Buffer
// Charge Pump Currnet Setting
// LDF
// LDP
// PD Polarity
// Power-Down
// CP Three-State
// Counter Reset
// Control


// Register 3
// 0x4B3
// 33222222|2|2|2|21|1|1|11|111110000000|000
// 10987654|3|2|1|09|8|7|65|432109876543|210
//         | | | |  | | |  |   010010110|011
// RRRRRRRR       RR   R                 CCC 
//
// Reserved
// Band Select Clock Mode
// APB
// Charge Cancel
// Reserved
// CSR
// Reserved
// Clock Div Mode
// 12-Bit Clock Divider Value
// Control

// Register 4
// 0xBC803C
// 33222222|2|222|11111111|1|1|0|0|00|0|00|000
// 10987654|3|210|98765432|1|0|9|8|76|5|43|210
//         |1|011|11001000|0|0|0|0|00|1|11|100
// RRRRRRRR                                CCC
//
// Reserved
// Feedback Select
// RF Divider Select
// 8-Bit Band Select Clock Divider Value
// VCO Power-Down
// MTLD
// Aux Output Select
// Aux Output Enable
// Aux Output Power
// RF Output Enable
// Output Power
// Control

// Register 5
// 0x580005
// 33222222|22|2|21|1111111110000000|000
// 10987654|32|1|09|8765432109876543|210
//         |01|0|11|0000000000000000|101
// RRRRRRRR    R RR RRRRRRRRRRRRRRRR CCC
//
// Reserved
// LD Pin Mode
// Reserved
// Reserved
// Reserved
// Control

uint32_t registers[6] =  {0x4580A8, 0x80080C9, 0x4E42, 0x4B3, 0xBC803C, 0x580005} ; // 437 MHz avec ref à 25 MHz


//uint32_t registers[6] =  {0, 0, 0, 0, 0xBC803C, 0x580005} ; // 437 MHz avec ref à 25 MHz
int address,modif=0,WEE=0,beacon=0;
int lcd_key = 0;
int adc_key_in  = 0;
int timer = 0,timer2=0; // utilisé pour mesurer la durée d'appui sur une touche
unsigned int i = 0;


double RFout, REFin, INT, PFDRFout, OutputChannelSpacing, FRACF;
double RFoutMin = 35, RFoutMax = 4400, REFinMax = 250, PDFMax = 32;
unsigned int long RFint, RFintold, INTA, RFcalc, PDRFout, MOD, FRAC;

int OutputDivider;
byte lock=2;
unsigned int long reg0, reg1;

unsigned long int shift=1; // FSK shift frequency
int cw_speed; // speed of CW (100ms = 12wpm, 240=5wpm))
int cw_WPM=12;
int FSK_shift=1;
int first_run=1;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

int sensorPin = A7; 

int ADF4351_LE = 5;   // NANO
//int ADF4351_LE = 3;  // UNO
int ADF4351_MUX = 2;

//************************************ Set CW Speed ****************************************
void SetCWSpeed (int WPM)
// Set the cw_speed variable using WPM.
  {
    cw_speed=WPM2secs(WPM);
    Serial.print(" Set CW speed to: ");
    Serial.print(WPM);
    Serial.print(" WPM\n");
   } 

//**************************** SP LECTURE BOUTONS ********************************************
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the buttons
  if (adc_key_in < 790)lcd.blink();
  
  if (adc_key_in < 50)return btnRIGHT;  // pour Afficheur ROBOT V1.0
  if (adc_key_in < 195)return btnUP;
  if (adc_key_in < 380)return btnDOWN;
  if (adc_key_in < 555)return btnLEFT;
  if (adc_key_in < 790)return btnSELECT; // Fin Afficheur ROBOT1.1

  //if (adc_key_in < 50)return btnRIGHT; // pour Afficheur ROBOT 1.1
  //if (adc_key_in < 250)return btnUP;
  //if (adc_key_in < 450)return btnDOWN;
  //if (adc_key_in < 650)return btnLEFT;
  //if (adc_key_in < 850)return btnSELECT; // fin Afficheur ROBOT 1.1

  return btnNONE;  // touches non appuyees
}

//***************************** SP Affichage Fréquence sur LCD ********************************
void printAll ()
{
  //RFout=1001.10 // test
  lcd.setCursor(0, 0);
  if (beacon==0) {lcd.print("RF = ");}
  else {lcd.print("BEC= ");}
  if (RFint < 100000) lcd.print(" ");
  if (RFint < 10000)  lcd.print(" ");
  lcd.print(RFint/100);lcd.print(".");
  RFcalc=RFint-((RFint/100)*100);
  if (RFcalc<10)lcd.print("0");
  lcd.print(RFcalc);
  lcd.print(" MHz");
  lcd.setCursor(0,1);
  if (WEE==0) {lcd.print("REE=");}
  else {lcd.print("WEE=");}
  if (memoire<10)lcd.print(" ");
  lcd.print(memoire,DEC);
  if  ((digitalRead(ADF4351_MUX)==1))lcd.print(" LOCKED ");
  else lcd.print(" NOLOCK ");
  lcd.print(PFDRFout,DEC);
  lcd.setCursor(poscursor,line);
}

//************************************ Check for Lock ****************************************
void check_lock() 
{
  if (digitalRead(ADF4351_MUX)==1) Serial.print(" Locked\n");
  else Serial.print(" No Lock\n");  
}

//************************************ Spur Reduction ****************************************
void reduce_spur(int val) 
{
  if (val==1){
  bitWrite (registers[2], 29, 1); // Low Spur
  bitWrite (registers[2], 30, 1);
  Serial.print(" Low spur mode.\n");
  }
  else
 {  
  bitWrite (registers[2], 29, 0); // Low noise
  bitWrite (registers[2], 30, 0);
  Serial.print(" Low noise mode.\n");
 }
}

//************************************ Write a register ****************************************
void WriteRegister32(const uint32_t value)   //Programme un registre 32bits
{
  digitalWrite(ADF4351_LE, LOW);
  for (int i = 3; i >= 0; i--)          // boucle sur 4 x 8bits
  SPI.transfer((value >> 8 * i) & 0xFF); // décalage, masquage de l'octet et envoi via SPI
  digitalWrite(ADF4351_LE, HIGH);
  digitalWrite(ADF4351_LE, LOW);
}

//************************************ Write all Registers ****************************************
void SetADF4351()  // Programme tous les registres de l'ADF4351
{ Serial.print("======== SetADF ========\n");
 for (int i = 5; i >= 0; i--)  // programmation ADF4351 en commencant par R5
    {
    WriteRegister32(registers[i]);
    Serial.print(i,DEC);
    Serial.print(" ");
    Serial.print(registers[i],BIN);
    Serial.print("\n");
    }
  Serial.print("========        ========\n");
  Serial.print(" ");
}

// *************** SP ecriture Mot long (32bits) en EEPROM  entre adress et adress+3 **************
void EEPROMWritelong(int address, long value)
      {
      //Decomposition du long (32bits) en 4 bytes
      //trois = MSB -> quatre = lsb
      byte quatre = (value & 0xFF);
      byte trois = ((value >> 8) & 0xFF);
      byte deux = ((value >> 16) & 0xFF);
      byte un = ((value >> 24) & 0xFF);

      //Ecrit 4 bytes dans la memoire EEPROM
      EEPROM.write(address, quatre);
      EEPROM.write(address + 1, trois);
      EEPROM.write(address + 2, deux);
      EEPROM.write(address + 3, un);
      }

// *************** SP lecture Mot long (32bits) en EEPROM situe entre adress et adress+3 **************
long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long quatre = EEPROM.read(address);
      long trois = EEPROM.read(address + 1);
      long deux = EEPROM.read(address + 2);
      long un = EEPROM.read(address + 3);

      //Retourne le long(32bits) en utilisant le shift de 0, 8, 16 et 24 bits et des masques
      return ((quatre << 0) & 0xFF) + ((trois << 8) & 0xFFFF) + ((deux << 16) & 0xFFFFFF) + ((un << 24) & 0xFFFFFFFF);
      }

//************************************ Setup ****************************************
void setup() {
  lcd.begin(16, 2); // two 16 characters lines
  lcd.display();

  analogReference(DEFAULT);

 // Don't do this in case it has the LCD bug.
  //analogWrite(10,255); //Luminosite LCD
  // Better to do
  digitalWrite(10, LOW);
  pinMode(10, INPUT);
  // Hardcoded for pin 10

  Serial.begin (19200); //  Serial to the PC via Arduino "Serial Monitor"  at 9600
  Serial.print("\n\n\nConnected at 19200 Baud\n");
  lcd.print("   GENERATEUR   ");
  lcd.setCursor(0, 1);
  lcd.print("    ADF4351     ");
  poscursor = 7; line = 0; 
  delay(100);
  lcd.setCursor(0, 0);
  lcd.print("   par F1CJN    ");
   delay(100);

  pinMode(ADF4351_MUX, INPUT);  // PIN 2 en entree pour lock
  pinMode(ADF4351_LE, OUTPUT);          // Setup pins
  digitalWrite(ADF4351_LE, HIGH);
  SPI.begin();                          // Init SPI bus
  SPI.setDataMode(SPI_MODE0);           // CPHA = 0 et Clock positive
  SPI.setBitOrder(MSBFIRST);            // poids forts en tête

  if (EEPROM.read(100)==55){PFDRFout=EEPROM.read(20*4);} // si la ref est ecrite en EEPROM, on la lit
  else {PFDRFout=25;}

  if (EEPROM.read(101)==55){RFint=EEPROMReadlong(memoire*4);} // si une frequence est ecrite en EEPROM on la lit
  else {RFint=43250;}

  RFintold=1234;//pour que RFintold soit different de RFout lors de l'init
  RFout = RFint/100 ; // fréquence de sortie
  OutputChannelSpacing = 0.01; // Pas de fréquence = 10kHz

  WEE=0;  address=0;
  lcd.blink();
  printAll(); delay(500);

  SetCWSpeed (cw_WPM);

} // Fin setup

//************************************ Power Set ****************************************
void power_set(int power)
{
/* Set the registers to tell ADF4351 that we'd like different power levels.
 *  Note that this funciton does not set the pwer, that's done when we write to 
 *  the synth.
 */
  
/* D4 D3
 * D2 D1
   1 0 -4dBm
   0 1 -1dBm
   1 0 +2dBm
   1 1 +5dBm
  */
Serial.print(" Set power to: ");
switch(power)
{
  case 1: bitWrite (registers[4], 4, 0); bitWrite (registers[4], 3, 0); Serial.print("-4dBm\n"); break;
  case 2: bitWrite (registers[4], 4, 0); bitWrite (registers[4], 3, 1); Serial.print("-1dBm\n"); break;
  case 3: bitWrite (registers[4], 4, 1); bitWrite (registers[4], 3, 0); Serial.print("+2dBm\n"); break;
  case 4: bitWrite (registers[4], 4, 1); bitWrite (registers[4], 3, 1); Serial.print("+4dBm\n"); break;
}
}


void set_MOD(unsigned long MOD)
{
    registers[1] = MOD << 3;
    registers[1] = registers[1] + 1 ; // ajout de l'adresse "001"
}


//************************************ Change Freq ****************************************
void change_freq()
/********************************************//**
 * Change the frequency of the PLL (if required) if
 * not just check the buttons and exit
 ***********************************************/
{
  
  //read_buttons(); // this a hack s.t. it will check the buttons
                  // often...
  RFout=RFint;
  RFout=RFout/100;
 //if ((RFint != RFintold)|| (modif==1) || (beacon)){
   
    OutputDivider = GetOutputDivider(RFout);

    Serial.print("OutputDivider: ");Serial.print(OutputDivider,DEC);Serial.print("\r\n");

    INTA = (RFout * OutputDivider) / PFDRFout;
    MOD = (PFDRFout / OutputChannelSpacing);
    FRACF = (((RFout * OutputDivider) / PFDRFout) - INTA) * MOD;
    FRAC = round(FRACF); // On arrondit le résultat

    if ((FSK_shift==0) && (beacon=1)) FRAC++; // if we're FSK I want the smallest shift

    registers[0] = 0;
    registers[0] = INTA << 15; // OK
    FRAC = FRAC << 3;
    registers[0] = registers[0] + FRAC;

    registers[1] = 0; // This needs sorting
    set_MOD(MOD);
    
    set_Prescaler(0); //dummy number '0'

    set_MUXOUT(0); //dummy number '0'
   
    //SetADF4351();  // Programme tous les registres de l'ADF4351
    RFintold=RFint;modif=0;
    printAll();  // Affichage LCD
    Serial.print("Changed frequency to: "); Serial.print(RFout); Serial.print("\n");
 // }
}

int GetOutputDivider(double RFout)
{
    if (RFout >= 2200) {
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
      return 1;
    }
    if (RFout >= 1100) {
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
      return 2;
    }
    if (RFout >= 550) {
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
      return 4;
    }
    if (RFout >= 275)  {
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 1);
      return 8;
    }
    if (RFout >= 137.5)  {
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
      return 16;
    }
    if (RFout >= 68.75) {
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
      return 32;
    }
    else {
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
      return 64;
    }

}

void set_FRAC(unsigned long FRAC)
{
  delay(1);
}

void set_MUXOUT(int MUX)
{
    // MUXOUT Digital lock == "110" sur b28 b27 b26
    bitWrite (registers[2], 28, 1);
    bitWrite (registers[2], 27, 1); 
    bitWrite (registers[2], 26, 0); 
}

void set_Prescaler(int prescaler)
{
      // Prescaler sur 8/9    
    bitWrite (registers[1], 27, 1); 
}
void read_buttons()
/********************************************//**
 * Function to work out what button was pressed and
 * decide what to do about it. It sets varibales
 * WEE - reading/writing eprom
 * RFint - the frequency to set the PLL
 * becaon - carrier/becon mode
 * PFDRFout - RF reference 25MHz internal 10MHz 
 *            external
 ***********************************************/
{
  lcd_key = read_LCD_buttons();  // read the buttons

  switch (lcd_key)               // Select action
  {
    case btnRIGHT: //Droit
      poscursor++; // cursor to the right
      if (line == 0) {
        if (poscursor == 1 ) {poscursor = 6; line = 0;};
        if (poscursor == 9 ) {
          poscursor = 10;
          line = 0; } //si curseur sur le .
        if (poscursor == 12 ) {
          poscursor = 0; line = 1; }; //si curseur à droite
      }
     if (line == 1) {
        if (poscursor == 1 ) {poscursor = 5; line = 1; } //si curseur sur le chiffre memoire 
        if (poscursor == 6 ) {poscursor = 15; line = 1; } //si curseur sur le chiffre memoire 
        if (poscursor==16) {poscursor=5; line=0;};     
      }  
      //Serial.print (" RIGHT Button\r\n");
      lcd.setCursor(poscursor, line);
      break;
      
    case btnLEFT: //Gauche
      poscursor--; // décalage curseur
      if (line == 0) {
        if (poscursor == 5) { poscursor = 0; line=0;};
        if (poscursor == 9) { poscursor = 8; line=0;}
      }
       if(line==1){
          if (poscursor==255) {poscursor=11; line=0;};
          if (poscursor==4) {poscursor=0; line=1;};
          if (poscursor==14) {poscursor=5; line=1;};
      }
      //Serial.print(poscursor,DEC);  
      lcd.setCursor(poscursor, line);
      break;
      
    case btnUP: //Haut
      if (line == 0)
      { // RFoutfrequency
        //Serial.print(oldRFint,DEC);
        if (poscursor == 5) RFint = RFint + 100000 ;
        if (poscursor == 6) RFint = RFint + 10000 ;
        if (poscursor == 7) RFint = RFint + 1000 ;
        if (poscursor == 8) RFint = RFint + 100 ;
        if (poscursor == 10) RFint = RFint + 10 ;
        if (poscursor == 11) RFint = RFint + 1 ;
        if (RFint > 440000)RFint = RFintold;
        //Serial.print(RFint,DEC);
        //Serial.print("  \r\n");

        if( (poscursor==1) && (beacon==1))beacon=0;
        else if ((poscursor==1)&&(beacon==0))beacon=1;
        
      }
      if (line == 1)
      { 
        if (poscursor == 5){ memoire++; 
        if (memoire==20)memoire=0;
        if (WEE==0){RFint=EEPROMReadlong(memoire*4); // lecture EEPROM et Affichage
           if (RFint>440000) RFint=440000; 
           } 
        }  
        if (poscursor==15){ 
        if( PFDRFout==10){PFDRFout=25;} //reglage FREF
        else if ( PFDRFout==25){PFDRFout=10;}
        else PFDRFout=25;// au cas ou PFDRF different de 10 et 25
        modif=1;  }
                    
        if( (poscursor==0) && (WEE==1))WEE=0;
        else if ((poscursor==0) && (WEE==0))WEE=1;                  
      }
        printAll();
      break; // fin bouton up

    case btnDOWN: //bas
      if (line == 0) {
        if (poscursor == 5) RFint = RFint - 100000 ;
        if (poscursor == 6) RFint = RFint - 10000 ;
        if (poscursor == 7) RFint = RFint - 1000 ;
        if (poscursor == 8) RFint = RFint - 100 ;
        if (poscursor == 10) RFint = RFint - 10 ;
        if (poscursor == 11) RFint = RFint - 1 ;
        if (RFint < 3450) RFint = RFintold;
        if (RFint > 440000)  RFint = RFintold;
       
        if((poscursor==0) && (beacon==1)) beacon=0;
        else if ((poscursor==0)&&(beacon==0)) beacon=1;

        printAll();
      //  break;
      }

     if (line == 1)
      { 
        if (poscursor == 5){memoire--; 
        if (memoire==255)memoire=19;
        if (WEE==0){RFint=EEPROMReadlong(memoire*4); // lecture EEPROM et Affichage
           if (RFint>440000) RFint=440000;
          // Serial.print(RFint,DEC);  
           } 
        } // fin poscursor =5 

       if (poscursor==15){ 
       if( PFDRFout==10){PFDRFout=25;} //reglage FREF
       else if ( PFDRFout==25){PFDRFout=10;}
       else PFDRFout=25;// au cas ou PFDRF different de 10 et 25
       modif=1;
       }
                   
       if( (poscursor==0) && (WEE==1))WEE=0;
       else if ((poscursor==0)&&(WEE==0))WEE=1;                          
      
       printAll();
      // Serial.print (" DOWN Button  \r\n");
      break; // fin bouton bas
      }

    case btnSELECT:
      do {
        adc_key_in = analogRead(0);      // Test release button
        delay(1); timer2++;        // timer inc toutes les 1 millisecondes
        if (timer2 > 600) { //attente 600 millisecondes
         if (WEE==1 || poscursor==15){ 
         if (line==1 && poscursor==15){ EEPROMWritelong(20*4,PFDRFout);EEPROM.write(100,55);} // ecriture FREF
         else if (WEE==1) {EEPROMWritelong(memoire*4,RFint);EEPROM.write(101,55);}// ecriture RF en EEPROM à adresse (memoire*4)
          lcd.setCursor(0,1); lcd.print("  MEMORISATION  ");}
          lcd.setCursor(poscursor,line);
          delay(500);timer2=0;
          printAll();
        }; // mes

        } 
      while (adc_key_in < 900); // attente relachement
      break;  // Fin bouton Select

     case btnNONE: {
        break;
      };
      break;
  }// Fin LCD keys

   do { adc_key_in = analogRead(0); delay(1);} while (adc_key_in < 900); // attente relachement touche
   delay (10);timer++; // inc timer
   //Serial.print(timer,DEC);
   if (timer>1000){lcd.noBlink();timer=0;} // curseur off

}   // fin loop


/********************************************//**
 * Basic building blocks for CW
 ***********************************************/
void key_down() {FSK_shift=1;change_freq();}

void key_up() {FSK_shift=0;change_freq();}

void dash()
{
  key_up();
  delay(cw_speed*3);
  key_down();
  delay(cw_speed);
}

void space() {delay(cw_speed*3);} 

void dot()
{
  key_up();
  delay(cw_speed);
  key_down();
  delay(cw_speed);
}

int WPM2secs (int WPM) {return 1200/WPM;}  // Convert from WPM to miliseconds.

/********************************************//**
 * Characters for CW
 ***********************************************/
void a() {dot();dash();space();Serial.print("a");}
void b() {dash();dot();dot();dot();space();Serial.print("b");}
void c() {dash();dot();dash();dot();space();Serial.print("c");}
void d() {dash();dot();dot();space();Serial.print("d");}
void e() {dot();space();Serial.print("e");}
void f() {dot();dot();dash();dot();space();Serial.print("f");}
void g() {dash();dash();dot();space();Serial.print("g");}
void h() {dot();dot();dot();dot();space();Serial.print("h");}
void eye() {dot();dot();space();Serial.print("i");}
void j() {dot();dash();dash();dash();space();Serial.print("j");}
void k() {dash();dot();dash();space();Serial.print("k");}
void l() {dot();dash();dot();dot();space();Serial.print("l");}
void m() {dash();dash();space(); Serial.print("m");}
void n() {dash();dot();space();Serial.print("n");}
void o() {dash();dash();dash();space();Serial.print("o");}
void p() {dot();dash();dash();dot();space();Serial.print("p");}
void q() {dash();dash();dot();dash();space();Serial.print("q");}
void r() {dot();dash();dot();space();Serial.print("r");}
void s() {dot();dot();dot();space();Serial.print("s");}
void t() {dash();space();Serial.print("t");}
void u() {dot();dot();dash();space();Serial.print("u");}
void v() {dot();dot();dot();dash();space();Serial.print("v");}
void w() {dot();dash();dash();space();Serial.print("w");}
void x() {dash();dot();dot();dash();space();Serial.print("x");}
void y() {dash();dot();dash();dash();space();Serial.print("y");}
void z() {dash();dash();dot();dot();space();Serial.print("z");}

/* Numbers */
void zero()  {dash();dash();dash();dash();dash();space();Serial.print("0");}
void one()   {dot();dash();dash();dash();dash();space();Serial.print("1");}
void two()   {dot();dot();dash();dash();dash();space();Serial.print("2");}
void three() {dot();dot();dot();dash();dash();space();Serial.print("3");}
void four()  {dot();dot();dot();dot();dash();space();Serial.print("4");}
void five()  {dot();dot();dot();dot();dot();space();Serial.print("5");}
void six()   {dash();dot();dot();dot();dot();space();Serial.print("6");}
void seven() {dash();dash();dot();dot();dot();space();Serial.print("7");}
void eight() {dash();dash();dash();dot();dot();space();Serial.print("8");}
void nine()  {dash();dash();dash();dash();dot();space();Serial.print("9");}

/* Special Characters */
void slash()     {dash();dot();dot();dash();dot();space();Serial.print("/");}
void full_stop() {dot();dash();dot();dash();dot();dash();space();Serial.print(".");}
void BT()        {dash();dot();dot();dot();dash();gap();Serial.print("<BT>\n");}
void minus()     {dash();dot();dot();dot();dot();dash();space();Serial.print("-");}
void gap()       {delay(cw_speed*7); Serial.print(" ");}

void run_beacon()
/********************************************//**
 * Hard coded becon message
 ***********************************************/
{
  m();zero();c();w();x();slash();b();gap();
  eye();o();eight();two();x();k();gap();
}

int readRotarySwitch(int in) // in=0 be silent, 1=report value
{
  int sensorValue = 0; 
  
  sensorValue = analogRead(sensorPin);
  if (in>0)
  {
   Serial.print(" Sensor Value:");
   Serial.println(sensorValue);
   Serial.print("\n");
  }
  if (sensorValue < 10) return 12;
  if (sensorValue < 100) return 11;
  if (sensorValue < 200) return 10;
  if (sensorValue < 300) return 9;
  if (sensorValue < 400) return 8;
  if (sensorValue < 500) return 7;
  if (sensorValue < 600) return 6;
  if (sensorValue < 700) return 5;
  if (sensorValue < 800) return 4;
  if (sensorValue < 900) return 3;
  if (sensorValue < 1000) return 2;

  else return 1;
}

unsigned long preset_frequency(int in)
{
       switch(in)
       {
        case  1: return 131700;  break; 
        case  2: return 177600;  break; 
        case  3: return 188800; break;
        case  4: return 266400;  break; 
        case  5: return 296800; break; 
        case  6: return 440000;  break; 
        case  7: return 43250; break;
        case  8: return 43251; break;
        case  9: return 43252; break;
        case  10: return 43253; break;
        case  11: return 43254; break;
        case  12: return 43255; break;
       }
  }

void VXOKnob(int in) // in=0 be silent, 1=report value
 {
   static int SwitchPos;
   int ReadSwitch;    
   ReadSwitch =  readRotarySwitch(in); 
   if (ReadSwitch!=SwitchPos) // switch has moved
  {   
   Serial.print(" Switch has moved to: "); Serial.println(SwitchPos); Serial.print("\n");    
   SwitchPos = ReadSwitch;
   RFint = preset_frequency(SwitchPos);
   change_freq();
   SetADF4351();
   check_lock();
  }
}

void test_mode()
{
  Serial.print(" Test Mode:\n ");  
  power_set(4);
  RFint=43250; change_freq(); SetADF4351();
}

#define MAX_MESSAGE 30

void read_buffer() {
  static char buffer[MAX_MESSAGE];
  static unsigned char index = 0;
  char inch;
  
  while (Serial.available() > 0) {
    inch = Serial.read();
    if (inch == '\r') {
      Serial.print("You entered: ");
      Serial.println(buffer);
      buffer[0] = 0;
      index = 0;
    } else {        
      if (index < MAX_MESSAGE-1) {
        buffer[index++] = inch;
        buffer[index] = 0;
      }
    }
  } 
}

void display_help() {
  Serial.print(" D  - Debug Mode \n"); 
  Serial.print(" Fx - Change frequency to x \n");   
  Serial.print(" H  - Display Help\n");  
  Serial.print(" L  - Check MUX lock\n"); 
  Serial.print(" Mx - Use Memory x\n"); 
  Serial.print(" Px - Change power level to x\n"); 
  Serial.print(" Sx - x=0 reduce noise x=1 reduce spurs\n"); 
  Serial.print(" W   - write changes to ADF4351\n"); 
}

void loop()
{
  signed int long  SerialIn;

  char inch;

  if(first_run==1) test_mode();
  if(beacon) run_beacon();
  VXOKnob(0); // check the rotary switch

  while (Serial.available() > 0) {
   inch = Serial.read();
  // ========================================================================
   // Debug Mode
   // ========================================================================
   // A mode for stuffing test functions in and calling them by number
   if(inch == 'D') { 
     Serial.print(" Debug Mode: \n"); 
     SerialIn=Serial.parseInt();
      switch(SerialIn){
         case  79: VXOKnob(1); break;          // DEBUG: Talk to VXOKnob() in verbose mode
         case 996: read_buffer(); break;       // DEBUG: test serial reading
         case 997: readRotarySwitch(1); break; // DEBUG: Get the value from the rotary switch
         case 998: test_mode(); break;         // DEBUG: go to a 432.5 MHz and run QRO
      }
   }
   // ========================================================================
   // Change Frequency
   // ========================================================================
    else if(inch == 'F') { 
     Serial.print(" Change frequency: \n");   
     SerialIn=Serial.parseInt();
     if(SerialIn < 35000 || SerialIn > 4400000) {
       Serial.print(" Current Freq: \n"); Serial.println(RFint); // Complain out of range
     }
     else {
       RFint=SerialIn/10; change_freq();SetADF4351(); 
     }
   } 
   // ========================================================================
   // Help
   // ========================================================================
   else if(inch == 'H') display_help();
   // ========================================================================
   // Check MUX Lock
   // ========================================================================
   else if (inch == 'L') { 
     Serial.print(" Check Lock: \n"); 
     check_lock();
   }
   // ========================================================================
   // Change frequency to a Memory
   // ========================================================================
    else if (inch == 'M') { // memory
     Serial.print(" Change to memory:\n"); 
     SerialIn=Serial.parseInt();
     if(SerialIn >= 1 && SerialIn <= 12) {
         RFint=preset_frequency(SerialIn); 
         change_freq(); 
         SetADF4351();
        }
      else Serial.print(" Unknown memory location\n"); 
    }
  // ========================================================================
   // Change Power Level 
   // ========================================================================
   else if (inch == 'P') { 
     Serial.print(" Change power:\n");   
     SerialIn=Serial.parseInt();
     if(SerialIn >= 1 && SerialIn <= 4) power_set(SerialIn);
     else Serial.print(" Unknown power level\n"); 
   }
   // ========================================================================
   // Switch between low noise and low spur
   // ========================================================================
   // 0 Reduce Noise
   // 1 Reduce Spur
   else if (inch == 'S') { 
     Serial.print(" Reduce Noise=0, Reduce Spur=1\n"); 
     if(SerialIn >= 0 && SerialIn <= 1) reduce_spur(SerialIn); 
     else Serial.print(" Unknown Spur/Noise level\n"); 
   }
   // ========================================================================
   // Write to ADF4351
   // ========================================================================
   else if (inch == 'W') { 
     Serial.print(" Write to ADF4351\n");      
     SetADF4351();
   }
  }
    
  first_run=0;
  delay(250);
} 

