#include <Arduino.h>
#include <U8g2lib.h>//for Oled diaplay
#include <math.h>//maths function 
#define DEBUG//use for toggling for specfic mode of library available at case scenario
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>//Serial Peripheral Interface use for communications short distance in multiple pheripheral device
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>//lets you communicate with I^2C devices(2 wires interface)
#endif

U8G2_SSD1306_64X48_ER_F_4W_SW_SPI u8g2(U8G2_R0,/* clock=*/ 9,/* data=*/ 8,/* cs=*/ 11,/* dc=*/ 10,/* reset=*/ 12);//defining pins for OLED display
int menu = 1;
int curr_menu = 1;
int prog_start = 0;
int menu_items = 3;
//DRV8835 motor driver
#define Aph 4//APhase connection
#define Apw 3//Apwn conncetion
#define Bph 5//BPhase connection
#define Bpw 6//Bpwn conncetion
#define Button_left 7//botton left
#define Button_right 2//button right

#define LED_Yellow 20//indicator for loop(yellow)
#define LED_Red 21//indicator for loop(red)


const int PatternCnt = 16;//possible case scenario
const int InptNde = 4;//inputs
const int HddNde = 5;//hidden layer
const int OutNde = 2;//output on motor nodes
const float LearnRt = 0.3;//higher learning rate for gardient descent
const float moment = 0.9;// gradient descent with momentum so that it doesnt overshoot the minimum error
const float IntWeighMX = 0.5;
const float Success = 0.0015;

float Input[PatternCnt][InptNde] = {//possible case scenarios of lights on 4 photo resistors
  { 0, 1, 1, 0 },  // LIGHT ON LEFT AND RIGHT
  { 0, 1, 0, 0 },  // LIGHT ON LEFT
  { 1, 1, 1, 0 },  // LIGHT ON TOP, LEFT, and RIGHT
  { 1, 1, 0, 0 },  // LIGHT ON TOP and LEFT
  { 0, 0, 1, 0 },  // LIGHT ON RIGHT
  { 1, 0, 0, 0 },  // LIGHT ON TOP
  { 0, 0, 0, 0 },  // NO LIGHT
  { 0, 0, 0, 1 },  // LIGHT ON BOTTOM
  { 0, 1, 0, 1 },  // LIGHT ON BOTTOM AND LEFT
  { 0, 0, 1, 1 },  // LIGHT ON BOTTOM AND RIGHT
  { 0, 1, 1, 1 },  // LIGHT ON BOTTOM, LEFT, and RIGHT
  { 1, 0, 0, 1 },  // LIGHT ON TOP AND BOTTOM
  { 1, 1, 0, 1 },  // LIGHT ON TOP, BOTTOM, and LEFT
  { 1, 0, 1, 1 },  // LIGHT ON TOP, BOTTOM, and RIGHT
  { 1, 0, 1, 0 },  // LIGHT ON TOP AND RIGHT
  { 1, 1, 1, 1 },  // LIGHT ON ALL
};

const float Target[PatternCnt][OutNde] = {//action to be taken after matching it to intenisty of light
  { 0.65, 0.55 },   //LEFT MOTOR SLOW
  { 0.75, 0.5 },    //LEFT MOTOR FASTER
  { 0.2, 0.2 },     //BOTH MOTORS FULL BACKWARDS
  { 1, 0.2 },       //MOTOR LEFT FULL FORWARD, RIGHT BACKWARDS
  { 0.5, 0.75 },    //MOTOR LEFT STOPPED, RIGHT FORWARDS
  { 0.3, 0.3 },     //BOTH BACKWARDS
  { 0.5, 0.5 },     //BOTH MOTORS STOPPED
  { 0.75, 0.75 },
  { 1, 0.75 },
  { 0.75, 1 },
  { 1, 1 },
  { 1, 0 },
  { 1, 0.75 },
  { 0.75, 1 },
  { 0.2, 1 },
  { 0.65, 0.65},
};
int i, j, p, q, r;//counters for loops
int RprtEvry100;// record every 100 cycle
int RndmInd[PatternCnt];//random start for node
long TrnCyc;
float Rando;
float Error = 2;
float Accumulator;
float Hidden[HddNde];//hidden Array for weight assignment
float Output[OutNde];//output array for motor rotation
float HddWeigh[InptNde + 1][HddNde];//2d array to work on  gradient descent and store weights
float OutWeigh[HddNde + 1][OutNde];
float HddDel[HddNde];//windrow-off/Delta rule for calculation of errors
float OutDel[OutNde];
float ChnHddWeigh[InptNde + 1][HddNde];//updated weights for hidden layer
float ChnOutWeigh[HddNde + 1][OutNde];
int ErrGrph[64];// fro plotting error graph
int P1,P2,P3,P4,lightSum;

void setup() {
  // put your setup code here, to run once:
  for (int x = 0; x < 64; x++) {
    ErrGrph[x] = 47;
  }

  pinMode(A0, INPUT);//photoresistor 1
  pinMode(A1, INPUT);//photoresistor 2
  pinMode(A2, INPUT);//photoresistor 3
  pinMode(A3, INPUT);//photoresistor 4
  pinMode(Button_left, INPUT_PULLUP);//button left
  pinMode(Button_right, INPUT_PULLUP);//button right

  randomSeed(analogRead(A0));       //Collect a random ADC sample for Randomization.
  RprtEvry100 = 1;
  for ( p = 0 ; p < PatternCnt ; p++ ) {
    RndmInd[p] = p ;
  }

  Serial.begin(115200);//baud rate 115200
  delay(100);
  u8g2.begin();//begin oled display
  u8g2.setFlipMode(0);//set the degree of rotation
  u8g2.setFont(u8g2_font_6x12_tr);//font to be used on OLed
  u8g2.setColorIndex(1);
  int testmode = 1;//button testing
  testmode = digitalRead(Button_left);//left button for changing menu items
  if (testmode == 0) {
    Motor_test();//motor testing if ideal
  }

  u8g2.firstPage();

  do {
    intro();
  } while ( u8g2.nextPage() );
  attachInterrupt(Button_left, Change_menu, LOW);//left button to change program
  attachInterrupt(Button_right, Load_program, LOW);//right button to execue the program
  delay(1000);
  while (prog_start == 0) {
    u8g2.firstPage();
    do {
      menu_select();
    } while ( u8g2.nextPage() );//menu select till interrupt
  }
  u8g2.clear();//clear screen
  delay(1000);//1 second delay

}

void loop() {

  //Motor_test() //Run this to test the direction of your motors;
  while (prog_start == 0) {
    u8g2.firstPage();
    do {
      menu_select();
    } while ( u8g2.nextPage() );
  }
  u8g2.clear();
  delay(1000);
  switch (curr_menu) {
    case 1:
      Draw_Ball();
      break;
    case 2:
      Training();
      break;
    case 3:
      Execute();
      break;
    default:
      break;

  }
}
void Draw_Ball() {//check photoresistors working
  P1 = analogRead(A0);//reading reading of resistor 1
  P2 = analogRead(A1);//reading reading of resistor 2
  P3 = analogRead(A2);//reading reading of resistor 3
  P4 = analogRead(A3);//reading reading of resistor 4
  lightSum = P1 + P2 + P3 + P4;
  P1 = map(P1, 400, 1024, 0, 48);
  P2 = map(P2, 400, 1024, 0, 64);
  P3 = map(P3, 400, 1024, 0, 64);
  P4 = map(P4, 400, 1024, 0, 48);
  int pull_x = 0;//indent from x axis
  int pull_y = 0;//indent from y axis
  pull_x = ((P2 - P3) / 2) + 32;
  pull_y = ((P1 - P4) / 2) + 24;
  u8g2.drawCircle(pull_x, pull_y, 4);//drawing circle on OLED to show intensity of light
//  int choice=digitalRead(Button_left);
  //if(choice==1)
  //{loop();}
}

void Motor_test() {//test motor
  Serial.println("Driving Forward");
  u8g2.firstPage();
  do {
    u8g2.drawStr( 1, 10, "Forward");
  } while ( u8g2.nextPage() );
  motA(70);//forward
  motB(70);//forward
  delay(1000);//1 second delay
  Serial.println("Stopping");
  u8g2.firstPage();
  do {
    u8g2.drawStr( 1, 10, "Stopping");
  } while ( u8g2.nextPage() );
  motA(50);//stationary
  motB(50);//stationary
  delay(1000);//1 second delay
  Serial.println("Turning");
  u8g2.firstPage();
  do {
    u8g2.drawStr( 1, 10, "Turning");
  } while ( u8g2.nextPage() );
  motA(70);//forward
  motB(20);//backward

  delay(1000);//1 second delay
  u8g2.firstPage();
  do {
    u8g2.drawStr( 1, 10, "Backwards");
  } while ( u8g2.nextPage() );
  motA(30);//backward
  motB(30);//backward
  delay(1000);//1 second delay
  u8g2.firstPage();
  do {
    u8g2.drawStr( 1, 10, "Stopped");
  } while ( u8g2.nextPage() );
  motA(50);//stationary
  motB(50);//stationary
  delay(1000);//1 second delay
  //while (1);
}


void motA(int percent) {//drive motor A
  int maxSpeed = 90;//max speed different from Motor B helpful in turns
  int minSpeed = 10;//min speed different from Motor B helpful in turns
  int dir = 0;
  if (percent < 50) {
    dir = 0;
  }
  if (percent > 50) {
    dir = 1;
  }
  if (dir == 1) {//funtioning of motor driver
    pinMode(Aph, INPUT);
    pinMode(Apw, INPUT);
    pinMode(Aph, OUTPUT);
    pinMode(Apw, OUTPUT);
    digitalWrite(Aph, LOW);
    int drive = map(percent, 51, 100, 0, 255);//driving forward
    drive = constrain(drive, 0, 1023);//contrain 2^10
     Serial.print("Driving A Forward: ");
    Serial.println(drive);
    analogWrite(Apw, drive);//output to motor A
  }
  if (dir == 0) {//funtioning of motor driver
    pinMode(Aph, INPUT);
    pinMode(Apw, INPUT);
    pinMode(Aph, OUTPUT);
    pinMode(Apw, OUTPUT);
    digitalWrite(Aph, HIGH);
    int drive = map(percent, 49, 0, 0, 255);//backward movement
    drive = constrain(drive, 0, 1023);//contrain 2^10
    Serial.print("Driving A Back: ");
    Serial.println(drive);
    analogWrite(Apw, drive);//output to motor A
  }
  if (percent == 50) {//stationary condtion waiting for input motor A
    pinMode(Aph, INPUT);
    pinMode(Apw, INPUT);
  }
}

void motB(int percent) {//drive motor B
  int maxSpeed = 85;//max speed different from Motor A helpful in turns
  int minSpeed = 45;//min speed different from Motor B helpful in turns
  int dir = 0;
  if (percent < 50) {
    dir = 0;
  }
  if (percent > 50) {
    dir = 1;
  }
  if (dir == 1) {//funtioning of motor driver
    pinMode(Bph, INPUT);
    pinMode(Bpw, INPUT);
    pinMode(Bph, OUTPUT);
    pinMode(Bpw, OUTPUT);
    digitalWrite(Bph, HIGH);
    int drive = map(percent, 51, 100, 0, 255);//driving forward
    drive = constrain(drive, 0, 1023);//contrain 2^10
    Serial.print("Driving B Forward: ");
    Serial.println(drive);
    analogWrite(Bpw, drive);//output to motor B
  }
  if (dir == 0) {//funtioning of motor driver
    pinMode(Bph, INPUT);
    pinMode(Bpw, INPUT);
    pinMode(Bph, OUTPUT);
    pinMode(Bpw, OUTPUT);
    digitalWrite(Bph, LOW);
    int drive = map(percent, 49, 0, 0, 255);//driving backward
    drive = constrain(drive, 0, 1023);//contrain 2^10
     Serial.print("Driving B Backward ");
    Serial.println(drive);
    analogWrite(Bpw, drive);//output to motor B
  }
  if (percent == 50) {//stationary condtion waiting for input motor B
    pinMode(Bph, INPUT);
    pinMode(Bpw, INPUT);
  }
}

void intro() {//intro message
  u8g2.drawStr( 7, 10, "ANN");
   Serial.println("ANN");
  u8g2.drawStr( 7, 25, "Rajat");
   Serial.println("Rajat");
  u8g2.drawStr( 7, 40, "Ajit");
   Serial.println("Ajit");
}

void menu_select() {//display menu
  menu_circle();
  u8g2.drawStr( 7, 10, "1)TestCir");
   Serial.println("1)TestCir");
  u8g2.drawStr( 7, 25, "2)TrainNN");
   Serial.println("2)TrainNN");
  u8g2.drawStr( 7, 40, "3)RunNN");
  Serial.println("3)TrainNN");
}


void menu_circle() {//create circle for selection of option in menu
  int yloc = curr_menu * 15;//location to draw circle for selection
  u8g2.drawCircle(3, yloc - 9, 2);//circle centre, indent and diameter in parameter
}


void Change_menu(void)
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 50)
  {
    int buttState = digitalRead(Button_left);//checking for on State
    if (buttState == 0) {
      curr_menu++;//change to next program of menu
      if (curr_menu > menu_items) {//if menu exceed menu item it brings back to option 1
        curr_menu = 1;
      }
    }
  }
  last_interrupt_time = interrupt_time;//update counter for future refernce
}

void Load_program() {
  prog_start = 1;
}

//TRAINS THE NEURAL NETWORK
void Training() {
 
  prog_start = 0;
  digitalWrite(LED_Yellow, LOW);//entering in a loop 1
  for ( i = 0 ; i < HddNde ; i++ ) {//  Initialize HddWeigh and ChnHddWeigh
    for ( j = 0 ; j <= InptNde ; j++ ) {
      ChnHddWeigh[j][i] = 0.0 ;//intializing hidden weights
      Rando = float(random(100)) / 100;//random assignment
      HddWeigh[j][i] = 2.0 * ( Rando - 0.5 ) * IntWeighMX ;
    }
  }
  digitalWrite(LED_Yellow, HIGH);//exiting in a loop 1 
 digitalWrite(LED_Red, LOW);//entering in a loop 2
  for ( i = 0 ; i < OutNde ; i ++ ) {//  Initialize OutWeigh and ChnOutWeigh
    for ( j = 0 ; j <= HddNde ; j++ ) {
      ChnOutWeigh[j][i] = 0.0 ;//intializing outer weights
      Rando = float(random(100)) / 100;//random assignment
      OutWeigh[j][i] = 2.0 * ( Rando - 0.5 ) * IntWeighMX ;
    }
  }
  digitalWrite(LED_Red, HIGH);//exiting loop 2

  for (TrnCyc = 1 ;TrnCyc < 2147483647 ;TrnCyc++) {//training

    for ( p = 0 ; p < PatternCnt ; p++) {// Randomize order of training patterns
      q = random(PatternCnt);//selecting random Input node to work with
      r = RndmInd[p] ;//swapping
      RndmInd[p] = RndmInd[q] ;
      RndmInd[q] = r ;
    }
    Error = 0.0 ;//error re-assignment
 
    for ( q = 0 ; q < PatternCnt ; q++ ) {
      p = RndmInd[q];//  Cycle through each training pattern in the randomized order
      digitalWrite(LED_Yellow, LOW);//entering loop 3
      for ( i = 0 ; i < HddNde ; i++ ) {//Compute hidden layer activations
        Accumulator = HddWeigh[InptNde][i] ;//Accumulatorulator for hidden layer
        for ( j = 0 ; j < InptNde ; j++ ) {
          Accumulator += Input[p][j] * HddWeigh[j][i] ;//Accumulatorulating hidden weight
        }
        Hidden[i] = 1.0 / (1.0 + exp(-Accumulator)) ;//Assigning weights hidden node
      }
      digitalWrite(LED_Yellow, HIGH);//exiting loop 3
      digitalWrite(LED_Red, LOW);//entering loop 4
      for ( i = 0 ; i < OutNde ; i++ ) {// Compute output layer activations and calculate errors
        Accumulator = OutWeigh[HddNde][i] ;
        for ( j = 0 ; j < HddNde ; j++ ) {
          Accumulator += Hidden[j] * OutWeigh[j][i] ;
        }
        Output[i] = 1.0 / (1.0 + exp(-Accumulator)) ;//Assigning weights output node
        OutDel[i] = (Target[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]) ;//delta calculation
        Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]) ;
      }
      //Serial.println(Output[0]*100);
      digitalWrite(LED_Red, HIGH);//exixting loop 4
      
      digitalWrite(LED_Yellow, LOW);//entering loop 5
      for ( i = 0 ; i < HddNde ; i++ ) {
        Accumulator = 0.0 ;
        for ( j = 0 ; j < OutNde ; j++ ) {
          Accumulator += OutWeigh[i][j] * OutDel[j] ;
        }
        HddDel[i] = Accumulator * Hidden[i] * (1.0 - Hidden[i]) ;// Backpropagate errors to hidden layer
      }
      digitalWrite(LED_Yellow, HIGH);//exiting loop 5

      digitalWrite(LED_Red, LOW);//entering loop 6
      for ( i = 0 ; i < HddNde ; i++ ) {
        ChnHddWeigh[InptNde][i] = LearnRt * HddDel[i] + moment * ChnHddWeigh[InptNde][i] ;//update hidden weights
        HddWeigh[InptNde][i] += ChnHddWeigh[InptNde][i] ;
        for ( j = 0 ; j < InptNde ; j++ ) {
          ChnHddWeigh[j][i] = LearnRt * Input[p][j] * HddDel[i] + moment * ChnHddWeigh[j][i];//update hidden weights matrix
          HddWeigh[j][i] += ChnHddWeigh[j][i] ;// Update Inner-->Hidden Weights
        }
      }
      digitalWrite(LED_Red, HIGH);//exiting loop 6
      digitalWrite(LED_Yellow, LOW);//enetering loop 7
      for ( i = 0 ; i < OutNde ; i ++ ) {
        ChnOutWeigh[HddNde][i] = LearnRt * OutDel[i] + moment * ChnOutWeigh[HddNde][i] ;//update output weights
        OutWeigh[HddNde][i] += ChnOutWeigh[HddNde][i] ;
        for ( j = 0 ; j < HddNde ; j++ ) {
          ChnOutWeigh[j][i] = LearnRt * Hidden[j] * OutDel[i] + moment * ChnOutWeigh[j][i] ;//update output weight matrix
          OutWeigh[j][i] += ChnOutWeigh[j][i] ;//    Update Hidden-->Output Weights
        }
      }
      digitalWrite(LED_Yellow, HIGH);//exiting loop 7
    }

    RprtEvry100 = RprtEvry100 - 1;
    if (RprtEvry100 == 0)
    {
      int graphNum =TrnCyc / 100;// Every 100 cycles send data to terminal
      int graphE1 = Error * 1000;
      int graphE = map(graphE1, 3, 80, 47, 0);//postioning of graph
      ErrGrph[graphNum] = graphE;// for display and draws the graph on OLED
      u8g2.firstPage();
      do {
        drawGraph();//draw graph function
      } while ( u8g2.nextPage() );
//to print on serial monitor
      Serial.println();
      Serial.println();
      Serial.print ("tarining cycle ");
      Serial.print (TrnCyc);
      Serial.print ("  Error = ");
      Serial.println (Error, 5);
      Serial.print ("  Graph Num: ");
      Serial.print (graphNum);
      Serial.print ("  Graph Error1 = ");
      Serial.print (graphE1);
      Serial.print ("  Graph Error = ");
      Serial.println (graphE);

      toTerminal();//print graph on serial montior or serial plotter

      if (TrnCyc == 1)
      {
        RprtEvry100 = 99;
      }
      else
      {
        RprtEvry100 = 100;
      }
    }

    if ( Error < Success ) break ;//  If error rate is less than pre-determined threshold then end
  }
}

//USES TRAINED NEURAL NETWORK TO DRIVE ROBOT
void Execute()
{
  if (Success < Error) {// failsafe condition to check the training of neural network
    prog_start = 0;
    Serial.println("NN not Trained");
  }
  while (Error < Success) {
    int num;
    int farDist = 35;//far distance 
    int closeDist = 7;//close distance
    float TestInput[] = {0, 0, 0, 0};//test condition when no light is present
    digitalWrite(LED_Yellow, LOW);//entering reading section

    int LL1 = analogRead(A0);   // Collect sonar distances.
    int LL2 = analogRead(A1);   // Collect sonar distances.
    int LL3 = analogRead(A2);   // Collect sonar distances.
    int LL4 = analogRead(A3);   // Collect sonar distances.

#ifdef DEBUG//ommited
    Serial.print("Light Level: ");
    Serial.print(LL1);
    Serial.print("\t");
    Serial.print(LL2);
    Serial.print("\t");
    Serial.print(LL3);
    Serial.print("\t");
    Serial.println(LL4);
#endif

    digitalWrite(LED_Yellow, HIGH);//exiting reading section

    LL1 = map(LL1, 400, 1024, 0, 100);
    LL2 = map(LL2, 400, 1024, 0, 100);
    LL3 = map(LL3, 400, 1024, 0, 100);
    LL4 = map(LL4, 400, 1024, 0, 100);

    LL1 = constrain(LL1, 0, 100);
    LL2 = constrain(LL2, 0, 100);
    LL3 = constrain(LL3, 0, 100);
    LL4 = constrain(LL4, 0, 100);

    TestInput[0] = float(LL1) / 100;
    TestInput[1] = float(LL2) / 100;
    TestInput[2] = float(LL3) / 100;
    TestInput[3] = float(LL4) / 100;
#ifdef DEBUG//ommitted
    Serial.print("Input: ");
    Serial.print(TestInput[3], 2);
    Serial.print("\t");
    Serial.print(TestInput[2], 2);
    Serial.print("\t");
    Serial.print(TestInput[1], 2);
    Serial.print("\t");
    Serial.println(TestInput[0], 2);
#endif

    InputToOutput(TestInput[0], TestInput[1], TestInput[2], TestInput[3]); //mapping test inputs to motor output percentage

    int SpdA = Output[0] * 100;
    int SpdB = Output[1] * 100;
    SpdA = int(SpdA);//typecasting
    SpdB = int(SpdB);//typecasting

    Serial.print("Speed: ");
    Serial.print(SpdA);
    Serial.print("\t");
    Serial.println(SpdB);

    motA(SpdA);//moving motor A
    motB(SpdB);//moving motor B
    delay(50);
  }
}

void drawGraph() {//graph function
  for (int x = 2; x < 64; x++) {
    u8g2.drawLine(x - 1, ErrGrph[x - 2], x - 1, ErrGrph[x - 1]);
  }
}


void toTerminal()
{//display info on serial plotter

  for ( p = 0 ; p < PatternCnt ; p++ ) {
    Serial.println();
    Serial.print ("  Training Pattern: ");
    Serial.println (p);
    Serial.print ("  Input ");
    for ( i = 0 ; i < InptNde ; i++ ) {
      Serial.print (Input[p][i], DEC);
      Serial.print (" ");
    }
    Serial.print ("  Target ");
    for ( i = 0 ; i < OutNde ; i++ ) 
    {
     Serial.print (Target[p][i], DEC);
     Serial.print (" ");
    }

    for ( i = 0 ; i < HddNde ; i++ ) {// Compute hidden layer activations
      Accumulator = HddWeigh[InptNde][i] ;
      for ( j = 0 ; j < InptNde ; j++ ) {
        Accumulator += Input[p][j] * HddWeigh[j][i] ;


      }
      Hidden[i] = 1.0 / (1.0 + exp(-Accumulator)) ;
    }

    for ( i = 0 ; i < OutNde ; i++ ) {// Compute output layer activations and calculate errors
      Accumulator = OutWeigh[HddNde][i] ;
      for ( j = 0 ; j < HddNde ; j++ ) {
        Accumulator += Hidden[j] * OutWeigh[j][i] ;
      }
      Output[i] = 1.0 / (1.0 + exp(-Accumulator)) ;
    }
    Serial.print ("  Output ");
    for ( i = 0 ; i < OutNde ; i++ ) {
      Serial.print (Output[i], 5);
      Serial.print (" ");
    }
  }
}

void InputToOutput(float In1, float In2, float In3, float In4)
{
  float TestInput[] = {0, 0, 0, 0};
  TestInput[0] = In1;
  TestInput[1] = In2;
  TestInput[2] = In3;
  TestInput[3] = In4;
  for ( i = 0 ; i < HddNde ; i++ ) {//  Compute hidden layer activations
    Accumulator = HddWeigh[InptNde][i] ;
    for ( j = 0 ; j < InptNde ; j++ ) {
      Accumulator += TestInput[j] * HddWeigh[j][i] ;
    }
    Hidden[i] = 1.0 / (1.0 + exp(-Accumulator)) ;
  }

  for ( i = 0 ; i < OutNde ; i++ ) {// Compute output layer activations and calculate errors
    Accumulator = OutWeigh[HddNde][i] ;
    for ( j = 0 ; j < HddNde ; j++ ) {
      Accumulator += Hidden[j] * OutWeigh[j][i] ;
    }
    Output[i] = 1.0 / (1.0 + exp(-Accumulator)) ;
  }
#ifdef DEBUG
  Serial.println("  Output ");
  for ( i = 0 ; i < OutNde ; i++ ) {
    Serial.print(Output[i], 5);
    Serial.print(" ");
  }
#endif
}
