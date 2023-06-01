                                                                                                         /////////////////////////
//------ Header -------//
/////////////////////////

#include <POP32.h> // นำเอาไลบราลี่บอร์ด POP-32 เข้ามาใช้งาน
#include <POP32_Pixy2.h>
POP32_Pixy2 pixy;

/////////////////////////
//------ DEFINE -------//
/////////////////////////

#define L analog(0)  // กำหนดเซนเซอร์ตำแหน่ง L เสียบที่ช่อง A0
#define R analog(1)  // กำหนดเซนเซอร์ตำแหน่ง R เสียบที่ช่อง A1
#define LL analog(2) // กำหนดเซนเซอร์ตำแหน่ง LL เสียบที่ช่อง A2
#define RR analog(3) // กำหนดเซนเซอร์ตำแหน่ง RR เสียบที่ช่อง A3
#define C analog(4)  // กำหนดเซนเซอร์ตำแหน่ง C เสียบที่ช่อง A4

#define getsonar int(analog(5) / 41) // กำหนดเซนเซอร์ตำแหน่งวัดระยะ เสียบที่ช่อง A5
#define obstacle_distance 7          // กำหนดค่าระยะห่าง เมื่อพบสิ่งกีดขวาง หน่วยคือ เซนติเมตร / cm.

#define SW_BL in(16)
#define SW_BR in(7)

#define reff_C_Silver 3850 // ค่ากึ่งกลาง ระหว่างสีขาวและสีเทาเงิน(ทางเข้าห้อง) ตำแหน่ง C

#define reff_L 3200 // ค่ากึ่งกลาง ระหว่างสีขาวและสีดำ ตำแหน่ง L
#define reff_R 3200 // ค่ากึ่งกลาง ระหว่างสีขาวและสีดำ ตำแหน่ง R
#define reff_C 2400 // ค่ากึ่งกลาง ระหว่างสีขาวและสีดำ ตำแหน่ง C

#define reff_LL 600  // ค่าสีเขียวที่วัดได้ จากเซนเซอร์ ตำแหน่ง LL
#define reff_RR 1160 // ค่าสีเขียวที่วัดได้ จากเซนเซอร์ ตำแหน่ง RR

#define diff_LL_Plus 300  // ค่าความแคบของค่าที่จับสีเขียว ฝั่งบวก
#define diff_LL_Minus 150 // ค่าความแคบของค่าที่จับสีเขียว ฝั่งลบ

#define diff_RR_Plus 400  // ค่าความแคบของค่าที่จับสีเขียว ฝั่งบวก
#define diff_RR_Minus 200 // ค่าความแคบของค่าที่จับสีเขียว ฝั่งลบ

#define powerNormal 30 // กำลังมอเตอร์ปกติ

#define powerTurn 50 // กำลังมอเตอร์ในการเลี้ยว

unsigned int function;
unsigned long currentTime;
boolean pauseMonitor = false;

int pos_robot = 0; // 0 = C, 1 = L, 2 = R; // ตำแหน่งหุ่นยนต์ที่หลุดออกจากเส้นสีดำ

double P, I, D, pv_error, PID_Value;
int error;
int count_foundgreen = 0;

////////////////////////////
//----- Function 1 -------//
////////////////////////////
void Stop(unsigned int t)
{
  motor(1, -powerNormal);
  motor(2, -powerNormal);
  delay(int(t / 2));

  ao();
  delay(int(t / 2));
}
void Wait()
{
  ao();
  delay(200);

  beep();

  while (SW_OK() == false && SW_BL == true && SW_BR == true)
  {
  }
}

void TL90()
{
  fd(powerNormal);
  delay(150);
  ao();
  delay(100);

  // TL90
  sl(powerTurn);
  delay(100);
  while (L <= reff_L)
  {
    sl(powerTurn);
    delay(20);
  }
  while (L >= reff_L)
  {
    sl(powerTurn);
    delay(20);
  }
  while (L <= reff_L)
  {
    sl(powerTurn);
    delay(20);
  }
  trackLine(100);
}
void TR90()
{
  fd(powerNormal);
  delay(150);
  ao();
  delay(100);

  // TR90
  sr(powerTurn);
  delay(100);
  while (R <= reff_R)
  {
    sr(powerTurn);
    delay(20);
  }
  while (R >= reff_R)
  {
    sr(powerTurn);
    delay(20);
  }
  while (R <= reff_R)
  {
    sr(powerTurn);
    delay(20);
  }
  trackLine(100);
}
void U90()
{
  bk(powerNormal);
  delay(200);
  ao();
  delay(100);

  // TR90
  sr(powerTurn);
  delay(100);
  while (R <= reff_R)
  {
    sr(powerTurn);
    delay(20);
  }
  while (R >= reff_R)
  {
    sr(powerTurn);
    delay(20);
  }
  while (R <= reff_R)
  {
    sr(powerTurn);
    delay(20);
  }
  trackLine(100);
}
void Obstacle()
{
  sl(powerNormal);
  delay(500);

  ao();
  delay(100);

  fd(powerNormal);
  delay(800);

  ao();
  delay(100);

  sr(powerNormal);
  delay(500);

  ao();
  delay(100);

  fd(powerNormal);
  delay(1550);

  ao();
  delay(100);

  sr(powerNormal);
  delay(500);

  ao();
  delay(100);


  while(L > reff_L)
  {
    fd(powerNormal);
    delay(10);
  }

  ao();
  delay(100);

  TL90();

  trackLine(powerNormal, 100);
}
////////////////////////////
//----- TrackLine[PID-Controller] -------////
////////////////////////////
void get_error()
{
  if (L < reff_L && C < reff_C && R < reff_R)
  {
    pos_robot = 0;
    error = 0;
  }
  else if (L < reff_L && C > reff_C && R > reff_R)
  {
    pos_robot = 1;
    error = -2;
  }
  else if (L < reff_L && C < reff_C && R > reff_R)
  {
    pos_robot = 1;
    error = -1;
  }
  else if (L > reff_L && C < reff_C && R > reff_R)
  {
    pos_robot = 0;
    error = 0;
  }
  else if (L > reff_L && C < reff_C && R < reff_R)
  {
    pos_robot = 2;
    error = 1;
  }
  else if (L > reff_L && C > reff_C && R < reff_R)
  {
    pos_robot = 2;
    error = 2;
  }
  else if (L > reff_L && C > reff_C && R > reff_R)
  {
    if (pos_robot == 1)
    {
      error = -4;
    }
    else if (pos_robot == 2)
    {
      error = 4;
    }
    else
    {
      error = 0;
    }
  }
}
void trackLine(int pw)
{
  get_error();

  P = error;
  I = I + error;
  D = error - pv_error;

  PID_Value = (20.0 * P) + (0.0 * I) + (30.0 * D);

  pv_error = error;

  int L_Motor_Speed = pw + PID_Value;
  int R_Motor_Speed = pw - PID_Value;

  constrain(L_Motor_Speed, -100, 100);
  constrain(R_Motor_Speed, -100, 100);

  motor(1, L_Motor_Speed);
  motor(2, R_Motor_Speed);
}
void trackLine(int pw, unsigned int Time)
{
  currentTime = millis();
  while (millis() - currentTime < Time)
  {
    trackLine(pw);
  }
}
////////////////////////////
//----- Main Run -------////
////////////////////////////
void main_run() // -- Main Run โค้ดการทำงานหลัก
{
  int count_for_detect_green = 26;
  if (getsonar <= obstacle_distance)
  { // พบวัตถุสิ่งกีดขวาง
    ao();
    delay(200);

    oled.clear();
    oled.textSize(2);
    oled.text(1, 0, "Obstacle!");
    oled.show();

    Obstacle();
  }
  
  if (C > reff_C_Silver) // พบทางเข้าห้องอพยพ
  {
    // ก่อนเข้าห้อง
    ao();
    delay(200);

    oled.clear();
    oled.textSize(2);
    oled.text(1, 0, "Ball Zone!");
    oled.show();

    // เข้าห้องไปแล้ว
    rescue_main();

  }

  // เจอแยก และเจอสีเขียวซ้าย และขวา
  else if ((L < reff_L && C < reff_C && R < reff_R) &&
           (((LL < reff_LL + diff_LL_Plus) && (LL > reff_LL - diff_LL_Minus)) &&
            ((RR < reff_RR + diff_RR_Plus) && (RR > reff_RR - diff_RR_Minus))))
  {
    count_foundgreen++;
    if (count_foundgreen > count_for_detect_green)
    {
      Stop(100);

      beep();

      oled.clear();
      oled.textSize(2);
      oled.text(1, 1, "U90!");
      oled.show();

      U90();
    }
  }

  // เจอแยก และเจอเขียวด้านซ้าย
  else if ((L < reff_L && C < reff_C && R < reff_R) &&
           ((LL < reff_LL + diff_LL_Plus) && (LL > reff_LL - diff_LL_Minus)))
  {
    count_foundgreen++;
    if (count_foundgreen > count_for_detect_green)
    {
      Stop(100);

      beep();

      oled.clear();
      oled.textSize(2);
      oled.text(1, 1, "TL90!");
      oled.show();

      TL90();
    }
  }

  // เจอแยก และเจอเขียวด้านขวา
  else if ((L < reff_L && C < reff_C && R < reff_R) &&
           ((RR < reff_RR + diff_RR_Plus) && (RR > reff_RR - diff_RR_Minus)))
  {
    count_foundgreen++;
    if (count_foundgreen > count_for_detect_green)
    {
      Stop(100);

      beep();

      oled.clear();
      oled.textSize(2);
      oled.text(1, 1, "TR90!");
      oled.show();

      TR90();
    }
  }
  else
  {
    count_foundgreen = 0;
  }
  trackLine(powerNormal);
}
////////////////////////////
//----- Function 2 ---------//
////////////////////////////
void showAllSensor()
{
  if (SW_A()) //  กดสวิตซ์ A จะพักการแสดงหน้าจอ เพื่อให้จดค่าแสง ที่แสดงบนหน้าจอ
  {
    if (pauseMonitor)
    {
      pauseMonitor = false;
      beep();
      delay(100);
    }
    else
    {
      pauseMonitor = true;
      beep();
      delay(100);
    }
  }
  if (SW_B()) //  กดสวิตซ์ B จะทำการแทร็กเส้น ไปหยุดที่แยกแล้วพักการแสดงหน้าจอ
  {
    beep();
    while (L > reff_L || R > reff_R)
    {
      trackLine(powerNormal);
    }
    ao();
    delay(200);

    oled.clear();
    delay(200);

    oled.textSize(1);
    oled.text(0, 1, "Track_L: %d ", L);
    oled.text(1, 1, "Track_C: %d ", C);
    oled.text(2, 1, "Track_R: %d ", R);
    oled.text(3, 1, "CheckG_L: %d ", LL);
    oled.text(4, 1, "CheckG_R: %d ", RR);
    oled.text(5, 0, "Press SW_A To Pause");
    oled.text(6, 0, "Press SW_B To Run");
    oled.text(7, 4, "                      ");
    oled.show();

    pauseMonitor = true;
    beep();
  }

  if (pauseMonitor) // ถ้ามีคำสั่งให้พักการแสดงหน้าจอ
  {
    oled.text(5, 0, "Press SW_A To Resume");
    oled.text(6, 0, "                      ");
    oled.text(7, 4, "PAUSE!! ");
    oled.show();
  }
  else // ถ้าไม่มีคำสั่งให้พักการแสดงหน้าจอ
  {
    oled.textSize(1);
    oled.text(0, 1, "Track_L: %d ", L);
    oled.text(1, 1, "Track_C: %d ", C);
    oled.text(2, 1, "Track_R: %d ", R);
    oled.text(3, 1, "CheckG_L: %d ", LL);
    oled.text(4, 1, "CheckG_R: %d ", RR);
    oled.text(5, 0, "Press SW_A To Pause");
    oled.text(6, 0, "Press SW_B To Run");
    oled.text(7, 4, "                      ");
    oled.show();
  }
}
void showNameFunction()
{
  function = knob(0, 6);
  oled.textSize(1);
  oled.text(0, 4, "Choose Program");
  oled.textSize(1);
  oled.text(2, 3, "Main Run");
  oled.text(3, 3, "Measure Sensor");
  oled.text(4, 3, "Measure Sonar");
  oled.text(5, 3, "Read Servo");

  // Cursor
  if (function == 0)
  {
    oled.text(2, 0, "->");
    oled.text(3, 0, "  ");
    oled.text(4, 0, "  ");
    oled.text(5, 0, "  ");
  }
  else if (function == 1)
  {
    oled.text(2, 0, "  ");
    oled.text(3, 0, "->");
    oled.text(4, 0, "  ");
    oled.text(5, 0, "  ");
  }
  else if (function == 2)
  {
    oled.text(2, 0, "  ");
    oled.text(3, 0, "  ");
    oled.text(4, 0, "->");
    oled.text(5, 0, "  ");
  }
  else if (function == 3)
  {
    oled.text(2, 0, "  ");
    oled.text(3, 0, "  ");
    oled.text(4, 0, "  ");
    oled.text(5, 0, "->");
  }
  else
  {
    oled.text(2, 0, "  ");
    oled.text(3, 0, "  ");
    oled.text(4, 0, "  ");
    oled.text(5, 0, "  ");
  }

  oled.show();
}
void showSonar()
{
  oled.clear();
  while (true)
  {
    oled.textSize(1);
    oled.text(1, 1, "dist: %d  cm.", getsonar);
    oled.show();
  }
}
void readServo()
{
  int x = knob(0,180);
  int y = knob(180,0);
  servo(1,x);
  servo(2,y);
  oled.text(1,0,"S1:%d  ",x);
  oled.text(2,0,"S2:%d  ",y);
  oled.show();
  delay(50);
}
//////////////////////////////
//-- Life Cycle of Arduino --//
//---------- MAIN ----------//
///////////////////////////////
void setup()
{
  // ทำงานครั้งเดียว เมื่อเปิดสวิตซ
  pixy.init();
  // Serial.begin(115200);
  Hand_up();
  
  // เมนู
  while (true)
  {

    if (SW_OK() || SW_BL == false || SW_BR == false)
    {
      oled.clear();
      // oled.textSize(2);
      // oled.text(1, 1, "Running...");
      oled.show();
      beep();
      while(SW_BL == false || SW_BR == false){}
      break;
    }

    showNameFunction();
  }
  // Hand_down();
  // pixy.setLamp(true,false);
}
void loop()
{
  // ส่วนการทำงานหลัก ทำงานวนซ้ำไปเรื่อย ๆ จนกว่าเราจะปิดสวิตซ์

  if (function == 0)
  {
    // Main Running algorithm...
    main_run();
  }
  else if (function == 1)
  {
    // Measure Sensor
    showAllSensor();
  }
  else if (function == 2)
  {
    // Measure Sonar
    showSonar();
  }
  else if (function == 3)
  {
    // Measure Sonar
    readServo();
  }
  else
  {
    Hand_down(); Wait();
    Hand_up_slow(); Wait();
  }
}

////////////////////////////
//---------- END ---------//
////////////////////////////
