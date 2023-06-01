
/////////////////////////
//------ Header -------//
/////////////////////////

/////////////////////////
//------ DEFINE -------//
/////////////////////////

// พอร์ตการเสียบสายเซอร์โว
#define hand_L 1
#define hand_R 2

// ค่าเซอร์โว ตอนวางมือลงเพื่อเก็บลูกบอล
#define hand_L_down 180
#define hand_R_down 0

// ค่าเซอร์โว ตอนยกมือขึ้นตรง
#define hand_L_up 76
#define hand_R_up 105

// ค่าเซอร์โว ตอนยกเทลูกบอล
#define hand_L_dump 30
#define hand_R_dump 150

////////////////////////////
//---- Function Servo ----//
////////////////////////////

// Raw function of servo
void Hand_down()
{
  for (int i = hand_L_up; i < hand_L_down; i += 1)
  {
    servo(hand_L, i);
    servo(hand_R, map(i, hand_R_down, hand_L_down, hand_L_down, hand_R_down));
    delay(5);
  }
  delay(100);
}
void Hand_up()
{
  servo(hand_L, hand_L_up); // ยกมือขึ้น
  servo(hand_R, hand_R_up);
  delay(500);
}
void Hand_dump()
{
  servo(hand_L, hand_L_dump); // ดั้มกระบะ
  servo(hand_R, hand_R_dump);
  delay(500);
}

void Hand_up_slow()
{
  int i = hand_L_down;
  int j = hand_R_down;
  while (i > hand_L_up || j < hand_R_up)
  {
    servo(hand_L, i); // ยกมือขึ้น
    servo(hand_R, j);

    i -= 1;
    j += 1;

    delay(6); // หน่วงความเร็ว ยิ่งเยอะ ยิ่งช้า
  }
}

void showServo()
{
  oled.clear();

  while (true)
  {
    int i = knob(0, 180);
    int j = knob(180, 0);

    oled.text(0, 0, "L:%d  ", i);
    oled.text(1, 0, "R:%d  ", j);
    oled.show();

    servo(hand_L, i);
    servo(hand_R, j);
  }
}

////////////////////////////
//---- Function Run ----//
////////////////////////////

// status [false = no! ball in bucket, true = ball in bucket]
boolean ball_red = false;
boolean ball_green = false;
boolean ball_blue = false;
boolean ball_yellow = false;

// Main
void rescue_main()
{
  pixy.setLamp(true, false); // สั่งเปิดไฟกล้อง

  rescue_start();
  rescue_findTriangle();
  rescue_ball();
}

// Sub Main
void rescue_start()
{
  fd(40);
  delay(1200);

  ao();
  delay(1000);

  sl(40);
  delay(550);

  

  ao();
  delay(500);

  bk_sw(2000); // ถอยจนกว่าสวิตซ์จะชน

  ao();
  delay(1000);

  fd(40);
  delay(500);

  ao();
  delay(1000);

  sl(40);
  delay(610); // เลี้ยวให้ตรงจุดอพยพ เพื่อถอยชน

  ao();
  delay(500);

  
}
void rescue_findTriangle() // หาจุดอพยพ
{
  unsigned int swTime = 0;
  unsigned int refSwTime = 280;
  while (true) // false = pressed, true = not press
  {
    if (SW_BL == false && SW_BR == false)
    {
      ao();
      delay(500);
      if (swTime > refSwTime)
      {
        // เจอจุดอพยพ เทลูกบาศก์ และออกจากฟังก์ชั่นนี้
        Dump();
        break;
      }
      else
      {
        bk(40);
        delay(500);

        ao();
        delay(1000);

        fd(40);
        delay(200);

        ao();
        delay(1000);

        sl(40);
        delay(550);

        

        ao();
        delay(500);

        swTime = 0;
      }
    }
    else if (SW_BL == false)
    {
      motor(1, 50);
      motor(2, -100);
      delay(1);
      swTime++;
    }
    else if (SW_BR == false)
    {
      motor(1, -100);
      motor(2, 50);
      delay(1);
      swTime++;
    }
    else
    {
      motor(1, -64);
      motor(2, -70);
      // delay(1);
    }
  }
}
void rescue_ball()
{
  rescue_01();     // เดินกวาดลูกบอลรอบที่ 1
  rescue_02();     // เดินกวาดลูกบอลรอบที่ 2
  rescue_03();     // เดินกวาดลูกบอลรอบที่ 3
  rescue_04();     // เลี้ยว และกลับไปหาจุดอพยพ เพื่อเทลูกบอล
  rescue_Finish(); // เดินกลับไปเท และหาจุดออกจากห้องอพยพ

  main_run(); // กลับไปทำภารกิจ เดินตามเส้น
}

// Part of rescue_ball
void rescue_01()
{
  motor(1, 50);
  motor(2, 47);
  delay(1700);

  ao();
  delay(2000);
  sl(40);
  delay(350);

  bk_sw(3500);

  fd_keep_ball_to_wall(6000);

  fd(40);
  delay(300);
  ao();
  delay(500);
  sl(40);
  delay(550);
  bk_sw(3500);
}
void rescue_02()
{
  fd_keep_ball_to_wall(5000);

  fd(40);
  delay(300);
  ao();
  delay(500);
  sl(40);
  delay(620);
  ao();
  delay(500);

  fd(40);
  delay(900);
  ao();
  delay(500);
  sl(40);
  delay(620);
  ao();
  delay(500);

  bk_sw(3500);
}
void rescue_03()
{
  fd_keep_ball_to_wall(5000);

  bk_sw(6000);
}
void rescue_04()
{
  fd(40);
  delay(500);

  ao();
  delay(1000);

  sl(40);
  delay(520);

  rescue_findTriangle();
}
void rescue_Finish()
{
  motor(1, 50);
  motor(2, 47);
  delay(1700);

  ao();
  delay(1000);

  sl(40);
  delay(350);

  bk_sw(3500);

  motor(1, 50);
  motor(2, 47);
  delay(800);

  ao();
  delay(500);

  sr(40);
  delay(490);

  ao();
  delay(500);

  while (true)
  {
    if (L < reff_L || C < reff_C || R < reff_R) // เมื่อเจอเส้น
    {
      trackLine(powerNormal, 100);
      break;
    }
    else
    { // ถ้าไม่เจอเส้น
      fd(40);
      delay(10);
    }
  }
}
// function of rescue_ball
void fd_keep_ball_to_wall(int Time)
{
  Hand_down();
  currentTime = millis();
  while (true)
  {
    if (getsonar <= 20 || millis() - currentTime > Time) // เจอกำแพง หรือใช้เวลาเยอะเกินกว่าที่กำหนด จะถือว่า ชนกำแพง
    {
      // found wall

      ao();
      delay(500);
      bk(30);
      delay(100);
      ao();
      delay(100);
      Hand_up_slow();
      break;
    }
    if (pixy.updateBlocks() && pixy.sigSize[1])
    {
      // found red ball
      
      ao(); delay(100);
      beep();
      beep();
      beep();

      redBall();

      currentTime=millis(); // reset time
      
    }
    motor(1, 40);
    motor(2, 40);
  }
}
void bk_sw(int Time)
{
  currentTime = millis();
  while (SW_BL == true || SW_BR == true) // false = pressed, true = not press
  {
    if (millis() - currentTime > Time) // กันสวิตซ์ไม่ชน ให้ใส่เวลาเผื่อไว้ ถ้าครบเวลาที่กำหนด จะออกจากลูป
    {
      break;
    }

    // การถอยหลัง
    motor(1, -68);
    motor(2, -70);
  }
  motor(1, -68);
  motor(2, -70);
  delay(500);
}

// function of servo
// เทกระบะ
void Dump()
{
  ao();
  delay(500);
  Hand_dump();

  fd(40);
  delay(50);
  bk(40);
  delay(300);
  ao();
  delay(500);

  Hand_up();
  ao();
  delay(500);
}

// เมื่อเจอลูกบอลสีแดง จะกระทำดังนี้
void redBall()
{
  motor(1, -2);
  motor(1, 50);
  delay(700);

  ao();
  delay(700);

  motor(1, 2);
  motor(1, -50);
  delay(700);

  ao();
  delay(500);
}

////////////////////////////
//---------- END ---------//
////////////////////////////
