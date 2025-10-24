#include "vex.h"
#include "robot-config.h"

using namespace vex;
competition Competition;
controller Controller1 = controller(primary);

motor LA = motor(PORT1, ratio6_1, true);
motor LB = motor(PORT2, ratio6_1, true);
motor LC = motor(PORT3, ratio6_1, true);
motor_group LM = motor_group(LA, LB, LC);

motor RA = motor(PORT6, ratio6_1, false);
motor RB = motor(PORT9, ratio6_1, false);
motor RC = motor(PORT4, ratio6_1, false);
motor_group RM = motor_group(RA, RB, RC);

motor_group Drivetrain = motor_group(LA, LB, LC, RA, RB, RC);

motor LowerRoller = motor(PORT7, ratio18_1, true);
motor UpperRoller = motor(PORT10, ratio18_1, true);
motor MiddleRoller = motor(PORT5, ratio18_1, true);
motor IntakeRoller = motor(PORT19, ratio18_1, true);

optical EYE = optical(PORT21);

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

    // Pick your drive setup from the list below:
    // ZERO_TRACKER_NO_ODOM
    // ZERO_TRACKER_ODOM
    // TANK_ONE_FORWARD_ENCODER
    // TANK_ONE_FORWARD_ROTATION
    // TANK_ONE_SIDEWAYS_ENCODER
    // TANK_ONE_SIDEWAYS_ROTATION
    // TANK_TWO_ENCODER
    // TANK_TWO_ROTATION
    // HOLONOMIC_TWO_ENCODER
    // HOLONOMIC_TWO_ROTATION
    //
    // Write it here:
    ZERO_TRACKER_NO_ODOM,

    // Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
    // You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

    // Left Motors:
    LM,

    // Right Motors:
    RM,

    // Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
    PORT12,

    // Input your wheel diameter. (4" omnis are actually closer to 4.125"):
    3.25,

    // External ratio, must be in decimal, in the format of input teeth/output teeth.
    // If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
    // If the motor drives the wheel directly, this value is 1:
    0.75,

    // Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
    // For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
    360,

    /*---------------------------------------------------------------------------*/
    /*                                  PAUSE!                                   */
    /*                                                                           */
    /*  The rest of the drive constructor is for robots using POSITION TRACKING. */
    /*  If you are not using position tracking, leave the rest of the values as  */
    /*  they are.                                                                */
    /*---------------------------------------------------------------------------*/

    // If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

    // FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
    // LF:      //RF:
    PORT17, -PORT20,

    // LB:      //RB:
    PORT16, -PORT18,

    // If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
    // If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
    // If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
    3,

    // Input the Forward Tracker diameter (reverse it to make the direction switch):
    2.75,

    // Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
    // For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
    // This distance is in inches:
    -2,

    // Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
    1,

    // Sideways tracker diameter (reverse to make the direction switch):
    -2.75,

    // Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
    5.5

);

int current_auton_selection = 0;
bool auto_started = false;
bool FrameUP = false;
bool FrameDOWN = false;

// --- Optical SET UP ---
const int EYE_MIN_BRIGHT = 30; // tune 20–40 on-field
bool colorSwapRedMode = false;

inline bool eyeSeesBlue()
{
  if (colorSwapRedMode == false)
    return true;
  return false;
}
inline bool eyeSeesRed()
{
  if (colorSwapRedMode == true)
    return true;
  return false;
}
inline const char *eyeStatus()
{
  int h = (int)EYE.hue();
  if (h > 150 && h < 280)
  {
    colorSwapRedMode = false;
    return "BLUE";
  }
  if (h < 50 || h >= 350)
  {
    colorSwapRedMode = true;
    return "RED";
  }
  return "NONE";
}

// ===== Auton Selector UI =====
struct Button
{
  int id;
  int x, y, w, h;
  color fill;
  const char *label;
};

inline bool inRect(int x, int y, const Button &b)
{
  return (x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h);
}

inline void drawButton(const Button &b, bool selected)
{
  Brain.Screen.setFillColor(b.fill);
  Brain.Screen.setPenColor(selected ? color::white : color::black);
  Brain.Screen.drawRectangle(b.x, b.y, b.w, b.h);

  if (selected)
  {
    Brain.Screen.setPenWidth(4);
    Brain.Screen.drawRectangle(b.x, b.y, b.w, b.h, color::transparent);
    Brain.Screen.setPenWidth(1);
  }

  // Center label
  Brain.Screen.setFont(mono40);
  int textW = Brain.Screen.getStringWidth(b.label);
  int cx = b.x + (b.w - textW) / 2;
  int cy = b.y + b.h / 2 + 7;
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.printAt(cx, cy, b.label);
}

inline const char *autonName(int sel)
{
  switch (sel)
  {
  case 0:
    return "RED"; // LEFT RED
  case 1:
    return "BLUE"; // LEFT BLUE
  case 2:
    return "SKILLS";
  case 3:
    return "RED"; // RIGHT RED
  case 4:
    return "BLUE"; // RIGHT BLUE
  }
  return "—";
}

inline void drawUI(const Button btns[], int n, int selectedId, double heading)
{
  Brain.Screen.clearScreen(color::black);

  Brain.Screen.setPenColor(color::yellow);
  Brain.Screen.setFont(mono40);
  Brain.Screen.printAt(380, 30, false, "3340E");
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.setFont(mono30);
  Brain.Screen.printAt(10, 50, false, "Battery: %d%%", Brain.Battery.capacity());
  Brain.Screen.printAt(10, 75, false, "Heading: %.1f", heading);
  Brain.Screen.printAt(10, 95, false, "Optical: %s   ", eyeStatus());
  Brain.Screen.setPenColor(color::yellow);
  Brain.Screen.setFont(mono30);
  Brain.Screen.printAt(10, 130, false, "%s", autonName(selectedId));

  for (int i = 0; i < n; ++i)
    drawButton(btns[i], btns[i].id == selectedId);

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Battery: %d%%", Brain.Battery.capacity());

  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Heading: %.1f", chassis.get_absolute_heading());

  // Controller1.Screen.setCursor(3,1);
  // Controller1.Screen.print("Color Mode: %s", (colorSwapRedMode? "RED" : "BLUE"));

  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Auton: %s", autonName(selectedId));
}

void pre_auton()
{
  vexcodeInit();
  default_constants();
  chassis.Gyro.calibrate();
  while (chassis.Gyro.isCalibrating())
    wait(200, msec);

  EYE.setLightPower(100);
  EYE.setLight(ledState::on);

  Button buttons[] = {
      {0, 10, 140, 140, 70, color::red, "RED"},
      {1, 170, 140, 140, 70, color::blue, "BLUE"},
      {2, 330, 140, 140, 70, color::orange, "SKILLS"}};
  const int BTN_COUNT = sizeof(buttons) / sizeof(buttons[0]);

  wait(200, msec);
  drawUI(buttons, BTN_COUNT, current_auton_selection, chassis.get_absolute_heading());

  // Loop until comp switches to autonomous
  while (!auto_started)
  {
    // Touch handling
    if (Brain.Screen.pressing())
    {
      const int x = Brain.Screen.xPosition();
      const int y = Brain.Screen.yPosition();
      for (int i = 0; i < BTN_COUNT; ++i)
      {
        if (inRect(x, y, buttons[i]))
        {
          current_auton_selection = buttons[i].id;
          drawUI(buttons, BTN_COUNT, current_auton_selection, chassis.get_absolute_heading());
          while (Brain.Screen.pressing())
            wait(5, msec); // debounce
          break;
        }
      }
    }

    // Periodic refresh so battery/heading/optical stay live
    static uint32_t last = 0;
    const uint32_t now = vex::timer::system();
    if (now - last > 150)
    { // ~6.6 Hz
      drawUI(buttons, BTN_COUNT, current_auton_selection, chassis.get_absolute_heading());
      last = now;
    }

    wait(10, msec); // <-- crucial for smooth auto-refresh
  }
}

void autonomous(void)
{
  auto_started = true;
  switch (current_auton_selection)
  {
  case 0:
    LEFT_RED_AUTON();
    break;
  case 1:
    LEFT_BLUE_AUTON();
    break;
  case 2:
    SKILLS_AUTON();
    break;
  case 3:
    RIGHT_RED_AUTON();
    break;
  case 4:
    RIGHT_BLUE_AUTON();
    break;
  }
}
// este funciona y es el code para el solenoide
bool FrameState = false;
void toggleFrame()
{
  FrameState = !FrameState;
  Frame.set(FrameState);
}

void usercontrol(void)
{
  chassis.control_arcade();

  while (1)
  {
    // Conducción

    bool up = Controller1.ButtonUp.pressing();
    if (Controller1.ButtonUp.pressing())
    {
      wait(20, msec);
      Controller1.ButtonUp.pressed(toggleFrame);
      wait(20, msec);
    }
    // Controller1.ButtonUp.pressed(toggleFrame);
    // bool color_sort_mode = (current_auton_selection == 0 || current_auton_selection == 2);

    // R2: -------- SCORE INTO LONG GOAL -----------------
    if (Controller1.ButtonR2.pressing())
    {
      IntakeRoller.spin(reverse, 100, percent);
      LowerRoller.spin(reverse, 100, percent);
      MiddleRoller.spin(forward, 100, percent);
      UpperRoller.spin(forward, 100, percent);
    }
    // L1: --------- Intake ----------------------------
    else if (Controller1.ButtonL1.pressing())
    {
      IntakeRoller.spin(reverse, 100, percent);
      LowerRoller.spin(forward, 100, percent);
      MiddleRoller.spin(forward, 100, percent);
    }
    // R1 -------- SCORE INTO UPPER GOAL -------------
    else if (Controller1.ButtonR1.pressing())
    {
      IntakeRoller.spin(reverse, 100, percent);
      LowerRoller.spin(reverse, 100, percent);
      MiddleRoller.spin(forward, 100, percent);
      UpperRoller.spin(reverse, 100, percent);
    }
    // L2 --------SCORE INTO LOWER GOAL------------
    else if (Controller1.ButtonL2.pressing())
    {
      IntakeRoller.spin(forward, 100, percent);
      LowerRoller.spin(reverse, 100, percent);
      MiddleRoller.spin(reverse, 100, percent);
      UpperRoller.spin(reverse, 100, percent);
    }
    // PRESSING ANYTHING
    else
    {
      LowerRoller.stop();
      UpperRoller.stop();
      MiddleRoller.stop();
      IntakeRoller.stop();
    }

    // Toggle del solenoide
    // este no funciona
    // if (up && !FrameDOWN) {
    //   if (!FrameUP) {
    //     FrameUP = true;
    //     Frame.set(FrameUP);
    //   }
    // }
    // FrameDOWN = up;

    wait(20, msec);
  }
}

int main()
{
  // Competition.autonomous(RIGHT_RED_AUTON);
  Competition.drivercontrol(usercontrol);

  // pre_auton();
  // vexcodeInit();

  while (true)
  {
    wait(100, msec);
  }
}

// No se si vas a ver este frase, pero si haces, necesitas saber que estas un maestro muy bueno. - Rishaan
