
/*
  Sand Clock

  This program listens to incoming commands on the serial port (9600 baud). A command
  is processed when a CR (13) or LF (10) character arrives.

  If you're using the serial monitor of the Arduino IDE, make sure you select one of
  the end-of-line options.

  The commands are subdivided in groups:

    Command group 'p': Pen commands. These commands expose the functionality used in autonomous
    mode to the user.

    - Command 'ps': Set pen position.
      Arg. #1: X position (floating point).
      Arg. #2: Y position (floating point).

      This command should be executed when the pen isn't down (out of the sand).

      Examples: ps 0 30
                ps -20.55 +55.8

    - Command 'pm': Pen move.
      Arg. #1: Destination X position (floating point).
      Arg. #2: Destination Y position (floating point).

      This command draws a line from the current pen position to the specified destination.

      Examples: pm 20 50
                pm -.56 16.8

    - Command 'pa': Pen arc.
      Arg. #1: Horizontal radius (floating point).
      Arg. #2: Vertical radius (floating point).
      Arg. #3: Start angle specified as radians or degrees. [1]
      Arg. #4: End angle specified as radians or degrees. [1]
      [1] add 'r' postfix to specify radians, 'd' for degrees. No postfix means radians.

      This command draws an arc starting on the current pen position.

      If the end angle is greater than the start angle, the path of the pen stroke goes
      counterclockwise, else the path goes clockwise.

      Examples:

        pa 10 10 0 360d
        -> Draws a circle with 10 mm radius (20 mm diameter) counterclockwise.

        pa 16.5 8.7 -3.14159 3.14159r
        -> Draws a 33 mm wide, 17.4 mm high ellipse clockwise.

        pa 10 15 -2.2 0.85
        -> Draws part of an ellips.

    - Command 'pg': Pen glyph.
      Arg. #1: Glyph index (0..max).
      Arg. #2: Horizontal scaling factor (floating point, max. value is 1.0).
      Arg. #3: Vertical scaling factor (floating point, max. value is 1.0).

      Draw a glyph. The base left-bottom corner of the glyph corresponds with the current pen
      position.

      Examples:

        pg 5 1 1
        -> Draws glyph 5 in its original dimensions.

        pg 8 .6 .75
        -> Draws glyph 8 with horizontal and vertical scaling factors.

    - Command 'pr': Draw a rectangle that spans a wide area of the sand.

      Example:  pr

    - Command 'pt': Draw time as in autonomous mode. This command won't acquire the
      current date and time from the realtime clock. Send 'cr' first if you want to
      draw the current time.

      Example:  pt

    - Command 'plu': Lift pen up.

      Example:  plu

    - Command 'plm': Lift pen to the middle position.

      Example:  plm

    - Command 'pld': Lift pen down i.e. plunge the pen into the sand.

      Example:  pld

    Command group 'sv': Servo motor.

    - Command 'svm': Move servo motor to position.
      Arg. #1: Motor identifier. 'l' means left, 'r' means right, 'p' means pen.

      You may omit spaces between command and motor identifier, i.e. 'svml' is the same
      as 'svm l'.

      Examples:

        svml 1600
        -> Set left servo motor to 1600 microseconds.

        svmr 1250
        -> Set right servo motor to 1250 microseconds.

        svmp 2100
        -> Set pen serbo motor to 2100 microseconds.

    - Command 'svs': Set the current position of a servo motor as a setting in RAM.
      Arg. #1: Motor identifier. 'l' means left, 'r' means right, 'p' means pen.
      Arg. #2: Setting identifier. Depends on motor identifier:
        For 'l' and 'r': 'h' means horizontal, 'v' means vertical.
        For 'p': 'u' means up, 'm' means middle, 'd' means down.

      You may omit spaces between command and sidentifier, i.e. 'svs l h' is the same
      as 'svslh'.

      All possibilities:

        svslh
        -> Set position of left servo motor as setting for horizontal position.

        svslv
        -> Set position of left servo motor as setting for vertical position.

        svsrh
        -> Set position of right servo motor as setting for horizontal position.

        svsrv
        -> Set position of right servo motor as setting for vertical position.

        svspu
        -> Set position of pen servo motor as setting for pen lifted up.

        svspm
        -> Set position of pen servo motor as setting for pen in the middle.

        svspd
        -> Set position of pen servo motor as setting for pen down.

    - Command 'svd': Dump position of servo motors.

      Example:  svd

    Command group 'vm': Vibration motor.

    - Command 'vme': Turn on (enable) vibration motor.
      Arg. #1: Period (seconds). Optional.

      Examples:

        vme
        -> Turn on the vibration motor for the duration of the setting in RAM.

        vme 0
        -> Turn on the vibration motor permanently. Turn of with command 'vmd'.

        vme 5
        -> Turn on the vibration motor for five seconsd.

    - Command 'vmd': Turn off (disable) vibration motor.

      Example:  vmd

    - Command 'vms': Set specfied vibration period as setting in RAM.

      Example:  vms 5

    Command group 'c': PCF2129A realtime clock chip.

    - Command 'cw': Set the date and time in the RTC.
      Arg. #1: Year (2000..2099).
      Arg. #2: Month (1..12).
      Arg. #3: Day (1..31).
      Arg. #4: Hour (0..23).
      Arg. #5: Minute (0..59).
      Arg. #6: Second (0..59).

      Example:  cw 2016 08 10 16 35 48

    - Command 'cr': Read status, date and time from the RTC chp.

    Command group 'm': Program mode.

    - Command 'ma': Switch to autonomous mode.

      Example:  ma

    - Command 'msa': Set program mode setting in RAM to autonomous mode.

      Example:  msa

    - Command 'msc': Set program mode setting in RAM to command mode.

      Example:  msc

    Command group 'se': Settings and EEPROM.

    - Command 'sed': Dump the settings that are currently stored in RAM. These are the settings
      currently in effect.

      Example:  sed

    - Command 'sev': Validate the active settings in RAM.

      Example:  sev

    - Command 'sew': Write the active settings into EEPROM.

      Example:  sew

    - Command 'ser': Read settings from EEPROM into RAM and apply them.

      Example:  ser

    - Command 'sec': Clear settings in EEPROM. To avoid typing mishaps, you've to specify a
      confirmation key after the command. The key is '12345'.

      Example:  sec 12345

    Other commands:

    - Command 'vb': Select verbosity of information sent over the serial interface.
      Arg. #1: 0..2. The higher the value, the more verbose the program becomes.

      Example:  vb 2

    - Command 'di': Select the drawing interval in autonomous mode.
      Arg. #1: 1, 5, 10, 15, 20, 30, 60. Draw time at multiple of this minutes.

      Example:  di 20

    - Command 'ece': Enable echoing of commands on the serial interface.

      Example:  ece

    - Command 'ecd': Disable echoing of commands on the serial interface.

      Example:  ecd

  Notes:

  * All lengths are calculated in millimeter units.

  * avr-gcc implements all floating-point arithmetic as IEEE 32-bit floats. Types float
    and double are the same.

  Tested with:
  * Arduino 1.6.12
  * Arduino 1.8.1

  Author:  Peter S'heeren, Axiris
  Date:    2017-02-25
  License: public domain

  Revision history:

    2016-08-13  Peter S'heeren, Axiris

      * Initial release.

    2017-02-25  Peter S'heeren, Axiris

      * Added drawing interval: command 'di', EEPROM setting.
      * Added command echoing: commands 'ece' and 'ecd', EEPROM setting.
      * Added support for push button for manual drawing of time.
*/


#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>


// I2C slave addresses

#define I2C_AD_PCF2129      0x51    // 0b1010001 - NXP PCF2129A


// Assignment of I/O pins

#define PIN_SERVO_LEFT    4
#define PIN_SERVO_RIGHT   5
#define PIN_SERVO_PEN     3
#define PIN_PUSH_BUTTON   6
#define PIN_VIB           A1


// Level of verbosity. The amount of information the program sends over the USB-to-serial interface.

byte   verbose_level = 1;       // 0..2


boolean   echo_commands = false;    // Echo received commands to host y/n

void  serial_println_echo_commands (byte spaces)
{
  Serial.print(F("echo commands: "));
  while (spaces) { Serial.print(' '); spaces--; }

  if (echo_commands)
    Serial.println(F("enabled"));
  else
    Serial.println(F("disabled"));
}


// Program mode of operation

#define PROG_MODE_CMD       0   // Process commands interactively
#define PROG_MODE_AUTO      1   // Autonomous
#define __PROG_MODE_CNT__   2

byte  prog_mode;                // Active program mode (assign PROG_MODE_xxx value)
byte  startup_prog_mode;        // Program mode after start-up (this variable is part of the RAM settings)


void  serial_println_startup_prog_mode (byte spaces)
{
  Serial.print(F("start-up program mode: "));
  while (spaces) { Serial.print(' '); spaces--; }

  if (startup_prog_mode == PROG_MODE_CMD)  Serial.println(F("command")); else
  if (startup_prog_mode == PROG_MODE_AUTO) Serial.println(F("autonomous"));
}


// Range of duty cycle values. Use limits as defined in Servo.h.

#define SV_MIN_US   MIN_PULSE_WIDTH
#define SV_MAX_US   MAX_PULSE_WIDTH


boolean  sv_us_is_valid (word us)
{
  return ((us >= SV_MIN_US) && (us <= SV_MAX_US));
}


Servo   sv_left;
Servo   sv_right;
Servo   sv_pen;

word    sv_left_cur_us   = 0;
word    sv_left_hor_us   = 1800;
word    sv_left_vert_us  = 1200;

word    sv_right_cur_us  = 0;
word    sv_right_hor_us  = 1200;
word    sv_right_vert_us = 1800;

word    sv_pen_cur_us    = 0;
word    sv_pen_up_us     = 2100;
word    sv_pen_mid_us    = 1800;
word    sv_pen_down_us   = 1500;


// Servo motor identifiers

typedef enum _SV_ID
{
  SV_ID_LEFT,
  SV_ID_RIGHT,
  SV_ID_PEN,
}
  SV_ID;


void  sv_left_write_us (word us)
{
  sv_left.writeMicroseconds(us);
  sv_left_cur_us = us;
}


void  sv_right_write_us (word us)
{
  sv_right.writeMicroseconds(us);
  sv_right_cur_us = us;
}


void  sv_pen_write_us (word us)
{
  sv_pen.writeMicroseconds(us);
  sv_pen_cur_us = us;
}


void  sv_write_us (byte sv_id, word us)
{
  if (sv_id == SV_ID_LEFT)  sv_left_write_us(us); else
  if (sv_id == SV_ID_RIGHT) sv_right_write_us(us); else
  if (sv_id == SV_ID_PEN)   sv_pen_write_us(us);
}


void  serial_println_sv_us (byte sv_id, char pos, word us, byte spaces)
{
  Serial.print(F("servo "));

  if (sv_id == SV_ID_LEFT)  Serial.print(F("left ")); else
  if (sv_id == SV_ID_RIGHT) Serial.print(F("right ")); else
  if (sv_id == SV_ID_PEN)   Serial.print(F("pen "));

  if (pos == 'h') Serial.print(F("horizontal")); else
  if (pos == 'v') Serial.print(F("vertical")); else
  if (pos == 'u') Serial.print(F("up")); else
  if (pos == 'm') Serial.print(F("in middle")); else
  if (pos == 'd') Serial.print(F("down"));

  Serial.print(F(": "));
  while (spaces) { Serial.print(' '); spaces--; }
  Serial.println(us);
}


// Convert radians to degrees

#define RAD_TO_DEG(rad) (((rad) * 180.0) / M_PI)


// Convert degrees to radians

#define DEG_TO_RAD(deg) (((deg) * M_PI) / 180.0)


#define SV_LEFT_MIN_RAD   DEG_TO_RAD(-20)
#define SV_LEFT_MAX_RAD   DEG_TO_RAD(135)

#define SV_RIGHT_MIN_RAD  DEG_TO_RAD(-20)
#define SV_RIGHT_MAX_RAD  DEG_TO_RAD(135)


// Position of left and right servo motor
#define pos_x1    -13.17
#define pos_y1    -30.0
#define dist_12    26.34

// Measurement of arms
#define dist_13    39.75
#define dist_24    39.75
#define dist_35    50.0
#define dist_46    57.0
#define dist_56    11.75
#define rad_564    DEG_TO_RAD(49.0)

// Position of pen
double  pos_x6  =   0.0;
double  pos_y6  =  30.0;

// Angles theta1 and theta2
double  rad_t1;
double  rad_t2;


// Glyph codes
//
// Each glyph fits in a rectangular area. The starting pen position is located in the bottom-left
// corner of the rectangle. The dimensions in the comments are true for scaling factor one in both
// directions.

#define PEN_GLYPH_0           0     // 20 mm x 30 mm
#define PEN_GLYPH_1           1     // 20 mm x 30 mm
#define PEN_GLYPH_2           2     // 20 mm x 30 mm
#define PEN_GLYPH_3           3     // 20 mm x 30 mm
#define PEN_GLYPH_4           4     // 20 mm x 30 mm
#define PEN_GLYPH_5           5     // 20 mm x 30 mm
#define PEN_GLYPH_6           6     // 20 mm x 30 mm
#define PEN_GLYPH_7           7     // 20 mm x 30 mm
#define PEN_GLYPH_8           8     // 20 mm x 30 mm
#define PEN_GLYPH_9           9     // 20 mm x 30 mm
#define PEN_GLYPH_COLON       10    //  4 mm x 22 mm
#define PEN_GLYPH_RECT        11    // 20 mm x 30 mm
#define __PEN_GLYPH_CNT__     12


// A pen stroke is the movement of the pen in the horizontal plane (XY plane) during drawing.
//
// The maximum pen stroke defines the precision of drawing. Most drawing functions apply this value
// exactly. However, the arc drawing function ensure the pen stroke is less than or equal to this
// value since it's not easy to draw exact strokes in the case of an arc.
//
// The delay prevents the pen from moving too quickly in the sand.

#define PEN_STROKE_MM         0.25  // Maximum pen stroke (millimeters) applied in drawing routines
#define PEN_STROKE_DELAY_MS   5     // Delay (milliseconds) in-between pen strokes


// Pen lift movement occurs in the Z plane.

#define PEN_LIFT_STEP_US      2
#define PEN_LIFT_DELAY_MS     2


void  serial_print_pen_pos ()
{
  Serial.print(F("pos: "));
  Serial.print(pos_x6);
  Serial.print(',');
  Serial.print(pos_y6);
}


void  serial_println_pen_pos ()
{
  serial_print_pen_pos();
  Serial.println();
}


boolean  pen_calc ()
{
  double  dx;
  double  dy;
  double  dist_26;
  double  rad_a2;
  double  rad_b2;
  double  rad_a6;
  double  rad_a5;
  double  x5;
  double  y5;
  double  dist_15;
  double  rad_a1;
  double  rad_b1;


  // Right-side triangle P2-P4-P6

  dx = pos_x6 - (pos_x1 + dist_12);
  dy = pos_y6 - pos_y1;

  rad_a2 = atan2(dy, dx);

  // Distance between pen and right servo motor
  dist_26 = sqrt(dx * dx + dy * dy);

  rad_b2 = acos
           (
             (dist_26 * dist_26 + dist_24 * dist_24 - dist_46 * dist_46) /
             (2 * dist_26 * dist_24)
           );

  rad_t2 = rad_a2 - rad_b2;

  if (!isfinite(rad_t2)) return false;
  if (rad_t2 < SV_LEFT_MIN_RAD) return false;
  if (rad_t2 > SV_LEFT_MAX_RAD) return false;


  rad_a6 = acos
           (
             (dist_26 * dist_26 + dist_46 * dist_46 - dist_24 * dist_24) /
             (2 * dist_26 * dist_46)
           );

  rad_a5 = rad_a2 + rad_a6 + M_PI - rad_564;

  x5 = pos_x6 + dist_56 * cos(rad_a5);
  y5 = pos_y6 + dist_56 * sin(rad_a5);


  // Left-side triangle P1-P3-P5

  dx = x5 - pos_x1;
  dy = y5 - pos_y1;

  rad_a1 = atan2(dy, dx);

  // Distance between P5 and left servo motor
  dist_15 = sqrt(dx * dx + dy * dy);

  rad_b1 = acos
           (
             (dist_15 * dist_15 + dist_13 * dist_13 - dist_35 * dist_35) /
             (2 * dist_15 * dist_13)
           );

  rad_t1 = M_PI - rad_b1 - rad_a1;

  if (!isfinite(rad_t1)) return false;
  if (rad_t1 < SV_RIGHT_MIN_RAD) return false;
  if (rad_t1 > SV_RIGHT_MAX_RAD) return false;


  return true;
}


boolean  pen_set (double x, double y)
{
  boolean   ok;
  word      delta;
  double    v;
  word      sv_left_us;
  word      sv_right_us;
  boolean   changed;

  pos_x6 = x;
  pos_y6 = y;
  ok = pen_calc();

  if (verbose_level == 2)
  {
    serial_print_pen_pos();
    Serial.print(F(" -> "));
    Serial.print(RAD_TO_DEG(rad_t1));
    Serial.print(F(" deg, "));
    Serial.print(RAD_TO_DEG(rad_t2));
    Serial.print(F(" deg "));
    if (!ok) Serial.println(F(" -> error!"));
  }
  else
  if (verbose_level == 1)
  {
    if (!ok) Serial.print(F("[E]"));
  }

  if (!ok) return false;

  delta = sv_left_hor_us - sv_left_vert_us;
  v = (rad_t1 * delta) / M_PI_2;
  sv_left_us = sv_left_hor_us - (word)v;

  delta = sv_right_vert_us - sv_right_hor_us;
  v = (rad_t2 * delta) / M_PI_2;
  sv_right_us = sv_right_hor_us + (word)v;

  if (verbose_level == 2)
  {
    Serial.print('(');
    Serial.print(sv_left_us);
    Serial.print(F(" us, "));
    Serial.print(sv_right_us);
    Serial.print(F(" us)"));
    Serial.println();
  }

  changed = false;

  if (sv_left_us != sv_left_cur_us)
  {
    sv_left_write_us(sv_left_us);
    changed = true;
  }

  if (sv_right_us != sv_right_cur_us)
  {
    sv_right_write_us(sv_right_us);
    changed = true;
  }

  if (changed)
    if (verbose_level <= 1)
      if (PEN_STROKE_DELAY_MS > 0) delay(PEN_STROKE_DELAY_MS);

  return true;
}


boolean  pen_set_rel (double dx, double dy)
{
  return pen_set(pos_x6 + dx,pos_y6 + dy);
}


// Set the pen in a resting position.

void  pen_set_resting ()
{
  pen_set(0.0,30.0);
}


void  pen_line_to (double x, double y)
{
  double  dx;
  double  dy;
  double  dist;
  double  start_x;
  double  start_y;
  double  n;

  dx = x - pos_x6;
  dy = y - pos_y6;

  // Distance the pen has to travel
  dist = sqrt(dx * dx + dy * dy);

  start_x = pos_x6;
  start_y = pos_y6;
  n = 0;
  for (;;)
  {
    n += PEN_STROKE_MM;
    if (n >= dist) break;

    pen_set
    (
      start_x + (n * dx) / dist,
      start_y + (n * dy) / dist
    );
  }

  pen_set(x,y);
}


void  pen_line_rel_to (double dx, double dy)
{
  pen_line_to(pos_x6 + dx,pos_y6 + dy);
}


boolean  pen_arc_to (double hradius, double vradius, double a1, double a2)
{
  boolean   neg;
  double    cx;
  double    cy;
  double    incr;

  if (hradius <= 0.0) return false;
  if (vradius <= 0.0) return false;

  // Calculate pen stroke increment.
  //
  // When working with a circle, the length of the arc is proportional to the size of the angle. When the
  // specified radius are not equal, the arc is elliptical. In the latter case, we use the smallest radius
  // for calculating the increment.
  incr = PEN_STROKE_MM / (vradius < hradius ? vradius : hradius);

  // If the increment is zero, the movement of the pen is neglectible and the function returns successfully.
  if (incr == 0.0) return true;

  // Direction:
  // * Positive: incrementing angle, a.k.a. counter clockwise.
  // * Negative: decrementing angle, a.k.a. clockwise.
  neg = (a1 > a2) ? true : false;

  // Center point
  cx = pos_x6 - hradius * cos(a1);
  cy = pos_y6 - vradius * sin(a1);

  for (;;)
  {
    if (neg)
    {
      a1 -= incr;
      if (a1 <= a2) break;
    }
    else
    {
      a1 += incr;
      if (a1 >= a2) break;
    }

    pen_line_to
    (
      cx + hradius * cos(a1),
      cy + vradius * sin(a1)
    );
  }

  pen_line_to
  (
    cx + hradius * cos(a2),
    cy + vradius * sin(a2)
  );

  return true;
}


void  pen_lift_to (word dest_us)
{
  boolean   neg;
  word      cur_us;

  // Direction:
  // * Positive: incrementing PWM duty cycle.
  // * Negative: decrementing PWM duty cycle.
  neg = (dest_us < sv_pen_cur_us) ? true : false;

  cur_us = sv_pen_cur_us;
  for (;;)
  {
    if (neg)
    {
      cur_us -= PEN_LIFT_STEP_US;
      if (cur_us <= dest_us) break;
    }
    else
    {
      cur_us += PEN_LIFT_STEP_US;
      if (cur_us >= dest_us) break;
    }

    sv_pen_write_us(cur_us);
    if (PEN_LIFT_DELAY_MS > 0) delay(PEN_LIFT_DELAY_MS);
  }

  sv_pen_write_us(dest_us);
  if (PEN_LIFT_DELAY_MS > 0) delay(PEN_LIFT_DELAY_MS);
}


void  pen_lift_up ()
{
  pen_lift_to(sv_pen_up_us);
}


void  pen_lift_mid ()
{
  pen_lift_to(sv_pen_mid_us);
}


void  pen_lift_down ()
{
  pen_lift_to(sv_pen_down_us);
}


void  pen_draw_glyph (byte glyph, double sx, double sy)
{
  double  x;
  double  y;

  // Don't accept scaling factors greater than zero, we only want to shrink glyphs, not
  // blow them up.
  if (sx > 1.0) return;
  if (sy > 1.0) return;

  // Get the pen out of the sand, if needed
  pen_lift_mid();

  x = pos_x6;
  y = pos_y6;

  switch (glyph)
  {
    case PEN_GLYPH_0:
    {
      // Move pen into position
      pen_set(x+10.0*sx,y);

      pen_lift_down();

      pen_arc_to(10.0*sx,15.0*sy,-M_PI_2,M_PI+M_PI_2);

      break;
    }

    case PEN_GLYPH_1:
    {
      pen_set(x,y);

      pen_lift_down();

      pen_line_rel_to(20.0*sx,0.0);

      pen_lift_mid();

      pen_set(x+10.0*sx,y);

      pen_lift_down();

      pen_line_rel_to(0.0,30.0*sy);

      pen_arc_to(40.0*sx,40.0*sy,DEG_TO_RAD(-20),DEG_TO_RAD(-45));

      break;
    }

    case PEN_GLYPH_2:
    {
      pen_set(x,y+20.0*sy);

      pen_lift_down();

      pen_arc_to(10.0*sx,10.0*sy,M_PI,-M_PI_4);

      pen_line_to(x,y);

      pen_line_rel_to(20.0*sx,0.0);

      break;
    }

    case PEN_GLYPH_3:
    {
      pen_set(x,y);

      pen_lift_down();

      pen_line_rel_to(10.0*sx,0.0);

      pen_arc_to(10.0*sx,7.5*sy,-M_PI_2,M_PI_2);

      pen_arc_to(10.0*sx,7.5*sy,-M_PI_2,M_PI_2);

      pen_line_rel_to(-10.0*sx,0.0*sy);

      break;
    }

    case PEN_GLYPH_4:
    {
      pen_set(x+16.0*sx,y);

      pen_lift_down();

      pen_line_rel_to(0.0,25.0*sy);

      pen_lift_mid();

      pen_set(x+20.0*sx,y+15.0*sy);

      pen_lift_down();

      pen_line_rel_to(-20.0*sx,0.0);

      pen_arc_to(15.0*sx,30.0*sy,-M_PI_4,DEG_TO_RAD(-12));

      break;
    }

    case PEN_GLYPH_5:
    {
      pen_set(x,y);

      pen_lift_down();

      pen_line_rel_to(10.0*sx,0.0);

      pen_arc_to(10.0*sx,7.5*sy,-M_PI_2,M_PI_2);

      pen_line_rel_to(-10.0*sx,0.0);

      pen_line_rel_to(0.0,15.0*sy);

      pen_line_rel_to(20.0*sx,0.0);

      break;
    }

    case PEN_GLYPH_6:
    {
      pen_set(x,y+7.5*sy);

      pen_lift_down();

      pen_arc_to(10.0*sx,7.5*sy,M_PI,-M_PI);

      pen_arc_to(30.4*sx,23.95*sy,M_PI,DEG_TO_RAD(180-70));

      break;
    }

    case PEN_GLYPH_7:
    {
      pen_set(x+10*sx,y);

      pen_lift_down();

      pen_line_rel_to(10.0*sx,30.0*sy);

      pen_line_rel_to(-20.0*sx,0.0);

      break;
    }

    case PEN_GLYPH_8:
    {
      pen_set(x+10.0,y);

      pen_lift_down();

      pen_arc_to(10.0*sx,7.5*sy,-M_PI_2,M_PI_2);

      pen_arc_to(10.0*sx,7.5*sy,M_PI+M_PI_2,-M_PI_2);

      pen_arc_to(10.0*sx,7.5*sy,M_PI_2,M_PI+M_PI_2);

      break;
    }

    case PEN_GLYPH_9:
    {
      pen_set(x+20.0*sx,y+22.5*sy);

      pen_lift_down();

      pen_arc_to(10.0*sx,7.5*sy,2*M_PI,0);

      pen_arc_to(30.4*sx,23.95*sy,0,DEG_TO_RAD(-70));

      break;
    }

    case PEN_GLYPH_COLON:
    {
      pen_set(x,y+10.0*sy);

      pen_lift_down();

      pen_arc_to(2.0*sx,2.0*sy,-M_PI,M_PI);

      pen_lift_mid();

      pen_set(x,y+20.0*sy);

      pen_lift_down();

      pen_arc_to(2.0*sx,2.0*sy,-M_PI,M_PI);

      break;
    }

    case PEN_GLYPH_RECT:
    {
      pen_set(x,y);

      pen_lift_down();

      pen_line_rel_to(20*sx,0.0);
      pen_line_rel_to(0.0,30*sx);
      pen_line_rel_to(-20*sx,0.0);
      pen_line_rel_to(0.0,-30*sx);

      break;
    }
  }

  // Get the pen out of the sand
  pen_lift_mid();
}


// Draw reference rectangle

void  pen_draw_ref_rect ()
{
  pen_lift_up();
  pen_set(pos_x1+dist_12/2-40.0,pos_y1+50.0);
  pen_lift_down();
  pen_line_rel_to(80.0,0.0);
  pen_line_rel_to(0.0,30.0);
  pen_line_rel_to(-80.0,0.0);
  pen_line_rel_to(0.0,-30.0);
  pen_lift_up();
}


// Time to turn on vibration motor (seconds)

#define VIB_PERIOD_MIN_S     1
#define VIB_PERIOD_DEF_S     5
#define VIB_PERIOD_MAX_S    20

byte  vib_period_s  = VIB_PERIOD_DEF_S;


// Turn on vibration motor

void  vib_turn_on ()
{
  digitalWrite(PIN_VIB,HIGH);
}


// Turn off vibration motor

void  vib_turn_off ()
{
  digitalWrite(PIN_VIB,LOW);
}


void  serial_println_vib_period_s (byte spaces)
{
  Serial.print(F("vibration period: "));
  while (spaces) { Serial.print(' '); spaces--; }
  Serial.println(vib_period_s);
}


#define DRAWING_INTERVAL_1MIN       0   // Every minute
#define DRAWING_INTERVAL_5MIN       1   // On multiples of 5 minutes
#define DRAWING_INTERVAL_10MIN      2   // On multiples of 10 minutes
#define DRAWING_INTERVAL_15MIN      3   // On multiples of 15 minutes
#define DRAWING_INTERVAL_20MIN      4   // On multiples of 20 minutes
#define DRAWING_INTERVAL_30MIN      5   // On multiples of 30 minutes
#define DRAWING_INTERVAL_1HOUR      6   // Every hour
#define __DRAWING_INTERVAL_CNT__    7

byte  drawing_interval  = DRAWING_INTERVAL_1MIN;


void  serial_println_drawing_interval (byte spaces)
{
  Serial.print(F("drawing interval: "));
  while (spaces) { Serial.print(' '); spaces--; }
  switch (drawing_interval)
  {
    case DRAWING_INTERVAL_1MIN:  Serial.println(F("1 min")); break;
    case DRAWING_INTERVAL_5MIN:  Serial.println(F("5 min")); break;
    case DRAWING_INTERVAL_10MIN: Serial.println(F("10 min")); break;
    case DRAWING_INTERVAL_15MIN: Serial.println(F("15 min")); break;
    case DRAWING_INTERVAL_20MIN: Serial.println(F("20 min")); break;
    case DRAWING_INTERVAL_30MIN: Serial.println(F("30 min")); break;
    case DRAWING_INTERVAL_1HOUR: Serial.println(F("1 hour")); break;
  }
}


// RTC date and time

typedef struct _PCF2129_DT
{
  byte        year;           // 0..99 (meaning 2000..2099)
  byte        month;          // 1..12
  byte        day;            // 1..31
  byte        hour;           // 0..23
  byte        minute;         // 0..59
  byte        second;         // 0..59
}
  PCF2129_DT;


byte  pbcd_to_bin (byte pbcd)
{
  return (((pbcd >> 4) * 10) + (pbcd & 15));
}


byte  bin_to_pbcd (byte bin)
{
  return (bin % 10) | ((bin / 10) << 4);
}


// PCF2129 state

typedef struct _PCF2129_STATE
{
  PCF2129_DT  dt;             // Date & time; valid if power_up is false
  boolean     power_up;       // Power-up detected y/n
  boolean     battery_low;    // Battery low y/n
}
  PCF2129_STATE;


const  byte  pcf2129_init_data[] PROGMEM =
{
  // Start register index
  0,

  // Register 00h - Control_1:
  // * Bit 7: EXT_TEST - external clock test mode
  // * Bit 6: Unused
  // * Bit 5: STOP - stop RTC clock y/n
  // * Bit 4: TSF1 - timestamp 1 interrupt generated y/n (write zero to clear)
  // * Bit 3: POR_OVRD - Power-On Reset Override (PORO) facility disabled; set
  //          logic 0 for normal operation
  // * Bit 2: 12_24 - 24 hour mode selected (0) or 12 hour mode selected (1)
  // * Bit 1: MI - minute interrupt enabled y/n
  // * Bit 0: SI - second interrupt enabled y/n
  // -> 0b00000000
  0x00,

  // Register 01h - Control_2:
  // * Bit 7: MSF - minute or second interrupt generated (write zero to clear)
  // * Bit 6: WDTF - watchdog timer interrupt or reset generated
  // * Bit 5: TSF2 - timestamp 2 interrupt generated y/n (write zero to clear)
  // * Bit 4: AF - alarm interrupt generated (write zero to clear)
  // * Bit 3: Unused
  // * Bit 2: TSIE - Timestamp interrupt enable
  // * Bit 1: AIE - Alarm interrupt enable
  // * Bit 0: Unused
  // -> 0b00000000
  0x00,

  // Register 02h - Control_3:
  // * Bit 7..5: PWRMNG[2..0] - power management setting
  // * Bit    4: BTSE - enable timestamp when battery switch-over y/n
  // * Bit    3: BF - set when battery switch-over occurs (write zero to clear)
  // * Bit    2: BLF - Battery low y/n
  // * Bit    1: BIE - interrupt generated when BF is set
  // * Bit    0: BLIE - interrupt generated when BLF is set
  // -> 0b00000000
  0x00
};


byte            pcf2129_poll_buf[10];
byte            pcf2129_dt_data[8];
PCF2129_STATE   pcf2129_info;


boolean  pcf2129_poll_main ()
{
  // Write start register
  Wire.beginTransmission(I2C_AD_PCF2129);
  Wire.write(byte(0x00));
  Wire.endTransmission();

  // Read registers 00h..09h
  Wire.requestFrom(I2C_AD_PCF2129, sizeof(pcf2129_poll_buf));
  if (Wire.available() < sizeof(pcf2129_poll_buf)) return false;
  for (byte u = 0; u  < sizeof(pcf2129_poll_buf); u++) pcf2129_poll_buf[u] = Wire.read();

  // Control_3 register, bit 2: BLF
  pcf2129_info.battery_low = (pcf2129_poll_buf[2] & 0x04) ? 1 : 0;

  // Check the OSF flag
  if (pcf2129_poll_buf[3] & 0x80)
  {
    // Mark power-up detected
    pcf2129_info.power_up = 1;

    // Resulting date and time - clear
    //memset(&pcf2129_info.dt,0,sizeof(pcf2129_info.dt));

    // Write the initialization data. The data block includes the start
    // register index value.
    Wire.beginTransmission(I2C_AD_PCF2129);
    for (byte u = 0; u  < sizeof(pcf2129_init_data); u++)
      Wire.write(pgm_read_byte_near(pcf2129_init_data + u));
    Wire.endTransmission();
  }
  else
  {
    // Power-up not detected
    pcf2129_info.power_up = 0;

    // Date and time
    pcf2129_info.dt.second = pbcd_to_bin(pcf2129_poll_buf[3] & 0x7F);
    pcf2129_info.dt.minute = pbcd_to_bin(pcf2129_poll_buf[4] & 0x7F);
    pcf2129_info.dt.hour   = pbcd_to_bin(pcf2129_poll_buf[5] & 0x3F);
    pcf2129_info.dt.day    = pbcd_to_bin(pcf2129_poll_buf[6] & 0x3F);
    pcf2129_info.dt.month  = pbcd_to_bin(pcf2129_poll_buf[8] & 0x1F);
    pcf2129_info.dt.year   = pbcd_to_bin(pcf2129_poll_buf[9]);
  }

  return true;
}


boolean  pcf2129_poll ()
{
  if (!pcf2129_poll_main())
  {
    Serial.println(F("Can't access PCF2129!"));
    return false;
  }

  return true;
}


boolean  pcf2129_set_dt ()
{
  byte  max_day;

  if (pcf2129_info.dt.month == 2)
  {
    // February
    max_day = ((pcf2129_info.dt.year & 3) == 0) ? 29 : 28;
  }
  else if ((pcf2129_info.dt.month >= 1) & (pcf2129_info.dt.month <= 7))
  {
    // January..July
    max_day = (pcf2129_info.dt.month & 1) ? 31 : 30;
  }
  else if (pcf2129_info.dt.month <= 12)
  {
    // August..December
    max_day = (pcf2129_info.dt.month & 1) ? 30 : 31;
  }
  else
  {
    // Invalid month
    return 0;
  }

  if ((pcf2129_info.dt.day < 1) || (pcf2129_info.dt.day > max_day)) return 0;
  if (pcf2129_info.dt.year > 99) return 0;
  if (pcf2129_info.dt.hour > 23) return 0;
  if (pcf2129_info.dt.minute > 59) return 0;
  if (pcf2129_info.dt.second > 59) return 0;

  pcf2129_dt_data[0] = 3;                                     // Start register index
  pcf2129_dt_data[1] = bin_to_pbcd(pcf2129_info.dt.second);   // Register 03h
  pcf2129_dt_data[2] = bin_to_pbcd(pcf2129_info.dt.minute);   // Register 04h
  pcf2129_dt_data[3] = bin_to_pbcd(pcf2129_info.dt.hour);     // Register 05h
  pcf2129_dt_data[4] = bin_to_pbcd(pcf2129_info.dt.day);      // Register 06h
  pcf2129_dt_data[5] = 0;                                     // Register 07h: Day of the week
  pcf2129_dt_data[6] = bin_to_pbcd(pcf2129_info.dt.month);    // Register 08h
  pcf2129_dt_data[7] = bin_to_pbcd(pcf2129_info.dt.year);     // Register 09h

  // Write the date & time data. The data block includes the start register index value.
  Wire.beginTransmission(I2C_AD_PCF2129);
  for (byte u = 0; u  < sizeof(pcf2129_dt_data); u++) Wire.write(pcf2129_dt_data[u]);
  Wire.endTransmission();

  return true;
}


// Confirmation key for clearing settings in EEPROM. See command "sec".

#define CLEAR_SETTINGS_KEY    12345


// Data structure of settings as stored in EEPROM.
//
// The structure is backwards ompatible with previous version(s) of the sketch.

typedef struct _SETTINGS
{
  // Settings from original sketch 2016-08-13

  byte      skip;
  byte      startup_prog_mode;
  word      sv_left_hor_us;
  word      sv_left_vert_us;
  word      sv_right_hor_us;
  word      sv_right_vert_us;
  word      sv_pen_down_us;
  word      sv_pen_mid_us;
  word      sv_pen_up_us;
  byte      vib_period_s;

  // Settings added in sketch 2017-02-25

  byte      rev;                // Revision of subsequent data
                                // Note: equals to FFh if not present

  // rev=0
  byte      drawing_interval;   // Drawing interval  (assign DRAWING_INTERVAL_xxx value)
  boolean   echo_commands;      // Echo commands y/n (true or false)
}
  SETTINGS;


// EEPROM address of the settings data.

#define SETTINGS_EEPROM_AD    0x0000


// Buffer for transferring data between EEPROM and RAM.

SETTINGS  settings;


// This function validates the contents of the settings structure. The function can
// validate both EEPROM data and RAM data:
// (a) The program reads data from the EEPROM into the settings structure.
// (b) The program stores RAM values into the settings structure.

boolean  settings_validate_main ()
{
  if (settings.startup_prog_mode >= __PROG_MODE_CNT__) return false;

  if (!sv_us_is_valid(settings.sv_left_hor_us)) return false;
  if (!sv_us_is_valid(settings.sv_left_vert_us)) return false;
  if (!(settings.sv_left_hor_us > settings.sv_left_vert_us)) return false;

  if (!sv_us_is_valid(settings.sv_right_hor_us)) return false;
  if (!sv_us_is_valid(settings.sv_right_vert_us)) return false;
  if (!(settings.sv_right_hor_us < settings.sv_right_vert_us)) return false;

  if (!sv_us_is_valid(settings.sv_pen_down_us)) return false;
  if (!sv_us_is_valid(settings.sv_pen_mid_us)) return false;
  if (!sv_us_is_valid(settings.sv_pen_up_us)) return false;
  if (!(settings.sv_pen_down_us < settings.sv_pen_mid_us)) return false;
  if (!(settings.sv_pen_mid_us < settings.sv_pen_up_us)) return false;

  if (settings.vib_period_s < VIB_PERIOD_MIN_S) return false;
  if (settings.vib_period_s > VIB_PERIOD_MAX_S) return false;

  if (settings.rev != 0xFF)
  {
    // Revision 0 or higher
    if (!(settings.drawing_interval < __DRAWING_INTERVAL_CNT__)) return false;
    if (!((settings.echo_commands == false) || (settings.echo_commands == true))) return false;
  }

  // The settings are valid
  return true;
}


boolean  settings_validate (boolean in_eeprom)
{
  if (!settings_validate_main())
  {
    if (in_eeprom)
      Serial.println(F("No settings found in EEPROM"));
    else
      Serial.println(F("Invalid settings in RAM, modify settings"));

    return false;
  }

  return true;
}


void  settings_write_eeprom ()
{
  // Update changed bytes in EEPROM
  for (byte u = 0; u < sizeof(settings); u++) EEPROM.write(SETTINGS_EEPROM_AD+u,((byte*)&settings)[u]);

  // Same thing, but not available in all versions of Arduino:
  //EEPROM.put(SETTINGS_EEPROM_AD,settings);
}


void  settings_read_eeprom ()
{
  // Read bytes from EEPROM into settings structure
  for (byte u = 0; u < sizeof(settings); u++) ((byte*)&settings)[u] = EEPROM.read(SETTINGS_EEPROM_AD+u);

  // Same thing, but not available in all versions of Arduino:
  //EEPROM.get(SETTINGS_EEPROM_AD,settings);
}


void  settings_clear_eeprom ()
{
  memset(&settings,0xFF,sizeof(settings));
  settings_write_eeprom();
}


void  settings_set_up ()
{
  settings.skip              = 0xFF;
  settings.startup_prog_mode = startup_prog_mode;
  settings.sv_left_hor_us    = sv_left_hor_us;
  settings.sv_left_vert_us   = sv_left_vert_us;
  settings.sv_right_hor_us   = sv_right_hor_us;
  settings.sv_right_vert_us  = sv_right_vert_us;
  settings.sv_pen_down_us    = sv_pen_down_us;
  settings.sv_pen_mid_us     = sv_pen_mid_us;
  settings.sv_pen_up_us      = sv_pen_up_us;
  settings.vib_period_s      = vib_period_s;
  settings.rev               = 0;
  settings.drawing_interval  = drawing_interval;
  settings.echo_commands     = echo_commands;
}


void  settings_accept ()
{
  // Update structure to the latest revision
  if (settings.rev == 0xFF)
  {
    // Settings are pre revision zero
    settings.drawing_interval = drawing_interval;
    settings.echo_commands    = echo_commands;
  }

  settings.rev = 0;

  startup_prog_mode = settings.startup_prog_mode;
  sv_left_hor_us    = settings.sv_left_hor_us;
  sv_left_vert_us   = settings.sv_left_vert_us;
  sv_right_hor_us   = settings.sv_right_hor_us;
  sv_right_vert_us  = settings.sv_right_vert_us;
  sv_pen_down_us    = settings.sv_pen_down_us;
  sv_pen_mid_us     = settings.sv_pen_mid_us;
  sv_pen_up_us      = settings.sv_pen_up_us;
  vib_period_s      = settings.vib_period_s;
  drawing_interval  = settings.drawing_interval;
  echo_commands     = settings.echo_commands;
}


void  set_prog_mode_cmd ()
{
  prog_mode = PROG_MODE_CMD;

  vib_turn_off();

  Serial.println(F("Enter commands"));
}


void  serial_print_2bcd (byte u)
{
  Serial.print(byte(u / 10));
  Serial.print(byte(u % 10));
}


#define RCV_BUF_LEN   32

char      rcv_buf[RCV_BUF_LEN];  // Receive buffer
byte      rcv_si    = 0;         // Store index
boolean   rcv_error = false;     // Receive error condition
byte      rcv_fi    = 0;         // Fetch index
char      rcv_fetch = 0;         // Last fetched character


void  echo_rcv_buf ()
{
  byte  u;

  for (u = 0; u < rcv_si; u++) Serial.print(rcv_buf[u]);
  Serial.println();
}


void  keep_rcv_char ()
{
  if (rcv_fetch != 0) rcv_fi--;
}


char  fetch_rcv_char ()
{
  if (rcv_fi == rcv_si)
  {
    // Reached the end of the buffer
    rcv_fetch = 0;
  }
  else
  {
    // Fetch the next character
    rcv_fetch = rcv_buf[rcv_fi++];
  }

  return rcv_fetch;
}


word  parse_u16_res;


boolean  parse_dec_u16 ()
{
  word    prev;
  char    c;
  boolean parsed;

  parsed        = false;
  parse_u16_res = 0;

  for (;;)
  {
    c = fetch_rcv_char();
    if ((c >= '0') && (c <= '9'))
    {
      prev = parse_u16_res;

      parse_u16_res = parse_u16_res * 10 + (c - '0');

      // Check for overflow
      if (parse_u16_res < prev) return false;

      // At least one digit has been parsed
      parsed = true;
    }
    else
    {
      keep_rcv_char();

      return parsed;
    }
  }
}


boolean  parse_dec_u16_range (word min, word max)
{
  if (!parse_dec_u16()) return false;
  if (parse_u16_res < min) return false;
  if (parse_u16_res > max) return false;
  return true;
}


int  parse_s16_res;


boolean  parse_dec_s16 ()
{
  char      c;
  boolean   minus;

  c = fetch_rcv_char();
  minus = false;
  if (c == '-') minus = true; else if (c != '+') keep_rcv_char();

  if (!parse_dec_u16()) return false;
  if (minus)
  {
    if (parse_u16_res > 32768) return false;
    parse_s16_res = -parse_u16_res;
  }
  else
  {
    if (parse_u16_res >= 32768) return false;
    parse_s16_res = parse_u16_res;
  }

  return true;
}


double  parse_double_res;


boolean  parse_dec_double ()
{
  boolean   parsed;
  char      c;
  boolean   minus;    // Minus sign y/n
  long      m;        // Mantissa
  long      prev_m;
  byte      e;        // Exponent
  boolean   dp;       // Decimal point parsed y/n

  c = fetch_rcv_char();
  minus = false;
  if (c == '-') minus = true; else if (c != '+') keep_rcv_char();

  parsed = false;
  m      = 0;
  e      = 0;
  dp     = false;

  for (;;)
  {
    c = fetch_rcv_char();
    if ((c >= '0') && (c <= '9'))
    {
      prev_m = m;

      m = m * 10 + (c - '0');

      // Check for overflow
      if (m < prev_m) return false;

      // At least one digit has been parsed
      parsed = true;

      if (dp) e++;
    }
    else if (c == '.')
    {
      if (dp) return false;
      dp = true;
    }
    else
    {
      double  res;

      keep_rcv_char();

      if (!parsed) return false;

      res = m;
      while (e) {
        res /= 10;
        e--;
      }
      if (minus) res = -res;

      parse_double_res = res;
      return true;
    }
  }
}


void  parse_spaces ()
{
  char  c;

  for (;;)
  {
    c = fetch_rcv_char();
    if (c != ' ') break;
  }

  keep_rcv_char();
}


boolean  parse_eol ()
{
  char    c;

  // End-of-line expected
  parse_spaces();
  c = fetch_rcv_char();
  if (c != 0) return false;
  return true;
}


boolean  parse_opt_eol ()
{
  if (parse_eol()) return true;
  keep_rcv_char();
  return false;
}


char  parse_r_d_res;


boolean  parse_r_d ()
{
  char  c;

  c = fetch_rcv_char();
  if ((c != 'r') && (c != 'd'))
  {
    // Default is radians
    c = 'r';
    keep_rcv_char();
  }

  parse_r_d_res = c;
  return true;
}


byte  parse_sv_id_res;

boolean  parse_sv_id ()
{
  char  c;

  c = fetch_rcv_char();
  if (c == 'l') parse_sv_id_res = SV_ID_LEFT; else
  if (c == 'r') parse_sv_id_res = SV_ID_RIGHT; else
  if (c == 'p') parse_sv_id_res = SV_ID_PEN; else
    return false;

  return true;
}


boolean  process_rcv ()
{
  char    c;

  // Parse separator space characters
  parse_spaces();

  c = fetch_rcv_char();
  if (c == 0)
  {
    // Empty line received
    return true;
  }

  if (c == 's')
  {
    c = fetch_rcv_char();
    if (c == 'e')
    {
      c = fetch_rcv_char();
      if (c == 'd')
      {
        // End-of-line expected
        if (!parse_eol()) return false;

        // Dump
        serial_println_startup_prog_mode(1);
        serial_println_sv_us(SV_ID_LEFT, 'h',sv_left_hor_us,   1);
        serial_println_sv_us(SV_ID_LEFT, 'v',sv_left_vert_us,  3);
        serial_println_sv_us(SV_ID_RIGHT,'h',sv_right_hor_us,  0);
        serial_println_sv_us(SV_ID_RIGHT,'v',sv_right_vert_us, 2);
        serial_println_sv_us(SV_ID_PEN,  'u',sv_pen_up_us,    10);
        serial_println_sv_us(SV_ID_PEN,  'm',sv_pen_mid_us,    3);
        serial_println_sv_us(SV_ID_PEN,  'd',sv_pen_down_us,   8);
        serial_println_vib_period_s(6);
        serial_println_drawing_interval(6);
        serial_println_echo_commands(9);

        return true;
      }
      else
      if (c == 'v')
      {
        // End-of-line expected
        if (!parse_eol()) return false;

        // Set up settings data
        settings_set_up();

        // Check validity of settings
        if (!settings_validate(false)) return true;

        Serial.println(F("Settings in RAM are valid"));

        return true;
      }
      else
      if (c == 'w')
      {
        // End-of-line expected
        if (!parse_eol()) return false;

        // Set up settings data
        settings_set_up();

        // Check validity of settings
        if (!settings_validate(false)) return true;

        // Write settings to EEPROM
        settings_write_eeprom();

        return true;
      }
      else
      if (c == 'r')
      {
        // End-of-line expected
        if (!parse_eol()) return false;

        // Read settings from EEPROM
        settings_read_eeprom();

        // Check validity of settings
        if (!settings_validate(true)) return true;

        // Accept settings
        settings_accept();

        return true;
      }
      else
      if (c == 'c')
      {
        // Confirmation key
        parse_spaces();
        if (!parse_dec_u16()) return false;
        if (parse_u16_res != CLEAR_SETTINGS_KEY) return false;
 
        // End-of-line expected
        if (!parse_eol()) return false;

        // Clear settings in EEPROM
        settings_clear_eeprom();

        return true;
      }
    }
    else
    if (c == 'v')
    {
      c = fetch_rcv_char();
      if (c == 'm')
      {
        // Servo identifier (one character)
        parse_spaces();
        if (!parse_sv_id()) return false;

        // PWM duty cycle a.k.a. ON time (microseconds)
        parse_spaces();
        if (!parse_dec_u16()) return false;

        // End-of-line expected
        if (!parse_eol()) return false;

        // Write the PWM duty cycle
        sv_write_us(parse_sv_id_res,parse_u16_res);

        return true;
      }
      else
      if (c == 's')
      {
        word   *src;
        word   *dest;

        // Servo identifier (one character)
        parse_spaces();
        if (!parse_sv_id()) return false;

        parse_spaces();
        c = fetch_rcv_char();
        if (parse_sv_id_res == SV_ID_LEFT)
        {
          src = &sv_left_cur_us;
          if (c == 'h') dest = &sv_left_hor_us; else
          if (c == 'v') dest = &sv_left_vert_us; else
            return false;
        }
        else
        if (parse_sv_id_res == SV_ID_RIGHT)
        {
          src = &sv_right_cur_us;
          if (c == 'h') dest = &sv_right_hor_us; else
          if (c == 'v') dest = &sv_right_vert_us; else
            return false;
        }
        else
        if (parse_sv_id_res == SV_ID_PEN)
        {
          src = &sv_pen_cur_us;
          if (c == 'u') dest = &sv_pen_up_us; else
          if (c == 'm') dest = &sv_pen_mid_us; else
          if (c == 'd') dest = &sv_pen_down_us; else
            return false;
        }

        // End-of-line expected
        if (!parse_eol()) return false;

        // Store servo setting
        *dest = *src;

        // Dump
        serial_println_sv_us(parse_sv_id_res,c,*src,0);

        return true;
      }
      else
      if (c == 'd')
      {
        // End-of-line expected
        if (!parse_eol()) return false;

        Serial.print(F("servo left:  "));
        Serial.println(sv_left_cur_us);
        Serial.print(F("servo right: "));
        Serial.println(sv_right_cur_us);
        Serial.print(F("servo pen:   "));
        Serial.println(sv_pen_cur_us);

        return true;
      }
    }
  }
  else if (c == 'p')
  {
    // Pen commands

    c = fetch_rcv_char();
    if (c == 's')
    {
      double  x, y;

      // X position (millimeters)
      parse_spaces();
      if (!parse_dec_double()) return false;
      x = parse_double_res;

      // Y position (millimeters)
      parse_spaces();
      if (!parse_dec_double()) return false;
      y = parse_double_res;

      // End-of-line expected
      if (!parse_eol()) return false;

      pen_set(x,y);

      if (verbose_level == 1) serial_println_pen_pos();

      return true;
    }
    else
    if (c == 'm')
    {
      // Pen move (line stroke to position)

      double  x, y;

      // X position (millimeters)
      parse_spaces();
      if (!parse_dec_double()) return false;
      x = parse_double_res;

      // Y position (millimeters)
      parse_spaces();
      if (!parse_dec_double()) return false;
      y = parse_double_res;

      // End-of-line expected
      if (!parse_eol()) return false;

      pen_line_to(x,y);

      if (verbose_level == 1) serial_println_pen_pos();

      return true;
    }
    else
    if (c == 'a')
    {
      double  hradius, vradius, a1, a2;

      // Horizontal radius (millimeters)
      parse_spaces();
      if (!parse_dec_double()) return false;
      hradius = parse_double_res;

      // Vertical radius (millimeters)
      parse_spaces();
      if (!parse_dec_double()) return false;
      vradius = parse_double_res;

      if (hradius == 0.0)
      {
        if (vradius == 0.0) return false;
        hradius = vradius;
      }
      else
      if (vradius == 0.0) vradius = hradius;

      // Start angle (radians or degrees)
      parse_spaces();
      if (!parse_dec_double()) return false;
      parse_spaces();
      if (!parse_r_d()) return false;
      a1 = (parse_r_d_res == 'r')
            ? parse_double_res
            : DEG_TO_RAD(parse_double_res);

      // End angle (radians or degrees)
      parse_spaces();
      if (!parse_dec_double()) return false;
      parse_spaces();
      if (!parse_r_d()) return false;
      a2 = (parse_r_d_res == 'r')
            ? parse_double_res
            : DEG_TO_RAD(parse_double_res);

      // End-of-line expected
      if (!parse_eol()) return false;

      pen_arc_to(hradius,vradius,a1,a2);

      if (verbose_level == 1) serial_println_pen_pos();

      return true;
    }
    else
    if (c == 'g')
    {
      // Draw glyph

      byte    glyph;
      double  sx, sy;

      // Glyph
      parse_spaces();
      if (!parse_dec_u16()) return false;
      if (parse_u16_res >= __PEN_GLYPH_CNT__) return false;
      glyph = (byte)parse_u16_res;

      // X scaling factor (millimeters)
      parse_spaces();
      if (!parse_dec_double()) return false;
      sx = parse_double_res;

      // Y scaling factor (millimeters)
      parse_spaces();
      if (!parse_dec_double()) return false;
      sy = parse_double_res;

      // End-of-line expected
      if (!parse_eol()) return false;

      pen_draw_glyph(glyph,sx,sy);

      if (verbose_level == 1) serial_println_pen_pos();

      return true;
    }
    else
    if (c == 'r')
    {
      // End-of-line expected
      if (!parse_eol()) return false;

      pen_draw_ref_rect();

      if (verbose_level == 1) serial_println_pen_pos();

      return true;
    }
    else
    if (c == 't')
    {
      // End-of-line expected
      if (!parse_eol()) return false;

      auto_mode_draw_dt();

      if (verbose_level == 1) serial_println_pen_pos();

      return true;
    }
    else
    if (c == 'l')
    {
      // Lift pen up, down, or to the middle

      c = fetch_rcv_char();
      if (c == 'u')
      {
        // End-of-line expected
        if (!parse_eol()) return false;
  
        pen_lift_up();
  
        if (verbose_level > 0) Serial.println(F("pen: up"));

        return true;
      }
      else
      if (c == 'm')
      {
        // End-of-line expected
        if (!parse_eol()) return false;
  
        pen_lift_mid();
  
        if (verbose_level > 0) Serial.println(F("pen: in middle"));

        return true;
      }
      else
      if (c == 'd')
      {
        // End-of-line expected
        if (!parse_eol()) return false;
  
        pen_lift_down();
  
        if (verbose_level > 0) Serial.println(F("pen: down"));

        return true;
      }
    }
  }
  else
  if (c == 'v')
  {
    c = fetch_rcv_char();
    if (c == 'b')
    {
      // Verbose setting

      byte  level;

      // Verbose level (0..2)
      parse_spaces();
      if (!parse_dec_u16_range(0,2)) return false;
      level = (byte)parse_u16_res;

      // End-of-line expected
      if (!parse_eol()) return false;

      verbose_level = level;

      return true;
    }
    else
    if (c == 'm')
    {
      // Vibration motor

      c = fetch_rcv_char();
      if (c == 'e')
      {
        byte  period;

        if (!parse_opt_eol())
        {
          // Time (0, or min..max seconds)
          parse_spaces();
          if (!parse_dec_u16()) return false;
          if (
              (parse_u16_res == 0) ||
              ((parse_u16_res >= VIB_PERIOD_MIN_S) && (parse_u16_res <= VIB_PERIOD_MAX_S))
             )
            period = (byte)parse_u16_res;
          else
           return false;

          // End-of-line expected
          if (!parse_eol()) return false;
        }
        else
        {
          // No argument specified, use active setting
          period = vib_period_s;
        }

        vib_turn_on();

        if (period > 0)
        {
          delay(period*1000);
          vib_turn_off();
        }

        return true;
      }
      else
      if (c == 'd')
      {
        // End-of-line expected
        if (!parse_eol()) return false;
  
        vib_turn_off();
  
        return true;
      }
      else
      if (c == 's')
      {
        byte  period;
 
        // Time (1..20 seconds)
        parse_spaces();
        if (!parse_dec_u16_range(VIB_PERIOD_MIN_S,VIB_PERIOD_MAX_S)) return false;
        period = (byte)parse_u16_res;

        // End-of-line expected
        if (!parse_eol()) return false;

        // Update setting
        vib_period_s = period;

        // Dump
        serial_println_vib_period_s(0);

        return true;
      }
    }
  }
  else
  if (c == 'c')
  {
    // Realtime clock commands

    c = fetch_rcv_char();
    if (c == 'r')
    {
      // End-of-line expected
      if (!parse_eol()) return false;

      if (!pcf2129_poll()) return true;

      Serial.print(F("battery "));
      if (pcf2129_info.battery_low) Serial.print(F("low")); else Serial.print(F("ok"));
      if (pcf2129_info.power_up)
      {
        Serial.print(F(", power-up"));
      }
      else
      {
        Serial.print(F(", 20"));
        serial_print_2bcd(pcf2129_info.dt.year);
        Serial.print('-');
        serial_print_2bcd(pcf2129_info.dt.month);
        Serial.print('-');
        serial_print_2bcd(pcf2129_info.dt.day);
        Serial.print(' ');
        serial_print_2bcd(pcf2129_info.dt.hour);
        Serial.print(':');
        serial_print_2bcd(pcf2129_info.dt.minute);
        Serial.print(':');
        serial_print_2bcd(pcf2129_info.dt.second);
      }
      Serial.println();

      return true;
    }
    else if (c == 'w')
    {
      // Year (2000..2099)
      parse_spaces();
      if (!parse_dec_u16_range(2000,2099)) return false;
      pcf2129_info.dt.year = parse_u16_res - 2000;

      // Month (1..12)
      parse_spaces();
      if (!parse_dec_u16_range(1,12)) return false;
      pcf2129_info.dt.month = (byte)parse_u16_res;

      // Day (1..31)
      parse_spaces();
      if (!parse_dec_u16_range(1,31)) return false;
      pcf2129_info.dt.day = (byte)parse_u16_res;

      // Hour (0..23)
      parse_spaces();
      if (!parse_dec_u16_range(0,23)) return false;
      pcf2129_info.dt.hour = (byte)parse_u16_res;

      // Minute (0..59)
      parse_spaces();
      if (!parse_dec_u16_range(0,59)) return false;
      pcf2129_info.dt.minute = (byte)parse_u16_res;

      // Second (0..59)
      parse_spaces();
      if (!parse_dec_u16_range(0,59)) return false;
      pcf2129_info.dt.second = (byte)parse_u16_res;

      // End-of-line expected
      if (!parse_eol()) return false;

      // Set date and time in realtime clock
      pcf2129_set_dt();

      return true;
    }
  }
  else
  if (c == 'm')
  {
    // Program mode

    c = fetch_rcv_char();
    if (c == 'a')
    {
      // End-of-line expected
      if (!parse_eol()) return false;

      set_prog_mode_auto(false);

      return true;
    }
    else
    if (c == 's')
    {
      byte  mode;

      c = fetch_rcv_char();
      if (c == 'c') mode = PROG_MODE_CMD; else
      if (c == 'a') mode = PROG_MODE_AUTO; else
        return false;

      // End-of-line expected
      if (!parse_eol()) return false;

      // Update setting
      startup_prog_mode = mode;

      // Dump
      serial_println_startup_prog_mode(0);

      return true;
    }
  }
  else
  if (c == 'd')
  {
    c = fetch_rcv_char();
    if (c == 'i')
    {
      byte  di;

      // Drawing interval
      parse_spaces();
      if (!parse_dec_u16()) return false;
      switch (parse_u16_res)
      {
        case 1:  di = DRAWING_INTERVAL_1MIN; break;
        case 5:  di = DRAWING_INTERVAL_5MIN; break;
        case 10: di = DRAWING_INTERVAL_10MIN; break;
        case 15: di = DRAWING_INTERVAL_15MIN; break;
        case 20: di = DRAWING_INTERVAL_20MIN; break;
        case 30: di = DRAWING_INTERVAL_30MIN; break;
        case 60: di = DRAWING_INTERVAL_1HOUR; break;
        default: return false;
      }

      // End-of-line expected
      if (!parse_eol()) return false;

      drawing_interval = di;

      // Dump
      serial_println_drawing_interval(0);

      return true;
    }
  }
  else
  if (c == 'e')
  {
    c = fetch_rcv_char();
    if (c == 'c')
    {
      boolean   enabled;

      c = fetch_rcv_char();
      if (c == 'e') enabled = 1; else
      if (c == 'd') enabled = 0; else
        return false;

      // End-of-line expected
      if (!parse_eol()) return false;

      echo_commands = enabled;

      // Dump
      serial_println_echo_commands(0);

      return true;
    }
  }

  return false;
}


// Receive characters from the serial port in non-blocking fashion.

void  rcv ()
{
  char  c;

  while (Serial.available() > 0)
  {
    c = Serial.read();
    if ((c == 13) || (c == 10))
    {
      if (rcv_error == false)
      {
        if (echo_commands) echo_rcv_buf();

        if (process_rcv())
          Serial.println(F("OK"));
        else
          Serial.println(F("Error in command!"));
      }

      // Reset the receive buffer
      rcv_si    = 0;
      rcv_error = false;
      rcv_fi    = 0;
    }
    else if ((c < 32) || (c > 126))
    {
      // Invalid character received
      rcv_error = true;
    }
    else if (rcv_si == RCV_BUF_LEN)
    {
      // Buffer overflow
      rcv_error = true;
    }
    else
    {
      // Store character in receive buffer
      rcv_buf[rcv_si] = c;
      rcv_si++;
    }
  }
}


boolean  rcv_eol ()
{
  boolean   eol;
  char      c;

  // Read all pending Rx bytes (purge Rx buffer) while checking for EOL characters
  eol = false;
  while (Serial.available() > 0)
  {
    c = Serial.read();
    if ((c == 13) || (c == 10)) eol = true;
  }

  return eol;
}


typedef struct _AUTO_MODE
{
  unsigned long   poll_millis;    // Timer for polling RTC (milliseconds resolution)
  byte            vib_min;        // Minute at which to start vibrating
  byte            draw_sec;       // Second at which to stop vibrating and start drawing
  byte            glyphs[4];      // The glyphs to draw
  byte            state;          // Current state of autonomous mode
  boolean         once;           // Draw once y/n
}
  AUTO_MODE;


AUTO_MODE     auto_mode;


void  set_prog_mode_auto (boolean once)
{
  prog_mode = PROG_MODE_AUTO;

  auto_mode.once = once;

  vib_turn_off();
  pen_lift_up();
  pen_set_resting();
  auto_mode_start();

  Serial.println(F("Autonomous mode, press ENTER to exit"));
}


void  auto_mode_draw_dt ()
{
  double  x,y;

  auto_mode.glyphs[0] = pcf2129_info.dt.hour / 10;
  auto_mode.glyphs[1] = pcf2129_info.dt.hour % 10;
  auto_mode.glyphs[2] = pcf2129_info.dt.minute / 10;
  auto_mode.glyphs[3] = pcf2129_info.dt.minute % 10;

  if (verbose_level >= 1)
  {
    Serial.print(F("Drawing: "));
    Serial.print(auto_mode.glyphs[0]);
    Serial.print(auto_mode.glyphs[1]);
    Serial.print(':');
    Serial.print(auto_mode.glyphs[2]);
    Serial.println(auto_mode.glyphs[3]);
  }

  x = pos_x1 + dist_12 / 2 - 42.0;
  y = pos_y1 + 30.0 + 23.0;

  pen_set(x,y);
  pen_draw_glyph(PEN_GLYPH_0 + auto_mode.glyphs[0],0.7,1);
  x += 20.0;

  pen_set(x,y);
  pen_draw_glyph(PEN_GLYPH_0 + auto_mode.glyphs[1],0.7,1);
  x += 20.0;

  pen_set(x,y);
  pen_draw_glyph(PEN_GLYPH_COLON,0.7,1);
  x += 9.0;

  pen_set(x,y);
  pen_draw_glyph(PEN_GLYPH_0 + auto_mode.glyphs[2],0.7,1);
  x += 20.0;

  pen_set(x,y);
  pen_draw_glyph(PEN_GLYPH_0 + auto_mode.glyphs[3],0.7,1);
}


boolean  auto_mode_poll_rtc ()
{
  if (!pcf2129_poll())
  {
    set_prog_mode_cmd();
    return false;
  }
  
  if (pcf2129_info.power_up)
  {
    Serial.println(F("PCF2129 power-up detected. Kindly set date and time."));

    set_prog_mode_cmd();
    return false;
  }

  return true;
}


void  auto_mode_set_vib_min ()
{
  byte  interval;
  byte  m;

  switch (drawing_interval)
  {
    case DRAWING_INTERVAL_1MIN:  interval = 1; break;
    case DRAWING_INTERVAL_5MIN:  interval = 5; break;
    case DRAWING_INTERVAL_10MIN: interval = 10; break;
    case DRAWING_INTERVAL_15MIN: interval = 15; break;
    case DRAWING_INTERVAL_20MIN: interval = 20; break;
    case DRAWING_INTERVAL_30MIN: interval = 30; break;
    case DRAWING_INTERVAL_1HOUR: interval = 60; break;
    default: return; // Shouldn't end up here
  }

  m = pcf2129_info.dt.minute;

  m -= (m % interval);  // Round down to multiple of interval     => m = 0..59
  m += interval;        // Increment to next multiple of interval => m = 1..60
  m--;                  // Start vibration one minute in advance  => m = 0..59

  auto_mode.vib_min = m;
}


void  auto_mode_handle_state ()
{
  for (;;)
  {
    switch (auto_mode.state)
    {
      case 0:
      {
        auto_mode.draw_sec = (pcf2129_info.dt.second + vib_period_s) % 60;
        vib_turn_on();
  
        auto_mode.state = 1;
  
        if (verbose_level >= 1)
        {
          Serial.print(F("AUTO state 0->1: "));
          Serial.print(pcf2129_info.dt.second);
          Serial.print(F(" s -> "));
          Serial.print(auto_mode.draw_sec);
          Serial.println(F(" s"));
        }
  
        return;
      }
  
      case 1:
      {
        if (auto_mode.draw_sec == pcf2129_info.dt.second)
        {
          if (verbose_level >= 1)
          {
            Serial.print(F("AUTO state 1: "));
            Serial.print(pcf2129_info.dt.second);
            Serial.println(F(" s"));
          }

          vib_turn_off();
          auto_mode_draw_dt();
          pen_lift_up();
          pen_set_resting();

          if (auto_mode.once)
          {
            set_prog_mode_cmd();
            return;
          }

          auto_mode_set_vib_min();
  
          auto_mode.state = 2;
  
          if (verbose_level >= 1)
          {
            Serial.println(F("AUTO state 1->2"));
          }
        }
  
        return;
      }
  
      case 2:
      {
        if (
            (digitalRead(PIN_PUSH_BUTTON) == LOW) ||
            (
             (pcf2129_info.dt.minute == auto_mode.vib_min) &&
             (pcf2129_info.dt.second >= (60 - vib_period_s))
            )
           )
        {
          auto_mode.state = 0;
  
          if (verbose_level >= 1)
          {
            Serial.print(F("AUTO state 2->0: "));
            Serial.print(pcf2129_info.dt.second);
            Serial.println(F(" s"));
          }

          // Continue executing without returning, to make sure pcf2129_info.dt.second remains unchanged
          break;
        }

        return;
      }
    }
  }
}


void  auto_mode_handle ()
{
  unsigned long   cur_millis;

  cur_millis = millis();
  if ((cur_millis - auto_mode.poll_millis) >= 800)
  {
    auto_mode.poll_millis = cur_millis;

    if (!auto_mode_poll_rtc()) return;
  }

  auto_mode_handle_state();
}


void  auto_mode_start ()
{
    auto_mode.poll_millis = millis();
    auto_mode.state       = 0;

    auto_mode_poll_rtc();
}


void  loop ()
{
  if (prog_mode == PROG_MODE_AUTO)
  {
    // Autonomous mode

    // Check for EOL (ENTER key for example)
    if (rcv_eol())
    {
      set_prog_mode_cmd();
    }
    else
    {
      auto_mode_handle();
    }
  }
  else
  {
    // Command mode

    // Receive and process commands
    rcv();

    if (digitalRead(PIN_PUSH_BUTTON) == LOW) set_prog_mode_auto(true);
  }
}

void  setup ()
{
  Serial.begin(9600);

  // Control line for vibration motor
  pinMode(PIN_VIB,OUTPUT);

  // Push button
  pinMode(PIN_PUSH_BUTTON,INPUT_PULLUP);

  // Servo motors
  sv_left.attach(PIN_SERVO_LEFT);
  sv_right.attach(PIN_SERVO_RIGHT);
  sv_pen.attach(PIN_SERVO_PEN);

  // Join the I2C bus as master
  Wire.begin();

  Serial.println(F("Reading EEPROM..."));
  settings_read_eeprom();

  if (settings_validate(true))
  {
    settings_accept();

    Serial.println(F("Settings applied"));

    // Move the pen up. This is the position farthest from the bottom panel. We don't want to
    // plunge the pen into the bottom panel in case the user has accidentally programmed a middle
    // or down position that's too low. Ofcourse, if the up position is wrong, the pen might hit
    // the bottom panel anyway. It's up to the user to provide correct settings.
    //
    // Don't call pen_lift_to() here! This function starts from the current PWM duty cycle
    // which is zero at this point meaning the pen would hit the bottom panel hard before being
    // lifted up.
    sv_pen_write_us(sv_pen_up_us);

    // Move the pen to the resting position.
    pen_set_resting();

    // Kick off in the program mode that was stored in EEPROM
    if (startup_prog_mode == PROG_MODE_AUTO)
      set_prog_mode_auto(false);
    else
      set_prog_mode_cmd();
  }
  else
  {
    // Put servo motors in their middle position. This is convenient during the construction of
    // the mechanical parts.
    sv_left_write_us(1500);
    sv_right_write_us(1500);
    sv_pen_write_us(1500);

    // Kick off in interactive command mode
    set_prog_mode_cmd();
  }
}

