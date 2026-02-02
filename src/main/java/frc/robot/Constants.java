// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final double ENCODER_TICKS_PER_MOTOR_REVOLUTION = 2048.0;

  public static final int SHOOTER_MAIN_MOTOR_MASTER_CAN_ID = 4;
  public static final int SHOOTER_MAIN_MOTOR_SLAVE_CAN_ID = 11;
  public static final int SHOOTER_KICKER_MOTOR_CAN_ID = 12;
  public static final int SHOOTER_INTAKE_MOTOR_CAN_ID = 3;
  public static final int SHOOTER_HOOD_MOTOR_CAN_ID = 9;

  // Hood
  public static final double HOOD_COMPETITION_HOME_POSITION_DEGREES = 0.0;
  public static final double HOOD_AUTO_HOME_POSITION_DEGREES = 0.0;
  public static final double HOOD_RETRACT_HOME_POSITION_DEGREES = 2.0;
  public static final double HOOD_AUTO_ZERO_SPEED = -0.1;
  public static final double HOOD_MIN_ANGLE_DEGREES = 0.0;
  public static final double HOOD_MAX_ANGLE_DEGREES = 55.0;
  public static final double HOOD_FENDER_ANGLE_DEGREES = 0.0;
  public static final double HOOD_KEY_ANGLE_DEGREES = 8.0;
  public static final double HOOD_AUTON_SHORT_ANGLE_DEGREES = 35.0;
  public static final double HOOD_AUTO_ANGLE_DEGREES = 37.0;
  public static final double HOOD_MEDIUM_ANGLE_DEGREES = 47.0;
  public static final double HOOD_LONG_ANGLE_DEGREES = 55.0;
  public static final double HOOD_LEG_ANGLE_DEGREES = 46.0;
  public static final double HOOD_AUTO_DOUBLE_ANGLE_DEGREES = 46;
  public static final double HOOD_DISTANCE_SLOPE = 3.5 / 59.0;
  public static final double HOOD_DISTANCE_INTERCEPT = 38.0;
}
