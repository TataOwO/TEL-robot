// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ShooterConstants {
    public static final int TOP_SHOOTER_ROTATION_SPEED = 1;
    public static final int BOTTOM_SHOOTER_ROTATION_SPEED = 1;

    public static final int TOP_SHOOTER_MOTOR_ID = 0;
    public static final int BOTTOM_SHOOTER_MOTOR_ID = 0;
  }

  public static class LoaderConstants {
    public static final int LOADER_ROTATION_SPEED = 1;

    public static final int LOADER_MOTOR_ID = 0;
  }

  public static class DriveConstants {
    public static final CANSparkLowLevel.MotorType DRIVE_MOTOR_TYPE = CANSparkLowLevel.MotorType.kBrushless;

    public static final int FRONT_LEFT_MOTOR_ID = 4;
    public static final int FRONT_RIGHT_MOTOR_ID = 31;
    public static final int REAR_LEFT_MOTOR_ID = 57;
    public static final int REAR_RIGHT_MOTOR_ID = 11;

    public static final double WHEEL_DISTANCE_X = 0; // x+ -> FRONT
    public static final double WHEEL_DISTANCE_y = 0; // y+ -> LEFT

    public static final double BASE_SPEED = 100;
  }
}
