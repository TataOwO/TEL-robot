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
    public static final int DRIVER_CONTROLLER_PORT = 0;

    public static final int COMPRESSOR_PORT_ID = 1; // TO BE REMOVED
  }

  public static class ShooterConstants {
    public static final int TOP_SHOOTER_ROTATION_SPEED = 1;
    public static final int BOTTOM_SHOOTER_ROTATION_SPEED = 1;

    public static final int SHOOTER_MOTOR_ID = 3;
  }

  public static class TransporterConstants {
    public static final int TRANSPORTER_ROTATION_SPEED = 1;

    public static final int TRANSPORTER_MOTOR_ID = 1;
  }

  public static class LoaderConstants {
    public static final int LOADER_ROTATION_SPEED = 1;

    public static final int FRONT_LOADER_MOTOR_ID = 7;
    public static final int LEFT_LOADER_MOTOR_ID = 11;

    public static enum loaderSide {
      FRONT (7),
      LEFT  (11);

      private final int motor_id;

      loaderSide(int motor_id) {
        this.motor_id = motor_id;
      }

      public int getMotor_id() {
        return this.motor_id;
      }
    }
  }

  public static class UltrasonicConstants {
    public static enum ultrasonicSide {
      TEST       (0, 1),
      FRONT      (0, 1),
      BACK       (0, 1),
      LEFT_FRONT (0, 1),
      LEFT_BACK  (0, 1),
      RIGHT_FRONT(0, 1),
      RIGHT_BACK (0, 1);

      private final int trig_channel;
      private final int echo_channel;

      ultrasonicSide(int trig_channel, int echo_channel) {
        this.trig_channel = trig_channel;
        this.echo_channel = echo_channel;
      }

      public int getTrigChannel() {
        return this.trig_channel;
      }

      public int getEchoChannel() {
        return this.echo_channel;
      }
    }
  }

  public static class DriveConstants {
    public static final CANSparkLowLevel.MotorType DRIVE_MOTOR_TYPE = CANSparkLowLevel.MotorType.kBrushless;

    public static final int FRONT_LEFT_MOTOR_ID = 57;
    public static final int FRONT_RIGHT_MOTOR_ID = 31;
    public static final int REAR_LEFT_MOTOR_ID = 4;
    public static final int REAR_RIGHT_MOTOR_ID = 11;

    public static final double WHEEL_DISTANCE_X = 0; // x+ -> FRONT
    public static final double WHEEL_DISTANCE_y = 0; // y+ -> LEFT

    public static final double BASE_SPEED = 600;

    public static final double DEFAULT_SPEED_MODIFIER = 0.4;

    public static final double POSITION_CLOSED_KP  = 5e-3;
    public static final double POSITION_CLOSED_KI  = 0.0;
    public static final double POSITION_CLOSED_KD  = 0.0;
    public static final double POSITION_CLOSED_KFF = 0.0;
  }
}
