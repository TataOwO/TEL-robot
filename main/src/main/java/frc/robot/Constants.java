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

    public static final int GYRO_PORT = 16;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_ID = 3;
    public static final int SHOOTER_SUPPORT_MOTOR_ID = 1;

    public static final double TOP_SPEED = 115;
    public static final double MID_SPEED = 91;
    public static final double BOT_SPEED = 69;
  }

  public static class TransporterConstants {
    public static final double TRANSPORTER_ROTATION_SPEED = 0.5;
    public static final double TRANSPORTER_LOAD_SPEED = -0.4;
    public static final double DIRECTION_ROTATION_SPEED = 0.6;

    public static final int TRANSPORTER_MOTOR_ID = 11;
    public static final int DIRECTION_MOTOR_ID = 12;

    public static final int DIRECTION_BUTTON_PIN = 0;
    public static final int TRANSPORT_BUTTON_PIN = 2;
  }

  public static class StorageConstants {
    public static final double STORAGE_RPM = 0.2;
    public static final double REVERSE_RPM = -0.2;

    public static enum StorageSide {
      RIGHT (2, 1),
      LEFT  (41, 3);

      private final int motor_id;
      private final int button_id;

      StorageSide(int motor_id, int button_id) {
        this.motor_id  = motor_id;
        this.button_id = button_id;
      }
      
      public int getMotorId() {
        return this.motor_id;
      }
      
      public int getButtonId() {
        return this.button_id;
      }
    }
  }

  public static class LoaderConstants {
    public static final double LOADER_SPEED = 0.8;
    public static final double REVERSE_SPEED = -0.8;

    public static final int FRONT_LOADER_MOTOR_ID = 7;
    public static final int LEFT_LOADER_MOTOR_ID = 13;

    public static final double TRANSPORT_LOAD_TIME = 0.5;

    public static final double LOAD_MINIMUM_TIME = 1.0;

    public static enum loaderSide {
      RIGHT (7, 16),
      LEFT  (13, 16);

      private final int motor_id;
      private final int discs_count;

      loaderSide(int motor_id, int discs_count) {
        this.motor_id = motor_id;
        this.discs_count = discs_count;
      }

      public int getMotorId() {
        return this.motor_id;
      }

      public int getDefaultDiscCount() {
        return this.discs_count;
      }
    }
  }

  public static class UltrasonicConstants {
    public static enum UltrasonicSide {
      TEST       (0, 1, 0, -7, 20),
      FRONT      (5, 4, 0, 0, 62),
      BACK       (0, 1, 180, 0, 0),
      LEFT_FRONT (7, 6, -90, -329, -40),
      LEFT_BACK  (0, 1, -90, 0, 0),
      RIGHT_FRONT(9, 8, 90, 252, -35),
      RIGHT_BACK (0, 1, 90, 0, 0);

      private final int trig_channel;
      private final int echo_channel;
      private final int base_angle;
      private final double offset_x;
      private final double offset_y;

      UltrasonicSide(int trig_channel, int echo_channel, int base_angle, double offset_x, double offset_y) {
        this.trig_channel = trig_channel;
        this.echo_channel = echo_channel;
        this.base_angle   = base_angle;
        this.offset_x     = offset_x;
        this.offset_y     = offset_y;
      }

      public int getTrigChannel() {
        return this.trig_channel;
      }

      public int getEchoChannel() {
        return this.echo_channel;
      }

      public int getBaseAngle() {
        return this.base_angle;
      }

      public double[] getOffset() {
        return new double[] {offset_x, offset_y};
      }
    }
  }

  public static class DriveConstants {
    public static final CANSparkLowLevel.MotorType DRIVE_MOTOR_TYPE = CANSparkLowLevel.MotorType.kBrushless;

    public static final int LEFT_FRONT_MOTOR_ID = 57;
    public static final int RIGHT_FRONT_MOTOR_ID = 11;
    public static final int REAR_LEFT_MOTOR_ID = 4;
    public static final int REAR_RIGHT_MOTOR_ID = 31;

    public static final double WHEEL_DISTANCE_X = 0; // x+ -> FRONT
    public static final double WHEEL_DISTANCE_y = 0; // y+ -> LEFT

    public static final double BASE_SPEED = 600;

    public static final double DEFAULT_SPEED_MODIFIER = 0.4;

    public static final double POSITION_CLOSED_KP  = 5e-3;
    public static final double POSITION_CLOSED_KI  = 0.0;
    public static final double POSITION_CLOSED_KD  = 0.0;
    public static final double POSITION_CLOSED_KFF = 0.0;
  }

  public static class LedStripConstants {
    public static final int PWM_PIN = 0;
    public static final int LED_COUNT = 12;
  }
}
