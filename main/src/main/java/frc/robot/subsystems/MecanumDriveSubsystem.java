// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.*;

public class MecanumDriveSubsystem extends SubsystemBase {
  private final MecanumDrive m_drive;

  private CANSparkMax front_left_motor;
  private CANSparkMax front_right_motor;
  private CANSparkMax rear_left_motor;
  private CANSparkMax rear_right_motor;

  private RelativeEncoder front_left_encoder;
  private RelativeEncoder front_right_encoder;
  private RelativeEncoder rear_left_encoder;
  private RelativeEncoder rear_right_encoder;

  private SparkPIDController front_left_PID;
  private SparkPIDController front_right_PID;
  private SparkPIDController rear_left_PID;
  private SparkPIDController rear_right_PID;

  private MecanumDrive.WheelSpeeds wheel_speeds = new MecanumDrive.WheelSpeeds(0,0,0,0);

  private static final CANSparkLowLevel.MotorType drive_motor_type = DriveConstants.DRIVE_MOTOR_TYPE;

  private double speed_modifier = DriveConstants.DEFAULT_SPEED_MODIFIER;
  private double current_speed = DriveConstants.BASE_SPEED * speed_modifier;

  private double position_closed_kP  = DriveConstants.POSITION_CLOSED_KP;
  private double position_closed_kI  = DriveConstants.POSITION_CLOSED_KI;
  private double position_closed_kD  = DriveConstants.POSITION_CLOSED_KD;
  private double position_closed_kFF = DriveConstants.POSITION_CLOSED_KFF;

  public MecanumDriveSubsystem() {
    front_left_motor  = new CANSparkMax(DriveConstants.FRONT_LEFT_MOTOR_ID , drive_motor_type);
    front_right_motor = new CANSparkMax(DriveConstants.FRONT_RIGHT_MOTOR_ID, drive_motor_type);
    rear_left_motor   = new CANSparkMax(DriveConstants.REAR_LEFT_MOTOR_ID  , drive_motor_type);
    rear_right_motor  = new CANSparkMax(DriveConstants.REAR_RIGHT_MOTOR_ID , drive_motor_type);

    front_left_motor.setInverted(true);
    rear_left_motor.setInverted(true);

    m_drive = new MecanumDrive(front_left_motor, rear_left_motor, front_right_motor, rear_right_motor);

    front_left_PID = front_left_motor.getPIDController();
    front_right_PID = front_right_motor.getPIDController();
    rear_left_PID = rear_left_motor.getPIDController();
    rear_right_PID = rear_right_motor.getPIDController();

    this.resetPID();

    SmartDashboard.putNumber("kP" , position_closed_kP);
    SmartDashboard.putNumber("kI" , position_closed_kI);
    SmartDashboard.putNumber("kD" , position_closed_kD);
    SmartDashboard.putNumber("kFF", position_closed_kFF);
  }

  public void driveWheelspeed(MecanumDrive.WheelSpeeds wheel_speeds) {
    this.front_left_motor .set(wheel_speeds.frontLeft );
    this.front_right_motor.set(wheel_speeds.frontRight);
    this.rear_left_motor  .set(wheel_speeds.rearLeft  );
    this.rear_right_motor .set(wheel_speeds.rearRight );
  }

  public void drive(double x_speed, double y_speed, double z_rotation) {
    x_speed *= this.speed_modifier;
    y_speed *= this.speed_modifier;
    z_rotation *= this.speed_modifier;

    m_drive.driveCartesian(x_speed, y_speed, z_rotation);
  }

  public void drivePositionClosedLoop(double degree) {
    front_left_PID .setReference(-degree * current_speed, ControlType.kPosition);
    front_right_PID.setReference( degree * current_speed, ControlType.kPosition);
    rear_left_PID  .setReference(-degree * current_speed, ControlType.kPosition);
    rear_right_PID .setReference( degree * current_speed, ControlType.kPosition);
  }

  public void resetPID() {
    front_left_PID. setP(position_closed_kP);
    front_right_PID.setP(position_closed_kP);
    rear_left_PID.  setP(position_closed_kP);
    rear_right_PID. setP(position_closed_kP);

    front_left_PID. setI(position_closed_kI);
    front_right_PID.setI(position_closed_kI);
    rear_left_PID.  setI(position_closed_kI);
    rear_right_PID. setI(position_closed_kI);

    front_left_PID. setD(position_closed_kD);
    front_right_PID.setD(position_closed_kD);
    rear_left_PID.  setD(position_closed_kD);
    rear_right_PID. setD(position_closed_kD);

    front_left_PID. setFF(position_closed_kFF);
    front_right_PID.setFF(position_closed_kFF);
    rear_left_PID.  setFF(position_closed_kFF);
    rear_right_PID. setFF(position_closed_kFF);
  }

  public void increaseSpeed() {
    if (speed_modifier == 1) return;
    speed_modifier += 0.1;

    this.current_speed = DriveConstants.BASE_SPEED * this.speed_modifier;
  }

  public void decreaseSpeed() {
    if (speed_modifier == 0.1) return;
    speed_modifier -= 0.1;

    this.current_speed = DriveConstants.BASE_SPEED * this.speed_modifier;
  }

  public double getSpeed_modifier() {
    return this.speed_modifier;
  }

  public double getCurrent_speed() {
    return this.current_speed;
  }

  @Override
  public void periodic() {
    double kP  = SmartDashboard.getNumber("kP" , position_closed_kP);
    double kI  = SmartDashboard.getNumber("kI" , position_closed_kI);
    double kD  = SmartDashboard.getNumber("kD" , position_closed_kD);
    double kFF = SmartDashboard.getNumber("kFF", position_closed_kFF);
    boolean isValueChanged = false;

    if (kP != position_closed_kP) {
      position_closed_kP = kP;
      isValueChanged = true;
    }
    if (kI != position_closed_kI) {
      position_closed_kI = kI;
      isValueChanged = true;
    }
    if (kD != position_closed_kD) {
      position_closed_kD = kD;
      isValueChanged = true;
    }
    if (kFF != position_closed_kFF) {
      position_closed_kFF = kFF;
      isValueChanged = true;
    }

    if (isValueChanged) this.resetPID();

    this.m_drive.driveCartesian(0, 0, 0);

    return;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
