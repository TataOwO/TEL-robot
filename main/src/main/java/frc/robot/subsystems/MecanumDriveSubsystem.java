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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

public class MecanumDriveSubsystem extends SubsystemBase {
  private final MecanumDrive m_drive;

  private CANSparkMax frontLeftMotor;
  private CANSparkMax frontRightMotor;
  private CANSparkMax rearLeftMotor;
  private CANSparkMax rearRightMotor;

  private RelativeEncoder frontLeftEncoder;
  private RelativeEncoder frontRightEncoder;
  private RelativeEncoder rearLeftEncoder;
  private RelativeEncoder rearRightEncoder;

  private SparkPIDController frontLeftPid;
  private SparkPIDController frontRightPid;
  private SparkPIDController rearLeftPid;
  private SparkPIDController rearRightPid;

  private MecanumDrive.WheelSpeeds wheelSpeeds = new MecanumDrive.WheelSpeeds(0,0,0,0);

  private static final CANSparkLowLevel.MotorType driveMotorType = DriveConstants.DRIVE_MOTOR_TYPE;

  private double speedModifier = 0.6;
  private double currentSpeed = DriveConstants.BASE_SPEED * speedModifier;

  public MecanumDriveSubsystem() {
    frontLeftMotor = new CANSparkMax(DriveConstants.FRONT_LEFT_MOTOR_ID, driveMotorType);
    frontRightMotor = new CANSparkMax(DriveConstants.FRONT_RIGHT_MOTOR_ID, driveMotorType);
    rearLeftMotor = new CANSparkMax(DriveConstants.REAR_LEFT_MOTOR_ID, driveMotorType);
    rearRightMotor = new CANSparkMax(DriveConstants.REAR_RIGHT_MOTOR_ID, driveMotorType);

    frontLeftMotor.setInverted(true);
    rearLeftMotor.setInverted(true);

    m_drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

    frontLeftEncoder = frontLeftMotor.getAlternateEncoder(1);
    frontRightEncoder = frontRightMotor.getAlternateEncoder(1);
    rearLeftEncoder = rearLeftMotor.getAlternateEncoder(1);
    rearRightEncoder = rearRightMotor.getAlternateEncoder(1);

    frontLeftPid = frontLeftMotor.getPIDController();
    frontRightPid = frontRightMotor.getPIDController();
    rearLeftPid = rearLeftMotor.getPIDController();
    rearRightPid = rearRightMotor.getPIDController();

    frontLeftPid. setP(.1);
    frontRightPid.setP(.1);
    rearLeftPid.  setP(.1);
    rearRightPid. setP(.1);

    frontLeftPid. setI(.0);
    frontRightPid.setI(.0);
    rearLeftPid.  setI(.0);
    rearRightPid. setI(.0);

    frontLeftPid. setD(.0);
    frontRightPid.setD(.0);
    rearLeftPid.  setD(.0);
    rearRightPid. setD(.0);
  }

  public void drive(double xSpeed, double ySpeed, double zRotation) {
    wheelSpeeds = MecanumDrive.driveCartesianIK(xSpeed, ySpeed, zRotation);

    frontLeftPid.setReference(wheelSpeeds.frontLeft * currentSpeed, ControlType.kVelocity);
    frontRightPid.setReference(wheelSpeeds.frontRight * currentSpeed, ControlType.kVelocity);
    rearLeftPid.setReference(wheelSpeeds.rearLeft * currentSpeed, ControlType.kVelocity);
    rearRightPid.setReference(wheelSpeeds.rearRight * currentSpeed, ControlType.kVelocity);
  }

  public void increaseSpeed() {
    if (speedModifier == 1) return;
    speedModifier += 0.1;

    this.currentSpeed = DriveConstants.BASE_SPEED * this.speedModifier;
  }

  public void decreaseSpeed() {
    if (speedModifier == 0.1) return;
    speedModifier -= 0.1;

    this.currentSpeed = DriveConstants.BASE_SPEED * this.speedModifier;
  }

  public double getSpeedModifier() {
    return this.speedModifier;
  }

  public double getCurrentSpeed() {
    return this.currentSpeed;
  }

  @Override
  public void periodic() {
    // poseEstimator.update(this.getGyroRotation(), this.getWheelPositions());

    return;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
