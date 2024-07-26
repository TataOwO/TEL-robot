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

    frontLeftPid = frontLeftMotor.getPIDController();
    frontRightPid = frontRightMotor.getPIDController();
    rearLeftPid = rearLeftMotor.getPIDController();
    rearRightPid = rearRightMotor.getPIDController();

    double kP = 1e-3;
    double kI = 1e-7;
    double kD = 1;

    frontLeftPid. setP(kP);
    frontRightPid.setP(kP);
    rearLeftPid.  setP(kP);
    rearRightPid. setP(kP);

    frontLeftPid. setI(kI);
    frontRightPid.setI(kI);
    rearLeftPid.  setI(kI);
    rearRightPid. setI(kI);

    frontLeftPid. setD(kD);
    frontRightPid.setD(kD);
    rearLeftPid.  setD(kD);
    rearRightPid. setD(kD);
  }

  public void drive(double xSpeed, double ySpeed, double zRotation) {
    xSpeed *= this.speedModifier;
    ySpeed *= this.speedModifier;
    zRotation *= this.speedModifier;

    m_drive.driveCartesian(xSpeed, ySpeed, zRotation);
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
