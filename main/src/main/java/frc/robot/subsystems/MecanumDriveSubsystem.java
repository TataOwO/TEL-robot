// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

public class MecanumDriveSubsystem extends SubsystemBase {
  private final MecanumDrive m_drive;

  private CANSparkMax frontLeftMotor;
  private CANSparkMax frontRightMotor;
  private CANSparkMax rearLeftMotor;
  private CANSparkMax rearFrontMotor;

  private MecanumDriveKinematics kinematics;
  private MecanumDrivePoseEstimator poseEstimator;

  private static final CANSparkLowLevel.MotorType driveMotorType = DriveConstants.DRIVE_MOTOR_TYPE;

  double speedModifier = 0.6;

  public MecanumDriveSubsystem() {
    frontLeftMotor = new CANSparkMax(DriveConstants.FRONT_LEFT_MOTOR_ID, driveMotorType);
    frontRightMotor = new CANSparkMax(DriveConstants.FRONT_RIGHT_MOTOR_ID, driveMotorType);
    rearLeftMotor = new CANSparkMax(DriveConstants.REAR_LEFT_MOTOR_ID, driveMotorType);
    rearFrontMotor = new CANSparkMax(DriveConstants.REAR_RIGHT_MOTOR_ID, driveMotorType);

    frontLeftMotor.setInverted(true);
    rearLeftMotor.setInverted(true);

    m_drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearFrontMotor);

    // TODO Translation2d(x, y);
    // Translation2d FL_location = new Translation2d();
    // Translation2d FR_location = new Translation2d();
    // Translation2d RL_location = new Translation2d();
    // Translation2d RR_location = new Translation2d();

    // kinematics = new MecanumDriveKinematics(FL_location, FR_location, RL_location, RR_location);

    // Pose2d initialPose = new Pose2d(0, 0, new Rotation2d());

    // poseEstimator = new MecanumDrivePoseEstimator(kinematics, this.getGyroRotation(), this.getWheelPositions(), initialPose);
  }

  public void driveWithSpeed(double xSpeed, double ySpeed, double zRotation, double speedModifier) {
    xSpeed *= speedModifier;
    ySpeed *= speedModifier;
    zRotation *= speedModifier;

    m_drive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  private Rotation2d getGyroRotation() {
    // TODO implement gyro
    return new Rotation2d();
  }

  private MecanumDriveWheelPositions getWheelPositions() {
    // TODO implement wheel position
    return new MecanumDriveWheelPositions();
  }

  public void increaseSpeed() {
    if (speedModifier == 1) return;
    speedModifier += 0.1;
  }

  public void decreaseSpeed() {
    if (speedModifier == 0.1) return;
    speedModifier -= 0.1;
  }

  public double getSpeedModifier() {
    return this.speedModifier;
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
