// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_topShooter;
  private final TalonFX m_botShooter;
  private final TalonFXConfiguration topShooterConfigs = new TalonFXConfiguration();

  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();

  private final StatusSignal<Double> topShooterVelocity;

  private StatusCode shooterStatus = StatusCode.StatusCodeNotInitialized;

  public ShooterSubsystem() {
    topShooterConfigs.Slot0.kP = 0.1;
    topShooterConfigs.Slot0.kI = 0.01;
    topShooterConfigs.Slot0.kD = 0;

    m_topShooter = new TalonFX(ShooterConstants.TOP_SHOOTER_MOTOR_ID);
    m_botShooter = new TalonFX(ShooterConstants.BOTTOM_SHOOTER_MOTOR_ID);

    for (int i=0; i<5; i++) {
      shooterStatus = m_topShooter.getConfigurator().apply(topShooterConfigs);
      if (shooterStatus.isOK()) break;
    }
    if (!shooterStatus.isOK()) System.out.println("An error occured at shooter subsystem: " + shooterStatus.toString());

    topShooterVelocity = m_topShooter.getVelocity();
  }

  /**
   * 
   * @param RPS rotation per second
   */
  public Command shootCommand(double RPS) {
    return this.runOnce(shootRunnable(RPS));
  }

  public Command stopCommand() {
    return this.runOnce(stopRunnable());
  }

  /**
   * 
   * @param RPS rotation per second
   */
  private Runnable shootRunnable(double RPS) {
    return () -> shoot(RPS);
  }

  private Runnable stopRunnable() {
    return () -> stop();
  }

  /**
   * 
   * @param RPS rotation per second
   */
  private void shoot(double RPS) {
    m_topShooter.setControl(m_velocityVoltage.withVelocity(RPS));
  }

  private void stop() {
    m_topShooter.setControl(m_brake);
  }

  @Override
  public void periodic() {
    topShooterVelocity.refresh();
  }

  @Override
  public void simulationPeriodic() {
    topShooterVelocity.refresh();
  }
}
