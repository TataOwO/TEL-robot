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

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_shooter;
  private final TalonFXConfiguration shooter_configs = new TalonFXConfiguration();

  private final VelocityVoltage m_velocity_voltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();

  private final StatusSignal<Double> shooter_velocity;

  private StatusCode shooter_status = StatusCode.StatusCodeNotInitialized;

  public ShooterSubsystem() {
    shooter_configs.Slot0.kP = 0.1;
    shooter_configs.Slot0.kI = 0.01;
    shooter_configs.Slot0.kD = 0;

    m_shooter = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID);

    for (int i=0; i<5; i++) {
      shooter_status = m_shooter.getConfigurator().apply(shooter_configs);
      if (shooter_status.isOK()) break;
    }
    if (!shooter_status.isOK()) System.out.println("An error occured at shooter subsystem: " + shooter_status.toString());

    shooter_velocity = m_shooter.getVelocity();
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
    m_shooter.setControl(m_velocity_voltage.withVelocity(RPS));
  }

  private void stop() {
    m_shooter.setControl(m_brake);
  }

  @Override
  public void periodic() {
    shooter_velocity.refresh();
  }

  @Override
  public void simulationPeriodic() {
    shooter_velocity.refresh();
  }
}
