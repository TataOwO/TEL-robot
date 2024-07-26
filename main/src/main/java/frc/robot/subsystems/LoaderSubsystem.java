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
import frc.robot.Constants.IntakeConstants;

public class LoaderSubsystem extends SubsystemBase {
  private final TalonFX m_loader;
  private final TalonFXConfiguration loaderConfigs = new TalonFXConfiguration();

  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut motorBrake = new NeutralOut();

  private final StatusSignal<Double> loaderVelocity; 

  private StatusCode loaderStatus = StatusCode.StatusCodeNotInitialized;

  public LoaderSubsystem() {
    loaderConfigs.Slot0.kP = 0.1;
    loaderConfigs.Slot0.kI = 0.01;
    loaderConfigs.Slot0.kD = 0;

    m_loader = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

    for (int i=0; i<5; i++) {
      loaderStatus = m_loader.getConfigurator().apply(loaderConfigs);
      if (loaderStatus.isOK()) break;
    }
    if (!loaderStatus.isOK()) System.out.println("An error occured at intake subsystem: " + loaderStatus.toString());

    loaderVelocity = m_loader.getVelocity();
  }

  public double getVelocity() {
    return loaderVelocity.getValueAsDouble();
  }

  /**
   * 
   * @param RPS rotation per second
   */
  public Command takeCommand(double RPS) {
    return this.runOnce(takeRunnable(RPS));
  }

  public Command stopCommand() {
    return this.runOnce(stopRunnable());
  }

  /**
   * 
   * @param RPS rotation per second
   */
  private Runnable takeRunnable(double RPS) {
    return () -> take(RPS);
  }

  private Runnable stopRunnable() {
    return () -> stop();
  }

  /**
   * 
   * @param RPS rotation per second
   */
  private void take(double RPS) {
    m_loader.setControl(m_velocityVoltage.withVelocity(RPS));
  }

  private void stop() {
    m_loader.setControl(motorBrake);
  }

  @Override
  public void periodic() {
    loaderVelocity.refresh();
  }

  @Override
  public void simulationPeriodic() {
    loaderVelocity.refresh();
  }
}
