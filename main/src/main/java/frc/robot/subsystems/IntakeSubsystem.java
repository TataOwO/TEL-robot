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

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_intake;
  private final TalonFXConfiguration configs = new TalonFXConfiguration();

  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();

  private final StatusSignal<Double> intakeVelocity; 

  private StatusCode status = StatusCode.StatusCodeNotInitialized;

  public IntakeSubsystem() {
    configs.Slot0.kP = 0.1;
    configs.Slot0.kI = 0.01;
    configs.Slot0.kD = 0;

    m_intake = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

    for (int i=0; i<5; i++) {
      status = m_intake.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) System.out.println("An error occured at intake subsys: " + status.toString());

    intakeVelocity = m_intake.getVelocity();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.runOnce(
        () -> {
          
        });
  }

  public double getVelocity() {
    return intakeVelocity.getValueAsDouble();
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
    m_intake.setControl(m_velocityVoltage.withVelocity(RPS));
  }

  private void stop() {
    m_intake.setControl(m_brake);
  }

  @Override
  public void periodic() {
    intakeVelocity.refresh();
  }

  @Override
  public void simulationPeriodic() {
    intakeVelocity.refresh();
  }
}
