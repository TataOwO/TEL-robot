// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransporterConstants;;

public class TransporterSubsystem extends SubsystemBase {
  private final WPI_VictorSPX m_transporter;
  private final VictorSPXConfiguration transporter_configs = new VictorSPXConfiguration();

  private final VelocityVoltage m_velocity_voltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut motor_brake = new NeutralOut();

  private StatusCode loader_status = StatusCode.StatusCodeNotInitialized;

  public TransporterSubsystem() {
    m_transporter = new WPI_VictorSPX(TransporterConstants.TRANSPORTER_MOTOR_ID);
    
    m_transporter.config_kP(0, 0.1);
    m_transporter.config_kI(0, 0.01);
    m_transporter.config_kD(0, 0.0);
  }

  /**
   * 
   * @param speed rotation per second
   */
  public Command transportCommand(double speed) {
    return this.runOnce(transportRunnable(speed));
  }

  public Command stopCommand() {
    return this.runOnce(stopRunnable());
  }

  /**
   * 
   * @param speed rotation per second
   */
  private Runnable transportRunnable(double speed) {
    return () -> transport(speed);
  }

  private Runnable stopRunnable() {
    return () -> stop();
  }

  /**
   * 
   * @param speed rotation per second
   */
  private void transport(double speed) {
    m_transporter.set(speed);
  }

  private void stop() {
    m_transporter.set(0);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
