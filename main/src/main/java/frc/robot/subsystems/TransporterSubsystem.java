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

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransporterConstants;;

public class TransporterSubsystem extends SubsystemBase {
  // motors
  private final WPI_VictorSPX m_transporter;
  
  public TransporterSubsystem() {
    m_transporter     = new WPI_VictorSPX(TransporterConstants.TRANSPORTER_MOTOR_ID);

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

  public Command stopTransportCommand() {
    return this.runOnce(stopTransportRunnable());
  }

  /**
   * 
   * @param speed rotation per second
   */
  private Runnable transportRunnable(double speed) {
    return () -> transport(speed);
  }

  private Runnable stopTransportRunnable() {
    return () -> stopTransport();
  }

  /**
   * 
   * @param speed rotation per second
   */
  public void transport(double speed) {
    m_transporter.set(speed);
  }

  public void stopTransport() {
    m_transporter.set(0);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
