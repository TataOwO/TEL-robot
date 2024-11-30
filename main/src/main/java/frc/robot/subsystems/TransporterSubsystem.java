// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransporterConstants;;

public class TransporterSubsystem extends SubsystemBase {
  // motors
  private final WPI_VictorSPX m_transporter;

  private final double transport_speed = TransporterConstants.TRANSPORTER_ROTATION_SPEED;
  private final double load_speed = TransporterConstants.TRANSPORTER_LOAD_SPEED;

  private final DigitalOutput m_button = new DigitalOutput(TransporterConstants.TRANSPORT_BUTTON_PIN);
  
  public TransporterSubsystem() {
    m_transporter     = new WPI_VictorSPX(TransporterConstants.TRANSPORTER_MOTOR_ID);
    // m_transporter.setInverted(true);

    m_transporter.config_kP(0, 0.1);
    m_transporter.config_kI(0, 0.01);
    m_transporter.config_kD(0, 0.0);
  }

  /**
   * 
   * @param speed rotation per second
   */
  public Command transportCommand() {
    return this.runOnce(transportRunnable());
  }

  public Command loadCommand() {
    return this.runOnce(()->load());
  }

  public Command stopTransportCommand() {
    return this.runOnce(stopTransportRunnable());
  }

  public boolean getButton() {
    return m_button.get();
  }

  /**
   * 
   * @param speed rotation per second
   */
  private Runnable transportRunnable() {
    return () -> transport();
  }

  private Runnable stopTransportRunnable() {
    return () -> stop();
  }

  public void load() {
    m_transporter.set(this.load_speed);
  }

  /**
   * 
   * @param speed rotation per second
   */
  public void transport() {
    m_transporter.set(transport_speed);
  }

  public void stop() {
    m_transporter.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("transport button", getButton());
  }

  @Override
  public void simulationPeriodic() {}
}
