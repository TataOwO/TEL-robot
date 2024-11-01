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
import frc.robot.Constants.LoaderConstants;

public class LoaderSubsystem extends SubsystemBase {
  private final WPI_VictorSPX m_loader;
  private final int loader_can_id;
  private final LoaderConstants.loaderSide loader_side;

  private final VelocityVoltage m_velocity_voltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut motor_brake = new NeutralOut();

  private StatusCode loader_status = StatusCode.StatusCodeNotInitialized;

  public LoaderSubsystem(LoaderConstants.loaderSide loader_side) {
    this.loader_side = loader_side;

    loader_can_id = loader_side.getMotor_id();

    m_loader = new WPI_VictorSPX(loader_can_id);

    m_loader.setInverted(true);
    
    m_loader.config_kP(0, 0.1);
    m_loader.config_kI(0, 0.01);
    m_loader.config_kD(0, 0.0);
  }

  /**
   * 
   * @param speed rotation per second
   */
  public Command loadCommand(double speed) {
    return this.runOnce(loadRunnable(speed));
  }

  public Command stopCommand() {
    return this.runOnce(stopRunnable());
  }

  /**
   * 
   * @param speed rotation per second
   */
  private Runnable loadRunnable(double speed) {
    return () -> load(speed);
  }

  private Runnable stopRunnable() {
    return () -> stop();
  }

  /**
   * 
   * @param speed rotation per second
   */
  private void load(double speed) {
    m_loader.set(speed);
  }

  private void stop() {
    m_loader.set(0);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
