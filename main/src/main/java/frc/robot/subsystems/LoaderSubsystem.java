// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LoaderConstants.loaderSide;

public class LoaderSubsystem extends SubsystemBase {
  private final WPI_VictorSPX m_loader;
  private final int loader_can_id;
  private final loaderSide loader_side;

  private int discs_count;

  public LoaderSubsystem(loaderSide loader_side) {
    this.loader_side = loader_side;

    loader_can_id = loader_side.getMotorId();

    m_loader = new WPI_VictorSPX(loader_can_id);

    this.discs_count = loader_side.getDefaultDiscCount();

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
  public void load(double speed) {
    m_loader.set(speed);
  }

  public void stop() {
    m_loader.set(0);
  }

  public int getDiscCount() {
    return this.discs_count;
  }

  public void useDisc() {
    this.discs_count--;
    System.out.println(this.loader_side.name() + " discs count: " + this.discs_count);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
