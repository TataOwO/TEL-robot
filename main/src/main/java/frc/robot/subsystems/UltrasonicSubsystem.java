// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.sound.midi.Soundbank;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.UltrasonicConstants;
import frc.robot.Constants.UltrasonicConstants.UltrasonicSide;;

public class UltrasonicSubsystem extends SubsystemBase {
  private final int trig_channel_id;
  private final int echo_channel_id;

  private final UltrasonicSide m_side;

  // private final DigitalOutput ping_channel;
  // private final DigitalInput echo_channel;

  private final Ultrasonic ultrasonic;

  /** Creates a new UltrasonicSubsystem. */
  public UltrasonicSubsystem(UltrasonicSide side) {
    trig_channel_id = side.getTrigChannel();
    echo_channel_id = side.getEchoChannel();
    m_side = side;

    // ping_channel = new DigitalOutput(ping_channel_id);
    // echo_channel = new DigitalInput(echo_channel_id);

    ultrasonic = new Ultrasonic(trig_channel_id, echo_channel_id);

    Ultrasonic.setAutomaticMode(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public double getRangeMM() {
    return this.ultrasonic.getRangeMM();
  }

  public UltrasonicSide getSide() {
    return this.m_side;
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public double[] calculatePosition(double current_angle_r) {
    // distance between ultrasonic and the center
    double base_x = Math.cos(current_angle_r);
    double base_y = Math.sin(current_angle_r);

    double sonic_angle_r = Math.toRadians(m_side.getBaseAngle()) + current_angle_r;

    double distance = this.getRangeMM();

    // distance between ultrasonic and object
    double distance_x = Math.cos(sonic_angle_r) * distance;
    double distance_y = Math.sin(sonic_angle_r) * distance;

    return new double[] {base_x+distance_x, -base_y-distance_y};
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    String name = this.getSide().name();
    SmartDashboard.putBoolean(String.join(" ", name, "ultrasonic valid"), this.ultrasonic.isRangeValid());
    SmartDashboard.putNumber(String.join(" ", name, "ultrasonic range mm"), this.getRangeMM());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
