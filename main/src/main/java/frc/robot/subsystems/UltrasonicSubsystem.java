// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.UltrasonicConstants;

public class UltrasonicSubsystem extends SubsystemBase {
  private final int trig_channel_id;
  private final int echo_channel_id;

  // private final DigitalOutput ping_channel;
  // private final DigitalInput echo_channel;

  private final Ultrasonic ultrasonic;

  /** Creates a new UltrasonicSubsystem. */
  public UltrasonicSubsystem(UltrasonicConstants.ultrasonicSide side) {
    trig_channel_id = side.getTrigChannel();
    echo_channel_id = side.getEchoChannel();

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
    // return this.ultrasonic.getRangeMM();
    return 0;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("ultrasonic on", this.ultrasonic.isEnabled());
    SmartDashboard.putBoolean("ultrasonic valid", this.ultrasonic.isRangeValid());
    SmartDashboard.putNumber("ultrasonic range mm", this.getRangeMM());
    SmartDashboard.putNumber("ultrasonic range inch", this.ultrasonic.getRangeInches());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
