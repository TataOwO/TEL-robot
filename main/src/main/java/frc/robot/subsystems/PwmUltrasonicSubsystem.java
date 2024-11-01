// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.UltrasonicConstants;

public class PwmUltrasonicSubsystem extends SubsystemBase {
  private final int trig_channel_id;
  private final int echo_channel_id;

  private final PWM trigger_PWM;
  private final DigitalInput echo_channel;

  private double distance;

  // private final Ultrasonic ultrasonic;

  /** Creates a new UltrasonicSubsystem. */
  public PwmUltrasonicSubsystem(UltrasonicConstants.ultrasonicSide side) {
    trig_channel_id = side.getTrigChannel();
    echo_channel_id = side.getEchoChannel();

    trigger_PWM = new PWM(trig_channel_id);
    echo_channel = new DigitalInput(echo_channel_id);

    trigger_PWM.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
    trigger_PWM.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);
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
    trigger_PWM.setSpeed(1.0);
    Timer.delay(0.00001);
    trigger_PWM.setSpeed(-1.0);

    Timer timer = new Timer();
    timer.start();

    while (!echo_channel.get() && timer.get() < 0.001) {}

    if (echo_channel.get()) {
      double startTime = Timer.getFPGATimestamp();
      while (echo_channel.get() && timer.get() < 0.1) {}
      double endTime = Timer.getFPGATimestamp();
      
      double pulseDuration = endTime - startTime;
      distance = (pulseDuration * 34300) / 2; // Speed of sound in cm/s
    } else {
      distance = -1;
    }
    
    timer.stop();

    SmartDashboard.putNumber("ultrasonic range cm", this.distance);
  }

  public double getDistance() {
    return this.distance;
  }
}
