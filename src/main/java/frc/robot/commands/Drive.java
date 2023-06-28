// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.StabilizedDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Drive extends PIDCommand {
  /** Creates a new Drive. */

  StabilizedDrivetrain drivetrain;

  public Drive(StabilizedDrivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(12, 0, 0.5),
        // This should return the measurement
        () -> drivetrain.gyro.getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0, /* maybe 90? */
        // This uses the output
        output -> {
          drivetrain.setLeft(output);
          drivetrain.setRight(output);
        });
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
