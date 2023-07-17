// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Drivetrain extends PIDSubsystem {

  private TalonSRX leftMotor;
  private TalonSRX rightMotor;

  public AHRS gyro = new AHRS();

  /** Creates a new Drivetrain. */
  public Drivetrain(int leftMotorID, int rightMotorID) {
    super(
        // The PIDController used by the subsystem
        new PIDController(0.1, 0.0015, 0.001));
        
    leftMotor = new TalonSRX(leftMotorID);
    rightMotor = new TalonSRX(rightMotorID);

    leftMotor.setInverted(true);
    
  }

  public void setMotors(double input) {
    leftMotor.set(ControlMode.PercentOutput, -(input / 2));
    rightMotor.set(ControlMode.PercentOutput, -(input / 2));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    setMotors(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return gyro.getPitch();
  }
}
