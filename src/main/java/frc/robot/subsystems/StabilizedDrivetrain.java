// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StabilizedDrivetrain extends SubsystemBase {

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;


  public AHRS gyro;

  public StabilizedDrivetrain(int leftMotorID, int rightMotorID) {
    rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushed);
    leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushed);
    leftMotor.setInverted(true);
  }

  public void setLeft(double input) {
    leftMotor.set(input);
  }

  public void setRight(double input) {
    rightMotor.set(input);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
