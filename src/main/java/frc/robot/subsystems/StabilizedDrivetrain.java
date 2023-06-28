// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StabilizedDrivetrain extends SubsystemBase {

  private TalonSRX leftMotor;
  private TalonSRX rightMotor;

  // private TalonFXConfiguration config;


  public AHRS gyro;

  public StabilizedDrivetrain(int leftMotorID, int rightMotorID) {
    // config = new TalonFXConfiguration();

    leftMotor = new TalonSRX(leftMotorID);
    rightMotor = new TalonSRX(rightMotorID);

    // config.supplyCurrLimit.enable = true;
    // config.supplyCurrLimit.triggerThresholdCurrent = 30;
    // config.supplyCurrLimit.currentLimit = 25;
    // config.supplyCurrLimit.triggerThresholdTime = 1.5;

    // leftMotor.configAllSettings(config);
    // rightMotor.configAllSettings(config);

  }
  public void set(double input) {
    leftMotor.set(ControlMode.PercentOutput, input);
    rightMotor.set(ControlMode.PercentOutput, input);
  }

  public void setLeft(double input) {
    leftMotor.set(ControlMode.PercentOutput, input);
  }

  public void setRight(double input) {
    rightMotor.set(ControlMode.PercentOutput, input);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
