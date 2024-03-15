// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  //private final TalonFX climbingMotor = new TalonFX(20);
  private final TalonFX climbingMotor = new TalonFX(20);

  public Climber() {
    climbingMotor.setNeutralMode(NeutralMode.Brake);
    climbingMotor.configForwardSoftLimitThreshold(900000.0);
    climbingMotor.configReverseSoftLimitThreshold(0);
    climbingMotor.configForwardSoftLimitEnable(true);
    climbingMotor.configReverseSoftLimitEnable(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void up(){
    climbingMotor.set(TalonFXControlMode.PercentOutput, 0.7);
  }

  public void down(){
    climbingMotor.set(TalonFXControlMode.PercentOutput, -0.7);
  }
  public void set(double speed){
    climbingMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void stop(){
    climbingMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public void toggleSoftStops(boolean tog){
    climbingMotor.configReverseSoftLimitEnable(tog);
  }
}
