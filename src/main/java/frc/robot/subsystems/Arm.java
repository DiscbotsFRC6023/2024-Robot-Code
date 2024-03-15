// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  // CHANGE ALL THESE LATER
  private final CANSparkMax leftArm = new CANSparkMax(14, MotorType.kBrushless);
  private final CANSparkMax rightArm = new CANSparkMax(15, MotorType.kBrushless);
  private final SparkAbsoluteEncoder armEncoder = rightArm.getAbsoluteEncoder(Type.kDutyCycle);
  private PIDController armController = new PIDController(0.013, 0.0001, 0.001);
  private double temp = 0.0;
  
  public Arm() {
    //armEncoder.reset();
    leftArm.restoreFactoryDefaults();
    rightArm.restoreFactoryDefaults();
    leftArm.setIdleMode(IdleMode.kBrake);
    leftArm.setOpenLoopRampRate(1.0);
    rightArm.setIdleMode(IdleMode.kBrake);
    rightArm.setOpenLoopRampRate(1.0);
    temp = 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void forward(){
    this.setArm(0.35);
  }

  public void backward(){
    this.setArm(-0.35);
  }

  public void setArm(double power){
    rightArm.set(-power);
    leftArm.set(power);
  }

  public void setArmPos(double degrees){
      temp = -armController.calculate(this.getEncoderinDegrees(), degrees);
      MathUtil.clamp(temp, -0.5, 0.5);
      this.setArm(temp);
  }

  public double getEncoderinDegrees(){
    return armEncoder.getPosition() * 360.0;
  }

  public void resetEncoder(){
    
  }

  public void stop(){
    rightArm.set(0.0);
    leftArm.set(0.0);
  }
}
