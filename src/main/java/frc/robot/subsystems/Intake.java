// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final CANSparkMax intakeRollers = new CANSparkMax(18, MotorType.kBrushless);
  private final CANSparkFlex bottomShooter = new CANSparkFlex(17, MotorType.kBrushless);
  private final CANSparkFlex topShooter = new CANSparkFlex(16, MotorType.kBrushless);
  private final DigitalInput beamBreak = new DigitalInput(1);
  private final DigitalInput beamBreak2 = new DigitalInput(2);
  private boolean shooterActive = false;
  private boolean notePresent = false;
  private Arm s_Arm;

  public Intake(Arm s_Arm) {
    topShooter.setInverted(true);
    topShooter.follow(bottomShooter);
    topShooter.setIdleMode(IdleMode.kCoast);
    bottomShooter.setIdleMode(IdleMode.kCoast);
    intakeRollers.setIdleMode(IdleMode.kCoast);
    this.s_Arm = s_Arm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Shooter Active:", shooterActive);
    
    if(this.getBeamBreak()){
      notePresent = true;
    } else {
      notePresent = false;
    }

    SmartDashboard.putBoolean("Note Present", notePresent);
  }

  public void intakeNote(){
    if(this.getBeamBreak()){
      if(shooterActive){
        intakeRollers.set(-0.25);
      } else {
        intakeRollers.set(0.0);
      }
    } else {
      intakeRollers.set(-0.25);
    }
  }

  public void outakeNote(){
    intakeRollers.set(0.2);
  }

  public void shootNote(){
    shooterActive = true;
    if(s_Arm.getEncoderinDegrees() <= 270.0){
      bottomShooter.set(-0.1);
    } else {
      bottomShooter.set(-0.5);
    }
    
  }

  public void stopIntake(){
    intakeRollers.set(0.0);
  }

  public void stopShooter(){
    shooterActive = false;
    bottomShooter.set(0.0);
  }

  public void stopAll(){
    shooterActive = false;
    intakeRollers.set(0.0);
    bottomShooter.set(0.0);
  }

  public boolean getBeamBreak(){
    if(!beamBreak.get() || !beamBreak2.get()){
      return true;
    }
    return false;
  }

  public boolean hasNote(){
    return hasNote();
  }
}
