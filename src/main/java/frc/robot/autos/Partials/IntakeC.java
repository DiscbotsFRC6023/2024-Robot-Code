// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.Partials;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeC extends Command {
  /** Creates a new Intake. */
  private Intake s_Intake;

  public IntakeC(Intake s_Intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = s_Intake;
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.intakeNote();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (s_Intake.hasNote()) {
      return true;
    } else {
      return false;
    }
  }
}
