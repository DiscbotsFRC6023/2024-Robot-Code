// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.Partials;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerShot extends SequentialCommandGroup {
  /** Creates a new SpeakerShot. */
  /*
   * Set arm to idle position
   * Turn on shooter
   * Shoot note
   * Stop Intake
   */
  public SpeakerShot(Swerve s_Swerve, Arm s_Arm, Intake s_Intake, Limelight s_Limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> s_Arm.setArmPos(Constants.armIdlePOS), s_Arm).withTimeout(2.0),
      new RunCommand(() -> s_Intake.shootNote(), s_Intake).withTimeout(1.0),
      new shootIntake(s_Intake).withTimeout(2.0),
      new InstantCommand(() -> s_Intake.stopAll(), s_Intake)
    );
  }
}
