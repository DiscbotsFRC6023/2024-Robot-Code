// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class Pivot extends Command {

  private Swerve s_Swerve;
  private Limelight s_Limelight;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier halfSpeedSup;
  private PhotonTrackedTarget tar;
  private PhotonPipelineResult res;
  PIDController rController = new PIDController(0.01, 0.0, 0.0);
         

  public Pivot(Limelight s_Limelight, Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier halfSpeedSup) {
      this.s_Swerve = s_Swerve;
      this.s_Limelight = s_Limelight;
      addRequirements(s_Limelight, s_Swerve);

      this.translationSup = translationSup;
      this.strafeSup = strafeSup;
      this.rotationSup = rotationSup;
      this.robotCentricSup = robotCentricSup;
      this.halfSpeedSup = halfSpeedSup;
  }

  @Override
  public void execute() {
    //s_Swerve.zeroYaw();

    res = s_Limelight.getLastResult();
      /* Get Values, Deadband*/
      double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
      double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
      double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

      /* Drive */
      s_Swerve.drive(
          new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
          rotationVal * Constants.Swerve.maxAngularVelocity, 
          !robotCentricSup.getAsBoolean(), 
          true,
          halfSpeedSup.getAsBoolean()
      );

      if(res.hasTargets()){
        tar = res.getBestTarget();
        if(tar.getFiducialId() == 4 || tar.getFiducialId() == 7){
          System.out.println("APRILTAG YAW:" + tar.getYaw() + "|| IMU YAW: " + s_Swerve.getYaw().getDegrees() + "Difference: " + (s_Swerve.getYaw().getDegrees() - (tar.getYaw())));

          rotationVal = rController.calculate(s_Swerve.getYaw().getDegrees(), tar.getYaw() + s_Swerve.getYaw().getDegrees());

        }

        s_Swerve.drive(
          new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
          rotationVal * Constants.Swerve.maxAngularVelocity, 
          !robotCentricSup.getAsBoolean(), 
          true,
          halfSpeedSup.getAsBoolean()
      );
      }
  }
}