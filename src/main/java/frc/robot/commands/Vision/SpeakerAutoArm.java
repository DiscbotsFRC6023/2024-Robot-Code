// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;

public class SpeakerAutoArm extends Command {
  /** Creates a new LimelightTest. */

  private final Limelight s_Limelight;
  private final Arm s_Arm;
  private PhotonPipelineResult res;
  private PhotonTrackedTarget tar;
  private double calculatedDistance = 0.0;
  private double calculatedAngle = 0.0;

  public SpeakerAutoArm(Limelight s_Limelight, Arm s_Arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Limelight = s_Limelight;
    this.s_Arm = s_Arm;
    addRequirements(s_Limelight, s_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    res = s_Limelight.getLastResult();
    if(res.hasTargets()){
      SmartDashboard.putNumber("Valid AprilTag ID:", res.getBestTarget().getFiducialId());

      tar = res.getBestTarget();
      
      if(tar.getFiducialId() == 4 || tar.getFiducialId() == 7){  //ID for the AprilTag
        // Start Angle Calculation
        calculatedDistance = PhotonUtils.calculateDistanceToTargetMeters(Constants.limelightHeight, Constants.speakerTagHeightCM / 100, Constants.limelightAngle, Units.degreesToRadians(tar.getPitch()));
        
        /* Calculate shooter angle by using arcsin(desiredHeight / calculated hypot) */
        calculatedAngle = -Math.asin(((Constants.speakerTagHeightCM + 61.35) / 100 /*Tag to speaker offset + 5.0 for error*/) / calculatedDistance);
        //calculatedAngle = Math.max(200, Math.max(354, Math.ceil(((calculatedAngle * 360) + 90 + 12)))); // Optional angle clamping
        calculatedAngle = (((calculatedAngle) * (180 / Math.PI)));
        calculatedAngle = (360 - calculatedAngle);

        System.out.println(calculatedAngle + "||" + s_Arm.getEncoderinDegrees());

        /* Set angle to system rotation motors */
        s_Arm.setArmPos(calculatedAngle);
      }

    } else {
      calculatedAngle = 0.0;
      s_Arm.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
