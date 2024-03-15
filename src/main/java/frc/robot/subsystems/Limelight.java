// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private final PhotonCamera camera;

  public Limelight() {
    camera = new PhotonCamera("arducam");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   public PhotonPipelineResult getLastResult(){
    return camera.getLatestResult();
  }
}
