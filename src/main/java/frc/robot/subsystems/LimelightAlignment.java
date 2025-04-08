// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;
import java.security.PublicKey;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class LimelightAlignment extends SubsystemBase {

  //private CommandSwerveDrivetrain drivetrain  = TunerConstants.createDrivetrain();
  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
  private Boolean run = false;

  //PID bad for Limelight don't use
  //private final PIDController xControl = new PIDController(1, 0, 0.3);
  //private final PIDController zControl = new PIDController(1, 0, 0.3);
  private final PIDController yawControl = new PIDController(0.3, 0, 0.4);
  private final double kix = 1.5;
  private final double kiy = 1.5;

  double ySpeed = 0;
  double xSpeed = 0;
  

  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** Creates a new LimelightAlignment. */
  public LimelightAlignment() {}

  public Command LimelightAlign(CommandSwerveDrivetrain drivetrain, boolean left){
    return run(() -> this.driveAtTag(drivetrain, left));
  }
  
  public Command setYaw(double yaw){
    return run(() -> yawControl.setSetpoint(yaw));
  }

  public Command LimelightAlignWithHeading(CommandSwerveDrivetrain drivetrain, boolean left, double heading){
    return run(() -> this.driveAtTagWithHeading(drivetrain, left, heading));
  }

  private void driveAtTagWithHeading(CommandSwerveDrivetrain driveT, boolean left, double heading){
    Pose3d cameraPose_TargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace("");
    yawControl.setTolerance(0.05);
    yawControl.enableContinuousInput(-180, 180);

    double xOffset = Constants.LimelightAlignment.kRightoffset;
    if(left)
    {
      xOffset = Constants.LimelightAlignment.kLeftoffset;
    }
    if (cameraPose_TargetSpace.getX() != 0 && cameraPose_TargetSpace.getY() != 0)
    {
      ySpeed = kiy * (cameraPose_TargetSpace.getX() + xOffset);
      xSpeed = -kix * (cameraPose_TargetSpace.getZ() + Constants.LimelightAlignment.kYofset);
    }
    else
    {
      System.out.println("can't see apriltag");
      ySpeed = 0;
      xSpeed = 0;
    }

    double yawSpeed = (driveT.getPigeon2().getYaw().getValueAsDouble() - heading) * Constants.LimelightAlignment.kiYaw;
    driveT.setControl(new SwerveRequest.RobotCentric().withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(yawSpeed));
  }

  // George Code
  private void driveAtTag(CommandSwerveDrivetrain driveT, boolean left){
      Pose3d cameraPose_TargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(""); // Camera's pose relative to tag (should use Robot's pose in the future)
   
      // when basing speed off offsets lets add an extra proportional term for each of these
      // lets not edit the yaw
      //double xPos = -0.16501350439035187;
      //double yPos = -0.08830078668779259;

      //xControl.setSetpoint(0);
      //zControl.setSetpoint(1);
      //xControl.setTolerance(0.05);
      //zControl.setTolerance(0.05);
  
      yawControl.setTolerance(0.05);
      yawControl.enableContinuousInput(-180, 180);

      // when lime light is not seeing the target, it will return 0 for x and y
      // if x and y are 0, then we should not move
      double xOffset = Constants.LimelightAlignment.kRightoffset;
      if(left){
        xOffset = Constants.LimelightAlignment.kLeftoffset;
      }
      if (cameraPose_TargetSpace.getX() != 0 && cameraPose_TargetSpace.getY() != 0)
      {
        ySpeed = kiy * (cameraPose_TargetSpace.getX() + xOffset);
        xSpeed = -kix * (cameraPose_TargetSpace.getZ() + Constants.LimelightAlignment.kYofset);
      }
      else
      {
        System.out.println("can't see apriltag");
        ySpeed = 0;
        xSpeed = 0;
      }

      // System.out.println("Z: " + cameraPose_TargetSpace.getZ());
      // double yawSpeed = -yawControl.calculate(driveT.getPigeon2().getYaw().getValueAsDouble())  * 0.01;
      // if(yawSpeed < 0.05){
      //   yawSpeed = 0;
      // }
      
      //driveT.applyRequest(() ->
      //  robotCentricRequest.withVelocityX(xSpeed).withVelocityY(ySpeed)
      //);

      // if tx or ty is not 0, then move
      
      driveT.setControl(new SwerveRequest.RobotCentric().withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(0));
      
      
  }

  @Override
  public void periodic() {
    // Basic targeting data
    double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
    boolean tv = LimelightHelpers.getTV(""); // Do you have a valid target?

    // Pose3d cameraPose_TargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(""); // Camera's pose relative to tag (should use Robot's pose in the future)

    // System.out.println("X Speed: " + xSpeed + " Y Speed: " + ySpeed + " X Pos: " + cameraPose_TargetSpace.getX() + " Y Pos: " + cameraPose_TargetSpace.getY());

    // System.out.println("tx: " + tx + " ty: " + ty + " ta: " + ta + " tv: " + tv);
  }
}
