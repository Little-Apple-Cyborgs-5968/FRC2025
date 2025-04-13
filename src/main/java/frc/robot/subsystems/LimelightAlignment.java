// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.io.Console;
import java.io.ObjectInputFilter.Config;
import java.security.PublicKey;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class LimelightAlignment extends SubsystemBase {

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  double toApplyOmega = 0.0;
   double timeStamp = 0.01; // 10ms

  //private CommandSwerveDrivetrain drivetrain  = TunerConstants.createDrivetrain();
  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();


  private final SwerveRequest.RobotCentricFacingAngle headingRequest = new SwerveRequest.RobotCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.05) // Add a 3% deadband to translation
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    ;


  private Boolean run = false;

  //PID bad for Limelight don't use
  //private final PIDController xControl = new PIDController(1, 0, 0.3);
  //private final PIDController zControl = new PIDController(1, 0, 0.3);
  private final PIDController yawControl = new PIDController(0.3, 0, 0.4);
  private final double kix = 1.5;
  private final double kiy = 1.5;

  double ySpeed = 0;
  double xSpeed = 0;

  public boolean AllignLeft = true;
  public boolean CurrentlyAlligning = false;

  public Command LimelightAlign(CommandSwerveDrivetrain drivetrain, boolean left){
    return run(() -> this.driveAtTag(drivetrain, left));
  }
  
  public Command setYaw(double yaw){
    return run(() -> yawControl.setSetpoint(yaw));
  }

  public Command LimelightAlignWithHeading(CommandSwerveDrivetrain drivetrain, boolean left, DoubleSupplier heading){
    System.out.println(heading);
    return run(() -> this.driveAtTagWithHeading(drivetrain, left, heading.getAsDouble()));
  
  }

  private void driveAtTagWithHeading(CommandSwerveDrivetrain driveT, boolean left, double h){
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
      ySpeed = 0;
      xSpeed = 0;
    }

    headingRequest.HeadingController.setP(Constants.Misc.kHeadingP);
    headingRequest.HeadingController.setI(Constants.Misc.kHeadingI);
    headingRequest.HeadingController.setD(Constants.Misc.kHeadingD);

    driveT.setControl(headingRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withTargetDirection(new Rotation2d(h)));

    System.out.println(h);

  }

  // George Code
  private void driveAtTag(CommandSwerveDrivetrain driveT, boolean left){
      CurrentlyAlligning = true;
      Pose3d cameraPose_TargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(""); // Camera's pose relative to tag (should use Robot's pose in the future)
      yawControl.setTolerance(0.05);
      yawControl.enableContinuousInput(-180, 180);
      

      
      double xOffset = Constants.LimelightAlignment.kRightoffset;
      //public bolean to keep track of which side we allign to
      AllignLeft = false;
      if(left){
        xOffset = Constants.LimelightAlignment.kLeftoffset;
        //public boolean
        AllignLeft = true;
      }
      // when lime light is not seeing the target, it will return 0 for x and y
      // if x and y are 0, then we should not move
      if (cameraPose_TargetSpace.getX() != 0 && cameraPose_TargetSpace.getY() != 0)
      {
        ySpeed = kiy * (cameraPose_TargetSpace.getX() + xOffset);
        xSpeed = -kix * (cameraPose_TargetSpace.getZ() + Constants.LimelightAlignment.kYofset);
      }
      else
      {
        ySpeed = 0;
        xSpeed = 0;
      }
      
      driveT.setControl(new SwerveRequest.RobotCentric().withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(0));
  }

  public Command getAprilTagHeading(){
    return run(() -> this.aprilTagheading());
  }

  double tagID = LimelightHelpers.getFiducialID("");
  public static double ApriltagBasedOrientation = 0;

  private void aprilTagheading() {
    ApriltagBasedOrientation = switch ((int) tagID) {
      case 18, 7 -> 1;
      case 17,8 -> 2;
      case 22,9 -> 3;
      case 21,10 -> 4;
      case 20,11 -> 5;
      case 19,12 -> 6;
      default -> 1;
    };
  }

  public Command idleCommand() {
    return run(() -> this.idle());
  }

  private void idle(){
    CurrentlyAlligning = false;
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
