// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.annotation.Target;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.Auto;

//subsystems imports
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.RoboSingSubsytem;
import frc.robot.subsystems.LimelightAlignment;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.AlgaeGroundtake;
import frc.robot.subsystems.Dealgaefier;


public class RobotContainer {

    private SlewRateLimiter xFilter = new SlewRateLimiter(Constants.Misc.kSlewRateLimit);
    private SlewRateLimiter yFilter = new SlewRateLimiter(Constants.Misc.kSlewRateLimit);
    private SlewRateLimiter yawFilter = new SlewRateLimiter(Constants.Misc.kYawSlewRateLimit);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //syom auto heading
    private double targetHeadingReef = 0; // in radians
    private DoubleSupplier targetHeadingReefSupplier = () -> targetHeadingReef;
    private double targetHeadingIntake = 2.2 +Math.PI;
    private DoubleSupplier AprilTagHeadingSupplier = () -> LimelightAlignment.ApriltagBasedOrientation;

    //multipleis the pov robot oriented to invert which side if front (only robot oriented, feild centric front should stay the same throughout the match)
    //should always be 1 or -1
    private double roboOrientedInverter = 1;


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    //syom auto heading
    private final SwerveRequest.FieldCentricFacingAngle headingRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.05) // Add a 3% deadband to translation
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    ;

    

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);



    private final CommandXboxController driveJoystick = new CommandXboxController(0);

    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    Joystick buttonBoard = new Joystick(2);
    private final JoystickButton back = new JoystickButton(buttonBoard, 10);
    private final JoystickButton right = new JoystickButton(buttonBoard, 11);
    private final JoystickButton forward = new JoystickButton(buttonBoard, 12);
    private final JoystickButton left = new JoystickButton(buttonBoard, 13);

    private final POVButton BbElevatorL4 = new POVButton(buttonBoard, 0);
    private final POVButton BbElevatorL3 = new POVButton(buttonBoard, 180);
    private final POVButton BbElevatorL2 = new POVButton(buttonBoard, 90);
    private final POVButton BbElevatorL1 = new POVButton(buttonBoard, 270);
    
    //initializing buttons, Bb stands for buttonboard
    private final JoystickButton BbReefBottomCenter = new JoystickButton(buttonBoard, 1);
    private final JoystickButton BbReefBottomRight = new JoystickButton(buttonBoard, 2);
    private final JoystickButton BbReefTopRight = new JoystickButton(buttonBoard, 3);
    private final JoystickButton BbReefTopCenter = new JoystickButton(buttonBoard, 4);
    private final JoystickButton BbReefTopLeft = new JoystickButton(buttonBoard, 5);
    private final JoystickButton BbReefBottomLeft = new JoystickButton(buttonBoard, 6);
    private final JoystickButton BbIntakeL = new JoystickButton(buttonBoard, 7);
    private final JoystickButton BbIntakeR = new JoystickButton(buttonBoard, 8);
    private final JoystickButton BbAlgaeProcessor = new JoystickButton(buttonBoard, 9);

    private final JoystickButton BbMasonReplacer = new JoystickButton(buttonBoard, 10);
    private final JoystickButton BbCoral2 = new JoystickButton(buttonBoard, 11);
    private final JoystickButton BbCoral3 = new JoystickButton(buttonBoard, 12);
     

    //instancing subsystems

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = Elevator.getInstance();
    private final Coral coral = Coral.getInstance();
    private final Dealgaefier dealgaefier = Dealgaefier.getInstance();

    public final AlgaeGroundtake algaeGroundtake = AlgaeGroundtake.getInstance();
    public final LimelightAlignment limelight = new LimelightAlignment();
    



    //syom sing intialize
    public final RoboSingSubsytem roboSingSubsytem = new RoboSingSubsytem();
    //public final LaserTest laserTest = new LaserTest();
    // private final RoboSingCommand singCommand = new RoboSingCommand(roboSing);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        registerPathPlannerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }
    

    private void registerPathPlannerNamedCommands(){
        //path planner named commands
        NamedCommands.registerCommand("LEFTlimelightAlign" ,limelight.LimelightAlign(drivetrain, true).withTimeout(Constants.Auto.kLimelightAllignTime));
        NamedCommands.registerCommand("RIGHTlimelightAlign" ,limelight.LimelightAlign(drivetrain,false).withTimeout(Constants.Auto.kLimelightAllignTime));


        NamedCommands.registerCommand("spitCoral" ,coral.IntakeAutoSpeed().withTimeout(Constants.Auto.kCoralSpinTime));

        NamedCommands.registerCommand("goToL1" ,elevator.goToElevatorStow().withTimeout(Constants.Auto.kElevatorTime));
        NamedCommands.registerCommand("goToL2" ,elevator.goToElevatorL2().withTimeout(Constants.Auto.kElevatorTime));
        NamedCommands.registerCommand("goToL3" ,elevator.goToElevatorL3().withTimeout(Constants.Auto.kElevatorTime));
        NamedCommands.registerCommand("goToL4" ,elevator.goToElevatorL4().withTimeout(Constants.Auto.kElevatorTime));
        NamedCommands.registerCommand("driveForward" ,drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)).withTimeout(Constants.Auto.kDriveForwardTime));
        NamedCommands.registerCommand("driveBackward" ,drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)).withTimeout(Constants.Auto.kDriveBackwardTime));

        NamedCommands.registerCommand("DealgFlopOut", dealgaefier.FlopOut().withTimeout(Constants.Auto.kDealgFlopInOutTime));
        NamedCommands.registerCommand("DealgFlopIn", dealgaefier.FlopIn().withTimeout(Constants.Auto.kDealgFlopInOutTime));

        NamedCommands.registerCommand("stopCoral", coral.stopIntake().withTimeout(Constants.Auto.kCoralStopTime));
        NamedCommands.registerCommand("stopDrive", drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0)).withTimeout(Constants.Auto.kCoralStopTime));
  

        //reef heading named commands
        NamedCommands.registerCommand("setHeading3",drivetrain.applyRequest(() -> 
                headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
                .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
                .withTargetDirection(new Rotation2d(2 * Math.PI / 3)))
            .withTimeout(Constants.Auto.kSetHeadingTime)
        );
        NamedCommands.registerCommand("setHeading4",drivetrain.applyRequest(() -> 
                headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
                .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
                .withTargetDirection(new Rotation2d(Math.PI)))
            .withTimeout(Constants.Auto.kSetHeadingTime)
        );
        NamedCommands.registerCommand("setHeading5",drivetrain.applyRequest(() -> 
                headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
                .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
                .withTargetDirection(new Rotation2d(4 * Math.PI / 3)))
            .withTimeout(Constants.Auto.kSetHeadingTime)
        );

        // most schizo named command of all time, reset odometry
        final Command resetOdoCommand = drivetrain.runOnce(() -> drivetrain.resetPose(getSecondPose()));
        NamedCommands.registerCommand("resetOdo", resetOdoCommand);



        //2 piece auto schizophreinc timings
        NamedCommands.registerCommand("2PdriveForward" ,drivetrain.applyRequest(() -> forwardStraight.withVelocityX(.8).withVelocityY(0)).withTimeout(Constants.Auto.k2PDriveForwardTime));
        NamedCommands.registerCommand("2PgoToL4" ,elevator.goToElevatorL4().withTimeout(Constants.Auto.k2PElevatorTime));
        NamedCommands.registerCommand("2PRIGHTlimelightAlign" ,limelight.LimelightAlign(drivetrain,false).withTimeout(Constants.Auto.k2PLimelightAllignTime));
        NamedCommands.registerCommand("2PgoToL1" ,elevator.goToElevatorStow().withTimeout(Constants.Auto.k2PElevatorTime));
        NamedCommands.registerCommand("2PspitCoral" ,coral.IntakeAutoSpeed().withTimeout(Constants.Auto.k2PCoralSpinTime));

        NamedCommands.registerCommand("slowCoral" ,coral.setSpeed(.08, 0).withTimeout(Constants.Auto.k2PCoralSpinTime));
        NamedCommands.registerCommand("reverseSlowCoral" ,coral.setSpeed(-.08, 0).withTimeout(Constants.Auto.k2PCoralSpinTime));

        NamedCommands.registerCommand("2PsetHeading2",drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(Math.PI / 3)))
            .withTimeout(Constants.Auto.k2PSetHeadingTime)
        );
        NamedCommands.registerCommand("2PsetHeading3",drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(2 * Math.PI / 3)))  
            .withTimeout(Constants.Auto.k2PSetHeadingTime)
        );
        NamedCommands.registerCommand("2PsetHeading4",drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(Math.PI)))
            .withTimeout(Constants.Auto.k2PSetHeadingTime)
        );
        NamedCommands.registerCommand("2PsetHeading5",drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(4 * Math.PI / 3)))
            .withTimeout(Constants.Auto.k2PSetHeadingTime)
        );
        NamedCommands.registerCommand("2PsetHeading6",drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(5 * Math.PI / 3)))
            .withTimeout(Constants.Auto.k2PSetHeadingTime)
        );

        NamedCommands.registerCommand("printMatchTime", drivetrain.runOnce(() -> {
            double matchTime = Timer.getMatchTime();
            System.out.println("Match Time: " + matchTime);
        }));

    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.


        //syom sing example binding

        driveJoystick.pov(90).onTrue(roboSingSubsytem.playRock());


/*---------------------------------- driver joystick stuff----------------------------------*/


        // Set the default command for the drivetrain to be a lambda that sets the drive request
        // most basic drive commandd, left joystick is drives, right joystick is yaw
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(xFilter.calculate(-driveJoystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(yFilter.calculate(-driveJoystick.getLeftX() )* MaxSpeed) // Drive left with negative X (left)
                    //filter.calculate is a limiter to make sure the robot accelarte  too fast
                    .withRotationalRate(-driveJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        System.out.println("Left Y: " + driveJoystick.getLeftY());

        // robot oriented drive forwad and backward, also left right
        driveJoystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5*roboOrientedInverter).withVelocityY(0))
        );
        driveJoystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.5*roboOrientedInverter))
        );
        driveJoystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5*roboOrientedInverter).withVelocityY(0))
        );
        driveJoystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(0.5*roboOrientedInverter))
        );

        //invert the pov controls, useful for when algea groundtake
        //the "back" button is in the middle of the controller slightly on the right
        driveJoystick.start().onTrue(drivetrain.runOnce(() -> {roboOrientedInverter *=-1;}));

        //resets which which way is considered forward
        //the "back" button is in the middle of the controller slightly on the left
        driveJoystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //auto gyro heading PID syom
        headingRequest.HeadingController.setP(Constants.Misc.kHeadingP);
        headingRequest.HeadingController.setI(Constants.Misc.kHeadingI);
        headingRequest.HeadingController.setD(Constants.Misc.kHeadingD);

        //reef SET target heading
        BbReefBottomCenter.onTrue(
            drivetrain.runOnce(() -> {targetHeadingReef = 0; }) // close
        );
        BbReefBottomRight.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = Math.PI / 3) // close right
        );
        BbReefTopRight.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = 2 * Math.PI / 3) // far right
        );
        BbReefTopCenter.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = Math.PI) // far
        );
        BbReefTopLeft.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = 4 * Math.PI / 3) // far left
        );
        BbReefBottomLeft.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = 5 * Math.PI / 3) // close left
        );

        //reef SET target heading for mason replacer
        BbMasonReplacer.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = (AprilTagHeadingSupplier.getAsDouble()-1)*Math.PI/3) // close
        );

        

        //intake station SET target heading
        BbIntakeL.onTrue(
            drivetrain.runOnce(() -> targetHeadingIntake = 2.2 +Math.PI) // left intake = face 126 degrees = 2.2 rads from forward 
        );
        BbIntakeR.onTrue(
            drivetrain.runOnce(() -> targetHeadingIntake = 4.084 + Math.PI) // right intake = face 234 degrees = 4.084 rads from forward
        );
        BbAlgaeProcessor.onTrue(
            drivetrain.runOnce(() -> targetHeadingIntake = 3 * Math.PI / 2) // algae processor = face right
        );

        //reef APPLY target heading
        driveJoystick.rightTrigger().whileTrue(
            limelight.LimelightAlignWithHeading(drivetrain, true, targetHeadingReefSupplier)
        );

        
        //intake station APPLY target heading
        driveJoystick.leftTrigger().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
                .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
                .withTargetDirection(new Rotation2d(targetHeadingIntake)))
        );

        // Limelight alignment
        driveJoystick.leftBumper().whileTrue(limelight.LimelightAlign(drivetrain, true));

        //align to right reef branch
        driveJoystick.rightBumper().whileTrue(limelight.LimelightAlign(drivetrain, false));



        //left trigger also go to fetal position (resests everythig to be compact)
        driveJoystick.leftTrigger().whileTrue(
            elevator.goToElevatorStow()
        );
        driveJoystick.leftTrigger().whileTrue(
            algaeGroundtake.goToPivotIn()
        );
        driveJoystick.leftTrigger().whileTrue(
            dealgaefier.FlopIn()
        );

        // Auto heading insta set and apply for four cardinal directions
        driveJoystick.y().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(0))) // Up
        );
        driveJoystick.b().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(3*Math.PI / 2))) // Right
        );
        driveJoystick.a().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(Math.PI))) // Down
        );
        driveJoystick.x().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d( Math.PI / 2))) // Left
        );




  


/*---------------------------------- operator joystick and button board stuff----------------------------------*/
        

//elevator controls
        BbElevatorL4.onTrue(
            elevator.goToElevatorL4() // go to elevator level 4 of reef
        );
        BbElevatorL3.onTrue(
            elevator.goToElevatorL3() // go to elevator level 3 of reef
        );
        BbElevatorL2.onTrue(
            elevator.goToElevatorL2() // go to elevator level 2 of reef
        );
        BbElevatorL1.onTrue(
            elevator.goToElevatorStow() // go to elevator stow position
        );

//coral controls
        operatorJoystick.y().whileTrue(
            coral.IntakeAutoSpeed() // runs the coral speed control depending on elevator height
        );

        operatorJoystick.a().whileTrue(
            coral.IntakeReverse() // runs the reverse intake
        );
        coral.setDefaultCommand(coral.stopIntake()); //whenever no button is pressed, intake doesnt spin

// dealgifier controls
        operatorJoystick.b().whileTrue(
            dealgaefier.FlopOut() // flops out the dealgaefier and start spinning lil thing at the end  
        );
        operatorJoystick.x().whileTrue(
            dealgaefier.FlopIn() // flops in the dealgaefier and stops spinning
        );


//algea groundtake controls
        operatorJoystick.pov(0).onTrue(
            algaeGroundtake.goToPivotOut() // flops out the algea groundtake
        );
        operatorJoystick.pov(180).onTrue(
            algaeGroundtake.goToPivotIn() // flops in the algea groundtake
        );
        operatorJoystick.pov(90).whileTrue(
            algaeGroundtake.Intake() // runs the intake
        );
        operatorJoystick.pov(270).whileTrue(
            algaeGroundtake.ReverseIntake() // runs the reverse intake
        );
        //whenever no button is pressed, intake doesnt spin
        algaeGroundtake.setDefaultCommand(algaeGroundtake.StopIntake());


// go to fetal position (resests everythig to be compact)
        operatorJoystick.leftTrigger().whileTrue(
            elevator.goToElevatorStow()
        );
        operatorJoystick.leftTrigger().whileTrue(
            algaeGroundtake.goToPivotIn()
        );
        operatorJoystick.leftTrigger().whileTrue(
            dealgaefier.FlopIn()
        );

//drivey stuff that opp(operator) can do

        //opp can also invert the pov controls, useful for when algea groundtake
        //the "back" button is in the middle of the controller slightly on the right
        operatorJoystick.start().onTrue(drivetrain.runOnce(() -> {roboOrientedInverter *=-1;}));

        //opp can also limelight allign
        // operatorJoystick.leftBumper().whileTrue(limelight.LimelightAlign(drivetrain, true));
        // operatorJoystick.rightBumper().whileTrue(limelight.LimelightAlign(drivetrain, false));



        



        





        // ong we dont need this shit wtf is "breaking"

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));



        //wtf is a sysid bru

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Pose2d getSecondPose (){
        List<PathPlannerPath> pathGroup;

        
        String autoFilename = getAutonomousCommand().getName();

        try {
            pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoFilename);
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println(" SCHIZO ERROR (syoma) Error loading path group from auto file: " + autoFilename);

            //if error will return the current pose of the robot
            return drivetrain.getState().Pose;
        }
        PathPlannerPath SecondPath = pathGroup.get(1);

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red){
            SecondPath = SecondPath.flipPath();
            
        }

        Pose2d secondPose2d = SecondPath.getStartingHolonomicPose().orElse(new Pose2d());

        return secondPose2d;
      
    }



    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

}
