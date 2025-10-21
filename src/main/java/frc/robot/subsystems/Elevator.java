package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANrange;
//sketch import i added 

public class Elevator extends SubsystemBase {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Elevator mInstance;
  private PeriodicIO mPeriodicIO;

  // private static final double kPivotCLRampRate = 0.5;
  // private static final double kCLRampRate = 0.5;

  public static Elevator getInstance() {
    if (mInstance == null) {
      mInstance = new Elevator();
    }
    return mInstance;
  }

  private SparkMax mLeftMotor;
  private RelativeEncoder mLeftEncoder;
  private SparkClosedLoopController mLeftPIDController;

  private CANrange mCANRange;

  private SparkMax mRightMotor;

  // private SparkMax beamBreakController;
  // private SparkLimitSwitch beamBreak;

  private TrapezoidProfile mProfile;
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  private double prevUpdateTime = Timer.getFPGATimestamp();

  private Elevator() {
    super("Elevator");

    mPeriodicIO = new PeriodicIO();

    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig.closedLoop
        .pid(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD)
        .iZone(Constants.Elevator.kIZone);

    elevatorConfig.smartCurrentLimit(Constants.Elevator.kMaxCurrent);

    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(true);

    // LEFT ELEVATOR MOTOR
    mLeftMotor = new SparkMax(Constants.Elevator.kElevatorLeftMotorId, MotorType.kBrushless);
    mLeftEncoder = mLeftMotor.getEncoder();
    mLeftPIDController = mLeftMotor.getClosedLoopController();
    mLeftMotor.configure(
        elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // RIGHT ELEVATOR MOTOR
    mRightMotor = new SparkMax(Constants.Elevator.kElevatorRightMotorId, MotorType.kBrushless);
    mRightMotor.configure(
        elevatorConfig.follow(mLeftMotor, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Initialize CAN range
    mCANRange = new CANrange(Constants.Elevator.kCANRangeId);

    mProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            Constants.Elevator.kMaxVelocity,
            Constants.Elevator.kMaxAcceleration));
    
    //beamBreakController = new SparkMax(Constants.Elevator.kBeamBreakId, MotorType.kBrushless);
    // beamBreak = beamBreakController.getForwardLimitSwitch();

    
  }

  public enum ElevatorState {
    NONE,
    STOW,
    L2,
    L3,
    L4,
    A1,
    A2
  }
  
  public static ElevatorState publicState;

  public static class PeriodicIO {
    double elevator_target = 0.0;
    double elevator_power = 0.0;

    boolean is_elevator_pos_control = false;

    ElevatorState state = ElevatorState.STOW;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    //can range stuff
    System.out.println("CAN range device id : " + mCANRange.getDeviceID());
    System.out.println("CAN range distance : " + mCANRange.getDistance(true));
    

    //periodically updates the elveator motors to turn to correct position based of what mPeriodicIO.elevator_target is set. syom

    //theres a bunch of publci function  to change elevator level, e.g 
    double curTime = Timer.getFPGATimestamp();
    double dt = curTime - prevUpdateTime;
    prevUpdateTime = curTime;
    if (mPeriodicIO.is_elevator_pos_control) {
      // Update goal
      mGoalState.position = mPeriodicIO.elevator_target;

      // Calculate new state
      prevUpdateTime = curTime;
      mCurState = mProfile.calculate(dt, mCurState, mGoalState);

      // Set PID controller to new state
      mLeftPIDController.setReference(
          mCurState.position,
          SparkBase.ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          Constants.Elevator.kG,
          ArbFFUnits.kVoltage);
    } else {
      mCurState.position = mLeftEncoder.getPosition();
      mCurState.velocity = 0;
      mLeftMotor.set(mPeriodicIO.elevator_power);
    }

    publicState = mPeriodicIO.state;

  }

  public void writePeriodicOutputs() {
    //useless ah function
  }

  public void stop() {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;

    mLeftMotor.set(0.0);
  }


  public void outputTelemetry() {
    // putNumber("Position/Current", mLeftEncoder.getPosition());
    // putNumber("Position/Target", mPeriodicIO.elevator_target);
    // putNumber("Velocity/Current", mLeftEncoder.getVelocity());

    // putNumber("Position/Setpoint", mCurState.position);
    // putNumber("Velocity/Setpoint", mCurState.velocity);

    // putNumber("Current/Left", mLeftMotor.getOutputCurrent());
    // putNumber("Current/Right", mRightMotor.getOutputCurrent());

    // putNumber("Output/Left", mLeftMotor.getAppliedOutput());
    // putNumber("Output/Right", mRightMotor.getAppliedOutput());

    // putNumber("State", mPeriodicIO.state);
  }
  
  public Command reset() {
    return run(() -> mLeftEncoder.setPosition(0.0));
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public Command getState() {
    return run(() -> getstate());
  }

  private ElevatorState getstate() {
    return mPeriodicIO.state;
  }

  public Command setElevatorPower(double power) {
    return run(() -> setelevatorpower(power));
  }

  private void setelevatorpower(double power) {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = power;
  }

  public Command goToElevatorStow() {
    return run(() -> gotoelevatorstow());
    
  }

  private void gotoelevatorstow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kStowHeight;
    mPeriodicIO.state = ElevatorState.STOW;
  }

  public Command goToElevatorL2() {
    return run(() -> gotoelevatorl2());
  }

  private void gotoelevatorl2() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kL2Height;
    mPeriodicIO.state = ElevatorState.L2;
    System.out.println("Elevator moving to Level 2 at night");
  }

  public Command goToElevatorL3() {
    return run(() -> gotoelevatorl3());
  }

  private void gotoelevatorl3() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kL3Height;
    mPeriodicIO.state = ElevatorState.L3;
  }

  public Command goToElevatorL4() {
    return run(() -> gotoelevatorl4());
  }

  private void gotoelevatorl4() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kL4Height;
    mPeriodicIO.state = ElevatorState.L4;
  }

  public Command goToAlgaeLow() {
    return run(() -> gotoalgaelow());
  }

  private void gotoalgaelow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kLowAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A1;
  }

  public Command goToAlgaeHigh() {
    return run(() -> gotoalgaehigh());
  }

  private void gotoalgaehigh() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kHighAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A2;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}