// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkLimitSwitch;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class LaserTest extends SubsystemBase {
//     private SparkMax beamBreakController;
//     private SparkLimitSwitch beamBreak;

//     public LaserTest(){
//         beamBreakController = new SparkMax(9, MotorType.kBrushless);
//         beamBreak = beamBreakController.getForwardLimitSwitch();
//     }


//     @Override
//     public void periodic() {
//         System.out.println(beamBreak.isPressed());
//         super.periodic();
//     }
// }
