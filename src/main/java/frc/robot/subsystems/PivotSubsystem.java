package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.PivotConstants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.Constants.LimeLightConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PivotSubsystem extends SubsystemBase{

private CANSparkMax m_pviot;
private SparkPIDController m_pidController;
private RelativeEncoder m_encoder;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


NetworkTable table2 = NetworkTableInstance.getDefault().getTable("limelight-lunas");
NetworkTable table1 = NetworkTableInstance.getDefault().getTable("limelight-rear");
NetworkTableInstance nTableInstance = NetworkTableInstance.getDefault();
LimelightHelpers limelightHelpers = new LimelightHelpers();

NetworkTableEntry tx2 = table1.getEntry("tx");
NetworkTableEntry ty2 = table1.getEntry("ty");
NetworkTableEntry ta2 = table1.getEntry("ta");

double x2 = tx2.getDouble(0);
double y2 = ty2.getDouble(0);
double area2 = ta2.getDouble(0);


public PivotSubsystem()
{
  m_pviot = new CANSparkMax(PivotConstants.pivot, MotorType.kBrushless);
  m_pviot.restoreFactoryDefaults();
  /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
  m_pidController = m_pviot.getPIDController();

   // Encoder object created to display position values
   m_encoder = m_pviot.getEncoder();

   // PID coefficients
   kP = 0.1; 
   kI = 1e-4;
   kD = 1; 
   kIz = 0; 
   kFF = 0; 
   kMaxOutput = 1; 
   kMinOutput = -1;

   // set PID coefficients
   m_pidController.setP(kP);
   m_pidController.setI(kI);
   m_pidController.setD(kD);
   m_pidController.setIZone(kIz);
   m_pidController.setFF(kFF);
   m_pidController.setOutputRange(kMinOutput, kMaxOutput);

   //Limelight SetUp
   
}

private void setVelocity(double setPoint)
{
  m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
}
 
@Override
public void periodic(){

  double targetOffsetAngle_Vertical = y2;
  double angleToGoalDegrees = LimeLightConstants.limelightMountAngledegrees + targetOffsetAngle_Vertical;
  double angletoGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
  double distanceFromLimelightGoalInches = (
  LimeLightConstants.goalHeightInches - LimeLightConstants.limelightLensHeightInches) / 
  Math.tan(angletoGoalRadians);

  SmartDashboard.putNumber("Pivot Encoder", m_encoder.getPosition());
  SmartDashboard.putNumber("LimelightX", x2);
  SmartDashboard.putNumber("LimelightY", y2);
  SmartDashboard.putNumber("LimelightArea", area2);
  SmartDashboard.putNumber("limelightDistance", distanceFromLimelightGoalInches);
  SmartDashboard.putNumber("AngleToGoalRadians", angletoGoalRadians);
   
}

private void setPosition(double setPoint)
{
  m_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
}

public Command withCalculatedPosition(double distance)
{
  //height / distance
  //feet
  double c_setPoint = Math.atan( 10 / distance);
  return runOnce(() -> this.setPosition(c_setPoint));
}

public Command withPosition(double setPoint)
{
  return runOnce(() -> this.setPosition(setPoint));
}

public Command high()
{
  return runOnce(() -> this.setPosition(80));
}

public Command mid()
{
  return runOnce(() -> this.setPosition(50));
}

public Command low()
{
  return runOnce(() -> this.setPosition(20));
}


public Command lowSpeedUp()
{
  return runOnce(() -> this.setVelocity(20));
}


public Command lowSpeedDown()
{
  return runOnce(() -> this.setVelocity(-20));
}


public Command stop()
{
  return runOnce(() -> this.setVelocity(0));
}

public double getDistance()
{
  double targetOffsetAngle_Vertical = y2;
  double angleToGoalDegrees = LimeLightConstants.limelightMountAngledegrees + targetOffsetAngle_Vertical;
  double angletoGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
  double distanceFromLimelightGoalInches = (
    LimeLightConstants.goalHeightInches - LimeLightConstants.limelightLensHeightInches) / 
    Math.tan(angletoGoalRadians);
  return distanceFromLimelightGoalInches;
}

public Command AutoAim(

  double goalHeightInches, 
  double heightOfShooter,  
  double distanceFromLimelightGoalInches,
  double limelightLensHeightInches, 
  double distanceFromShooter) {

  return run(() -> m_pviot.getEncoder().setPosition((180/3.14159)*(Math.atan(
    (goalHeightInches - limelightLensHeightInches + heightOfShooter) / 
    (distanceFromLimelightGoalInches + distanceFromShooter))
    )));
}


}
