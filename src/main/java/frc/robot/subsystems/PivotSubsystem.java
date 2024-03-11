package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.PivotConstants;
import frc.robot.generated.Constants.LimeLightConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PivotSubsystem extends SubsystemBase{

private CANSparkMax m_pviot;
private SparkPIDController m_pidController;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

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

public Command lightlightAutoAim (double distance) {
  return run(() -> this.setPosition(((distance - 36.125)*0.1194) +24.96));
}

}
