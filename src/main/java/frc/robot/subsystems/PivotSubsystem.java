package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.generated.Constants.ShooterConstants;;

public class PivotSubsystem extends SubsystemBase{

private CANSparkFlex m_pviot;
private SparkPIDController m_pidController;
private RelativeEncoder m_encoder;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


public PivotSubsystem()
{
  m_pviot = new CANSparkFlex(ShooterConstants.shooterTOP, MotorType.kBrushless);
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
}

private void setVelocity(double setPoint)
{
  m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
}
 
@Override
public void periodic(){
  SmartDashboard.putNumber("Pivot Encoder", m_encoder.getPosition());
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


}
