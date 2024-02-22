package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.ElevatorConstants;
import frc.robot.generated.Constants.ShooterConstants;;

public class ElevatorSubsystem extends SubsystemBase{

private CANSparkFlex m_leftElevator, m_rightElevator;
private SparkPIDController l_pidController,r_pidController;
private RelativeEncoder l_encoder,r_encoder;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


public ElevatorSubsystem()
{
  m_rightElevator = new CANSparkFlex(ElevatorConstants.rightElevator, MotorType.kBrushless);
  m_leftElevator = new CANSparkFlex(ElevatorConstants.leftElevator, MotorType.kBrushless);
  m_rightElevator.restoreFactoryDefaults();
  m_leftElevator.restoreFactoryDefaults();

  /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
  r_pidController = m_rightElevator.getPIDController();
  l_pidController = m_leftElevator.getPIDController();

   // Encoder object created to display position values
   l_encoder = m_leftElevator.getEncoder();
   r_encoder = m_rightElevator.getEncoder();

   // PID coefficients
   kP = 6e-5; 
   kI = 0;
   kD = 0; 
   kIz = 0; 
   kFF = 0.000015; 
   kMaxOutput = 1; 
   kMinOutput = -1;
   maxRPM = 5700;

   // set PID coefficients
   r_pidController.setP(kP);
   r_pidController.setI(kI);
   r_pidController.setD(kD);
   r_pidController.setIZone(kIz);
   r_pidController.setFF(kFF);
   r_pidController.setOutputRange(kMinOutput, kMaxOutput);

   // set PID coefficients
   l_pidController.setP(kP);
   l_pidController.setI(kI);
   l_pidController.setD(kD);
   l_pidController.setIZone(kIz);
   l_pidController.setFF(kFF);
   l_pidController.setOutputRange(kMinOutput, kMaxOutput);

}

private void setVelocity(double setPoint)
{
  r_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  l_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
}

private void setPosition(double setPoint)
{
  r_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  l_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
}

private void setVelocitydiff(double tsetPoint,double bsetPoint)
{
  r_pidController.setReference(tsetPoint, CANSparkMax.ControlType.kVelocity);
  l_pidController.setReference(bsetPoint, CANSparkMax.ControlType.kVelocity);
}

public Command withVelocity(double setPoint)
{
  return runOnce(() -> this.setVelocity(setPoint));
}

public Command highSpeed()
{
  return runOnce(() -> this.setVelocity(80));
}


public Command lowSpeed()
{
  return runOnce(() -> this.setVelocity(20));
}


public Command stop()
{
  return runOnce(() -> this.setVelocity(0));
}


public Command withPosition(double setPoint)
{
  return runOnce(() -> this.setPosition(setPoint));
}


public Command setHomePosition()
{
  return runOnce(() -> this.setPosition(0)); //need to find
}


public Command setUpPosition()
{
  return runOnce(() -> this.setPosition(50)); // need to find
}


public Command setClimbPosition()
{
  return runOnce(() -> this.setPosition(20)); // need to find
}

}
