package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.generated.Constants.IntakeConstants;
import frc.robot.generated.Constants.ShooterConstants;;

public class ShooterSubsystem extends SubsystemBase{

private CANSparkFlex m_topshooter,m_bottomshooter;
private SparkPIDController t_pidController,b_pidController;
private RelativeEncoder b_encoder,t_encoder;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


public ShooterSubsystem()
{
  m_topshooter = new CANSparkFlex(ShooterConstants.shooterTOP, MotorType.kBrushless);
  m_bottomshooter = new CANSparkFlex(ShooterConstants.shooterBOTTOM, MotorType.kBrushless);
  m_topshooter.restoreFactoryDefaults();
  m_bottomshooter.restoreFactoryDefaults();

  /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
  t_pidController = m_topshooter.getPIDController();
  b_pidController = m_bottomshooter.getPIDController();

   // Encoder object created to display position values
   t_encoder = m_topshooter.getEncoder();
   b_encoder = m_bottomshooter.getEncoder();

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
   t_pidController.setP(kP);
   t_pidController.setI(kI);
   t_pidController.setD(kD);
   t_pidController.setIZone(kIz);
   t_pidController.setFF(kFF);
   t_pidController.setOutputRange(kMinOutput, kMaxOutput);

   // set PID coefficients
   b_pidController.setP(kP);
   b_pidController.setI(kI);
   b_pidController.setD(kD);
   b_pidController.setIZone(kIz);
   b_pidController.setFF(kFF);
   b_pidController.setOutputRange(kMinOutput, kMaxOutput);

}

private void setVelocity(double setPoint)
{
  t_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  b_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
}

private void setVelocitydiff(double tsetPoint,double bsetPoint)
{
  t_pidController.setReference(tsetPoint, CANSparkMax.ControlType.kVelocity);
  b_pidController.setReference(bsetPoint, CANSparkMax.ControlType.kVelocity);
}

public Command withVelocity(double setPoint)
{
  return runOnce(() -> this.setVelocity(setPoint));
}

public Command highSpeed()
{
  return runOnce(() -> this.setVelocity(80));
}

public Command medSpeed()
{
  return runOnce(() -> this.setVelocity(50));
}

public Command lowSpeed()
{
  return runOnce(() -> this.setVelocity(20));
}

public Command ampSpeed()
{
  return runOnce(() -> this.setVelocitydiff(50,20));
}

public Command stop()
{
  return runOnce(() -> this.setVelocity(0));
}


}
