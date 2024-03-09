package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.generated.Constants.LimeLightConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase{

private final NetworkTable limelight_table;
private double tx, ty, ta;
private double distance;

public VisionSubsystem() {
    limelight_table = NetworkTableInstance.getDefault().getTable("limelight-lunas");
}

@Override
public void periodic() {
    tx = limelight_table.getEntry("tx").getDouble(0);
    ty = limelight_table.getEntry("ty").getDouble(0);
    ta = limelight_table.getEntry("ta").getDouble(0);

    distance = this.getDistance();

    SmartDashboard.putNumber("LimelightX", tx);
    SmartDashboard.putNumber("LimelightY", ty);
    SmartDashboard.putNumber("LimelightArea", ta);
    SmartDashboard.putNumber("distance", distance);
}

public double getTY () {
    return ty;
}

public double getDistance()
{
  double targetOffsetAngle_Vertical = ty;
  double angleToGoalDegrees = LimeLightConstants.limelightMountAngledegrees + targetOffsetAngle_Vertical;
  double angletoGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
  double distanceFromLimelightGoalInches = (
    LimeLightConstants.goalHeightInches - LimeLightConstants.limelightLensHeightInches) / 
    Math.tan(angletoGoalRadians);
  return distanceFromLimelightGoalInches;
}

}

