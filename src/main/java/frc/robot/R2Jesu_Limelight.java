package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

public class R2Jesu_Limelight {

    private NetworkTable limelight_Table;
    double currentDistance;
    double targetOffsetAngle_Vertical;
    double ourDist;
    
    public R2Jesu_Limelight() {
        limelight_Table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void postDashboard() {
        SmartDashboard.putNumber("LimelightX", limelight_Table.getEntry("tx").getDouble(0.0));
        SmartDashboard.putNumber("LimelightY", limelight_Table.getEntry("ty").getDouble(0.0));
        SmartDashboard.putNumber("LimelightArea", limelight_Table.getEntry("ta").getDouble(0.0));
        
    }

    public double getTX() {
        return limelight_Table.getEntry("tx").getDouble(0.0);
    }

    public double getDistance() {
        //(Target height - camera height) / tan((camera angle + target offset angle from limelight)) * (PI / 180)))
        targetOffsetAngle_Vertical = limelight_Table.getEntry("ty").getDouble(0.0);
        ourDist = (((double)57.5 - (double)23.125) / (Math.tan(((double)23.12 + targetOffsetAngle_Vertical) * (Math.PI / 180.00))));
        currentDistance = ourDist;
        if (targetOffsetAngle_Vertical == 0.0){
            currentDistance = 0.0;
        }
        SmartDashboard.putNumber("current distance", currentDistance);
        return currentDistance;
    }
}