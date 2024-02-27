package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class R2Jesu_Limelight {

    private NetworkTable limelight_Table;
    
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
}