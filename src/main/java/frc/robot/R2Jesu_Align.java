package frc.robot;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import java.lang.Math;

public class R2Jesu_Align {
    // last year we used pid controller, used alignPpid (and d and i)
    // PIDController​(double kp, double ki, double kd, double period) is on the docs
    // I put the same values as last year, may change idk
     //April Tag Align
    boolean localAlign;
    double alignPpid = 0.047; //.047
    double alignIpid = 0.005; // .001
    double alignDpid = 0.0148; // .0148
    double alignPidOutput = 0.0;
    private PIDController m_alignController = new PIDController(alignPpid, alignIpid, alignDpid, 0.05);
    double aTurn2Ppid = 0.042;
    double aTurn2Ipid = 0.00;
    double aTurn2Dpid = 0.00;
    double aTurn2PidOutput = 0.0;
    private PIDController m_aTurn2Controller = new PIDController(aTurn2Ppid, aTurn2Ipid, aTurn2Dpid, 0.02);
    //NavX
    //AHRS *ahrs;
    

    //Align
    double facingError;
    double facingCorrection;
    double alignYaw;
    double aprilError;
    double aprilCorrection;
    double aprilID = 0.0;

    public boolean align(double targetYaw, R2Jesu_Drive alignDrive, R2Jesu_Limelight alignLimelight, AHRS alignAhrs) {
        localAlign = true;
    aprilCorrection = 0.0;
    aTurn2PidOutput = 0.0;
    //if ((ahrs->GetYaw() > -177.0) && (ahrs->GetYaw() < 177.0))
    if (Math.abs(alignAhrs.getYaw()) < (targetYaw - 3) || Math.abs(alignAhrs.getYaw()) > (targetYaw + 3))
    {
        alignYaw = alignAhrs.getYaw();
        aTurn2PidOutput = m_aTurn2Controller.calculate(Math.abs(alignYaw), targetYaw);
        if (alignYaw < 0)
        {
            aTurn2PidOutput = aTurn2PidOutput * -1.0;
        }
        localAlign = false;
        alignDrive.drive(0.0, 0.0, aTurn2PidOutput, alignAhrs);
    }
    //if (((limelight_Table->GetNumber("tx",0.0) < -1.5) || (limelight_Table->GetNumber("tx",0.0) > 1.5)) && !((ahrs->GetYaw() > -177.0) && (ahrs->GetYaw() < 177.0)))
    if (((alignLimelight.getTX() < -1.5) || (alignLimelight.getTX() > 1.5)) && !(Math.abs(alignAhrs.getYaw()) < (targetYaw - 3) || Math.abs(alignAhrs.getYaw()) > (targetYaw + 3)))
    {
        aprilError = alignLimelight.getTX();
        aprilCorrection = m_alignController.calculate(aprilError, 0.0);
        localAlign = false;
        if (targetYaw == 0.0) {
            alignDrive.drive((aprilCorrection * -1.0), 0.0, 0.0, alignAhrs);
        }
        else {
            alignDrive.drive(aprilCorrection, 0.0, 0.0, alignAhrs);
        }
    }
    return localAlign;
    }

}
