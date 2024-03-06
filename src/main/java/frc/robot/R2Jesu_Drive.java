package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

public class R2Jesu_Drive {

  // swerve 1
  private CANSparkMax m_SwerveDrive1 = new CANSparkMax(5, MotorType.kBrushless);
  private RelativeEncoder m_DriveEncoder1 = m_SwerveDrive1.getEncoder();
  private WPI_VictorSPX m_SwerveTurn1 = new WPI_VictorSPX(1);
  private AnalogInput m_SwerveAnalog1 = new AnalogInput(0);

  // swerve 2
  private CANSparkMax m_SwerveDrive2 = new CANSparkMax(6, MotorType.kBrushless);
  private RelativeEncoder m_DriveEncoder2 = m_SwerveDrive2.getEncoder();
  private WPI_VictorSPX m_SwerveTurn2 = new WPI_VictorSPX(2);
  private AnalogInput m_SwerveAnalog2 = new AnalogInput(1);

  // swerve 3
  private CANSparkMax m_SwerveDrive3 = new CANSparkMax(7, MotorType.kBrushless);
  private RelativeEncoder m_DriveEncoder3 = m_SwerveDrive3.getEncoder();
  private WPI_TalonSRX m_SwerveTurn3 = new WPI_TalonSRX(3);
  private AnalogInput m_SwerveAnalog3 = new AnalogInput(2);

  // swerve 4
  private CANSparkMax m_SwerveDrive4 = new CANSparkMax(8, MotorType.kBrushless);
  private RelativeEncoder m_DriveEncoder4 = m_SwerveDrive4.getEncoder();
  private WPI_TalonSRX m_SwerveTurn4 = new WPI_TalonSRX(4);
  private AnalogInput m_SwerveAnalog4 = new AnalogInput(3);

  private double inputAngle = 0.0;
  private double r = 0.0;
  private double fieldOrientedAngle = 0.0;
  private double newX = 0.0;
  private double newY = 0.0;
  private double A = 0.0;
  private double B = 0.0;
  private double C = 0.0;
  private double D = 0.0;
  private double LENGTH = 17.375;
  private double WIDTH = 21.25;
  private double R = Math.sqrt((LENGTH*LENGTH) + (WIDTH*WIDTH));
  private double fullSpeed = .3;
  private double turnSpeed = .2;
  private double speedChoice;
  private double wSpeed1=0.0;
  private double wAngle1=0.0;
  private double wSpeed2=0.0;
  private double wAngle2=0.0;
  private double wSpeed3=0.0;
  private double wAngle3=0.0;
  private double wSpeed4=0.0;
  private double wAngle4=0.0;
  private double Ppid = 0.050;//45;
  private double Ipid = 0.000;
  private double Dpid = 0.001;//0.0008;//.0005
  private double pidOutput1 = 0.0;
  private double pidOutput2 = 0.0;
  private double pidOutput3 = 0.0;
  private double pidOutput4 = 0.0;
  private PIDController m_angleController1 = new PIDController(Ppid, Ipid, Dpid);
  private PIDController m_angleController2 = new PIDController(Ppid, Ipid, Dpid);
  private PIDController m_angleController3 = new PIDController(Ppid, Ipid, Dpid);
  private PIDController m_angleController4 = new PIDController(Ppid, Ipid, Dpid);
  private double conversion1 = 360.0/3.3;
  private double conversion2 = 360.0/3.3;
  private double conversion3 = 360.0/3.3;
  private double conversion4 = 360.0/3.3;
  private AHRS ahrsDrive;
  private double autoSpeed = .15;

  public R2Jesu_Drive(AHRS x) {
    ahrsDrive = x;
  }


  public void drive(double x, double y, double z) {
    y = y * -1.0;
    
    if (Math.abs(x) < 0.1)
    {
        x = 0.0;
    }
    if (Math.abs(y) < 0.1)
    {
        y = 0.0;
    }
    if (Math.abs(z) < 0.1)
    {
        z = 0.0;
    }

    inputAngle = Math.atan2(y, x) * 180.0/Math.PI;
    r = Math.sqrt((x*x) + (y*y));
    fieldOrientedAngle = ahrsDrive.getYaw() + inputAngle;
    newX = r * (Math.cos(fieldOrientedAngle * Math.PI/180.0));
    newY = r * (Math.sin(fieldOrientedAngle * Math.PI/180.0));
    A = newY - z*(LENGTH/R);
    B = newY + z*(LENGTH/R);
    C = newX - z*(WIDTH/R);
    D = newX + z*(WIDTH/R);

    if (z != 0.0)
    {
        speedChoice = turnSpeed;
    } else
    {
        speedChoice = fullSpeed;
    }

    wSpeed1 = speedChoice *  (Math.sqrt(B*B + C*C));
	wAngle1 = Math.atan2(B,C) * 180.0/Math.PI; 
    if (wAngle1 < 0.0)
    {
        wAngle1 = wAngle1 + 360.0;
    }
    if (wAngle1 >= 185.0)
    {
        wAngle1 = wAngle1 - 180.0;
        wSpeed1 = -1.0 * wSpeed1;
    }

	wSpeed2 = speedChoice *  (Math.sqrt(B*B + D*D));
	wAngle2 = Math.atan2(B,D) * 180.0/Math.PI;
    if (wAngle2 < 0.0)
    {
        wAngle2 = wAngle2 + 360.0;
    }
    if (wAngle2 >= 185.0)
    {
        wAngle2 = wAngle2 - 180.0;
        wSpeed2 = -1.0 * wSpeed2;
    }

	wSpeed3 = speedChoice* (Math.sqrt(A*A + D*D));
	wAngle3 = Math.atan2(A,D) * 180.0/Math.PI;
    if (wAngle3 < 0.0)
    {
        wAngle3 = wAngle3 + 360.0;
    }
    if (wAngle3 >= 185.0)
    {
        wAngle3 = wAngle3 - 180.0;
        wSpeed3 = -1.0 * wSpeed3;
    }

	wSpeed4 = speedChoice*(Math.sqrt(A*A + C*C));
	wAngle4 = Math.atan2(A,C) * 180.0/Math.PI;
    if (wAngle4 < 0.0)
    {
        wAngle4 = wAngle4 + 360.0;
    }
    if (wAngle4 >= 185.0)
    {
        wAngle4 = wAngle4 - 180.0;
        wSpeed4 = -1.0 * wSpeed4;
    }



if (Math.abs(x) > 0.1 || Math.abs(y) > 0.1 || Math.abs(z) > 0.1)
{
    
    m_SwerveDrive1.set(wSpeed1);
    pidOutput1 = m_angleController1.calculate((m_SwerveAnalog1.getVoltage() * conversion1), wAngle1);
    m_SwerveTurn1.set(pidOutput1);

    m_SwerveDrive2.set(wSpeed2);
    pidOutput2 = m_angleController2.calculate((m_SwerveAnalog2.getVoltage() * conversion2), wAngle2);
    m_SwerveTurn2.set(pidOutput2);

    m_SwerveDrive3.set(wSpeed3);
    pidOutput3 = m_angleController3.calculate((m_SwerveAnalog3.getVoltage() * conversion3), wAngle3);
    m_SwerveTurn3.set(pidOutput3);

    m_SwerveDrive4.set(wSpeed4);
    pidOutput4 = m_angleController4.calculate((m_SwerveAnalog4.getVoltage() * conversion4), wAngle4);
    m_SwerveTurn4.set(pidOutput4);
}

else
{
    m_SwerveDrive1.set(0.0);
    m_SwerveTurn1.set(0.0);

    m_SwerveDrive2.set(0.0);
    m_SwerveTurn2.set(0.0);

    m_SwerveDrive3.set(0.0);
    m_SwerveTurn3.set(0.0);

    m_SwerveDrive4.set(0.0);
    m_SwerveTurn4.set(0.0);
}
SmartDashboard.putNumber("NavX", ahrsDrive.getYaw());
SmartDashboard.putNumber("Pitch", ahrsDrive.getPitch());
SmartDashboard.putNumber("Wheel 1 Voltage", m_SwerveAnalog1.getVoltage());
SmartDashboard.putNumber("Wheel 1 Angle", (m_SwerveAnalog1.getVoltage()*conversion1));
SmartDashboard.putNumber("Wheel 2 Voltage", m_SwerveAnalog2.getVoltage());
SmartDashboard.putNumber("Wheel 2 Angle", (m_SwerveAnalog2.getVoltage()*conversion2));
SmartDashboard.putNumber("Wheel 3 Voltage", m_SwerveAnalog3.getVoltage());
SmartDashboard.putNumber("Wheel 3 Angle", (m_SwerveAnalog3.getVoltage()*conversion3));
SmartDashboard.putNumber("Wheel 4 Voltage", m_SwerveAnalog4.getVoltage());
SmartDashboard.putNumber("Wheel 4 Angle", (m_SwerveAnalog4.getVoltage()*conversion4));
SmartDashboard.putNumber("X", x);
SmartDashboard.putNumber("Y", y);
SmartDashboard.putNumber("Z", z);
SmartDashboard.putNumber("Wheel 1 Speed", wSpeed1);
SmartDashboard.putNumber("Wheel 2 Speed", wSpeed2);
SmartDashboard.putNumber("Wheel 3 Speed", wSpeed3);
SmartDashboard.putNumber("Wheel 4 Speed", wSpeed4);

  }

  public void driveAuto() {
      while (m_DriveEncoder1.getPosition() < 43.0 && DriverStation.isAutonomousEnabled())
    {
        m_SwerveDrive1.set(autoSpeed);
        pidOutput1 = m_angleController1.calculate((m_SwerveAnalog1.getVoltage() * conversion1), 90.0);
        m_SwerveTurn1.set(pidOutput1);

        m_SwerveDrive2.set(autoSpeed);
        pidOutput2 = m_angleController2.calculate((m_SwerveAnalog2.getVoltage() * conversion2), 90.0);
        m_SwerveTurn2.set(pidOutput2);

        m_SwerveDrive3.set(autoSpeed);
        pidOutput3 = m_angleController3.calculate((m_SwerveAnalog3.getVoltage() * conversion3), 90.0);
        m_SwerveTurn3.set(pidOutput3);

        m_SwerveDrive4.set(autoSpeed);
        pidOutput4 = m_angleController4.calculate((m_SwerveAnalog4.getVoltage() * conversion4), 90.0);
        m_SwerveTurn4.set(pidOutput4);
        //frc::SmartDashboard::PutNumber("position", m_DriveEncoder1.GetPosition());
    }
    m_SwerveDrive1.set(0.0);
    m_SwerveDrive2.set(0.0);
    m_SwerveDrive3.set(0.0);
    m_SwerveDrive4.set(0.0);
    m_SwerveTurn1.set(0.0);
    m_SwerveTurn2.set(0.0);
    m_SwerveTurn3.set(0.0);
    m_SwerveTurn4.set(0.0);
    m_DriveEncoder1.setPosition(0.0);
    m_DriveEncoder2.setPosition(0.0);
    m_DriveEncoder3.setPosition(0.0);
    m_DriveEncoder4.setPosition(0.0);

  }
  }


