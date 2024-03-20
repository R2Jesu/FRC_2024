package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Compressor;

public class R2Jesu_Hanger {
    private Compressor hangerCompressor = new Compressor(22, PneumaticsModuleType.CTREPCM);
    private DoubleSolenoid hangerPneumatics = new DoubleSolenoid(22, PneumaticsModuleType.CTREPCM, 0, 1);
    //open close pneumatics from 2022

    public R2Jesu_Hanger() {
        hangerCompressor.enableDigital();
    }
    
    public void hang(XboxController hangerController) {

        if (hangerController.getRightTriggerAxis() > 0.0) {
            hangerPneumatics.set(DoubleSolenoid.Value.kForward);
        }
        else {
            hangerPneumatics.set(DoubleSolenoid.Value.kReverse);
        }

    }
}