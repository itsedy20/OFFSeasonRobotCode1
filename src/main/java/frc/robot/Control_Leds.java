// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/** Add your docs here. */
public class Control_Leds {
    
    I2C arduino = new I2C(Port.kOnboard, 10);
    
    /**
     * 0 -> Idle<P>
     * 1 -> Yellow Flash<P>
     * 2 -> Green Flash<P>
     * 3 -> Cylon Mode<P>
     * @param data Valor a enviar al arduino
     */
    public void write(int data){
        byte[] datas = {(byte) data};
        try {
            arduino.writeBulk(datas);
        } catch (Exception e) {
            DriverStation.reportError("Arduino I2C error", null);
        }
    }
}
