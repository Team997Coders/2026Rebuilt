package frc.robot.subsystems;
import frc.robot.subsystems.Drivebase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;

public class Bump extends SubsystemBase {

    //note to self: add the gyro and use to detect when robot is going over bump and speed up motors to give boost

    private Drivebase drivebase;
    private Canandgyro gyro;

    public Bump(Drivebase m_drivebase, Canandgyro m_gyro) {

        this.m_drivebase = drivebase;
        this.m_gyro = gyro;
    }

    
    
}


