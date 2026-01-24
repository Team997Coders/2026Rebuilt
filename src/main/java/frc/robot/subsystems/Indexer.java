package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase{

   private final SparkMax indexMotor = new SparkMax(Constants.indexerMotorID, MotorType.kBrushless);
   
   public Indexer() {

   }

   public void spinIndexerMotor(double voltage){
    indexMotor.setVoltage(voltage);
   } 

   public Command startIdexCommand(){
      return this.run(() -> spinIndexerMotor(1));
   }

}
