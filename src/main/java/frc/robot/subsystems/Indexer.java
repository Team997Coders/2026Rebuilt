package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase{

   private final SparkMax indexMotor; 


   public Indexer() {
   indexMotor = new SparkMax(Constants.IndexerConstants.indexerMotorID, MotorType.kBrushless);

   }

   public void spinIndexerMotor(double voltage){
    indexMotor.setVoltage(voltage);
   } 

   public void setIndexerMotor(double setpoint)
   {
      indexMotor.set(setpoint);
   }

   public Command startIndexer(){
      return this.run(() -> spinIndexerMotor(Constants.IndexerConstants.defaultVolts));
   }

   public Command reverseIndexer() {
      return this.run(() -> spinIndexerMotor(Constants.IndexerConstants.reverseVolts));
   }

   public Command stopIndexer() {
      return this.runOnce(() -> spinIndexerMotor(0));
   }

   public double getIndexVoltage(){
      return indexMotor.getAppliedOutput() * indexMotor.getBusVoltage();
   }

   public boolean unstickFuel(){
      if (indexMotor.getOutputCurrent() > Constants.IndexerConstants.typicalIndexOutputCurrent) {
         return true;
      } else {
         return false;
      }
   }
}
