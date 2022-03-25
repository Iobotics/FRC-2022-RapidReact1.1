/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /**
   * Creates a new Limelight.
   */
  
  private  double x = 0;
  private  double y = 0;
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  public Limelight() {
    
    table = NetworkTableInstance.getDefault().getTable("limelight"); //setting up network tables for limelight
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");


    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);


  }
  
  public double getTX(){ //returns distance in degrees from the target
    x = tx.getDouble(0.0);
    return x;
  }
  
  public double getTY(){ //returns distance in degrees from the target
    y = ty.getDouble(0.0);
    return y;
  }


  public void outputs(){
  SmartDashboard.putNumber("LimelightX", tx.getDouble(0.0));
  SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));
  SmartDashboard.putNumber("LimelightArea", ta.getDouble(0.0));
    //Ignore this part.
    //  SmartDashboard.putBoolean("Limelight TV", this.isTargetDetected());
  }

  

  @Override
  public void periodic() {


      SmartDashboard.putNumber("LimelightX", tx.getDouble(0.0));
      SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));
      SmartDashboard.putNumber("LimelightArea", ta.getDouble(0.0));
      //Ignore this part in the future
      //SmartDashboard.putBoolean("Limelight TV", this.isTargetDetected());

      Boolean onTarget;
      if (5>tx.getDouble(0.0) && -5<tx.getDouble(0.0) && 5>ty.getDouble(0.0) && -5<ty.getDouble(0.0)){
        onTarget = true;
      } else{
        onTarget = false;
      }

      Boolean topTarget;
      if (5>ty.getDouble(0.0) && ty.getDouble(0.0)!= 0 ){
        topTarget=true;
      } else{
        topTarget=false;
      }     

      Boolean bottomTarget;
      if (-5<ty.getDouble(0.0) && ty.getDouble(0.0)!= 0){
        bottomTarget=true;
      } else{
        bottomTarget=false;
      }    

      Boolean rightTarget;
      if (5>tx.getDouble(0.0)  && tx.getDouble(0.0)!= 0 ){
        rightTarget=true;
      } else{
        rightTarget=false;
      }          

      Boolean leftTarget;
      if (-5<tx.getDouble(0.0) && tx.getDouble(0.0)!= 0){
        leftTarget=true;
      } else{
        leftTarget=false;
      }     


      SmartDashboard.putBoolean("On Target?", onTarget);
      SmartDashboard.putBoolean("Above", topTarget);
      SmartDashboard.putBoolean("Right", rightTarget);
      SmartDashboard.putBoolean("Left", leftTarget);
      SmartDashboard.putBoolean("Bottom", bottomTarget);
    
  }
}