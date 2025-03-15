package frc.robot.commands;

import java.util.function.DoubleSupplier;
import lib.frc706.cyberlib.subsystems.LimelightHelpers;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import java.lang.Thread;

public class AButtonPrintCommand extends Command{

    private final SwerveSubsystem swerveSubsystem;

    public AButtonPrintCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem; 
        System.out.println("A Button Command Constructor");
    }

    @Override
    public void initialize(){
        
        // int[] validAprilTags = {4};
        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-three",validAprilTags);
        double tx = LimelightHelpers.getTX("limelight-three");
        System.out.println("Init TX = "  + (tx));
        boolean tv = LimelightHelpers.getTV("limelight-three");
        double xSpeed = (tx/50);
        System.out.println(("xSpeed =") + (xSpeed));
    
            
        while (tv == true && Math.abs(tx) > 0.1) {
            System.out.println("debug 0");
            
                System.out.println("debug 1");
                // start to drive
                this.swerveSubsystem.driveRobotOriented
                (this.swerveSubsystem.swerveDrive.swerveController.getRawTargetSpeeds(
                    0.0,-xSpeed,0.0));
                System.out.println("debug 2");
                try {
                    // sleep to let it drive for a while
                    long sleepTime = (long)(tx*tx/5);
                    System.out.println("sleeping for " + (sleepTime));
                    Thread.sleep(sleepTime);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                this.swerveSubsystem.driveRobotOriented
                (this.swerveSubsystem.swerveDrive.swerveController.getRawTargetSpeeds(
                0.0,0.0,0.0));
        
        
                tx = LimelightHelpers.getTX("limelight-three");
                tv = LimelightHelpers.getTV("limelight-three");
                xSpeed = (tx/50);
                System.out.println("Loop TX = "  + (tx));
            }

    }
            

        
        

    @Override
    public void execute() {
        
        
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    
}
            
            //if (loopbreak == true) {
            //}



        
            //else {
                // System.out.println("No valid apriltags");
            //  }

        // if  (tv == true) {
            
            //double aButtonGetTurnRate= this.swerveSubsystem.swerveDrive.getTurnRate();
            //System.out.println(("RobotTurnRate = ") + (aButtonGetTurnRate));


            // double aButtonRobotRelativeSpeeds = this.swerveSubsystem.getRobotRelativeSpeeds();
            // System.out.println(("relativeSpeeds = ") + (aButtonRobotRelativeSpeeds));


            // double tx = LimelightHelpers.getTX("limelight-three");
            // double aButtonHeading = this.swerveSubsystem.getHeading();
            // System.out.println(("TX Value =")+(tx));
            // System.out.println(("Heading = ") + (aButtonHeading));
            
    //     }
        
    //     else {
    //         System.out.println("No Valid AprilTags!");
    //     }