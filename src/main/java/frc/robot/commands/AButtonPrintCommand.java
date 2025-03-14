package frc.robot.commands;

import java.util.function.DoubleSupplier;
import lib.frc706.cyberlib.subsystems.LimelightHelpers;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import java.lang.Thread;

//getRobotVelocity getYaw
    public class AButtonPrintCommand extends Command{

        private final SwerveSubsystem swerveSubsystem;

        public AButtonPrintCommand(SwerveSubsystem swerveSubsystem) {
            this.swerveSubsystem = swerveSubsystem; 
            System.out.println("A Button Command Constructor");
        }

    boolean loopbreak;
        
        @Override
        public void initialize(){
            
            // int[] validAprilTags = {4};
            // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-three",validAprilTags);
            double tx = LimelightHelpers.getTX("limelight-three");
            boolean tv = LimelightHelpers.getTV("limelight-three");
               

            while (tv == true && tx > 0) {
                
                     tx = LimelightHelpers.getTX("limelight-three");
                     tv = LimelightHelpers.getTV("limelight-three");
                    // if (tx > 0) {
                    //     loopbreak = false;
                    //     break;
                    // }
                        this.swerveSubsystem.driveRobotOriented(this.swerveSubsystem.swerveDrive.swerveController.getRawTargetSpeeds(
                         0.0,-0.5,0.0));
                        try {
                           long sleepTime = (long)(tx+1*10);
                           System.out.println("sleeping for " + (sleepTime));
                        Thread.sleep(sleepTime);
                        } catch (InterruptedException e) {
                     e.printStackTrace();
                    }
                    // if (tx < 0) {
                    //     loopbreak = true;
                    //     break;
                    // }
               }
            
                //if (loopbreak == true) {
                    System.out.println("TX value is negative! Loop TX = "  + (tx));
                //}



            }
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
         }

        @Override
        public void execute() {
            
            
        }

        @Override
        public boolean isFinished(){
            return true;
        }
    
        
    }

