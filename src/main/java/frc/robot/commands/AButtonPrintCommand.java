package frc.robot.commands;

import java.util.function.DoubleSupplier;
import lib.frc706.cyberlib.subsystems.LimelightHelpers;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

//getRobotVelocity getYaw
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
            
            boolean tv = LimelightHelpers.getTV("limelight-three");
                //System.out.println(tv); //Temporary line for testing tv.
            
            if  (tv == true) {
                
                //double aButtonGetTurnRate= this.swerveSubsystem.swerveDrive.getTurnRate();
                //System.out.println(("RobotTurnRate = ") + (aButtonGetTurnRate));


                // double aButtonRobotRelativeSpeeds = this.swerveSubsystem.getRobotRelativeSpeeds();
                // System.out.println(("relativeSpeeds = ") + (aButtonRobotRelativeSpeeds));


                double tx = LimelightHelpers.getTX("limelight-three");
                double aButtonHeading = this.swerveSubsystem.getHeading();
                System.out.println(("TX Value =")+(tx));
                System.out.println(("Heading = ") + (aButtonHeading));
                
            }
            
            else {
                System.out.println("No Valid AprilTags!");
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

