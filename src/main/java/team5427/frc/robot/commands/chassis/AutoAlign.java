package team5427.frc.robot.commands.chassis;

import java.lang.Thread.State;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AutoAlign extends Command{
    private Pose2d target;
    private Pose2d robot;

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController rotController;

    private SwerveSubsystem driveBase;
    private boolean isCorrect;


    private Pose2d getTargetPose2d() {
        List<Pose2d> accPoses;
            List<Pose2d> targetPoses = new ArrayList<>();

            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                accPoses = List.copyOf(List.of(RobotConfigConstraints.kAutoAlignPosesBlue)); // add robot config constraints later
            } else {
                accPoses = List.copyOf(List.of(RobotConfigConstraints.kAutoAlignPosesRed));
            }
            for (int i = isCorrect? 0 : 1; i < accPoses.size(); i += 2) {
                targetPoses.add(accPoses.get(i));
                // System.out.println(accPoses.get(i));
            }

            // return the target pose
            return RobotState.getInstance().getAdaptivePose().nearest(targetPoses); // todo fix getInstance
    }


    public AutoAlign(SwerveSubsystem driveBase, boolean isCorrect) {
        this.isCorrect = isCorrect;
        this.driveBase = driveBase;

        rotController = new ProfiledPIDController(Constants.SwerveConstants.kAutoAlignRotationalKp,0.0,0.0, new Constraints(10.0, 10.0));
        xController = new ProfiledPIDController(Constants.SwerveConstants.kAutoAlignTranslationKp, 0.0, 0.0, new Constraints(10.0, 10.0));
        yController = new ProfiledPIDController(Constants.SwerveConstants.kAutoAlignTranslationKp, 0.0, 0.0, new Constraints(10.0, 10.0));

        this.isCorrect = isCorrect;
        this.driveBase = driveBase;
        addRequirements(driveBase);

        robot = new Pose2d();
    }

    @Override
    public void initialize() {
        target = getTargetPose2d();
        Pose2d robotPose = RobotState.getInstance().getEstimatedPose();

        rotController.reset(robotPose.getRotation().getRadians());
        rotController.setGoal(target.getRotation().getRadians());
        rotController.setTolerance(0.05);

        xController.reset(robotPose.getX());
        xController.setGoal(target.getX());
        xController.setTolerance(0.04);

        yController.reset(isCorrect ? robotPose.getY() : robotPose.getY() * -1);
        yController.setGoal(isCorrect ? target.getY() : target.getY() * -1);
        yController.setTolerance(0.04);
    }
}