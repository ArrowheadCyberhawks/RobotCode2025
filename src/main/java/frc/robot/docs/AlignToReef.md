# AlignToReef

## Purpose
It moves the robot to a pose corresponding to a position on the reef using:
- **Trajectory generation** by `Pathplanner`
- **PID Movements** using Command `DriveToPose`

### Variables
```java
    private final SwerveSubsystem swerveSubsystem;
    private final ReefPoint reef;
    //private final double modeVal = 0.3;
```
- `swerveSubsystem`: A reference to the subsystem responsible for controlling the robot's swerve drive, which allows for precise movement.
- `reef`: A `Pose2d` corresponding to a position on the reef that the robot is aligning to.

### Constructor
```java
    public AlignToReef(SwerveSubsystem mSwerve, AprilTagFieldLayout field, ReefPoint reef) {
        this.swerveSubsystem = mSwerve;
        this.reef = reef;
    }
```
- The constructor accepts a `SwerveSubsystem` object, the field layout (which is not used directly here), and the `ReefPoint` object that represents the target point for alignment.

### Logging
```java
    private final LoggedNetworkNumber modeVal = new LoggedNetworkNumber("AlignToReef/modeVal", 0.5);
    private final StructPublisher<Pose2d> desiredBranchPublisher = NetworkTableInstance.getDefault().getTable("logging").getStructTopic("desired branch", Pose2d.struct).publish();
```
- `modeVal`: A `LoggedNetworkNumber` that is used for controlling a value through NetworkTables
- `desiredBranchPublisher`: A `StructPublisher` that publishes the `Pose2d` of the desired alignment point (reef) to NetworkTables

### Generate Command
```java
    public Command generateCommand(ReefPoint reef) {
        return Commands.defer(() -> {
            desiredBranchPublisher.accept(reef.getPose());
            return getPathFromWaypoint(getWaypointFromBranch(reef));
        }, Set.of());
    }
```
- `generateCommand`: This method generates a command to align the robot to the specified `ReefPoint`.
- It publishes the target pose (position and orientation) of the reef to the network table.
- The command then calculates and returns the path from the robot's current position to the desired waypoint using the `getPathFromWaypoint` method.

### Get Path from Waypoint
```java
    private Command getPathFromWaypoint(Pose2d waypoint) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(swerveSubsystem.getPose().getTranslation(), getPathVelocityHeading(swerveSubsystem.swerveDrive.getFieldVelocity(), waypoint)),
            waypoint
        );

        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < modeVal.get()) {
            return 
            Commands.sequence(
                Commands.print("start position PID loop"),
                new DriveToPose(swerveSubsystem, waypoint),
                Commands.print("end position PID loop")
            );
        }

        PathConstraints constraints = new PathConstraints(
            swerveSubsystem.swerveDrive.getMaximumChassisVelocity(), 4.0,
            swerveSubsystem.swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            constraints,
            new IdealStartingState(getVelocityMagnitude(swerveSubsystem.swerveDrive.getFieldVelocity()), swerveSubsystem.getPose().getRotation()), 
            new GoalEndState(0.0, waypoint.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path).andThen(
            Commands.print("Starting position PID loop"),
            new DriveToPose(swerveSubsystem, waypoint),
            Commands.print("Ending position PID loop")
        );
    }
```
- `getPathFromWaypoint`: This method generates a path to align the robot to a specific waypoint using bezier curves.
    - First, it calculates a list of waypoints using the `PathPlannerPath.waypointsFromPoses` method.
    - If the distance between the start and end waypoint is smaller than the threshold (`modeVal`), it drives to the position using **PID** using the `DriveToPose` command that directly drives the robot to the waypoint.
    - Otherwise, it constructs a full path using `PathPlannerPath` and applies constraints for maximum velocity and angular velocity.
- The path is followed using `AutoBuilder`, and after that, it continues to align the robot precisely to the target position.

### Get Path Velocity Heading
```java
    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            var diff = target.minus(swerveSubsystem.getPose()).getTranslation();
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();//.rotateBy(Rotation2d.k180deg);
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }
```
- `getPathVelocityHeading`: This method determines the velocity heading based on the robot’s current movement.
- If the robot’s velocity is low (less than 0.25 m/s), it calculates the direction to the target waypoint and adjusts the heading accordingly.
- If the robot is moving faster, it uses the robot’s current velocity to determine the heading.

### Get Velocity Magnitude
```java
    private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }
```
- `getVelocityMagnitude`: This method calculates the magnitude of the robot’s velocity (i.e., how fast it's moving) by calculating the norm of the velocity vector.

### Get Waypoint from Branch
```java
    private Pose2d getWaypointFromBranch(ReefPoint reef) {
        return reef.getPose();
    }
}
```
- `getWaypointFromBranch`: This method simply returns the pose (position and orientation) of the specified `ReefPoint` that the robot needs to align to.

---