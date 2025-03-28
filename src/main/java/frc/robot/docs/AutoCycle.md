# AutoCycle  

### **Purpose**  
The `AutoCycle` command automates the robot's scoring cycle by:  
- Identifying the **next best scoring position** (`Pose2d`).  
- Aligning with the target using the **AlignToReef** command.  
- Returning to the **human player station** for resupply.  

---  

### **Variables**  

#### **Subsystems and Dependencies**  
```java
private final SwerveSubsystem swerveSubsystem;
private final Superstructure superstructure;
private final Grabber grabber;
private final AlignToReef alignmentCommandFactory;
```
- **`swerveSubsystem`** → Controls robot movement.  
- **`superstructure`** → Manages scoring positions (L1–L4).  
- **`grabber`** → Handles game piece manipulation.  
- **`alignmentCommandFactory`** → Generates alignment commands for precise positioning.  

#### **Scoring and Navigation**  
```java
private final List<Pose2d> reefPoints;
public int[][] algaeArray = new int[12][4];
private int scoredL4 = 0;
private int scoredL3 = 0;
private int scoredL2 = 0;
private int scoredL1 = 0;
```
- **`reefPoints`** → List of predefined scoring positions.  
- **`algaeArray`** → Tracks obstacles and scoring status at reef points.  
- **`scoredL1 – scoredL4`** → Counters for scored objects at different levels.  

---  

### **`run()` Method**  
The main execution method for the `AutoCycle` command.  

```java
public Command run() {
    Pose2d targetPose = getNextBestScorePose(false);
    alignmentCommandFactory.generateCommand(targetPose);
    return returnToHumanPlayerStation();
}
```
- **Finds the next best scoring position (`getNextBestScorePose`)**.  
- **Aligns the robot** using `AlignToReef`.  
- **Returns to the human player station** for more game pieces.  

---  

### **Scoring Logic**  
#### **Selecting the Next Target**  
```java
private Pose2d getNextBestScorePose(boolean coopertitionActive) {
    int L4_THRESHOLD = 5, L3_THRESHOLD = 5, L2_THRESHOLD = 5, L1_THRESHOLD = 5;
    
    if (scoredL4 < L4_THRESHOLD) {
        return findClosestPoseWithStatus("L4", 0);
    } else if (scoredL3 < L3_THRESHOLD) {
        return findClosestPoseWithStatus("L3", 0);
    } else if (scoredL2 < L2_THRESHOLD) {
        return findClosestPoseWithStatus("L2", 0);
    } else if (scoredL1 < L1_THRESHOLD && !coopertitionActive) {
        return findClosestPoseWithStatus("L1", 0);
    } else {
        return findClosestPoseWithStatus("L4", 0);
    }
}
```
- **Prioritizes scoring at higher levels (L4 → L1).**  
- **Accounts for coopertition rules** when deciding where to score.  

#### **Finding the Closest Scoring Position**  
```java
private Pose2d findClosestPoseWithStatus(String level, int status) {
    Pose2d closestPose = null;
    double closestDistance = Double.MAX_VALUE;
    Superstructure.nextSuperStructureState = SuperStructureState.valueOf(level);

    for (int i = 0; i < reefPoints.size(); i++) {
        Pose2d nextPose = reefPoints.get(i);
        int currentStatus = algaeArray[i][levelToIndex(level)];

        if (currentStatus == status) {
            double distance = nextPose.getTranslation().getDistance(swerveSubsystem.getPose().getTranslation());
            if (distance < closestDistance) {
                closestPose = nextPose;
                closestDistance = distance;
            }
        }
    }
    return closestPose;
}
```
- **Finds the closest unoccupied scoring position** based on the robot’s location.  
- **Updates the `SuperStructureState`** for scoring adjustments.  

---  

### **Returning to the Human Player Station**  
```java
private Command returnToHumanPlayerStation() {
    Pose2d humanPlayerStationPose = findClosestHumanPlayerStation();
    return alignmentCommandFactory.generateCommand(humanPlayerStationPose);
}
```
- **Finds the closest station** for resupply.  
- **Aligns the robot** using `AlignToReef`.  

#### **Finding the Closest Station**  
```java
private Pose2d findClosestHumanPlayerStation() {
    Pose2d leftHuman = new Pose2d(16.697198, 0.65532, new Rotation2d(Units.degreesToRadians(30)));
    Pose2d rightHuman = new Pose2d(16.697198, 7.396, new Rotation2d(Units.degreesToRadians(30)));

    if (leftHuman.getTranslation().getDistance(swerveSubsystem.getPose().getTranslation()) <
        rightHuman.getTranslation().getDistance(swerveSubsystem.getPose().getTranslation())) {
        return leftHuman;
    } else {
        return rightHuman;
    }
}
```
- **Calculates distance to both human player stations** and chooses the closest one.  

---  

### **Logging Scoring Data**  
```java
private void logScoring() {
    Logger.recordOutput("Scoring/L4", scoredL4);
    Logger.recordOutput("Scoring/L3", scoredL3);
    Logger.recordOutput("Scoring/L2", scoredL2);
    Logger.recordOutput("Scoring/L1", scoredL1);
}
```
- **Logs the number of scores per level** for debugging and analytics.  

---  

### **Helper Methods**  
#### **Convert Reef Points to Poses**  
```java
public List<Pose2d> convertReefPointsToPoses(List<ReefPoint> reefPoints) {
    List<Pose2d> poseList = new ArrayList<>();
    for (ReefPoint point : reefPoints) {
        poseList.add(point.getPose());
    }
    return poseList;
}
```
- **Converts reef point locations into `Pose2d` objects** for navigation.  

#### **Get Reef Points**  
```java
public List<ReefPoint> getReefPoints() {
    List<ReefPoint> reefPoints = new ArrayList<>();
    for (ReefPoint point : ReefPoint.values()) {
        reefPoints.add(point);
    }
    reefPoints.remove(12);
    return reefPoints;
}
```
- **Retrieves reef point locations** but removes index `12` for an unspecified reason.  

#### **Initialize Scoring Array**  
```java
public void initScoringArray() {
    algaeArray = new int[][] {
        { 0, 2, 2, 0 }, { 0, 2, 2, 0 }, { 0, 2, 2, 0 }, { 0, 2, 2, 0 },
        { 0, 2, 2, 0 }, { 0, 2, 2, 0 }, { 0, 2, 2, 0 }, { 0, 2, 2, 0 },
        { 0, 2, 2, 0 }, { 0, 2, 2, 0 }, { 0, 2, 2, 0 }, { 0, 2, 2, 0 }
    };
}
```
- **Initializes `algaeArray`** to store reef point status.  

#### **Convert Level to Index**  
```java
private int levelToIndex(String level) {
    switch (level) {
        case "L4": return 0;
        case "L3": return 1;
        case "L2": return 2;
        case "L1": return 3;
        default: return -1;
    }
}
```
- **Maps scoring levels (`L1–L4`) to indices** for array access.  

---  