package OpModes.Autos.Specimen;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Common.Robot.IntakeOuttakeV2;


@Autonomous
@Config
//@Disabled
public class Copy2 extends OpMode {
    private Timer pathTimer;
    private int pathState;
    private Follower follower;
    private Path cyclePos, push1;
    private PathChain samplePush1, goBack1, samplePush2, goBack2, samplePush3, preload, eat1, hang1, eat2, hang2, eat3, hang3, eat4, hang4, park;
    private final Pose startPose = new Pose(6.7291,67.9626,Math.toRadians(0));//x was 6.7291
    private final Pose scorePose = new Pose(40.3738-2.25,67.9626+.5,Math.toRadians(0));

    private final Pose sample1 = new Pose(60.1121-1-2,30.9533+.5,Math.toRadians(180));//was x=66.8411, y=25.7944; then 65.7196,28.4861, then (61.9065,31.4019)
    private final Pose sample1Control = new Pose(5.8318,48.6729,Math.toRadians(180));//was 2.6916,53.1589
    private final Pose observation1 = new Pose(7.4019+3+8, 18.1682+.5, Math.toRadians(180));//was 7.1776, 20.8598
    private final Pose sample2 = new Pose(57.4206-2.25-2.5, 18.1682+1.75, Math.toRadians(180));//was x=68.6355, y=23.3271, then (77.3832, 24.8972), then 69.0841, 20.8598, then (61.0093, 18.3925), then (62.1308, 14.3551), then (60.1121, 17.4953)
    private final Pose sample2Control = new Pose(54.9533,42.8411,Math.toRadians(180));//was (71.9999,35.6636), then (60.3364,32.9720), then (59.9495,39.5215), then (61.9065,43.2897)
    private final Pose observation2 = new Pose(9.8692+2+6, 13.0093, Math.toRadians(180));//was (10.5421, 10.7664)
    private final Pose sample3 = new Pose(57.4206-3.225-1, 10.9907+5, Math.toRadians(180)); //was (x,y)(71.3271, 9.1963), then (71.1028, 10.9907), then (62.8037, 9.4206), then (59.8879, 9.8692), then (59.43925, 10.5421)
    private final Pose sample3Control = new Pose(57.1963,25.7944+9,Math.toRadians(180));//was (80.0748,24.4486)
    private final Pose observation3 = new Pose(8.9720+8,10.9907,Math.toRadians(180)); //was (x,y) (9.8692,8.2991), then (10.7664,9.4206), then (10.0935,9.1963)
    //private final Pose geraldPickup = new Pose(10,24,Math.toRadians(180));
    private final Pose eatPose = new Pose(7+2,24+8,Math.toRadians(180));//was 180
    private final Pose eatPose3 = new Pose(7+2,24+8,Math.toRadians(180));//was 180
    private final Pose eatPoseControl = new Pose(27.5,18.8,Math.toRadians(180));
    private final Pose specimen1 = new Pose(40.374-3.5-5.5,70+9,Math.toRadians(180));
    private final Pose specimen1Control = new Pose(33.87,89.72,Math.toRadians(0));
    private final Pose specimen1Control2 = new Pose(17.05, 84.79, Math.toRadians(0));
    private final Pose specimen2 = new Pose(40.375-6.25-3.5  ,66+10,Math.toRadians(180));
    private final Pose specimen3 = new Pose(40.375-3.5-6,61+5,Math.toRadians(180));
    private final Pose specimen4 = new Pose(40.375-3.5-3.25,58+7,Math.toRadians(180));


    IntakeOuttakeV2 jamocha = new IntakeOuttakeV2();


    public void pathBuilder(){
        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose.getX(), startPose.getY()),new Point(scorePose.getX()-.75, scorePose.getY())))
                .setConstantHeadingInterpolation((startPose.getHeading()))
                .build();

        push1 = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(sample1Control), new Point(sample1)));
        push1.setLinearHeadingInterpolation(scorePose.getHeading(), sample1.getHeading());

        samplePush1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(observation1)))
                .setConstantHeadingInterpolation(sample1.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        goBack1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(observation1), new Point(sample2Control), new Point(sample2)))
                .setConstantHeadingInterpolation(observation1.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        samplePush2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2), new Point(observation2)))
                .setConstantHeadingInterpolation(sample2.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        goBack2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(observation2), new Point(sample3Control),new Point(sample3)))
                .setConstantHeadingInterpolation(observation2.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        samplePush3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3), new Point(observation3)))
                .setConstantHeadingInterpolation(sample3.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        eat1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(observation3), new Point(eatPoseControl), new Point(eatPose.getX()-3, eatPose.getY()-2)))
                .setConstantHeadingInterpolation(observation3.getHeading())
                .build();

        /*eat1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(geraldPickup), new Point(eatPose)))
                .setConstantHeadingInterpolation(geraldPickup.getHeading())
                .build();*/

        hang1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eatPose), new Point(specimen1.getX()-.25, specimen1.getY())))
                .setConstantHeadingInterpolation(eatPose.getHeading())
                .build();

        eat2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1), new Point(eatPose.getX()-1.5, eatPose.getY()-1)))
                .setConstantHeadingInterpolation(specimen1.getHeading())
                .setPathEndTimeoutConstraint(125)
                .build();

        hang2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eatPose.getX(), eatPose.getY()), new Point(specimen2.getX()-2, specimen2.getY())))
                .setConstantHeadingInterpolation(eatPose.getHeading())
                .build();

        eat3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2), new Point(eatPose3.getX()-2, eatPose3.getY()-1.2)))
                .setConstantHeadingInterpolation(specimen2.getHeading())
                .setPathEndTimeoutConstraint(125)
                .build();
        hang3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eatPose3), new Point(specimen3.getX()-1.75, specimen3.getY())))
                .setConstantHeadingInterpolation(eatPose3.getHeading())
                .build();

        eat4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen3), new Point(eatPose.getX()-1, eatPose.getY())))
                .setConstantHeadingInterpolation(specimen3.getHeading())
                .setPathEndTimeoutConstraint(125)
                .build();

        hang4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eatPose), new Point(specimen4.getX()-5.25, specimen4.getY()+7)))
                .setConstantHeadingInterpolation(eatPose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen4), new Point(eatPose)))
                .setConstantHeadingInterpolation(specimen4.getHeading())
                .build();
    }
    public void autoPathUpdate(){
        switch(pathState){
            case 0:
                jamocha.clawClosed();
                jamocha.armChamber();
                jamocha.armTurretForward();
                jamocha.horizontalSlidesHome();
                jamocha.fourBarPitchStowed();
                jamocha.fourBarStowed();
                //jamocha.liftHighChamber(); //was -1410
                pathTimer.resetTimer();
                setPathState(1000);
                break;
            case 1000:
                //follower.setMaxPower(0.85);
                follower.followPath(preload, true);
                setPathState(10000);
                break;
            case 10000:
                if (follower.getCurrentTValue()>.05){
                    jamocha.liftHighChamber();
                    setPathState(100);
                }
                break;
            case 100:
                if(!follower.isBusy()){
                    jamocha.armHorizontal();
                    //jamocha.liftStowed();
                    setPathState(101);
                }
                break;
            case 101:
                if(pathTimer.getElapsedTimeSeconds()>.25){
                    jamocha.clawOpen();
                    jamocha.liftSwing();
                    //jamocha.armPickup();
                    //fix
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()){
                    //follower.setMaxPower(0.95);
                    follower.followPath(push1);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    //jamocha.liftStowed();
                    //follower.setMaxPower(0.90);//was.7
                    follower.followPath(samplePush1);
                    setPathState(21);
                }
                break;
            case 21:
                if (follower.getCurrentTValue()>.1){
                    jamocha.liftStowed();
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    //follower.setMaxPower(0.95);//was .7
                    follower.followPath(goBack1);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.setMaxPower(0.90);
                    follower.followPath(samplePush2);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    //follower.setMaxPower(0.95);
                    follower.followPath(goBack2);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    follower.setMaxPower(0.90-.05);
                    follower.followPath(samplePush3);
                    setPathState(61);
                }
                break;
            case 61:
                if(follower.atParametricEnd()){
                    //lift.autoLift(-200);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds()>.1){
                    follower.setMaxPower(0.95);
                    follower.followPath(eat1);
                    setPathState(71);
                }
                break;
            case 71:
                if(follower.atParametricEnd()) {
                    jamocha.clawClosed();
                    setPathState(73);
                }
                break;
            case 73:
                if (pathTimer.getElapsedTimeSeconds()>.4) {
                    jamocha.armChamber();
                    setPathState(8);
                }
                break;
            case 8:
                if(pathTimer.getElapsedTimeSeconds()>.1){ //follower.getCurrentTValue()>0.995)
                    //follower.setMaxPower(0.95);
                    follower.followPath(hang1);
                    setPathState(72);
                }
                break;
            case 72:
                if (follower.getCurrentTValue()>.1){
                    jamocha.liftHighChamber();
                    setPathState(74);
                }
                break;
            case 74:
                if (follower.getCurrentTValue()>.15){
                    jamocha.armTurretBackward();
                    setPathState(81);
                }
                break;
            case 81:
                if(follower.atParametricEnd()){
                    jamocha.armHorizontal();
                    setPathState(82);
                }
                break;
            case 82:
                if(pathTimer.getElapsedTimeSeconds()>.25){
                    jamocha.clawOpen();
                    jamocha.liftSwing();
                    jamocha.armTurretForward();
                    //jamocha.armPickup();
                    setPathState(9);
                }
                break;
            case 9:
                if(pathTimer.getElapsedTimeSeconds()>.1){
                    follower.setMaxPower(0.85);
                    follower.followPath(eat2);
                    //jamocha.liftStowed();
                    setPathState(911);
                }
                break;
            case 911:
                if (follower.getCurrentTValue()>.2){
                    jamocha.liftStowed();
                    setPathState(91);
                }
                break;
            case 91:
                if(follower.atParametricEnd()) {
                    setPathState(92);
                }
                break;
            case 92:
                if (pathTimer.getElapsedTimeSeconds()>.2) {
                    jamocha.clawClosed();
                    //.armChamber();
                    setPathState(93);
                }
                break;
            case 93:
                if (pathTimer.getElapsedTimeSeconds()>.4){
                    jamocha.armChamber();
                    setPathState(10);
                }
                break;
            case 10:
                if(pathTimer.getElapsedTimeSeconds()>.75){
                    follower.setMaxPower(0.95);
                    follower.followPath(hang2);
                    setPathState(1003);
                }
                break;
            case 1003:
                if (follower.getCurrentTValue()>.1){
                    jamocha.liftHighChamber();
                    setPathState(1004);
                }
                break;
            case 1004:
                if (follower.getCurrentTValue()>.15){
                    jamocha.armTurretBackward();
                    setPathState(1001);
                }
                break;
            case 1001:
                if(follower.atParametricEnd()){
                    jamocha.armHorizontal();
                    setPathState(1002);
                }
                break;
            case 1002:
                if(pathTimer.getElapsedTimeSeconds()>.25){
                    jamocha.clawOpen();
                    jamocha.liftSwing();
                    jamocha.armTurretForward();
                    //jamocha.armPickup();
                    setPathState(11);
                }
                break;
            case 11:
                if(pathTimer.getElapsedTimeSeconds()>.1){
                    follower.setMaxPower(0.85);
                    follower.followPath(eat3);
                    //jamocha.liftStowed();
                    setPathState(111);
                }
                break;
            case 111:
                if (follower.getCurrentTValue()>.1){
                    jamocha.liftStowed();
                    setPathState(112);
                }
                break;
            case 112:
                if (follower.atParametricEnd()){
                    setPathState(113);
                }
                break;
            case 113:
                if(pathTimer.getElapsedTimeSeconds()>.1) {
                    jamocha.clawClosed();
                    setPathState(114);
                }
                break;
            case 114:
                if (pathTimer.getElapsedTimeSeconds()>.4) {
                    jamocha.armChamber();
                    setPathState(12);
                }
                break;
            case 12:
                if(pathTimer.getElapsedTimeSeconds()>.1){
                    follower.setMaxPower(0.90);
                    follower.followPath(hang3);
                    setPathState(121);
                }
                break;
            case 121:
                if (follower.getCurrentTValue()>.1){
                    jamocha.liftHighChamber();
                    setPathState(122);
                }
                break;
            case 122:
                if (follower.getCurrentTValue()>.15){
                    jamocha.armTurretBackward();
                    setPathState(123);
                }
                break;
            case 123:
                if(follower.atParametricEnd()){
                    jamocha.armHorizontal();
                    setPathState(124);
                }
                break;
            case 124:
                if(pathTimer.getElapsedTimeSeconds()>.25){
                    jamocha.clawOpen();
                    jamocha.liftSwing();
                    jamocha.armTurretForward();
                    //jamocha.armPickup();
                    setPathState(13);
                }
                break;
            case 13:
                if(pathTimer.getElapsedTimeSeconds()>.1){
                    follower.setMaxPower(0.95);
                    follower.followPath(park);
                    //jamocha.liftStowed();
                    setPathState(-1);
                }
                break;
            case 130:
                if (follower.getCurrentTValue()>.1){
                    jamocha.liftStowed();
                    setPathState(1300);
                }
                break;
            case 1300:
                if (follower.atParametricEnd()){
                    setPathState(131);
                }
                break;
            case 131:
                if(pathTimer.getElapsedTimeSeconds()>.1) {
                    jamocha.clawClosed();
                    setPathState(132);
                }
                break;
            case 132:
                if (pathTimer.getElapsedTimeSeconds()>.4) {
                    jamocha.armChamber();
                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.getElapsedTimeSeconds()>.1){
                    follower.setMaxPower(0.95);
                    follower.followPath(hang4);
                    setPathState(140);
                }
                break;
            case 140:
                if (follower.getCurrentTValue()>.1){
                    jamocha.liftHighChamber();
                    setPathState(133);
                }

            case 133:
                if (follower.getCurrentTValue()>.15){
                    jamocha.armTurretBackward();
                    setPathState(141);
                }
                break;
            case 141:
                if(follower.atParametricEnd()){
                    jamocha.armHorizontal();
                    setPathState(142);
                }
                break;
            case 142:
                if(pathTimer.getElapsedTimeSeconds()>.75){
                    jamocha.clawOpen();
                    jamocha.liftSwing();
                    jamocha.armTurretForward();
                    //jamocha.armPickup();
                    setPathState(15);
                }
                break;
            case 15:
                if(follower.getCurrentTValue()>0.995){
                    follower.setMaxPower(0.95);
                    follower.followPath(park);
                    jamocha.liftStowed();
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        // opmodeTimer = new Timer();

        //opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        pathBuilder();

        // lift.initLiftAuto(hardwareMap);

        jamocha.initIntake(hardwareMap);

        jamocha.initOuttake(hardwareMap);

        // claw.initSpecimen_Claw(hardwareMap);

        //claw.Specimen_Claw_Closed();
        // claw.Closer_Servo_Closed();

    }

    @Override
    public void loop() {
        follower.update();
        autoPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void start(){
        setPathState(0);
    }
}



