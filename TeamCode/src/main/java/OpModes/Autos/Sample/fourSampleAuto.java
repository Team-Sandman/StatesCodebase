package OpModes.Autos.Sample;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Common.Robot.IntakeOuttakeV2;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous
@Config
public class fourSampleAuto extends OpMode {
    private Timer pathTimer;
    private int pathState;
    private Follower follower;
    private Path eat1;
    private PathChain preload, score1, eat2, score2, eat3, score3, backOff;
    private final Pose startPose = new Pose(8.4014,110.3944,Math.toRadians(90));
    private final Pose preScore = new Pose(8.4014,124.3407+2,Math.toRadians(90));

    private final Pose sample1 = new Pose(35.1179-4,121.1482,Math.toRadians(180));
    private final Pose scorePose = new Pose(15.2905+1,131.0618,Math.toRadians(140));
    private final Pose sample2 = new Pose(35.6219-2, 131.734, Math.toRadians(180));
    private final Pose sample3 = new Pose(43.35, 131.4-2, Math.toRadians(260));//was x=68.6355, y=23.3271, then (77.3832, 24.8972), then 69.0841, 20.8598, then (61.0093, 18.3925), then (62.1308, 14.3551), then (60.1121, 17.4953)

    private final Pose End = new Pose(16, 131.5, Math.toRadians(140));


    IntakeOuttakeV2 jamocha = new IntakeOuttakeV2();


    public void pathBuilder(){
        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose.getX(), startPose.getY()),new Point(preScore.getX(), preScore.getY())))
                .setConstantHeadingInterpolation((startPose.getHeading()))
                .build();

        eat1 = new Path(new BezierLine(new Point(preScore), new Point(sample1)));
        eat1.setLinearHeadingInterpolation(preScore.getHeading(), sample1.getHeading());

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1.getX(), sample1.getY()),new Point(scorePose.getX(), scorePose.getY())))
                .setLinearHeadingInterpolation(sample1.getHeading(), scorePose.getHeading())
                .build();

        eat2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose.getX(), scorePose.getY()),new Point(sample2.getX(), sample2.getY())))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sample2.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2.getX(), sample2.getY()),new Point(scorePose.getX(), scorePose.getY())))
                .setLinearHeadingInterpolation(sample2.getHeading(), scorePose.getHeading())
                .build();

        eat3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose.getX(), scorePose.getY()),new Point(sample3.getX(), sample3.getY())))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sample3.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3.getX(), sample3.getY()),new Point(scorePose.getX(), scorePose.getY())))
                .setLinearHeadingInterpolation(sample2.getHeading(), scorePose.getHeading())
                .build();

        backOff = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose.getX(), scorePose.getY()),new Point(End.getX(), End.getY())))
                .setConstantHeadingInterpolation((startPose.getHeading()))
                .build();

    }

    public void autoPathUpdate(){
        switch(pathState){
            case 0:
                jamocha.clawClosed();
                jamocha.armChamber();
                jamocha.armTurretForward();
                jamocha.horizontalSlidesHome();
                jamocha.fourBarPitchSearch();
                jamocha.fourBarStowed();
                jamocha.intakeOpen();
                jamocha.liftHighBasket(); //was -1410
                pathTimer.resetTimer();
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds()>2) {
                    //follower.setMaxPower(0.85);
                    follower.followPath(preload, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds()>1.5){
                    jamocha.clawOpen();
                    setPathState(3);
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds()>.1) {
                    follower.setMaxPower(.5);
                    follower.followPath(eat1);
                    //jamocha.liftStowed();
                    setPathState(4);
                }
                break;

            case 4:
                if (follower.getCurrentTValue()>.2){
                    jamocha.armTurretBackward();
                    jamocha.liftPreTransfer();
                    jamocha.fourBarSearch();
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()){
                    jamocha.armTransfer();
                    jamocha.fourBarDown();
                    jamocha.intakeClosed();
                    //jamocha.liftTransfer();
                    setPathState(6);
                }
                break;

            case 6:
                if (pathTimer.getElapsedTimeSeconds()>.5) {
                    jamocha.fourBarPitchTransfer();
                    jamocha.fourBarSearch();
                    //jamocha.liftTransfer();
                    setPathState(7);
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds()>.75){
                    jamocha.liftTransfer();
                    jamocha.clawClosed();
                    setPathState(8);
                }
                break;

            case 8:
                if (pathTimer.getElapsedTimeSeconds()>.75) {
                    //jamocha.fourBarPitchTransfer();
                    jamocha.clawClosed();
                    setPathState(9);
                }
            case 9:
                if (pathTimer.getElapsedTimeSeconds()>.2){
                    jamocha.intakeOpen();
                    setPathState(10);
                }
                break;

            case 10:
                if(pathTimer.getElapsedTimeSeconds()>.2){
                    jamocha.armChamber();
                    setPathState(11);
                }
                break;

            case 11:
                if(pathTimer.getElapsedTimeSeconds()>.25){
                    jamocha.liftHighBasket();
                    jamocha.armTurretForward();
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(score1);
                    //jamocha.liftStowed();
                    setPathState(13);
                }
                break;

            case 13:
                if(!follower.isBusy()){
                    jamocha.clawOpen();
                    setPathState(14);
                }
                break;

            case 14:
                    follower.followPath(eat2);
                    //jamocha.liftStowed();
                    setPathState(15);
                break;

            case 15:
                if (follower.getCurrentTValue()>.2){
                    jamocha.armTurretBackward();
                    jamocha.liftPreTransfer();
                    jamocha.fourBarSearch();
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()){
                    jamocha.armTransfer();
                    jamocha.fourBarDown();
                    jamocha.intakeClosed();
                    //jamocha.liftTransfer();
                    setPathState(17);
                }
                break;

            case 17:
                if (pathTimer.getElapsedTimeSeconds()>.5) {
                    jamocha.fourBarPitchTransfer();
                    jamocha.fourBarSearch();
                    setPathState(18);
                }
                break;

            case 18:
                if (pathTimer.getElapsedTimeSeconds()>1){
                    jamocha.liftTransfer();
                    jamocha.clawClosed();
                    setPathState(19);
                }
                break;

            case 19:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    //jamocha.fourBarPitchTransfer();
                    jamocha.clawClosed();
                    setPathState(20);
                }
            case 20:
                if (pathTimer.getElapsedTimeSeconds()>.5){
                    jamocha.intakeOpen();
                    setPathState(21);
                }
                break;

            case 21:
                if(pathTimer.getElapsedTimeSeconds()>.25){
                    jamocha.armChamber();
                    setPathState(22);
                }
                break;

            case 22:
                if(pathTimer.getElapsedTimeSeconds()>.1){
                    jamocha.liftHighBasket();
                    jamocha.armTurretForward();
                    setPathState(23);
                }
                break;

            case 23:
                if(!follower.isBusy()){
                    follower.followPath(score2);
                    //jamocha.liftStowed();
                    setPathState(24);
                }
                break;

            case 24:
                if(!follower.isBusy()){
                    jamocha.clawOpen();
                    setPathState(25);
                }
                break;

            case 25:
                if(!follower.isBusy()){
                    follower.followPath(eat3);
                    //jamocha.liftStowed();
                    setPathState(26);
                }
                break;

            case 26:
                if (follower.getCurrentTValue()>.2){
                    jamocha.armTurretBackward();
                    jamocha.liftPreTransfer();
                    jamocha.fourBarSearch();
                    jamocha.intakeRotateWall();
                    setPathState(27);
                }
                break;

            case 27:
                if (!follower.isBusy()){
                    jamocha.armTransfer();
                    jamocha.fourBarDown();
                    jamocha.intakeClosed();
                    //jamocha.liftTransfer();
                    setPathState(28);
                }
                break;

            case 28:
                if (pathTimer.getElapsedTimeSeconds()>.5) {
                    jamocha.intakeOff();
                    setPathState(29);
                }
                break;

            case 29:
                if (pathTimer.getElapsedTimeSeconds()>.25) {
                    jamocha.fourBarPitchTransfer();
                    setPathState(30);
                }
                break;

            case 30:
                if (pathTimer.getElapsedTimeSeconds()>1){
                    jamocha.liftTransfer();
                    setPathState(31);
                }
                break;

            case 31:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    //jamocha.fourBarPitchTransfer();
                    jamocha.clawClosed();
                    setPathState(32);
                }
            case 32:
                if (pathTimer.getElapsedTimeSeconds()>.5){
                    jamocha.intakeOpen();
                    setPathState(33);
                }
                break;

            case 33:
                if(pathTimer.getElapsedTimeSeconds()>.25){
                    jamocha.armChamber();
                    setPathState(34);
                }
                break;

            case 34:
                if(pathTimer.getElapsedTimeSeconds()>.1){
                    jamocha.liftHighBasket();
                    jamocha.armTurretForward();
                    setPathState(35);
                }
                break;

            case 35:
                if(!follower.isBusy()){
                    follower.followPath(score3);
                    //jamocha.liftStowed();
                    setPathState(36);
                }
                break;

            case 36:
                if(!follower.isBusy()){
                    follower.followPath(backOff);
                    //jamocha.liftStowed();
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
        Constants.setConstants(FConstants.class, LConstants.class);
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
        telemetry.addData("Velocity Magnitude", follower.getVelocityMagnitude());
        telemetry.addData("Is the pinpoint screwed", follower.isLocalizationNAN());
        telemetry.addData("Parametric Path Completion", follower.getCurrentTValue());
        telemetry.update();
    }
    @Override
    public void start(){
        setPathState(0);
    }
}


