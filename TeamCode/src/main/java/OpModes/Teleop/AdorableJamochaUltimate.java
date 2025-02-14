package OpModes.Teleop;



import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import Common.Robot.IntakeOuttakeV2;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp
public class AdorableJamochaUltimate  extends OpMode {
    // Used for teleop driving enhancement
    private Follower follower;
    private Timer jamochaPaws;

    IntakeOuttakeV2 jamocha = new IntakeOuttakeV2();

    private boolean TurretStateForward;

    //Drive Motors
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    enum State {
        //START,\
        MANUAL_CONTROL,
        DRIVE_ARM_BACK,
        DRIVE_ARM_FORWARD,

        SPECIMEN_FORWARD,
        SPECIMEN_FORWARD_GRAB,
        SPECIMEN_TO_BACK_1,
        SPECIMEN_TO_BACK_2,
        SPECIMEN_BACK,
        SPECIMEN_BACK_GRAB,
        SPECIMEN_TO_FORWARD_1,
        SPECIMEN_TO_FORWARD_2,

        INTAKE_SEARCH,
        INTAKE_EXPEL,
        INTAKE_GRAB,
        INTAKE_RETRACT,
        INTAKE_TRANSFER_1,
        INTAKE_TRANSFER_2,
        INTAKE_TRANSFER_3,
        INTAKE_TRANSFER_4,

        PRE_LOW_BASKET_NEUTRAL,
        PRE_LOW_BASKET_FORWARD,
        PRE_LOW_BASKET_FORWARD_1,
        SCORE_LOW_BASKET_FORWARD,
        DROP_LOW_BASKET_FORWARD,
        PRE_LOW_BASKET_BACK,
        PRE_LOW_BASKET_BACK_1,
        SCORE_LOW_BASKET_BACK,
        DROP_LOW_BASKET_BACK,

        PRE_HIGH_BASKET_NEUTRAL,
        PRE_HIGH_BASKET_FORWARD,
        PRE_HIGH_BASKET_FORWARD_1,
        SCORE_HIGH_BASKET_FORWARD,
        DROP_HIGH_BASKET_FORWARD,
        PRE_HIGH_BASKET_BACK,
        PRE_HIGH_BASKET_BACK_1,
        SCORE_HIGH_BASKET_BACK,
        DROP_HIGH_BASKET_BACK,

        PRE_LOW_CHAMBER_NEUTRAL,
        PRE_LOW_CHAMBER_FORWARD,
        PRE_LOW_CHAMBER_FORWARD_1,
        SCORE_LOW_CHAMBER_FORWARD,
        DROP_LOW_CHAMBER_FORWARD,
        PRE_LOW_CHAMBER_BACK,
        PRE_LOW_CHAMBER_BACK_1,
        SCORE_LOW_CHAMBER_BACK,
        DROP_LOW_CHAMBER_BACK,

        PRE_HIGH_CHAMBER_NEUTRAL,
        PRE_HIGH_CHAMBER_FORWARD,
        PRE_HIGH_CHAMBER_FORWARD_1,
        SCORE_HIGH_CHAMBER_FORWARD,
        DROP_HIGH_CHAMBER_FORWARD,
        PRE_HIGH_CHAMBER_BACK,
        PRE_HIGH_CHAMBER_BACK_1,
        SCORE_HIGH_CHAMBER_BACK,
        DROP_HIGH_CHAMBER_BACK,

    }



    //JamochaRemote.State
    State state = State.DRIVE_ARM_BACK;
    @Override
    public void init(){
        Constants.setConstants(FConstants .class, LConstants .class);
        //teleop driving enhancement initialization
        follower = new Follower(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();


        jamocha.initIntake(hardwareMap);
        jamocha.initOuttake(hardwareMap);
        jamocha.clawClosed();

        jamochaPaws = new Timer();


    }
    @Override
    public void loop() {
        //Teleop Driving Enhanced
        follower.setTeleOpMovementVectors(-gamepad1.right_stick_y + gamepad1.right_trigger - gamepad1.left_trigger, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
        follower.update();
        telemetry.addData("State", state);
        telemetry.update();
        //State Change

        //Prescore States

        if (gamepad1.back) {
            state = State.MANUAL_CONTROL;
        }
        if (gamepad1.start) {
            state = State.DRIVE_ARM_BACK;
        }
        switch (state){
            case MANUAL_CONTROL:
                jamocha.liftManualControl(gamepad2.right_stick_y);
                jamocha.horizontalSlidesManualControl(gamepad2.left_stick_y);
                if (gamepad2.back){
                state = State.DRIVE_ARM_BACK;
                }
            case DRIVE_ARM_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw

                //lift
                jamocha.liftSwing();

                TurretStateForward = false;
                if (gamepad2.a){
                    state = State.DRIVE_ARM_FORWARD;
                }
                if (gamepad2.y){
                    state = State.SPECIMEN_BACK;
                }
                if (gamepad2.left_bumper){
                    state = State.INTAKE_SEARCH;
                }
                if (gamepad2.dpad_left){
                    state = State.PRE_LOW_BASKET_NEUTRAL;
                }
                if (gamepad2.dpad_right){
                    state = State.PRE_HIGH_BASKET_NEUTRAL;
                }
                if (gamepad2.dpad_down){
                    state = State.PRE_LOW_CHAMBER_NEUTRAL;
                }
                if (gamepad2.dpad_up){
                    state = State.PRE_HIGH_CHAMBER_NEUTRAL;
                }


                break;

            case DRIVE_ARM_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armChamber();

                //claw

                //lift
                jamocha.liftSwing();

                TurretStateForward = true;

                if (gamepad2.y){
                    state = State.SPECIMEN_FORWARD;
                }
                if (gamepad2.b){
                    state = State.DRIVE_ARM_BACK;
                }
                if (gamepad2.dpad_left){
                    state = State.PRE_LOW_BASKET_NEUTRAL;
                }
                if (gamepad2.dpad_right){
                    state = State.PRE_HIGH_BASKET_NEUTRAL;
                }
                if (gamepad2.dpad_down){
                    state = State.PRE_LOW_CHAMBER_NEUTRAL;
                }
                if (gamepad2.dpad_up){
                    state = State.PRE_HIGH_CHAMBER_NEUTRAL;
                }
                break;

            case SPECIMEN_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armHorizontal();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftStowed();

                TurretStateForward = true;

                if (gamepad2.right_bumper){
                    state = State.SPECIMEN_FORWARD_GRAB;
                }

                if (gamepad2.b){
                        jamochaPaws.resetTimer();
                   state = State.SPECIMEN_TO_BACK_1;
                }

                break;

            case SPECIMEN_FORWARD_GRAB:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward(); //?

                //arm
                jamocha.armHorizontal();

                //claw
                jamocha.clawClosed(); //?

                //lift
                jamocha.liftStowed();

                TurretStateForward = false;

                if (gamepad2.a){
                    state = State.DRIVE_ARM_FORWARD;
                }
                if (gamepad2.x){
                    state = State.SPECIMEN_FORWARD;
                }

                break;

            case SPECIMEN_TO_BACK_1:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftStowed();

                TurretStateForward = true;

                if (jamochaPaws.getElapsedTimeSeconds() >.2){
                    jamochaPaws.resetTimer();
                    state = State.SPECIMEN_TO_BACK_2;
                }

                break;

            case SPECIMEN_TO_BACK_2:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftStowed();

                TurretStateForward = false;

                if (jamochaPaws.getElapsedTimeSeconds()>.2){
                    state = State.SPECIMEN_BACK;
                }

                break;

            case SPECIMEN_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                //jamocha.armHorizontal();
                jamocha.armPickup();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftStowed();

                TurretStateForward = false;

                if (gamepad2.right_bumper){
                    state = State.SPECIMEN_BACK_GRAB;
                }
                if (gamepad2.a){
                    jamochaPaws.resetTimer();
                    state = State.SPECIMEN_TO_FORWARD_1;
                }

                break;

            case SPECIMEN_BACK_GRAB:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armHorizontal();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftStowed();

                TurretStateForward = false;

                if (gamepad2.x){
                    state = State.SPECIMEN_BACK;
                }

                if (gamepad2.b){
                    state = State.DRIVE_ARM_BACK;
                }

                break;

            case SPECIMEN_TO_FORWARD_1:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftStowed();

                TurretStateForward = false;

                if (jamochaPaws.getElapsedTimeSeconds()>.2){
                    jamochaPaws.resetTimer();
                    state = State.SPECIMEN_TO_FORWARD_2;
                }

                break;

            case SPECIMEN_TO_FORWARD_2:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftStowed();

                TurretStateForward = true;


                if (jamochaPaws.getElapsedTimeSeconds()>.2){
                    state = State.SPECIMEN_FORWARD;
                }

                break;

            case INTAKE_SEARCH:
                //four bar
                jamocha.fourBarSearch();
                jamocha.fourBarPitchSearch();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesManualControl(gamepad2.left_stick_y);

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armTransfer();

                //claw
                jamocha.clawOpen();

                //lift
                //jamocha.liftSwing();

                jamocha.liftPreTransfer();

                TurretStateForward = false;


                if (gamepad2.x){
                    state = State.INTAKE_EXPEL;
                }
                if (gamepad2.left_trigger>.1){
                    state = State.INTAKE_GRAB;
                }
                if (gamepad2.a){
                    state = State.INTAKE_RETRACT;
                }
                if (gamepad2.b){
                    state = State.DRIVE_ARM_BACK;
                }

                break;

            case INTAKE_EXPEL:
                //four bar
                jamocha.fourBarSearch();
                jamocha.fourBarPitchSearch();

                //intake
                jamocha.intakeOut();

                //horizontal slides
                jamocha.horizontalSlidesManualControl(gamepad2.left_stick_y);

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armTransfer();

                //claw
                jamocha.clawOpen();

                //lift
                //jamocha.liftSwing();
                jamocha.liftPreTransfer();

                TurretStateForward = false;

                if (!gamepad2.x){
                    state = State.INTAKE_SEARCH;
                }

                break;

            case INTAKE_GRAB:
                //four bar
                jamocha.fourBarDown();
                jamocha.fourBarPitchSearch();

                //intake
                jamocha.intakeIn();

                //horizontal slides
                jamocha.horizontalSlidesManualControl(gamepad2.left_stick_y);

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armTransfer();

                //claw
                jamocha.clawOpen();

                //lift
                //jamocha.liftSwing();
                jamocha.liftPreTransfer();

                TurretStateForward = false;

                if (gamepad2.left_trigger <.1){
                    state = State.INTAKE_SEARCH;
                }

                break;

            case INTAKE_RETRACT:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armTransfer();

                //claw
                jamocha.clawOpen();

                //lift
                //jamocha.liftSwing();
                jamocha.liftPreTransfer();

                TurretStateForward = false;

                if (!gamepad2.a){
                    state = State.INTAKE_TRANSFER_1;
                }
                if (gamepad2.left_bumper){
                    state = State.INTAKE_SEARCH;
                }

                break;

            case INTAKE_TRANSFER_1:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOut();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armTransfer();

                //claw
                jamocha.clawOpen();

                //lift
                //jamocha.liftSwing();
                jamocha.liftPreTransfer();

                TurretStateForward = false;

                if (gamepad2.a){
                    state = State.INTAKE_TRANSFER_2;
                }

                break;

            case INTAKE_TRANSFER_2:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOut();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armTransfer();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftTransfer();

                TurretStateForward = false;

                if (!gamepad2.a){
                    state = State.INTAKE_TRANSFER_3;
                }

                break;

            case INTAKE_TRANSFER_3:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOut();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armTransfer();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftTransfer();

                TurretStateForward = false;

                if (gamepad2.a){
                    state = State.INTAKE_TRANSFER_4;
                }

                break;

            case INTAKE_TRANSFER_4:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOut();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armTransfer();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftSwing();

                TurretStateForward = false;

                if (!gamepad2.a){
                    state = State.DRIVE_ARM_BACK;
                }

                break;

            case PRE_LOW_BASKET_NEUTRAL:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                //jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftLowBasket();


                if (gamepad2.a){
                    state = State.PRE_LOW_BASKET_FORWARD;
                }
                if (gamepad2.b){
                    state = State.PRE_LOW_BASKET_BACK;
                }
                if (gamepad2.right_bumper){
                    if (TurretStateForward=true) {
                        state = State.SCORE_LOW_BASKET_FORWARD;
                    }
                }
                if (gamepad2.right_bumper){
                    if (TurretStateForward=false) {
                        state = State.SCORE_LOW_BASKET_BACK;
                    }
                }

                break;

            case PRE_LOW_BASKET_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftLowBasket();

                TurretStateForward = true;

                if (gamepad2.right_bumper){
                    state = State.SCORE_LOW_BASKET_FORWARD;
                }
                if (gamepad2.b){
                    state = State.PRE_LOW_BASKET_BACK;
                }

                break;

            case PRE_LOW_BASKET_FORWARD_1:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();

                TurretStateForward = false;

                if (gamepad2.a){
                    state = State.SCORE_LOW_BASKET_FORWARD;
                }

                break;

            case SCORE_LOW_BASKET_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armHorizontal();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftLowBasket();

                TurretStateForward = true;

                if (!gamepad2.right_bumper){
                    state = State.DROP_LOW_BASKET_FORWARD;
                }
                if (gamepad2.x){
                    state = State.PRE_LOW_BASKET_FORWARD_1;
                }

                break;

            case DROP_LOW_BASKET_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armHorizontal();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftLowBasket();

                TurretStateForward = true;

                if (gamepad2.a){
                    state = State.DRIVE_ARM_FORWARD;
                }
                if (gamepad2.b){
                    state = State.DRIVE_ARM_BACK;
                }

                break;

            case PRE_LOW_BASKET_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftLowBasket();

                TurretStateForward = false;

                if (gamepad2.right_bumper){
                    state = State.SCORE_LOW_BASKET_BACK;
                }
                if (gamepad2.a){
                    state = State.PRE_LOW_BASKET_FORWARD;
                }

                break;

            case PRE_LOW_BASKET_BACK_1:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();

                TurretStateForward = false;

                if (gamepad2.a){
                    state = State.SCORE_LOW_BASKET_BACK;
                }

                break;

            case SCORE_LOW_BASKET_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armHorizontal();//?

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftLowBasket();

                TurretStateForward = false;

                if (!gamepad2.right_bumper){
                    state = State.DROP_LOW_BASKET_BACK;
                }

                if (gamepad2.x){
                    state = State.PRE_LOW_BASKET_BACK_1;
                }

                break;

            case DROP_LOW_BASKET_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armHorizontal();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftLowBasket();

                TurretStateForward = false;

                if (gamepad2.a){
                    state = State.DRIVE_ARM_FORWARD;
                }
                if (gamepad2.b){
                    state = State.DRIVE_ARM_BACK;
                }

                break;

            case PRE_HIGH_BASKET_NEUTRAL:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                //jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighBasket();


                if (gamepad2.a){
                    state = State.PRE_HIGH_BASKET_FORWARD;
                }
                if (gamepad2.b){
                    state = State.PRE_HIGH_BASKET_BACK;
                }
                if (gamepad2.right_bumper){
                    if (TurretStateForward=true) {
                        state = State.SCORE_HIGH_BASKET_FORWARD;
                    }
                }
                if (gamepad2.right_bumper){
                    if (TurretStateForward=false) {
                        state = State.SCORE_HIGH_BASKET_BACK;
                    }
                }

                break;

            case PRE_HIGH_BASKET_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighBasket();

                TurretStateForward = true;

                if (gamepad2.right_bumper){
                    state = State.SCORE_HIGH_BASKET_FORWARD;
                }
                if (gamepad2.b){
                    state = State.PRE_HIGH_BASKET_BACK;
                }

                break;

            case PRE_HIGH_BASKET_FORWARD_1:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();

                TurretStateForward = false;

                if (gamepad2.a){
                    state = State.SCORE_HIGH_CHAMBER_FORWARD;
                }

                break;

            case SCORE_HIGH_BASKET_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armHorizontal();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighBasket();

                TurretStateForward = true;

                if (!gamepad2.right_bumper){
                    state = State.DROP_HIGH_BASKET_FORWARD;
                }
                if (gamepad2.x){
                    state = State.PRE_HIGH_BASKET_FORWARD_1;
                }

                break;

            case DROP_HIGH_BASKET_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armHorizontal();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftHighBasket();

                TurretStateForward = true;

                if (gamepad2.a){
                    state = State.DRIVE_ARM_FORWARD;
                }
                if (gamepad2.b){
                    state = State.DRIVE_ARM_BACK;
                }

                break;

            case PRE_HIGH_BASKET_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighBasket();

                TurretStateForward = false;

                if (gamepad2.right_bumper){
                    state = State.SCORE_HIGH_BASKET_BACK;
                }
                if (gamepad2.a){
                    state = State.PRE_HIGH_BASKET_FORWARD;
                }

                break;

            case PRE_HIGH_BASKET_BACK_1:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();

                TurretStateForward = false;

                if (gamepad2.a){
                    state = State.SCORE_HIGH_BASKET_BACK;
                }

                break;

            case SCORE_HIGH_BASKET_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armHorizontal();//?

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighBasket();

                TurretStateForward = false;

                if (!gamepad2.right_bumper){
                    state = State.DROP_HIGH_BASKET_BACK;
                }

                if (gamepad2.x){
                    state = State.PRE_HIGH_BASKET_BACK_1;
                }

                break;



            case PRE_LOW_CHAMBER_NEUTRAL:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                //jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftLowChamber();


                if (gamepad2.a){
                    state = State.PRE_LOW_CHAMBER_FORWARD;
                }
                if (gamepad2.b){
                    state = State.PRE_LOW_CHAMBER_BACK;
                }
                if (gamepad2.right_bumper){
                    if (TurretStateForward=true) {
                        state = State.SCORE_LOW_CHAMBER_FORWARD;
                    }
                }
                if (gamepad2.right_bumper){
                    if (TurretStateForward=false) {
                        state = State.SCORE_LOW_CHAMBER_BACK;
                    }
                }

                break;

            case PRE_LOW_CHAMBER_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftStowed();

                TurretStateForward = true;

                if (gamepad2.right_bumper){
                    state = State.SCORE_LOW_CHAMBER_FORWARD;
                }
                if (gamepad2.b){
                    state = State.PRE_LOW_CHAMBER_BACK;
                }

                break;

            case PRE_LOW_CHAMBER_FORWARD_1:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftStowed();

                TurretStateForward = false;

                if (gamepad2.a){
                    state = State.SCORE_LOW_CHAMBER_FORWARD;
                }

                break;

            case SCORE_LOW_CHAMBER_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armPickup();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftStowed();

                TurretStateForward = true;

                if (!gamepad2.right_bumper){
                    state = State.DROP_LOW_CHAMBER_FORWARD;
                }
                if (gamepad2.x){
                    state = State.PRE_LOW_CHAMBER_FORWARD_1;
                }

                break;

            case DROP_LOW_CHAMBER_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armPickup();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftStowed();

                TurretStateForward = true;

                if (gamepad2.a){
                    state = State.DRIVE_ARM_FORWARD;
                }
                if (gamepad2.b){
                    state = State.DRIVE_ARM_BACK;
                }

                break;

            case PRE_LOW_CHAMBER_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftStowed();

                TurretStateForward = false;

                if (gamepad2.right_bumper){
                    state = State.SCORE_LOW_CHAMBER_BACK;
                }
                if (gamepad2.a){
                    state = State.PRE_LOW_CHAMBER_FORWARD;
                }

                break;

            case PRE_LOW_CHAMBER_BACK_1:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftStowed();

                TurretStateForward = false;

                if (gamepad2.a){
                    state = State.SCORE_LOW_CHAMBER_BACK;
                }

                break;

            case SCORE_LOW_CHAMBER_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armPickup();//?

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftStowed();

                TurretStateForward = false;

                if (!gamepad2.right_bumper){
                    state = State.DROP_LOW_CHAMBER_BACK;
                }

                if (gamepad2.x){
                    state = State.PRE_LOW_CHAMBER_BACK_1;
                }

                break;

            case DROP_LOW_CHAMBER_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armPickup();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftStowed();
                TurretStateForward = false;

                if (gamepad2.a){
                    state = State.DRIVE_ARM_FORWARD;
                }
                if (gamepad2.b){
                    state = State.DRIVE_ARM_BACK;
                }

                break;

            case PRE_HIGH_CHAMBER_NEUTRAL:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                //jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();


                if (gamepad2.a){
                    state = State.PRE_HIGH_CHAMBER_FORWARD;
                }
                if (gamepad2.b){
                    state = State.PRE_HIGH_CHAMBER_BACK;
                }
                if (gamepad2.right_bumper){
                    if (TurretStateForward=true) {
                        state = State.SCORE_HIGH_CHAMBER_FORWARD;
                    }
                }
                if (gamepad2.right_bumper){
                    if (TurretStateForward=false) {
                        state = State.SCORE_HIGH_CHAMBER_BACK;
                    }
                }

                break;

            case PRE_HIGH_CHAMBER_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();

                TurretStateForward = true;

                if (gamepad2.right_bumper){
                    state = State.SCORE_HIGH_CHAMBER_FORWARD;
                }
                if (gamepad2.b){
                    state = State.PRE_HIGH_CHAMBER_BACK;
                }

                break;

            case PRE_HIGH_CHAMBER_FORWARD_1:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();

                TurretStateForward = false;

                if (gamepad2.a){
                    state = State.SCORE_HIGH_CHAMBER_FORWARD;
                }

                break;

            case SCORE_HIGH_CHAMBER_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //arm
                jamocha.armHorizontal();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();

                TurretStateForward = true;

                if (!gamepad2.right_bumper){
                    state = State.DROP_HIGH_CHAMBER_FORWARD;
                }
                if (gamepad2.x){
                    state = State.PRE_HIGH_CHAMBER_FORWARD_1;
                }

                break;

            case DROP_HIGH_CHAMBER_FORWARD:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretForward();

                //jamocha.armScore();

                //arm
                jamocha.armHorizontal();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftHighChamber();

                TurretStateForward = true;

                if (gamepad2.a){
                    state = State.DRIVE_ARM_FORWARD;
                }
                if (gamepad2.b){
                    state = State.DRIVE_ARM_BACK;
                }

                break;

            case PRE_HIGH_CHAMBER_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();

                TurretStateForward = false;

                if (gamepad2.right_bumper){
                    state = State.SCORE_HIGH_CHAMBER_BACK;
                }
                if (gamepad2.a){
                    state = State.PRE_HIGH_CHAMBER_FORWARD;
                }

                break;

            case PRE_HIGH_CHAMBER_BACK_1:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armChamber();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();

                TurretStateForward = false;

                if (gamepad2.a){
                    state = State.SCORE_HIGH_CHAMBER_BACK;
                }

                break;

            case SCORE_HIGH_CHAMBER_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //arm
                jamocha.armHorizontal();//?

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();

                TurretStateForward = false;

                if (!gamepad2.right_bumper){
                    state = State.DROP_HIGH_CHAMBER_BACK;
                }

                if (gamepad2.x){
                    state = State.PRE_HIGH_CHAMBER_BACK_1;
                }

                break;

            case DROP_HIGH_CHAMBER_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm turret
                jamocha.armTurretBackward();

                //jamocha.armScore();

                //arm
                jamocha.armHorizontal();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftHighChamber();

                TurretStateForward = false ;

                if (gamepad2.a){
                    state = State.DRIVE_ARM_FORWARD;
                }
                if (gamepad2.b){
                    state = State.DRIVE_ARM_BACK;
                }

                break;

        }

    }



}



