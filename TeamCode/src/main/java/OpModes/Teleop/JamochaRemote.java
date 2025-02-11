package OpModes.Teleop;



import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import Common.Robot.IntakeOuttakeV2;


@TeleOp
public class JamochaRemote  extends OpMode {
    // Used for teleop driving enhancement
    private Follower follower;

    IntakeOuttakeV2 jamocha = new IntakeOuttakeV2();


    //Drive Motors
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    enum State{
        //START,

        //Start state to fit within 18 inches
        SCORE_HIGH_CHAMBER,

        SCORE_HIGH_CHAMBER_FORWARD,

        SCORE_HIGH_CHAMBER_DROP,

        SCORE_HIGH_CHAMBER_FORWARD_DROP,

        PRE_SCORE_HIGH_CHAMBER,
        //Line up to score at high chamber
      //  SCORE_CHAMBER,
        //Score at the high chamber
        SCORE_LOW_CHAMBER,

        SCORE_LOW_CHAMBER_FORWARD,

        SCORE_LOW_CHAMBER_DROP,

        SCORE_LOW_CHAMBER_FORWARD_DROP,

        PRE_SCORE_LOW_CHAMBER,
        //Line up to score at low chamber
        PRE_SCORE_HIGH_BASKET,
        //Line up to score at the low chamber
        SCORE_HIGH_BASKET,

        SCORE_HIGH_BASKET_FORWARD,

        SCORE_HIGH_BASKET_DROP,

        SCORE_HIGH_BASKET_FORWARD_DROP,
        //Score at the high basket
        PRE_SCORE_LOW_BASKET,
        //Line up to score at the low basket
        SCORE_LOW_BASKET,

        SCORE_LOW_BASKET_FORWARD,

        SCORE_LOW_BASKET_DROP,

        SCORE_LOW_BASKET_FORWARD_DROP,

        //Score at the low basket
        DRIVE,
        //Pick up specimen from wall
        PICK_UP_SPECIMEN,

        PICK_UP_SPECIMEN_BACK,

        SPECIMEN_BACK_TO_DRIVE,

        DRIVE_TO_SPECIMEN_BACK,
        GRAB_SPECIMEN,
        //Retract slides for driving
        SEARCH_FOR_SAMPLE,
        //Deploys intake to search for a sample
        PICK_UP_SAMPLE,
        //Pick up sample from the submersible
        TRANSFER_SAMPLE,
        //Hand off sample from intake to outtake

        EXPEL_SAMPLE,

        EXPEL_WRONG_SAMPLE,
        MANUAL_CONTROL,
        //Driver takes manual control of slides
        RECALIBRATE
        //Rezero motor encoders
    }
    //JamochaRemote.State
    State state = State.DRIVE;
    @Override
    public void init(){
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

    }
    @Override
    public void loop() {
        //Teleop Driving Enhanced
        follower.setTeleOpMovementVectors(-gamepad1.right_stick_y + gamepad1.right_trigger - gamepad1.left_trigger, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
        follower.update();
        telemetry.addData("State", state);
        telemetry.update();
        //State Change
        if (gamepad1.a) {
            state = State.DRIVE;
        }

        //Prescore States
        if (gamepad2.dpad_up) {
            state = State.PRE_SCORE_HIGH_CHAMBER;
        }

        if (gamepad2.dpad_down) {
            state = State.PRE_SCORE_LOW_CHAMBER;
        }

        if (gamepad2.dpad_right) {
            state = State.PRE_SCORE_HIGH_BASKET;
        }

        if (gamepad2.dpad_left) {
            state = State.PRE_SCORE_LOW_BASKET;
        }







        if (gamepad1.back) {
            state = State.MANUAL_CONTROL;
        }

        switch (state){

            case DRIVE:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm
                jamocha.armChamber();

                //arm turret
                jamocha.armTurretBackward();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftSwing();

                if (gamepad2.y){
                    state = State.DRIVE_TO_SPECIMEN_BACK;
                }

                break;

            case PRE_SCORE_HIGH_CHAMBER:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm
                jamocha.armChamber();

                //arm turret


                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();

                if (gamepad2.b){
                    state= State.SCORE_HIGH_CHAMBER;
                }
                if (gamepad2.a){
                    state=State.SCORE_HIGH_CHAMBER_FORWARD;
                }
                break;


            case SCORE_HIGH_CHAMBER:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm
                jamocha.armChamber();

                //arm turret
                jamocha.armTurretBackward();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();

                    if (gamepad2.right_bumper){
                        state= State.SCORE_HIGH_CHAMBER_DROP;
                    }
                break;


            case SCORE_HIGH_CHAMBER_DROP:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm
                jamocha.armHorizontal();

                //arm turret
                jamocha.armTurretBackward();

                //claw
                if(!gamepad2.right_bumper) {
                    jamocha.clawOpen();
                }

                //lift
                jamocha.liftHighChamber();

                if (gamepad2.a) {
                    state = State.PICK_UP_SPECIMEN;
                }

                if (gamepad2.b) {
                    state = State.PICK_UP_SPECIMEN_BACK;
                }
                if (gamepad2.x) {
                    state = State.DRIVE;
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

                //arm
                jamocha.armChamber();

                //arm turret
                jamocha.armTurretForward();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighChamber();

                if (gamepad2.right_bumper){
                    state= State.SCORE_HIGH_CHAMBER_FORWARD_DROP;
                }

                break;


            case SCORE_HIGH_CHAMBER_FORWARD_DROP:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm
                jamocha.armHorizontal();

                //arm turret
                jamocha.armTurretForward();

                //claw
               if(!gamepad2.right_bumper) {
                   jamocha.clawOpen();
               }
                //lift
                jamocha.liftHighChamber();

                if (gamepad2.a) {
                    state = State.PICK_UP_SPECIMEN;
                }

                if (gamepad2.b) {
                    state = State.PICK_UP_SPECIMEN_BACK;
                }
                if (gamepad2.x) {
                    state = State.DRIVE;
                }

                break;


            case PRE_SCORE_HIGH_BASKET:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm
                jamocha.armChamber();

                //arm turret


                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighBasket();

                if (gamepad2.b){
                    state= State.SCORE_HIGH_BASKET;
                }
                if (gamepad2.a){
                    state=State.SCORE_HIGH_BASKET_FORWARD;
                }
                break;

            case SCORE_HIGH_BASKET:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm
                jamocha.armChamber();

                //arm turret
                jamocha.armTurretBackward();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighBasket();

                if (gamepad2.right_bumper){
                    state= State.SCORE_HIGH_BASKET_DROP;
                }


                break;

            case SCORE_HIGH_BASKET_DROP:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm
                jamocha.armChamber();

                //arm turret
                jamocha.armTurretBackward();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftHighBasket();

                if (gamepad2.a) {
                    state = State.PICK_UP_SPECIMEN;
                }

                if (gamepad2.b) {
                    state = State.PICK_UP_SPECIMEN_BACK;
                }
                if (gamepad2.x) {
                    state = State.DRIVE;
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

                //arm
                jamocha.armChamber();

                //arm turret
                jamocha.armTurretForward();

                //claw
                jamocha.clawClosed();

                //lift
                jamocha.liftHighBasket();
                if (gamepad2.right_bumper){
                    state= State.SCORE_HIGH_BASKET_FORWARD_DROP;
                }

                break;

            case SCORE_HIGH_BASKET_FORWARD_DROP:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm
                jamocha.armChamber();

                //arm turret
                jamocha.armTurretForward();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftHighBasket();

                if (gamepad2.a) {
                    state = State.PICK_UP_SPECIMEN;
                }

                if (gamepad2.b) {
                    state = State.PICK_UP_SPECIMEN_BACK;
                }
                if (gamepad2.x) {
                    state = State.DRIVE;
                }

                break;

            case PICK_UP_SPECIMEN:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm
                jamocha.armHorizontal();

                //arm turret
                jamocha.armTurretForward();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftStowed();

                if (gamepad2.b){
                    state =State.PICK_UP_SPECIMEN_BACK;
                }

                break;

            case PICK_UP_SPECIMEN_BACK:
                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm
                jamocha.armHorizontal();

                //arm turret
                jamocha.armTurretBackward();

                //claw
                jamocha.clawOpen();

                //lift
                jamocha.liftStowed();

                if (gamepad2.a){
                    state =State.PICK_UP_SPECIMEN;
                }

                if (gamepad2.y){
                    state = State.SPECIMEN_BACK_TO_DRIVE;
                }

                break;

            case DRIVE_TO_SPECIMEN_BACK:
                //intermediate between drive and specimen back, lift slides so arm can switch position

                //lift
                jamocha.liftSwing();

                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm

                if (jamocha.liftHasArrived()){
                    jamocha.armHorizontal();

                }
                else {
                    jamocha.armTransfer();
                }

                //arm turret
                jamocha.armTurretBackward();

                //claw
                jamocha.clawOpen();


                if (gamepad2.b){
                    state =State.PICK_UP_SPECIMEN_BACK;

                }
                break;

            case SPECIMEN_BACK_TO_DRIVE:
                //intermediate between drive and specimen back, lift slides so arm can switch position

                //lift
                jamocha.liftSwing();

                //four bar
                jamocha.fourBarTransfer();
                jamocha.fourBarPitchTransfer();

                //intake
                jamocha.intakeOff();

                //horizontal slides
                jamocha.horizontalSlidesHome();

                //arm

                if (jamocha.liftHasArrived()){
                    jamocha.armTransfer();

                }
                else {
                    jamocha.armHorizontal();
                }

                //arm turret
                jamocha.armTurretBackward();

                //claw
                jamocha.clawOpen();

                if (gamepad2.x){
                    state =State.DRIVE;
                }
                break;

        }

    }



}


