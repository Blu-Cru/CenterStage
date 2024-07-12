# Road Runner Quickstart

Check out the roadrunner [docs](https://rr.brott.dev/docs/v1-0/tuning/).

## hi this is blu crew center stage 2023-2024

### Installation:
- Clone the repository
- Open the project in Android Studio

### Build:
- Connect to the control hub (usb or wifi through REV hardware client)
- Build and run on Android Studio

### Code:

Our code is in the `TeamCode/src/main/java/org/firstinspires/ftc/teamcode` directory.
OpModes are in the `blucru/opmodes` directory.

### Controls:

driver 1:

    driving: 
        - left stick: field centric mecanum drive (forward, strafe)
        - right stick: mecanum drive (rotation)
        - right stick press: reset heading
        - left trigger: slow mode
        - right trigger: fast mode

        - b (circle): turn to face robot front to the left
        - x (square): turn to face robot front to the right

    dpad up: launch airplane

driver 2:

    intake controls:
        - a (cross): drop down intake
        - left bumper: intake rollers
        - right bumper: outtake rollers

    outtake presets:
        - a (cross): retract robot
        - b (circle): outtake high preset
        - x (square): outtake low preset
        - y (triangle): outtake mid preset

        - left trigger (hold): turn turret left
        - right trigger (hold): turn turret right

        - dpad left: release bottom pixel
        - dpad right: release both pixels
        - dpad down: toggle wrist

    hang:
        - left stick: manual hang control

    manual slide controls:
        - left joystick: increment by one pixel height
        - left joystick press: manual slide control