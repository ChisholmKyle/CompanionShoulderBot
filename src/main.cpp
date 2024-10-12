#include <Arduino.h>

/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These drivers use I2C to communicate, 2 pins are required to
  interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVO_PULSE_MIN 100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVO_PULSE_MAX 530 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_ANGLE_MIN -105.0
#define SERVO_ANGLE_MAX 105.0
// trajectory parameters
#define SERVO_ACC_MAX 300.0
#define SERVO_VEL_MAX 90.0
// timings
#define SERVO_FREQ 50            // Analog servos run at ~50 Hz updates
#define SERVO_CONTROL_STEP_MS 40 // Controller running at 25Hz
// number of servos
#define NUM_SERVOS 5
// IDs of each servo
#define SERVO_ID_SHOULDER_ROTATE 0
#define SERVO_ID_SHOULDER_RAISE 1
#define SERVO_ID_ELBOW 2
#define SERVO_ID_WRIST 3
#define SERVO_ID_EYE 4
// joystick config
#define JOY_X A0
#define JOY_Y A1
#define JOY_SWITCH 9

struct JoyConfig
{
    int deadzone;
    int max;
    int min;
    float scale;
};

enum JoyQuadrant
{
    CENTRE,
    NORTH,
    NORTH_EAST,
    EAST,
    SOUTH_EAST,
    SOUTH,
    SOUTH_WEST,
    WEST,
    NORTH_WEST
};

struct TrajectoryParameters
{
    float step_size;
    float max_velocity;
    float max_acceleration;
    float min_displacement;
    float min_velocity;
    float min_acceleration;
};

struct Trajectory
{
    float position;
    float velocity;
    float acceleration;
};

enum SequenceMode
{
    STOP,
    ZERO,
    WIGGLE,
    INTIMIDATE
};

struct SequenceTarget
{
    float speed_scale;
    uint32_t duration;
    int target[NUM_SERVOS];
};

struct SequenceState
{
    SequenceMode mode;
    uint32_t start;
    int index;
};

// function declarations - servo control
void set_servo_angle(uint8_t servo_id, int angle);
Trajectory smooth_goto(const Trajectory &prev, const TrajectoryParameters &params, const float target);
SequenceState change_mode(const SequenceMode mode, const SequenceTarget sequence[], const uint32_t now,
                          int target[], float &velocity);
void update_mode_sequence(const SequenceTarget sequence[], const int sequence_count, const uint32_t now,
                          SequenceState &state, int target[], float &velocity);

// function declarations - servo control
JoyQuadrant xy_to_quadrant(int x, int y);
JoyQuadrant get_joy_quadrant(const JoyConfig &config_x, const JoyConfig &config_y, int val_x, int val_y);

// objects - servo
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint8_t servos_list[NUM_SERVOS] = {SERVO_ID_SHOULDER_ROTATE, SERVO_ID_SHOULDER_RAISE, SERVO_ID_ELBOW, SERVO_ID_WRIST, SERVO_ID_EYE};

const int command_zero[NUM_SERVOS] = {0, 0, 0, 90, 0};

const int sequence_wiggle_count = 2;
const SequenceTarget sequence_wiggle[2] = {
    {.speed_scale = 0.5,
     .duration = 2000,
     .target = {20, -30, 40, 100, 10}},
    {.speed_scale = 0.5,
     .duration = 3000,
     .target = {-20, 0, -40, 70, -10}}};

const int sequence_intimidate_count = 2;
const SequenceTarget sequence_intimidate[2] = {
    {.speed_scale = 1.0,
     .duration = 2000,
     .target = {-5, -40, -40, 90, 0}},
    {.speed_scale = 1.0,
     .duration = 3000,
     .target = {5, -10, -10, 45, 10}}};

const int sequence_zero_count = 1;
const SequenceTarget sequence_zero[1] = {
    {.speed_scale = 1.0,
     .duration = 10000,
     .target = {0, 0, 0, 90, 0}}};

SequenceState servo_state = {};
Trajectory servo_traj[NUM_SERVOS] = {};
TrajectoryParameters servo_traj_params = {};
uint32_t timing_prev = 0;

JoyConfig joy_x = {};
JoyConfig joy_y = {};

void setup()
{
    Serial.begin(9600);
    // Serial.println("8 channel Servo test!");

    pinMode(JOY_SWITCH, INPUT);
    digitalWrite(JOY_SWITCH, HIGH);

    joy_x.deadzone = 30;
    joy_x.max = 1023;
    joy_x.min = 0;
    joy_x.scale = 1.0;

    joy_y.deadzone = 30;
    joy_y.max = 1023;
    joy_y.min = 0;
    joy_y.scale = 1.0;

    pwm.begin();
    /*
     * In theory the internal oscillator (clock) is 25MHz but it really isn't
     * that precise. You can 'calibrate' this by tweaking this number until
     * you get the PWM update frequency you're expecting!
     * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     * is used for calculating things like writeMicroseconds()
     * Analog servos run at ~50 Hz updates, It is importaint to use an
     * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
     *    the I2C PCA9685 chip you are setting the value for.
     * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
     *    expected value (50Hz for most ESCs)
     * Setting the value here is specific to each individual I2C PCA9685 chip and
     * affects the calculations for the PWM update frequency.
     * Failure to correctly set the int.osc value will cause unexpected PWM results
     */
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

    delay(10);

    // trajectory parameters
    servo_traj_params.step_size = static_cast<float>(SERVO_CONTROL_STEP_MS) / 1000.0;
    servo_traj_params.max_velocity = SERVO_VEL_MAX;
    servo_traj_params.max_acceleration = SERVO_ACC_MAX;
    servo_traj_params.min_displacement = 0.2;
    servo_traj_params.min_velocity = 0.5;
    servo_traj_params.min_acceleration = 0.5;

    // initialize trajectory stopped at zero position
    servo_state.mode = STOP;
    for (int k = 0; k < NUM_SERVOS; k++)
    {
        servo_traj[k].position = command_zero[k];
        set_servo_angle(servos_list[k], servo_traj[k].position);
    }

    timing_prev = millis();
    servo_state.start = timing_prev;
}

void loop()
{
    static int target[NUM_SERVOS] = {0, 0, 0, 0, 0};

    const uint32_t now = millis();
    const uint32_t elapsed = (now - timing_prev);
    if (elapsed >= SERVO_CONTROL_STEP_MS)
    {
        timing_prev = timing_prev + SERVO_CONTROL_STEP_MS;

        const JoyQuadrant quadrant = get_joy_quadrant(joy_x, joy_y, analogRead(JOY_X), analogRead(JOY_Y));
        const bool button_down = (digitalRead(JOY_SWITCH) == LOW);

        if (button_down && (servo_state.mode != STOP))
        {
            // Change mode to stop
            servo_state.mode = STOP;
            // stop target same as current traj position
            for (int k = 0; k < NUM_SERVOS; k++)
            {
                target[k] = servo_traj[k].position;
                servo_traj[k].velocity = 0.0;
                servo_traj[k].acceleration = 0.0;
            }
            Serial.println("STOP");
        }
        else if ((quadrant == NORTH) && (servo_state.mode != WIGGLE))
        {
            // Change mode to wiggle
            servo_state = change_mode(WIGGLE, sequence_wiggle, now, target, servo_traj_params.max_velocity);
            Serial.println("WIGGLE");
        }
        else if ((quadrant == SOUTH) && (servo_state.mode != INTIMIDATE))
        {
            // Change mode to intimidate
            servo_state = change_mode(INTIMIDATE, sequence_intimidate, now, target, servo_traj_params.max_velocity);
            Serial.println("INTIMIDATE");
        }
        else if ((quadrant == EAST) && (servo_state.mode != ZERO))
        {
            // Change mode to zero
            servo_state = change_mode(ZERO, sequence_zero, now, target, servo_traj_params.max_velocity);
            Serial.println("ZERO");
        }

        // run sequence
        if (servo_state.mode == WIGGLE)
        {
            update_mode_sequence(sequence_wiggle, sequence_wiggle_count, now,
                                 servo_state, target, servo_traj_params.max_velocity);
        }
        else if (servo_state.mode == INTIMIDATE)
        {
            update_mode_sequence(sequence_intimidate, sequence_intimidate_count, now,
                                 servo_state, target, servo_traj_params.max_velocity);
        }
        else if (servo_state.mode == ZERO)
        {
            update_mode_sequence(sequence_zero, sequence_zero_count, now,
                                 servo_state, target, servo_traj_params.max_velocity);
        }

        // evaluate trajectory
        for (int k = 0; k < NUM_SERVOS; k++)
        {
            servo_traj[k] = smooth_goto(servo_traj[k], servo_traj_params, target[k]);
            set_servo_angle(servos_list[k], servo_traj[k].position);
        }
    }
}

void set_servo_angle(uint8_t servo_id, int angle)
{
    angle = constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
    const uint16_t pulse = map(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_PULSE_MAX, SERVO_PULSE_MIN);
    pwm.setPWM(servo_id, 0, pulse);
}

Trajectory smooth_goto(const Trajectory &prev, const TrajectoryParameters &params, const float target)
{
    Trajectory cmd = {};

    // displacement
    const float displacement = target - prev.position;

    if (fabs(displacement) < params.min_displacement)
    {
        cmd.position = target;
        cmd.velocity = 0.0;
        cmd.acceleration = 0.0;
    }
    else
    {
        // desired velocity to reach target
        cmd.velocity = displacement / params.step_size;
        if (fabs(cmd.velocity) < params.min_velocity)
        {
            cmd.velocity = 0.0;
        }
        else if (cmd.velocity > params.max_velocity)
        {
            cmd.velocity = params.max_velocity;
        }
        else if (cmd.velocity < -params.max_velocity)
        {
            cmd.velocity = -params.max_velocity;
        }

        // check max acceleration
        cmd.acceleration = (cmd.velocity - prev.velocity) / params.step_size;
        if (fabs(cmd.acceleration) < params.min_acceleration)
        {
            cmd.acceleration = 0.0;
            cmd.velocity = prev.velocity;
        }
        else if (cmd.acceleration > params.max_acceleration)
        {
            cmd.acceleration = params.max_acceleration;
            cmd.velocity = prev.velocity + cmd.acceleration * params.step_size;
        }
        else if (cmd.acceleration < -params.max_acceleration)
        {
            cmd.acceleration = -params.max_acceleration;
            cmd.velocity = prev.velocity + cmd.acceleration * params.step_size;
        }

        // new position with constant acceleration
        cmd.position = prev.position + (prev.velocity + cmd.velocity) * params.step_size / 2.0;
    }

    return cmd;
}

SequenceState change_mode(const SequenceMode mode, const SequenceTarget sequence[], const uint32_t now,
                          int target[], float &velocity)
{
    SequenceState state = {};
    state.mode = mode;
    state.index = 0;
    state.start = now;
    for (int k = 0; k < NUM_SERVOS; k++)
    {
        target[k] = sequence[state.index].target[k];
    }
    velocity = sequence[state.index].speed_scale * SERVO_VEL_MAX;
    return state;
}

void update_mode_sequence(const SequenceTarget sequence[], const int sequence_count, const uint32_t now,
                          SequenceState &state, int target[], float &velocity)
{
    if ((now - state.start) >= sequence[state.index].duration)
    {
        // advance to next item in sequence
        state.start = now;
        state.index++;
        if (state.index >= sequence_count)
        {
            state.index = 0;
        }
        // apply next target in sequence
        for (int k = 0; k < NUM_SERVOS; k++)
        {
            target[k] = sequence[state.index].target[k];
        }
        velocity = sequence[state.index].speed_scale * SERVO_VEL_MAX;
    }
}

JoyQuadrant xy_to_quadrant(int x, int y)
{
    JoyQuadrant quadrant = CENTRE;
    if ((x != 0) || (y != 0))
    {
        if (x == 0)
        {
            return (y > 0 ? NORTH : SOUTH);
        }
        else if (y == 0)
        {
            return (x > 0 ? EAST : WEST);
        }
        else if (x > 0)
        {
            return (y > 0 ? NORTH_EAST : SOUTH_EAST);
        }
        else if (x < 0)
        {
            return (y > 0 ? NORTH_WEST : SOUTH_WEST);
        }
    }
    return quadrant;
}

JoyQuadrant get_joy_quadrant(const JoyConfig &config_x, const JoyConfig &config_y, int val_x, int val_y)
{
    int x = map(val_x, config_x.min, config_x.max, -100, 100);
    int y = map(val_y, config_y.min, config_y.max, -100, 100);

    if (abs(x) < config_x.deadzone)
    {
        x = 0;
    }
    if (abs(y) < config_y.deadzone)
    {
        y = 0;
    }
    return xy_to_quadrant(x, y);
}
