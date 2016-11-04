// *************** SERVO LIB SETUP
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  125 // 'minimum' pulse length count
#define SERVOMAX  550 // 'maximum' pulse length count 

// *************** SERVO IDS
#define HIP_FRONT_RIGHT 0
#define KNEE_FRONT_RIGHT 1

#define HIP_FRONT_LEFT 2
#define KNEE_FRONT_LEFT 3

#define HIP_BACK_RIGHT 4
#define KNEE_BACK_RIGHT 5

#define HIP_BACK_LEFT 6
#define KNEE_BACK_LEFT 7

// **************** SERVO SLACK DELTA
// add or remove degres from the joints to compensate for misaligment 
#define HIP_FRONT_RIGHT_DELTA -4
#define KNEE_FRONT_RIGHT_DELTA 2

#define HIP_FRONT_LEFT_DELTA -9
#define KNEE_FRONT_LEFT_DELTA 0

#define HIP_BACK_RIGHT_DELTA -3
#define KNEE_BACK_RIGHT_DELTA 0

#define HIP_BACK_LEFT_DELTA 8
#define KNEE_BACK_LEFT_DELTA 12

// *************** LEGS
#define MAX_HIP_ANGLE 180
#define MIN_HIP_ANGLE 0
#define MAX_KNEE_ANGLE 180
#define MIN_KNEE_ANGLE 0

#define FRONT_RIGHT 0
#define FRONT_LEFT 1
#define BACK_RIGHT 2
#define BACK_LEFT 3

#define TRAJECTORY_LENGTH 50 
#define SAMPLING_GRANULARITY 4


int SERVOS[4][2] =
{
  {HIP_FRONT_RIGHT,KNEE_FRONT_RIGHT},
  {HIP_FRONT_LEFT,KNEE_FRONT_LEFT},
  {HIP_BACK_RIGHT,KNEE_BACK_RIGHT},
  {HIP_BACK_LEFT,KNEE_BACK_LEFT}
};

int SERVO_SLACK[4][2] =
{
  {HIP_FRONT_RIGHT_DELTA,KNEE_FRONT_RIGHT_DELTA},
  {HIP_FRONT_LEFT_DELTA,KNEE_FRONT_LEFT_DELTA},
  {HIP_BACK_RIGHT_DELTA,KNEE_BACK_RIGHT_DELTA},
  {HIP_BACK_LEFT_DELTA,KNEE_BACK_LEFT_DELTA}
};


// **************** ROBOT GEOMETRY
#define FEMUR_SIZE 44
#define TIBIA_SIZE 67

// **************** POSE/STEP GEOMETRY
#define HIP_ORIGIN_X 0
#define HIP_ORIGIN_Y 0
#define POSE_ORIGIN_Y 100
#define POSE_ORIGIN_X 0

#define MAX_X  50
#define MIN_X -50
#define MAX_Y 110
#define MIN_Y 10

int POSE_WIDTH = 30;
int POSE_HEIGHT = 95;
int PIN_HEIGHT = 10;


// ***************** TEST VALUES
#define NUDGE_LEFT 8
#define NUDGE_RIGHT 7
#define NUDGE_UP 5
#define NUDGE_DOWN 6
#define NUDGE_DISTANCE 5


int CURRENT_X = 0;
int CURRENT_Y = 95;

// ***************** DATA STRUCTURES 
struct point {
    int x;
    int y;
};

struct line {
    point P1;
    point P2;
};

struct arc {
  point origin;
  point radius;
  int start_angle;
  int end_angle;
};

struct pose {
    int hip_angle;
    int knee_angle;
};


struct leg {
  int idx;
  struct point trajectory[TRAJECTORY_LENGTH];
  int trajectory_length = 0;
  point current_position = { POSE_ORIGIN_X, POSE_ORIGIN_Y };
};

struct leg LEGS[4] = {};

// ***************** CONTROLLER STRUCTURES 
int TRAJECTORY_INDEX[4] = { -1, -1, -1, -1 };
unsigned long LAST_TIMER;
unsigned long FRAME_INTERVAL = 60; //run each second
int LOOP = 0;

//action types
#define ACTION_DRAG 0
#define ACTION_PIN 1
#define MOVING_FORWARD 0
#define MOVING_BACKWARD 1
#define MOVING_LEFT 2
#define MOVING_RIGHT 3
#define TURN_INC 5 //turning increment - how much to shrink or enlarge the step size to facilite turning
#define LEGS_COUNT 4

int MOVING = 0;
int CURRENT_WALK = 0;
int CURRENT_STEP_COUNT = 4;

struct keyframe {
  int ACTION_TYPE;
  struct point from;
  struct point to;
};


int STEP_IDX = 0;
int KEYFRAME_LENGTH = 0;
int CURRENT_KEYFRAME = -1;
bool LOAD_NEXT_KEYFRAME = false;
int CURRENT_DIRECTION = MOVING_FORWARD;
keyframe KEYFRAMES[20][4];


int WALKS[1][4][LEGS_COUNT] ={
                      {
                          {1,2,3,4},
                          {2,3,4,1},
                          {3,4,1,2},
                          {4,1,2,3}
                       }
                   };


 int TILTS[1][4][LEGS_COUNT] ={
                      {
                          {-2, 1, -1, 0},
                          {1, -2, 0, -1},
                          {-1, 0, -2, 1},
                          {0, -1, 1, -2}
                       }
                   };  


int POSE_WIDTHS[4][LEGS_COUNT] = {
                          {POSE_WIDTH, POSE_WIDTH, POSE_WIDTH, POSE_WIDTH }, //FORWARD
                          {POSE_WIDTH - TURN_INC, POSE_WIDTH + TURN_INC, POSE_WIDTH - TURN_INC, POSE_WIDTH + TURN_INC }, //LEFT
                          {POSE_WIDTH + TURN_INC, POSE_WIDTH - TURN_INC, POSE_WIDTH + TURN_INC, POSE_WIDTH - TURN_INC }, //RIGHT
                          {POSE_WIDTH, POSE_WIDTH, POSE_WIDTH, POSE_WIDTH } //BACKWARD
                        };

bool shouldMirrowStep(int leg)
{
  return (leg == BACK_RIGHT || leg == BACK_LEFT);
}

#define COMMAND_FORWARD 5
#define COMMAND_BACKWARD 6
#define COMMAND_LEFT 7
#define COMMAND_RIGHT 8
#define COMMAND_STAND 1
#define COMMAND_SIT 2  
#define COMMAND_DECK 3
#define COMMAND_CALIBRATE 4


// ******************* MESSAGING ITEMS
// COMMON SETTINGS
// ----------------------------------------------------------------------------------------------
// These settings are used in both SW UART, HW UART and SPI mode
// ----------------------------------------------------------------------------------------------
#define BUFSIZE                        128   // Size of the read buffer for incoming data
#define VERBOSE_MODE                   true  // If set to 'true' enables debug output
#define BLE_READPACKET_TIMEOUT         50 //500   // Timeout in ms waiting to read a response


// SOFTWARE UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins that will be used for 'SW' serial.
// You should use this option if you are connecting the UART Friend to an UNO
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SWUART_RXD_PIN       9    // Required for software serial!
#define BLUEFRUIT_SWUART_TXD_PIN       10   // Required for software serial!
#define BLUEFRUIT_UART_CTS_PIN         11   // Required for software serial!
#define BLUEFRUIT_UART_RTS_PIN         -1   // Optional, set to -1 if unused


// HARDWARE UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the HW serial port you are using. Uncomment
// this line if you are connecting the BLE to Leonardo/Micro or Flora
// ----------------------------------------------------------------------------------------------
#ifdef Serial1    // this makes it not complain on compilation if there's no Serial1
  #define BLUEFRUIT_HWSERIAL_NAME      Serial1
#endif


// SHARED UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following sets the optional Mode pin, its recommended but not required
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_UART_MODE_PIN        12    // Set to -1 if unused


// SHARED SPI SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins to use for HW and SW SPI communication.
// SCK, MISO and MOSI should be connected to the HW SPI pins on the Uno when
// using HW SPI.  This should be used with nRF51822 based Bluefruit LE modules
// that use SPI (Bluefruit LE SPI Friend).
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SPI_CS               8
#define BLUEFRUIT_SPI_IRQ              7
#define BLUEFRUIT_SPI_RST              4    // Optional but recommended, set to -1 if unused

// SOFTWARE SPI SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins to use for SW SPI communication.
// This should be used with nRF51822 based Bluefruit LE modules that use SPI
// (Bluefruit LE SPI Friend).
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SPI_SCK              13
#define BLUEFRUIT_SPI_MISO             12
#define BLUEFRUIT_SPI_MOSI             11

//****************************

