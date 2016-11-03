/**
 * legSetup
 * Initialize legs and servos
 */
void legSetup()
{
  pwm.begin();
  pwm.setPWMFreq(60);  
  
  for(int i = 0; i < LEGS_COUNT; i++)
  {
    leg lg;
    lg.idx = i;
    LEGS[i] = lg;
  }
}

/**
 * homePosition
 * Move legs to default "ready" position
 */
void homePosition()
{
  for(int i = 0; i < LEGS_COUNT; i++)
  {
    moveTo(i, { POSE_ORIGIN_X, POSE_ORIGIN_Y });
  }
}

/**
 * calibrate
 * Move all servos to 90 degrees (incluede delta/slack)
 * to control leg angles 
 */
void calibrate()
{
  for(int i = 0; i < LEGS_COUNT; i++)
  {
    calibrateLeg(i);
  }
}
void calibrateLeg(int leg)
{
    Serial.print("calibrating leg:");
    Serial.println(leg);
    
    pwm.setPWM(SERVOS[leg][0],0,map(90 + SERVO_SLACK[leg][0], 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(SERVOS[leg][1],0,map(90 + SERVO_SLACK[leg][1], 0, 180, SERVOMIN, SERVOMAX));
}

void pin(int leg, struct point from, struct point to, bool append)
{
  int radX= radiusX(from.x,to.x);
  int cx = midPointX(radX,from.x,to.x); 

  struct arc ac;
  ac.origin = { cx, to.y };
  ac.radius = { radX,PIN_HEIGHT };

  ac.start_angle = 360;
  ac.end_angle = 180;

  /*
  if(shouldMirrow(leg))
  {
    ac.start_angle = 180;
    ac.end_angle = 360;
  }*/

  /*
  Serial.print("radx:");
  Serial.print(radX);
  Serial.print(", cx:");
  Serial.println(cx);
  */

  ellipticalTrajectoryForLeg(ac, &LEGS[leg], append, (to.x > from.x) );
  
  registerTrajectory(leg);
}

int midPointX(int cx, int x1, int x2) 
{
  return (x1 > x2)? x2 + cx: x1 + cx;
}

int radiusX(int x1, int x2)
{
  float cx = (x1 > x2)? x1 - x2: x2 - x1;
  cx = cx  / 2;
  int cx_ = (int) cx;
  return cx_;
}

/**
 * drag
 * Calculate the linear trajectory from the current x to the requested position
 * register the leg for moving 
 *
 * @param int leg  - index to the active leg
 * @param struct point p  - where to drag to
 */
void drag(int leg, struct point from, struct point to, bool append)
{
  struct line ln;
  ln.P1 = from;
  ln.P2 = to;
  linearTrajectoryForLeg(ln,&LEGS[leg], append);
  
  //reg for moving...
  registerTrajectory(leg);
}

/***************************************************
 * Given a requested endpoint find the joint angles and pass the values to the serovs
 * @param struct line ln - line to lerp
 * @param struct leg L - leg reference to setup trajectory 
****************************************************/
void ellipticalTrajectoryForLeg(struct arc ac, struct leg *L, bool append, bool reverse)
{
    float step_size = (ac.end_angle - ac.start_angle) / SAMPLING_GRANULARITY;
    int c_angle = ac.start_angle;
    c_angle += step_size;

    if(!append) L->trajectory_length = 0;

    int n = SAMPLING_GRANULARITY;
    for(int i = 0; i < SAMPLING_GRANULARITY; i++)
    {
        float x = ac.origin.x + ac.radius.x * cos(c_angle/RAD_TO_DEG);
        float y = ac.origin.y + ac.radius.y * sin(c_angle/RAD_TO_DEG);

        int x_ = (int) x;
        int y_ = (int) y;
        struct point p = { x_, y_ };

        n--;
        int dir = (reverse)? n:i;
        int idx = L->trajectory_length + dir;
        L->trajectory[idx] = p; 

        c_angle+= step_size; 
    }
    L->trajectory_length += SAMPLING_GRANULARITY;
}

/***************************************************
 * Given a requested endpoint find the joint angles and pass the values to the serovs
 * @param struct line ln - line to lerp
 * @param struct leg L - leg reference to setup trajectory 
****************************************************/
void linearTrajectoryForLeg(struct line ln, struct leg *L, bool append)
{
    //find the slope/delta
    float delta_x = ln.P2.x - ln.P1.x;
    float delta_y = ln.P2.y - ln.P1.y;

    //clear the trajectory
    if(!append) L->trajectory_length = 0;

    float distance = sqrt( ((delta_x) * (delta_x)) + ((delta_y) * (delta_y)) );

    if(distance < 1) return;

    //divide the line int the required number of points
    float step_size = distance / SAMPLING_GRANULARITY;
    float current_step = step_size; //skip the first point 

    
    for(int i = 0; i < SAMPLING_GRANULARITY; i++)
    {
       float increment = current_step / distance;
       float x = ln.P1.x + (increment * delta_x);
       float y = ln.P1.y + (increment * delta_y);

       int x_ = (int) x;
       int y_ = (int) y;
       struct point p = { x_, y_ };

       L->trajectory[L->trajectory_length + i] = p; 

       current_step += step_size;
    }

    L->trajectory_length += SAMPLING_GRANULARITY;
}

/***************************************************
 * Given a requested endpoint find the joint angles and pass the values to the serovs
 * @param int leg - index to leg
 * @param struct point p - requested position of endpoint
 **************************************************/
void moveTo(int leg, struct point p)
{
  pose po = IK(FEMUR_SIZE,TIBIA_SIZE,p);  
  if(shouldMirrow(leg))
  {
     po.hip_angle = 180 - po.hip_angle; 
     po.knee_angle = 180 - po.knee_angle;
  }
  setRotationAngles(leg, po.hip_angle, po.knee_angle);
}

bool shouldMirrow(int leg)
{
  return (leg == BACK_RIGHT || leg == FRONT_LEFT);
}

/***************************************************
 * Command the rotation angle of the hip and knee to the servos
 * @param int leg - index to leg
 * @param int hip - hip angle
 * @param int knee - knee angle
 ***************************************************/
void setRotationAngles(int leg, int hip, int knee)
{
  int h = hip + SERVO_SLACK[leg][0];
  int k = knee + SERVO_SLACK[leg][1];

  pwm.setPWM(SERVOS[leg][0],0,map(h, 0, 180, SERVOMIN, SERVOMAX));
  pwm.setPWM(SERVOS[leg][1],0,map(k, 0, 180, SERVOMIN, SERVOMAX));

}

/**********************************************************
 * Inverse Kinematic function for a two link planar system.
 * Given the size of the two links an a desired position, 
 * it returns a pose struct with angles for both links
 * @param int L1 - length of 1. segment
 * @param int L2 - length of 2. segment
 * @param struct point P2 - endpoint position to compute
 **********************************************************/
struct pose IK(int L1, int L2, struct point P2)
{
    struct pose a;
    struct point P1 = { HIP_ORIGIN_X, HIP_ORIGIN_Y };  //origin point (hip)

    int H1 = P2.x - P1.x;  //delta on the x axis
    int H2 = P2.y - P1.y;  //delta on the y axis

    //this is the hypothenuse between the origin and the target
    float K = sqrt( pow(H1,2) + pow(H2,2));

    //the hypothenuse can not be larget the the sum of the segments
    if( K > (L1 + L2 )) K = L1 + L2;

    //angle between L1 & L2 - the to segments, femur and tibia
    float A1 = acos(( pow(L1,2) + pow(L2,2) - pow(K,2))/ (2 * L1 * L2));

    //convert from radians to degrees and cast as an integer
    float A1_ = A1 * RAD_TO_DEG;
    int A1__ = (int) A1_;

    //get the angle between the hypothenuse and the first segment (femur)
    float A2 = acos( (pow(K,2) + pow(L1,2) - pow(L2,2)) / (2 * K * L1));

    //get the angle between the hypothenuse and the x axis
    float A3 = asin(H1/K);
    
    //get the hip rotational angle
    float A4 = (PI / 2) - (A2 + A3);  //add the two angles, substract it from half a circle

    float A4_ = A4 * RAD_TO_DEG;
    int A4__ = (int) A4_;

    a.hip_angle = A4__;
    a.knee_angle = A1__;

    float A2_ = A2 * RAD_TO_DEG;
    int A2__ = (int) A2_;
    
    float A3_ = A3 * RAD_TO_DEG;
    int A3__ = (int) A3_;

    /*
    Serial.print("L1:");
    Serial.print(L1);
    Serial.print(", L2:");
    Serial.print(L2);
    Serial.print(", P2.x:");
    Serial.print(P2.x);
    Serial.print(", P2.y:");
    Serial.print(P2.y);
    Serial.print(", H1:");
    Serial.print(H1);
    Serial.print(", H2:");
    Serial.print(H2);
    Serial.print(", K:");
    Serial.print(K);
    Serial.print(", A1:");
    Serial.print(A1__);
    Serial.print(", A2:");
    Serial.print(A2__);
    Serial.print(", A3:");
    Serial.print(A3__);
    Serial.print(", A4:");
    Serial.println(A4__);
    */

    return a;
}
