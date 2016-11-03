void walkLoadingTest()
{
  loadWalk(CURRENT_WALK, CURRENT_STEP_COUNT, MOVING_FORWARD, 0);

  for(int i=0; i < KEYFRAME_LENGTH;i++)
  {
    Serial.println("-------------------------");
    Serial.print("KEYFRAME:");
    Serial.println(i);

    for(int j = 0; j < 4; j++)
    {
      struct keyframe kf = KEYFRAMES[i][j];

      Serial.print((kf.ACTION_TYPE == ACTION_PIN)? "pin":"drag");
      Serial.print("leg:");
      Serial.print(j);
      Serial.print(", from: (");
      Serial.print(kf.from.x);
      Serial.print(",");
      Serial.print(kf.from.y);
      Serial.print(") to (");
      Serial.print(kf.to.x);
      Serial.print(",");
      Serial.print(kf.to.y);
      Serial.println(")");
    }

  }
}

void printKeyframe(int keyframe)
{
  Serial.print("Keyframe: {");
  for(int i=0; i < 4; i++)
  {
    if(i!=0) Serial.print(",");
    Serial.print(WALKS[0][keyframe][i]); 
  }
  Serial.println("}");
}

void testSetupKeyFrame(int pose_from)
{
  /*
  keyframe kf = setupKeyframe(pose_from);

  Serial.print((kf.ACTION_TYPE == ACTION_PIN)? "pin":"drag");
  Serial.print(", from: (");
  Serial.print(kf.from.x);
  Serial.print(",");
  Serial.print(kf.from.y);
  Serial.print(") to (");
  Serial.print(kf.to.x);
  Serial.print(",");
  Serial.print(kf.to.y);
  Serial.println(")");
  */
}

void stepTest(int leg)
{
 
  struct point from = { 30, CURRENT_Y };
  struct point to = { -30, CURRENT_Y };

  if(shouldMirrowStep(leg))
  {
      from = { -30, CURRENT_Y };
      to = { 30, CURRENT_Y };
  }
  
  pin(leg,from,to,false);

  from = { -30, CURRENT_Y };
  to = { 30, CURRENT_Y };

  if(shouldMirrowStep(leg))
  {
      from = { 30, CURRENT_Y };
      to = { -30, CURRENT_Y };
  }
  drag(leg,from,to,true);

  LOOP = 10;
}


void pinTest()
{
  Serial.println("Pin test");
  
  struct point from = { 30, CURRENT_Y };
  struct point to = { -30, CURRENT_Y };
  pin(FRONT_RIGHT,from,to,false);

  Serial.print("trajectory_length A:");
  Serial.println(LEGS[FRONT_RIGHT].trajectory_length);

  
  from = { -30, CURRENT_Y };
  to = { 30, CURRENT_Y };
  pin(FRONT_RIGHT,from,to,true);

  Serial.print("trajectory_length B:");
  Serial.println(LEGS[FRONT_RIGHT].trajectory_length);
  
  
  for(int i = 0; i < LEGS[FRONT_RIGHT].trajectory_length; i++)
  {
    Serial.print("(");
    Serial.print(LEGS[FRONT_RIGHT].trajectory[i].x);
    Serial.print(",");
    Serial.print(LEGS[FRONT_RIGHT].trajectory[i].y);
    Serial.println(")");
  }
  LOOP = 2;
}

void dragTest()
{
  
  struct point from = { -30, CURRENT_Y };
  struct point to = { 30, CURRENT_Y };
  drag(FRONT_RIGHT,from,to,false);

  Serial.print("trajectory_length A:");
  Serial.println(LEGS[FRONT_RIGHT].trajectory_length);

  from = { 30, CURRENT_Y };
  to = { -30, CURRENT_Y };
  drag(0,from,to,true);

  Serial.print("trajectory_length B:");
  Serial.println(LEGS[FRONT_RIGHT].trajectory_length);

  for(int i = 0; i < LEGS[FRONT_RIGHT].trajectory_length; i++)
  {
    Serial.print("(");
    Serial.print(LEGS[FRONT_RIGHT].trajectory[i].x);
    Serial.print(",");
    Serial.print(LEGS[FRONT_RIGHT].trajectory[i].y);
    Serial.println(")");
  }

  LOOP = 5;
  
}
void testCommand(uint8_t nr)
{
  nudge(nr);
}

void nudge(uint8_t dir)
{
  if(dir == NUDGE_LEFT) CURRENT_X+= NUDGE_DISTANCE;
  if(dir == NUDGE_RIGHT) CURRENT_X-= NUDGE_DISTANCE;
  if(dir == NUDGE_UP) CURRENT_Y-= NUDGE_DISTANCE;
  if(dir == NUDGE_DOWN) CURRENT_Y += NUDGE_DISTANCE;

  if(CURRENT_X > MAX_X) CURRENT_X = MAX_X;
  if(CURRENT_X < MIN_X) CURRENT_X = MIN_X;
  if(CURRENT_Y > MAX_Y) CURRENT_Y = MAX_Y;
  if(CURRENT_Y < MIN_Y) CURRENT_Y = MIN_Y;

  struct point p = { CURRENT_X, CURRENT_Y };
  moveTo(0, p);
}

void testLinearTrajectories()
{
  struct line ln;
  ln.P1 = { 0, 0 };
  ln.P2 = { -30, 30 };

  testLinearTrajectory(ln);

  return;
  ln.P2 = { 0, 20 };

  testLinearTrajectory(ln);

  ln.P1 = { 20, 20 };

  testLinearTrajectory(ln);

}

void testLinearTrajectory(struct line ln){
  
  struct leg lg;
  linearTrajectoryForLeg(ln,&lg, false);

  Serial.print("Linear trajectory for (");
  Serial.print(ln.P1.x);
  Serial.print(",");
  Serial.print(ln.P1.y);
  Serial.print(") - (");
  Serial.print(ln.P2.x);
  Serial.print(",");
  Serial.print(ln.P2.y);
  Serial.println(")");

  for(int i = 0; i < lg.trajectory_length; i++)
  {
    Serial.print("(");
    Serial.print(lg.trajectory[i].x);
    Serial.print(",");
    Serial.print(lg.trajectory[i].y);
    Serial.println(")");
  }
}

void testEllipticalTrajectories()
{
  struct arc ac;
  ac.origin = { 0, 0 };
  ac.radius = { 35, 20 };
  ac.start_angle = 0;
  ac.end_angle = 180;

  testEllipticalTrajectory(ac);
}

void testEllipticalTrajectory(struct arc ac)
{
  struct leg lg;
  ellipticalTrajectoryForLeg(ac, &lg, false, false);

  Serial.print("Elliptical trajectory for with origin (");
  Serial.print(ac.origin.x);
  Serial.print(",");
  Serial.print(ac.origin.y);
  Serial.print(") and radius (");
  Serial.print(ac.radius.x);
  Serial.print(",");
  Serial.print(ac.radius.y);
  Serial.print(") start_angle:");
  Serial.print(ac.start_angle);
  Serial.print(") end_angle:");
  Serial.print(ac.end_angle);
  Serial.println(")");

  for(int i = 0; i < lg.trajectory_length; i++)
  {
    Serial.print("(");
    Serial.print(lg.trajectory[i].x);
    Serial.print(",");
    Serial.print(lg.trajectory[i].y);
    Serial.println(")");
  }
}

void testIKS(){
  struct point o1 = { 0, 96 };
  testIK(o1);

  struct point o2 = { 30, 96 };
  testIK(o2);

  struct point o3 = { 15, 81 };
  testIK(o3);

  struct point o4 = { 0, 66 };
  testIK(o4);
    
  struct point o5 = { -15, 81 };
  testIK(o5);

  struct point o6 = { -30, 96 };
  testIK(o6);
}
void testIK(struct point o){
  Serial.println("---------------------------");
  pose p = IK(FEMUR_SIZE,TIBIA_SIZE,o);  
  //p.hip_angle = 180 - p.hip_angle;
  //p.knee_angle = 180 - p.knee_angle; //+90
  
  Serial.print("point:");
  Serial.print(o.x);
  Serial.print(",");
  Serial.print(o.y);
  Serial.print(" pose:");
  Serial.print(p.hip_angle);
  Serial.print(",");
  Serial.println(p.knee_angle);

  pwm.setPWM(HIP_FRONT_RIGHT,0,map(p.hip_angle + HIP_FRONT_RIGHT_DELTA, 0, 180, SERVOMIN, SERVOMAX));
  pwm.setPWM(KNEE_FRONT_RIGHT,0,map(p.knee_angle + KNEE_FRONT_RIGHT_DELTA, 0, 180, SERVOMIN, SERVOMAX));
}
