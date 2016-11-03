
void controllerSetup()
{
  LAST_TIMER = millis();
}

void controllerLoop()
{
  //run only every n seconds
  if((millis() - LAST_TIMER) < FRAME_INTERVAL) return;
  LAST_TIMER = millis();

  if(MOVING < 0) return;

  if(KEYFRAME_LENGTH < 1) return; //no keyframes
  if(LOAD_NEXT_KEYFRAME) loadNextKeyFrame();

  for(int i = 0; i < LEGS_COUNT; i++)
  {
    if(TRAJECTORY_INDEX[i] < 0) continue; 

    if(TRAJECTORY_INDEX[i] >= LEGS[i].trajectory_length)
    {
      LOAD_NEXT_KEYFRAME = true;
      continue;
    }

   
    moveTo(i,LEGS[i].trajectory[TRAJECTORY_INDEX[i]]);
    TRAJECTORY_INDEX[i]++;
  } 
}

void controllerCommand(int command)
{
  //
  Serial.print("controllerCommand:");
  Serial.println(command);

  switch(command)
  {
    case COMMAND_FORWARD:
      commandForward();
      break;
    case COMMAND_LEFT:
      commandTurn(MOVING_LEFT);
      break;
    case COMMAND_RIGHT:
      commandTurn(MOVING_RIGHT);
      break;
    default:
      Serial.print("command not implemented:");
      Serial.println(command);
  }
  
}


void commandForward()
{
   if(MOVING == MOVING_FORWARD)
    {
      Serial.println("Stop moving forward");
      MOVING = -1;
      return;
    }

    if(MOVING == MOVING_LEFT || MOVING == MOVING_RIGHT)
    {
      //update the direction
      loadWalk(CURRENT_WALK, CURRENT_STEP_COUNT, MOVING_FORWARD, CURRENT_KEYFRAME);
    }
    MOVING = MOVING_FORWARD;
    Serial.println("Moving forward");
}

void commandTurn(int dir)
{
  MOVING = dir;
  loadWalk(CURRENT_WALK, CURRENT_STEP_COUNT, dir, CURRENT_KEYFRAME);
  Serial.print("Turning ");
  Serial.println( (dir == MOVING_LEFT)? "left":"right");
}

void loadWalk(int walk, int step_count, int direction, int starting_step)
{
  CURRENT_KEYFRAME = starting_step;
  KEYFRAME_LENGTH = 0;
  LOAD_NEXT_KEYFRAME = true;

  for(int i = 0; i < step_count;i++)
  {
    Serial.println("---------------------------------");
    printKeyframe(i);
    
    for(int j = 0; j < LEGS_COUNT;j++)
    {
      KEYFRAMES[KEYFRAME_LENGTH][j] = setupKeyframe(walk,i,j, direction);
    }
    KEYFRAME_LENGTH++;
  }

  Serial.print("loading walk:");
  Serial.println(walk);
  Serial.print("keyframe_length:");
  Serial.println(KEYFRAME_LENGTH);
}


struct keyframe setupKeyframe(int walk, int walk_step, int leg, int direction)
{
   int pose_from = WALKS[walk][walk_step][leg];
   struct keyframe kf;

   kf.ACTION_TYPE = (pose_from == 4)? ACTION_PIN:ACTION_DRAG;
   int pose_to = (pose_from == 4)? 1:pose_from + 1;

   Serial.print("pose_from:");
   Serial.print(pose_from);
   Serial.print(",pose_to:");
   Serial.print(pose_to);

   int tilt_from = TILTS[walk][walk_step][leg];
   int tilt_to = TILTS[walk][pose_to - 1][leg];

   Serial.print(",tilt_from:");
   Serial.print(tilt_from);
   Serial.print(",tilt_to:");
   Serial.print(tilt_to);
   
   int x[] = { 
                (POSE_WIDTHS[direction][leg] / 2) * -1,
                ((POSE_WIDTHS[direction][leg] / 3)/2) * -1,
                ((POSE_WIDTHS[direction][leg] / 3)/2),
                (POSE_WIDTHS[direction][leg] / 2) 
              };

   kf.from = { x[pose_from - 1], POSE_HEIGHT + tilt_from }; 
   kf.to = { x[pose_to - 1], POSE_HEIGHT + tilt_to }; 

   Serial.print((kf.ACTION_TYPE == ACTION_PIN)?"PIN ":"DRAG ");
   Serial.print("from: (");
   Serial.print(kf.from.x);
   Serial.print(",");
   Serial.print(kf.from.y);
   Serial.print("), to: (");
   Serial.print(kf.to.x);
   Serial.print(",");
   Serial.print(kf.to.y);
   Serial.println(")");
   
   return kf;
}

void loadNextKeyFrame()
{
  //load the next keyframe with the trajectory for each leg

  if(CURRENT_KEYFRAME >= KEYFRAME_LENGTH)
  {
    Serial.print("New step:");
    Serial.println(STEP_IDX);
    CURRENT_KEYFRAME = 0; //loop
    STEP_IDX++;
  }

  for(int i=0; i < LEGS_COUNT; i++)
  {
    struct keyframe kf = KEYFRAMES[CURRENT_KEYFRAME][i];

    if(shouldMirrowStep(i))
    {
      kf.from.x *= -1;
      kf.to.x  *= -1; 
    }
    
    if(kf.ACTION_TYPE == ACTION_PIN) pin(i, kf.from, kf.to, false);
    if(kf.ACTION_TYPE == ACTION_DRAG) drag(i, kf.from, kf.to, false);
  }

  CURRENT_KEYFRAME++;
  LOAD_NEXT_KEYFRAME = false;
}



/*
void controllerLoop_()
{
  //run only every n seconds
  if((millis() - LAST_TIMER) < FRAME_INTERVAL) return;
  LAST_TIMER = millis();

  for(int i = 0; i < 4; i++)
  {
    if(TRAJECTORY_INDEX[i] < 0) continue; 

    Serial.print("trajectory_length:");
    Serial.println(LEGS[i].trajectory_length);

    Serial.print("trajectory_index:");
    Serial.println(TRAJECTORY_INDEX[i]);
    
    if(TRAJECTORY_INDEX[i] >= LEGS[i].trajectory_length)
    {
      Serial.print("Done with trajectory for leg:");
      Serial.println(i); 

      if(LOOP <= 0)
      {
        LOOP = 0;
        TRAJECTORY_INDEX[i] = -1;
        continue;
      }
      
      //loop
      LOOP--;
      TRAJECTORY_INDEX[i] = 0;  
      
    }

    Serial.print("moving:");
    Serial.print(LEGS[i].trajectory[TRAJECTORY_INDEX[i]].x);
    Serial.print(",");
    Serial.println(LEGS[i].trajectory[TRAJECTORY_INDEX[i]].y);
    
    moveTo(i,LEGS[i].trajectory[TRAJECTORY_INDEX[i]]);
    TRAJECTORY_INDEX[i]++;
  }
}
*/

void registerTrajectory(int leg)
{
  TRAJECTORY_INDEX[leg] = 0;
}
