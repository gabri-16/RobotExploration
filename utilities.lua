---- Utilities ----

---- Sensors preprocessing ----

-- Count the number of stopped robots sensed close to the items sensed on a given channel
function count_RAB(channel, sensing_range)
  number_robot_sensed = 0
  for i = 1, #robot.range_and_bearing do
    -- for each robot seen, check they it is close enough
    if robot.range_and_bearing[i].range < sensing_range and robot.range_and_bearing[i].data[channel] == 1 then
      number_robot_sensed = number_robot_sensed + 1
    end
  end
  return number_robot_sensed
end


-- Check if a robot is in the base
function is_in_base()
  max = 0
  for i = 1, #robot.motor_ground do
    v = robot.motor_ground[i].value
    if (v > max) then
      max = v
    end  
  end
  return max <= WHITE_GROUND_THRESHOLD
end

function is_near_obstacle()

  local obstacle = false
  for i=1, #robot.proximity do
    if robot.proximity[i].value > 0.9 then 
      obstacle = true 
    end
  end
  return obstacle
end

---- Actuators postprocessing ----

-- Bring velocity back to range [MIN_VELOCITY, MAX_VELOCITY] 
function restrain_velocity(velocity)

  local restrained_v = velocity
  restrained_v.left = math.max(restrained_v.left, MIN_VELOCITY)   
  restrained_v.left = math.min(restrained_v.left, MAX_VELOCITY)
  restrained_v.right = math.max(restrained_v.right, MIN_VELOCITY)
  restrained_v.right = math.min(restrained_v.right, MAX_VELOCITY)
  return restrained_v
end

---- Miscellaneous ----

-- Ternary operator like
function ternary(cond, T, F)
  if cond then return T else return F end
end