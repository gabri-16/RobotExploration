local vector = require "vector"

-- Velocity

MIN_VELOCITY = -10
MAX_VELOCITY = 10
CRUISE_VELOCITY = 10

left_v, right_v = 0 -- Velocity of left and right wheels

WHEELS_DISTANCE = robot.wheels.axis_length / 2 -- 7
PI = math.pi

-- Leds color
RESTING_LED_COLOR = "red"
EXPLORING_LED_COLOR = "green"
BIASED_EXPLORING_LED_COLOR = "orange"
WAITING_LED_COLOR = "yellow"
RETURNING_BASE_LED_COLOR = "blue"
RECONNAISSANCE_LED_COLOR = "lightblue"

-- Range and bearing
MAX_SENSING_RANGE = 50
LANDMARK_SIGNAL_CHANNEL = 1

-- Motor ground
WHITE_GROUND_THRESHOLD = 0.5

-- Probabilities --

-- Calculate the probability the robot has to start exploration.
-- The probability increases proportionally to the time already spent resting.
-- t = 0 => p = 0, t -> +inf => p -> 1
START_EXPLORATION_DELAY_FACTOR = 1000
function start_exploring_p(t)
  return math.atan(t / START_EXPLORATION_DELAY_FACTOR) -- 1 - (1 / (t + 1))
end

--
--
function join_cluster_p(t)
  pass()
end

--
--
function leave_cluster_p(t, n)
  return math.atan(t / 50) * ((3 - n) / 10)
end

-- Calculate the probability to quit exploration and return to the base
--
function quit_exploring_p(t)
  return 0.001
end

---- States ----
current_state = "resting" -- It will be the strating state too

local states = {}

-- Waiting in base
rest_time = 0
states.resting = function()

  local start_exploring_cond = math.random() < start_exploring_p(rest_time)
  -- log("t " .. rest_time .. " p " .. start_exploring_p(rest_time)) 
  
  if start_exploring_cond then
    current_state = "exploring"
    robot.leds.set_all_colors(EXPLORING_LED_COLOR)
    exploring_time = 0
  end
  rest_time = ternary(start_exploring_cond, 0, rest_time + 1)
end

-- Explore the arena and look for any not already explored landmark
exploring_time = 0
states.exploring = function()
  
  exploring_time = exploring_time + 1

  velocity = vector.zero()
  velocity.length = CRUISE_VELOCITY
  velocity.angle = ternary(near_obstacle, math.rad(math.random(0, 360)), 0)
  velocity = restrain_velocity(to_differential_model(velocity)) 
  --log("L " .. velocity.left)
  --log("R " .. velocity.right)
 
  if (near_landmark) then 
    current_state = "waiting_for_cluster"
    robot.wheels.set_velocity(0, 0)
    robot.leds.set_all_colors(WAITING_LED_COLOR) 
    wait_time = 0
  else 
    if (math.random() < quit_exploring_p(exploring_time)) then
      current_state = "returning_base"
      robot.leds.set_all_colors(RETURNING_BASE_LED_COLOR)
      biased_time = 0
    else
      robot.wheels.set_velocity(velocity.left, velocity.right)
    end
  end  
end

-- Wait for cluster to be completed
wait_time = 0
MAX_WAIT_TIME = 10
states.waiting_for_cluster = function()

  wait_time = wait_time + 1
  if (wait_time >= MAX_WAIT_TIME) then
    current_state = "reconnaissance"
    robot.leds.set_all_colors(RECONNAISSANCE_LED_COLOR)
    reconnaissance_time = 0
  end
end

reconnaissance_time = 0
RECONNAISSANCE_LEGNTH = 20
states.reconnaissance = function()

  reconnaissance_time = reconnaissance_time + 1
  
  if reconnaissance_time > RECONNAISSANCE_LEGNTH then
    current_state = "returning_base"
    robot.leds.set_all_colors(RETURNING_BASE_LED_COLOR)
  end
end

-- Exploration without considering any landmark 
biased_time = 0
MAX_BIASED_TIME = 100
states.biased_exploration = function()

  velocity = vector.zero()
  velocity.length = CRUISE_VELOCITY
  velocity.angle = ternary(near_obstacle, math.rad(math.random(0, 360)), 0)
  velocity = restrain_velocity(to_differential_model(velocity)) 
  robot.wheels.set_velocity(velocity.left, velocity.right)
  
  biased_time = biased_time + 1
  if (biased_time >= MAX_BIASED_TIME) then
    current_state = "exploring"
    robot.leds.set_all_colors(EXPLORING_LED_COLOR)
  end
end

-- Returning to base by executing phototaxis
GAIN_FACTOR = 10
states.returning_base = function()
  
  local light_force = phototaxis()
  --log(polar_vector_to_string(light_force))
  
  local obstacle_force = obstacle_avoidance()
  --log(polar_vector_to_string(obstacle_force))

  local schemas = {light_force, obstacle_force}
  local resultant = reduce_vec2_array_polar(schemas);
  resultant.length = resultant.length / #schemas * GAIN_FACTOR
  --log(polar_vector_to_string(resultant))
  
  velocity = restrain_velocity(to_differential_model(resultant)) 
  --log("L " .. velocity.left)
  --log("R " .. velocity.right)
  
  robot.wheels.set_velocity(velocity.left, velocity.right)
  
  if in_base then
      robot.wheels.set_velocity(0, 0) 
      current_state = "resting"
      robot.leds.set_all_colors(RESTING_LED_COLOR)
      rest_time = 0
  end
      
end

---- Potential fields ----

-- Potential field: phototaxis
-- It create an attracting field so the robot can return to the base
-- The attraction increases proportionally to the distance for the light source
function phototaxis()

  max_light = 0
  max_light_idx = -1
  for i=1, #robot.light do
    local v = robot.light[i].value
    if v > max_light then
      max_light = v
      max_light_idx = i
    end
  end

  return {
    length = 1 - robot.light[max_light_idx].value, 
    angle = robot.light[max_light_idx].angle
  }
 
end

-- Potential field: obstacle avoidance
-- It creates a tangential field so the robot can circumnavigate it
function obstacle_avoidance()
  
  local forces = {}
  for i=1, #robot.proximity do
    local prox = robot.proximity[i]
    forces[i] = {
      length = prox.value,
      angle = prox.angle - PI / 2 -- -PI/2 makes the perpendicular direction 
    }
  end

  local summation = reduce_vec2_array_polar(forces)
  return {
    length = summation.length / #forces, 
    angle = summation.angle
  }
end

---- Utilities ----

function ternary(cond, T, F)
  if cond then return T else return F end
end

-- Convert a polar vector representing a perceived force the differential model
-- (length, angle) -> (velocity left, velocity right) 
function to_differential_model(force)
 return {
   left = force.length - WHEELS_DISTANCE * force.angle,
   right = force.length + WHEELS_DISTANCE * force.angle
  }
end

-- Bring velocity back to range [MIN_VELOCITY, MAX_VELOCITY] 
function restrain_velocity(velocity)

  local restrained_v = velocity
  restrained_v.left = math.max(restrained_v.left, MIN_VELOCITY)   
  restrained_v.left = math.min(restrained_v.left, MAX_VELOCITY)
  restrained_v.right = math.max(restrained_v.right, MIN_VELOCITY)
  restrained_v.right = math.min(restrained_v.right, MAX_VELOCITY)
  return restrained_v
end

-- Get robot orientation angle in radiants
function retrieve_robot_orientation()
    local orientationString = tostring(robot.positioning.orientation)
    -- Drop decimal part so it can be converted to a Lua number using "tonumber" function
    local i, _ = string.find(orientationString, ",", 1) 
    local int_orientation_value = tonumber(string.sub(orientationString, 1, i-1))
    return math.rad(int_orientation_value)
end

-- Count the number of stopped robots sensed close to the items sensed on a given channel
function count_RAB(channel)
  number_robot_sensed = 0
  for i = 1, #robot.range_and_bearing do
    -- for each robot seen, check they it is close enough
    if robot.range_and_bearing[i].range < MAX_SENSING_RANGE and robot.range_and_bearing[i].data[channel] == 1 then
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

-- Reduce an array of polar vectors by summing them
function reduce_vec2_array_polar(array)
  
  local res = vector.zero()
  for i=1, #array do
    res = vector.vec2_polar_sum(res, array[i])
  end
  return res
end

-- Nice representation of a polar vector
function polar_vector_to_string(vector)
  return "length: " .. vector.length .. "; angle: " .. vector.angle
end

---- Core ----

function init()
  robot.wheels.set_velocity(0, 0)
  robot.leds.set_all_colors(RESTING_LED_COLOR)
end


t_t = 0
n = 2
function step()
  
  v = math.atan(t_t / 50) * ((3 - n) / 10)
  --log("lc n ".. n .. "p " .. v)
  t_t = t_t + 1
  
  -- Check if near landmark
  near_landmark = count_RAB(LANDMARK_SIGNAL_CHANNEL) > 0

  -- Check if it is in the base
  in_base = is_in_base()
  
  -- Check if near to any obstacle
  near_obstacle = false
  for i=1, #robot.proximity do
    if robot.proximity[i].value > 0.9 then 
      near_obstacle = true 
    end
  end
  
  -- Retrieve robot orientation
  robot_orientation = retrieve_robot_orientation()
  
  -- Select behavior depending on my state
  states[current_state]()
end

function reset()
  init()
end

function destroy()
   -- put your code here
end
