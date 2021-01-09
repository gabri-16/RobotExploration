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
BIASED_EXPLORING_LED_COLOR = "blue"
WAITING_LED_COLOR = "yellow"

-- Range and bearing
MAX_SENSING_RANGE = 50
LANDMARK_SIGNAL_CHANNEL = 1

-- Probabilities
START_EXPLORING_P = 0.1


---- States ----
current_state = "resting" -- It will be the strating state too

local states = {}

-- Waiting in base
states.resting = function()
  
  if math.random() < START_EXPLORING_P then
    current_state = "exploring"
    robot.leds.set_all_colors(EXPLORING_LED_COLOR)
  end
end

-- Explore the arena and look for any not already explored landmark
states.exploring = function()
  
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
    robot.wheels.set_velocity(velocity.left, velocity.right)
  end  
end

-- Wait for cluster to be completed
wait_time = 0
MAX_WAIT_TIME = 10
states.waiting_for_cluster = function()

  wait_time = wait_time + 1
  if (wait_time >= MAX_WAIT_TIME) then
    current_state = "biased_exploration"
    robot.leds.set_all_colors(BIASED_EXPLORING_LED_COLOR)
    biased_time = 0
  end
end

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

-- Nice representation of a polar vector
function polar_vector_to_string(vector)
  return "length: " .. vector.length .. "; angle: " .. vector.angle
end

---- Core ----

function init()
  robot.wheels.set_velocity(0, 0)
  robot.leds.set_all_colors(RESTING_LED_COLOR)
end

function step()
  
  -- Check if near landmark
  near_landmark = count_RAB(LANDMARK_SIGNAL_CHANNEL) > 0

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
