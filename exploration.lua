local vector = require "vector"

-- Velocity

MIN_VELOCITY = -10
MAX_VELOCITY = 10
CRUISE_VELOCITY = 5

left_v, right_v = 0 -- Velocity of left and right wheels

WHEELS_DISTANCE = robot.wheels.axis_length / 2 -- 7
PI = math.pi

-- Leds color
RESTING_LED_COLOR = "red"
EXPLORING_LED_COLOR = "green"

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
  log("L " .. velocity.left)
  log("R " .. velocity.right)
  
  robot.wheels.set_velocity(velocity.left, velocity.right)
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

-- Get robot oreintation angle in radiants
function retrieve_robot_orientation()
    local orientationString = tostring(robot.positioning.orientation)
    -- Drop decimal part so it can be converted to a Lua number using "tonumber" function
    local i, _ = string.find(orientationString, ",", 1) 
    local int_orientation_value = tonumber(string.sub(orientationString, 1, i-1))
    return math.rad(int_orientation_value)
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
