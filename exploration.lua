local vector = require "vector"
require "shared_vars"
require "motor_schema"
require "utilities"

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
RECONNAISSANCE_LED_COLOR = "brown"

-- Motor ground
WHITE_GROUND_THRESHOLD = 0.5

-- Probabilities --
t = 0 -- Time elapsed in the current state

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
states = {}

-- Waiting in base and start randomly start exploration
states.resting = function()

  local start_exploring_cond = math.random() < start_exploring_p(t)
  -- log("t " .. t .. " p " .. start_exploring_p(t)) 
  
  if start_exploring_cond then
    current_state = "exploring"
    robot.leds.set_all_colors(EXPLORING_LED_COLOR)
    t = 0
  end
  t = ternary(start_exploring_cond, 0, t + 1)
end

-- Explore the arena and look for any not already explored landmark
states.exploring = function()
  
  t = t + 1

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
    t = 0
  else 
    if (math.random() < quit_exploring_p(exploring_time)) then
      current_state = "returning_base"
      robot.leds.set_all_colors(RETURNING_BASE_LED_COLOR)
      t = 0
    else
      robot.wheels.set_velocity(velocity.left, velocity.right)
    end
  end  
end

-- Wait for cluster to be completed
MAX_WAIT_TIME = 10
states.waiting_for_cluster = function()

  t = t + 1
  if (t >= MAX_WAIT_TIME) then
    current_state = "reconnaissance"
    robot.leds.set_all_colors(RECONNAISSANCE_LED_COLOR)
    t = 0
  end
end

-- The robots "explore" the area near the landmark before heading back to the base
RECONNAISSANCE_LEGNTH = 20
states.reconnaissance = function()

  t = t + 1
  robot.range_and_bearing.set_data(2, 1)
  
  if t > RECONNAISSANCE_LEGNTH then
    current_state = "returning_base"
    robot.leds.set_all_colors(RETURNING_BASE_LED_COLOR)
  end
end

-- Exploration without considering any landmark 
MAX_BIASED_TIME = 100
states.biased_exploration = function()

  velocity = vector.zero()
  velocity.length = CRUISE_VELOCITY
  velocity.angle = ternary(near_obstacle, math.rad(math.random(0, 360)), 0)
  velocity = restrain_velocity(to_differential_model(velocity)) 
  robot.wheels.set_velocity(velocity.left, velocity.right)
  
  t = t + 1
  if (t >= MAX_BIASED_TIME) then
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
      t = 0
  end
      
end

---- Core ----

function init()
  robot.wheels.set_velocity(0, 0)
  robot.leds.set_all_colors(RESTING_LED_COLOR)
end

function step() 
  
  -- Preprocess input
  near_landmark = count_RAB(LANDMARK_SIGNAL_CHANNEL) > 0
  
  near_obstacle = false
  for i=1, #robot.proximity do
    if robot.proximity[i].value > 0.9 then 
      near_obstacle = true 
    end
  end
  
  in_base = is_in_base()
  
  robot_orientation = retrieve_robot_orientation()
  
  -- Select behavior depending on the current state
  states[current_state]()
end

function reset()
  init()
end

function destroy()
   pass()
end
