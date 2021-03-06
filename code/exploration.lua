local vector = require "vector"
require "shared_vars"
require "motor_schema"
require "utilities"
require "swarm_intelligence"

---- Global vars ----

-- Velocity
MIN_VELOCITY = -15
MAX_VELOCITY = 15
CRUISE_VELOCITY = 10

ROBOT_DRIVE_SHAFT = robot.wheels.axis_length / 2 -- 7
PI = math.pi

-- Led colors
RESTING_LED_COLOR = "red"
EXPLORING_LED_COLOR = "green"
BIASED_EXPLORING_LED_COLOR = "black"
WAITING_LED_COLOR = "yellow"
RECONNAISSANCE_LED_COLOR = "white"
RETURNING_BASE_LED_COLOR = "blue"

-- Range and bearing
LANDMARK_SENSING_RANGE = MAX_LANDMARK_SENSING_RANGE --math.random(20, MAX_LANDMARK_SENSING_RANGE)
NEIGHBOR_SIGNAL_CHANNEL = 3
NEIGHBOR_SENSING_RANGE = MAX_LANDMARK_SENSING_RANGE * 2 + 5

-- Others 
t = 0 -- Time elapsed in the current state
WHITE_GROUND_THRESHOLD = 0.5 -- Motor ground

RECONNAISSANCE_LENGTH = 20
BIASED_EXPLORATION_LENGTH = 100

---- States ----
current_state = "resting" -- It will be the starting state too
states = {}

-- Waiting in base and randomly start exploration
states.resting = function()

  local start_exploring_cond = non_deterministic_transition(start_exploring_p(t))
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

  --velocity = vector.zero()
  --velocity.length = CRUISE_VELOCITY
  --velocity.angle = ternary(near_obstacle, math.rad(math.random(0, 360)), 0)
  --velocity = restrain_velocity(to_differential_model(velocity)) 
  velocity = ballistic_random_walk()
  
  if (near_landmark) then 
    current_state = "waiting_for_cluster"
    robot.wheels.set_velocity(0, 0)
    robot.leds.set_all_colors(WAITING_LED_COLOR) 
    t = 0
  else 
    if non_deterministic_transition(quit_exploring_p(exploring_time)) then
      current_state = "returning_base"
      robot.leds.set_all_colors(RETURNING_BASE_LED_COLOR)
      t = 0
    else
      robot.wheels.set_velocity(velocity.left, velocity.right)
    end
  end  
end

-- Wait for cluster to be completed
states.waiting_for_cluster = function()

  t = t + 1
  
  robot.range_and_bearing.set_data(NEIGHBOR_SIGNAL_CHANNEL, 1)

  if n_neighbors >= EXPECTED_CLUSTER_SIZE - 1  then
    current_state = "reconnaissance"
    robot.leds.set_all_colors(RECONNAISSANCE_LED_COLOR)
    t = 0
  else
    if non_deterministic_transition(leave_cluster_p(t, n_neighbors + 1)) then
      current_state = "biased_exploration"
      robot.leds.set_all_colors(BIASED_EXPLORING_LED_COLOR)
      t = 0
    end
  end
end

-- The robots "explore" the area near the landmark before heading back to the base
states.reconnaissance = function()

  t = t + 1
  
  robot.range_and_bearing.set_data(LANDMARK_EXPLORED_NOTIIFICATION_CHANNEL, 1)
  
  if t > RECONNAISSANCE_LENGTH then
    current_state = "returning_base"
    robot.leds.set_all_colors(RETURNING_BASE_LED_COLOR)
  end
end

-- Exploration without considering any landmark 
states.biased_exploration = function()

  t = t + 1
  
  robot.range_and_bearing.set_data(NEIGHBOR_SIGNAL_CHANNEL, 0)
  
  velocity = ballistic_random_walk()
  robot.wheels.set_velocity(velocity.left, velocity.right)
  
  if t >= BIASED_EXPLORATION_LENGTH then
    current_state = "exploring"
    robot.leds.set_all_colors(EXPLORING_LED_COLOR)
  end
end

-- Returning to base by executing phototaxis
states.returning_base = function()
  
  robot.range_and_bearing.set_data(NEIGHBOR_SIGNAL_CHANNEL, 0)
  robot.range_and_bearing.set_data(LANDMARK_EXPLORED_NOTIIFICATION_CHANNEL, 0)
  
  local light_force = phototaxis()
  --log("light " .. polar_vector_to_string(light_force))

  local obstacle_force = obstacle_avoidance(PI / 2)
  --log("obs " .. polar_vector_to_string(obstacle_force))
  
  local schemas = {light_force, obstacle_force} 
  local resultant = reduce_vec2_array_polar(schemas);
  resultant.length = resultant.length / #schemas * GAIN_FACTOR
  --log("res " .. polar_vector_to_string(resultant))
  
  velocity = restrain_velocity(to_differential_model(resultant))   
  robot.wheels.set_velocity(velocity.left, velocity.right)
   
  if in_base then
      robot.wheels.set_velocity(0, 0) 
      current_state = "resting"
      robot.leds.set_all_colors(RESTING_LED_COLOR)
      t = 0
  end
end

---- Random walk ----

-- Ballistic random walk: go straight till an obstacle is perceived, then turn randomly
function ballistic_random_walk()

  velocity = vector.zero()
  velocity.length = CRUISE_VELOCITY
  velocity.angle = ternary(near_obstacle > 0, math.rad(math.random(0, 360)), 0)
  return restrain_velocity(to_differential_model(velocity))
end

---- Core ----

function init()

  t = 0
  robot.wheels.set_velocity(0, 0)
  robot.leds.set_all_colors(RESTING_LED_COLOR)
end

function step() 
  
  -- Preprocess input
  near_landmark = count_RAB(LANDMARK_SIGNAL_CHANNEL, LANDMARK_SENSING_RANGE) > 0
  
  n_neighbors = count_RAB(NEIGHBOR_SIGNAL_CHANNEL, NEIGHBOR_SENSING_RANGE)
  
  near_obstacle = is_near_obstacle()

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
