-- Leds color
NOT_EXPLORED_LED_COLOR = "red"
EXPLORED_LED_COLOR = "green"

-- Range and bearing
MAX_SENSING_RANGE = 20
LANDMARK_SIGNAL_CHANNEL = 1
LANDMARK_EXPLORED_NOTIIFICATION_CHANNEL = 2

---- States ----
current_state = "not_explored" -- It will be the strating state too

local states = {}

-- Not explored
states.not_explored = function()

  robot.range_and_bearing.set_data(LANDMARK_SIGNAL_CHANNEL, 1)
  if (explored) then
    current_state = "explored"
    robot.leds.set_all_colors(EXPLORED_LED_COLOR)
  end
end

-- Explored
states.explored = function()
  pass()  
end

---- Utilities ----

function ternary(cond, T, F)
  if cond then return T else return F end
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

---- Core ----

function init()
  robot.leds.set_all_colors(NOT_EXPLORED_LED_COLOR)
end

function step()
 
  explored = count_RAB(LANDMARK_EXPLORED_NOTIIFICATION_CHANNEL)

  -- Select behavior depending on my state
  states[current_state]()
end

function reset()
  init()
end

function destroy()
   -- put your code here
end
