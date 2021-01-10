require "shared_vars"
require "utilities"

-- Leds colors
NOT_EXPLORED_LED_COLOR = "red"
EXPLORED_LED_COLOR = "green"

---- States ----
current_state = "not_explored" -- It will be the strating state too

local states = {}

-- Not explored
states.not_explored = function()

  robot.range_and_bearing.set_data(LANDMARK_SIGNAL_CHANNEL, 1)

  explored = count_RAB(LANDMARK_EXPLORED_NOTIIFICATION_CHANNEL, LANDMARK_SENSING_RANGE) > 0
  if (explored) then
    current_state = "explored"
    robot.leds.set_all_colors(EXPLORED_LED_COLOR)
  end
end

-- Explored
states.explored = function()
  robot.range_and_bearing.set_data(LANDMARK_SIGNAL_CHANNEL, 0)  
end

---- Core ----

function init()
  robot.leds.set_all_colors(NOT_EXPLORED_LED_COLOR)
end

function step()
  states[current_state]() -- Select behavior depending on the landmark state
end

function reset()
  init()
end

function destroy()
   pass()
end
