-- Leds color
NOT_EXPLORED_LED_COLOR = "red"
EXPLORED_LED_COLOR = "green"

-- Range and bearing
MAX_SENSING_RANGE = 20
LANDMARK_SIGNAL_CHANNEL = 1

---- States ----
current_state = "not_explored" -- It will be the strating state too

local states = {}

-- Not explored
states.not_explored = function()

  robot.range_and_bearing.set_data(LANDMARK_SIGNAL_CHANNEL, 1)
end

-- Explored
states.explored = function()
  pass()  
end

---- Utilities ----

function ternary(cond, T, F)
  if cond then return T else return F end
end

---- Core ----

function init()
  robot.leds.set_all_colors(NOT_EXPLORED_LED_COLOR)
end

function step()
    
  -- Select behavior depending on my state
  states[current_state]()
end

function reset()
  init()
end

function destroy()
   -- put your code here
end
