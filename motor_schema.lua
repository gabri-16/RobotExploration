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

---- Utility functions ----

-- Get robot orientation angle in radiants
function retrieve_robot_orientation()
    local orientationString = tostring(robot.positioning.orientation)
    -- Drop decimal part so it can be converted to a Lua number using "tonumber" function
    local i, _ = string.find(orientationString, ",", 1) 
    local int_orientation_value = tonumber(string.sub(orientationString, 1, i-1))
    return math.rad(int_orientation_value)
end

-- Convert a polar vector representing a perceived force the differential model
-- (length, angle) -> (velocity left, velocity right) 
function to_differential_model(force)
 return {
   left = force.length - WHEELS_DISTANCE * force.angle,
   right = force.length + WHEELS_DISTANCE * force.angle
  }
end