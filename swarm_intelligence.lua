---- Swarm intelligence ----

-- Parameters --
EXPECTED_CLUSTER_SIZE = 3

F_SHIFT = 500
PATIENCE = 200
NEIGHBOR_INFLUENCE_LIMITER = 5
PATIENCE_ENHANCER = 10

START_EXPLORATION_DELAY_FACTOR = 1000

-- Non deterministic transactions --

-- Calculate the probability the robot has to start exploration.
-- The probability increases proportionally to the time already spent resting.
-- t = 0 => p = 0, t -> +inf => p -> 1
function start_exploring_p(t)
  return math.tanh((t - F_SHIFT) / PATIENCE) + 1
end

-- Probability for a robot to jon a cluster when near a landmark
--
function join_cluster_p(t)
  return (math.tanh((t - F_SHIFT) / PATIENCE) + 1) 
    * math.tanh(n / NEIGHBOR_INFLUENCE_LIMITER) 
    / (NEIGHBOR_INFLUENCE_LIMITER * PATIENCE_ENHANCER)
end

-- Probability for a robot to leave a joined cluster (that is still incomplete)
--
function leave_cluster_p(t, n)
  return (math.tanh((t - F_SHIFT) / PATIENCE) + 1) 
    / math.tanh(n / NEIGHBOR_INFLUENCE_LIMITER) 
    / (NEIGHBOR_INFLUENCE_LIMITER * PATIENCE_ENHANCER)
end

-- probability to quit exploration and return to the base
--
function quit_exploring_p(t)
  return 0.001
end

-- Utilities --

function non_deterministic_transition(p)
  return math.random() < p
end 
