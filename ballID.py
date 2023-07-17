


ball_entity_id = #entity ID of the ball object?

env.start()

#state?
state = env.state()

# ball position
ball_position = state[ball_entity_id].kinematic().pose().translation()

# Print the ball's position
print("Ball Position:", ball_position)

# Close the environment
env.close()