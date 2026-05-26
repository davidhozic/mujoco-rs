import pyo3_application

# Create the application
sim = pyo3_application.Simulation()

# Loop until the viewer is closed
sim.render_loop()
