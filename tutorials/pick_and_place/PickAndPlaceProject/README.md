# Autonomous AI Vision Pick-and-Place

This project transforms a static robotic arm into an intelligent, continuous, and autonomous pick-and-place simulation. By integrating the Gemini Robotics Vision API natively into Unity, the robot visually parses its own environment to locate and manipulate randomized objects.

---

## Background & Foundation

This project started from the official **Unity Robotics Hub Pick-and-Place Tutorial**. 
The foundational setup uses:
*   **Unity**: The 3D physics environment rendering the simulation and the Niryo One robot arm model.
*   **ROS (Robot Operating System)**: The backend acting as the robotics framework.
*   **MoveIt**: The robotic motion planner. Unity pushes target poses over the `ROS TCP Connector`, MoveIt mathematically calculates the 6-DOF joint trajectories (avoiding table collisions), and returns those waypoints to Unity for execution.

In the original tutorial, the cube and the placement zone were placed at static, hardcoded coordinates. Once the robot picked up the cube and placed it once, the simulation permanently ended.

---

## Key Upgrades & Features

I expanded the simulation in three distinct evolutionary phases:

### 1. Continuous Gameplay Loop & Randomization
I refactored the hardcoded simulation into a dynamic environment. 
*   **Continuous Looping:** Once a cube is successfully placed into the target drop-off zone, the engine immediately initiates a fresh environment wipe.
*   **Spatial Randomization:** Both the Pick Target (the cube) and the Place Target (the drop zone) are randomly regenerated within strict mathematical radii limits around the robot base.
*   **Kinematic Fail-safes:** MoveIt sometimes aborts a trajectory if the generated start/end coordinates exceed the robot's physical reach or joint contortion limits. I built a system that actively intercepts these ROS abortion errors, automatically re-randomizes the environment, and restarts the flow gracefully without breaking Unity.

### 2. Interactive Raycast Targeting (Mouse Click)
To simulate the flow of a future Vision AI, I decoupled the robot from the cube's true 3D game coordinates. Instead of the robot innately "knowing" where the cube spawned, it pauses and waits for a 2D pixel input.
*   The human acts as the "vision system", clicking anywhere on the Game Viewer screen.
*   Unity casts a ray from the camera lens through the clicked screen pixel, landing on the simulated 3D tabletop.
*   The system extracts this exact 3D point and passes it to ROS to execute the Pick operation exactly where the user clicked.

### 3. Gemini Vision (Autonomous AI Integration)
The ultimate mode replaces the human mouse click with **Google's Gemini API** (`gemini-robotics-er-1.6-preview`).
*   **Native Unity Integration:** The entire REST API architecture operates implicitly through C#, meaning no external Python terminals need to be managed. 
*   **Live Rendering:** When the simulation requires spatial input, Unity intrinsically mounts a virtual screen capture of the main camera, processes the texture frame, and encodes the pixels into a Base64 string payload.
*   **Spatial Parsing:** The payload is shipped to Gemini alongside an internal prompt asking the model to locate `"cube"`.
*   **Autonomous Driving:** The Google AI parses the frame organically and returns the estimated `[y, x]` coordinates of the object. Unity extracts this through JSON regex validation, automatically aligns those coordinates to your screen aspect ratio, and natively computes the 3D raycast to physically command the robot.

---

## Data Logs & Observability

Because AI operates as an invisible layer, the project includes deep traceability loops. You can check the `Save Gemini Logs` setting on the `Trajectory Planner` script in Unity. 

When active, the simulation generates subfolders (`/GeminiOutputs/Action_[TIMESTAMP]`) outside your repo tracking for every action it evaluates, storing:
1.  `raw_frame.png`: The exact uncolored shot exported to Google.
2.  `gemini_response.json`: The exact raw text output received.
3.  `labeled_frame.png`: A verified debugging screenshot. Unity procedurally parses the array coordinates from the Google API and physically paints a red crosshair over those returned pixels, allowing you to instantly assess if the AI pinpointed the cube accurately or hallucinated the spatial coordinates.

