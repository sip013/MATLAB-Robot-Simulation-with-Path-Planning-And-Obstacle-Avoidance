## **WLR: Wheeled & Legged Robot Simulation**
### **MATLAB Robot Simulation with Path Planning & Obstacle Avoidance**

### **Project Overview**
This project demonstrates a **differential drive robot** navigating a predefined environment with obstacles using **Hybrid A* path planning, Monte Carlo Localization (MCL), and reactive obstacle avoidance**. The robot autonomously determines an **optimized path**, corrects for localization errors, and adjusts its movement in real time to avoid collisions.

### **Project Members**
- **Shikhar Panchal** (22BCE331)  
- **Mann Patel** (22BCE174)  
- **Jigar Bhoye** (22BCE041)  

### **Features & Methodologies**
✔ **Path Planning with Hybrid A*** – Generates an initial path from the **start position [1,1,π/4]** to the **goal position [8,2,0]**.

✔ **Path Optimization** – The `optimizePath` function refines the generated path for smooth navigation while maintaining a **safe margin from obstacles**.  

✔ **Monte Carlo Localization (MCL)** – Uses **1000 particles** to estimate the robot’s position dynamically, improving navigation accuracy.  

✔ **Reactive Obstacle Avoidance** – The robot detects nearby obstacles and dynamically **adjusts its heading** (`omega`) to prevent collisions.  

✔ **Simulated Environment in MATLAB** – Uses a **binary occupancy map** and visualizes the planned, optimized, and actual paths taken by the robot.

---

## **How It Works**
### **1. Robot Kinematics & Environment Setup**
- The robot is modeled using **differentialDriveKinematics** with a **wheel radius of 0.05m** and a **track width of 0.5m**.
- A **binary occupancy map** (`simpleMap`) represents the environment, including obstacles at specific locations.

### **2. Path Planning & Optimization**
- The **Hybrid A*** planner finds an **initial feasible path** to the goal while avoiding obstacles.
- The `optimizePath` function refines the path by:
  - Ensuring a **minimum turning radius of 2m**.
  - Keeping the number of path states **three times the original path states**.
  - Maintaining an **obstacle safety margin of 0.75m**.

### **3. Localization & Sensor Model**
- A **range sensor** with **10 beams** scans the environment within a **horizontal angle of -π/3 to π/3**.
- The **Monte Carlo Localization (MCL)** algorithm refines the robot’s estimated position using **odometry motion models and likelihood field sensor models**.

### **4. Obstacle Avoidance & Navigation**
- The robot calculates the **heading error** to align with the target.
- If an obstacle is detected within **1m**, the robot **adjusts its angular velocity (`omega`)** to avoid a collision.
- The system **remembers past avoidance maneuvers** and **resumes its original trajectory** after clearing the obstacle.

### **5. Visualization**
The simulation generates **real-time visual feedback**, displaying:
- The **initial planned path** (blue dashed line).
- The **optimized path** (red solid line).
- The **robot's actual trajectory** (magenta line).
- The **target position** (green cross).

---

## **How to Run the Simulation**
1. **Open MATLAB** and navigate to the project folder.
2. Run the script:  
   ```matlab
   WLR_PROJECT.m
   ```
3. The simulation will execute, displaying:
   - The **robot moving toward its target** while avoiding obstacles.
   - The **path planning and localization process**.
   - A final **comparison of planned vs. optimized vs. actual paths**.

---
> Screenshots of the simulation:

![Screenshot 1](/Wheeled%20and%20Legged%20Robots/Images/Fig1.png)

![Screenshot 2](/Wheeled%20and%20Legged%20Robots/Images/Fig2.png)

---

> [!NOTE]
> You can change the **obstacle locations, robot parameters, or target position** to test different scenarios.

---

## **Future Enhancements**
🚀 **Advanced Localization:** Incorporate **LiDAR, ultrasonic, or infrared sensors** for better position accuracy.  
🧠 **AI-Based Obstacle Handling:** Train a **machine learning model** to recognize different obstacle types and apply intelligent avoidance strategies.  

---

## **Demo Video 🎥**
To see the simulation in action, watch the demo video:  
[📺 Demo Video](/Wheeled%20and%20Legged%20Robots/WLR_PROJECT_VID.mp4)


---

## **References**
📌 **MATLAB Robotics Toolbox** – Official MATLAB documentation  
📌 **Hybrid A* Algorithm** – Path planning reference  
📌 **Monte Carlo Localization** – Probabilistic robotics concepts  

--- 
