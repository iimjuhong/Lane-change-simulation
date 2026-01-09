%% Highway Lane Change Planner and Controller
% This example shows how to simulate an automated lane change maneuver
% system for highway driving scenario.
% 
%% Introduction
% An automated lane change maneuver (LCM) system enables the ego vehicle to
% automatically move from one lane to another lane. The LCM system models
% the longitudinal and lateral control dynamics for automated lane change.
% An LCM system senses the environment for most important objects (MIOs)
% using on-board sensors, identifies an optimal trajectory that avoids
% these objects, and steers the ego vehicle along this trajectory.
%
% This example shows how to design and test the planner and controller
% components of an LCM system. In this example, the lane change planner 
% uses ground truth information from the scenario to detect MIOs. It then  
% generates a feasible trajectory to negotiate a lane change that is 
% executed by the lane change controller. In this example, you:
%
% * *Explore the test bench model* &mdash; The model contains planning,
% controls, vehicle dynamics, scenario, and metrics to assess
% functionality.
% * *Model the lane change planner* &mdash; The reference model finds the
% MIO, samples terminal states of the ego vehicle, and generates an optimal
% trajectory.
% * *Model the lane change controller* &mdash; This model generates control
% commands for the ego vehicle based on the generated trajectory.
% * *Simulate and visualize system behavior* &mdash; The test bench model
% is configured to test the integration of planning and controls to perform
% lane change maneuvers on a curved road with multiple vehicles.
% * *Explore other scenarios* &mdash; These scenarios test the system under
% additional conditions.
% 
% You can apply the modeling patterns used in this example to test your own
% planner and controller components of an LCM system.

%% Explore Test Bench Model
% In this example, you use a simulation test bench model to explore the
% behavior of the planner and controller components for a lane change
% maneuver system.
%
% To explore the test bench model, load the highway lane change planner and
% controller project.
openProject("HLCPlannerAndController");
%%
% Open the simulation test bench model.
%%
open_system('HLCPlannerAndControllerTestBench');

%%
% Opening this model runs the |helperSLHLCPlannerAndControllerSetup| script
% that initializes the road scenario using the <docid:driving_ref#bvm8jbf
% drivingScenario> object in the base workspace. It also configures the
% planner configuration parameters, controller design parameters, vehicle
% model parameters, and the Simulink&reg; bus signals required for defining
% the inputs and outputs for the |HLCPlannerAndControllerTestBench| model.
%% 
% The test bench model contains the following subsystems.
%
% * Scenario and Environment &mdash; Subsystem that specifies the scene,
% vehicles, and map data used for simulation.
% * Planner Configuration Parameters &mdash; Subsystem that specifies the
% configuration parameters required for the planner algorithm.
% * Highway Lane Change Planner &mdash; Subsystem that implements the
% lane change planner algorithm for highway.
% * Lane Change Controller &mdash; Subsystem that specifies the path
% following controller that generates control commands to steer the ego
% vehicle along the generated trajectory.
% * Vehicle Dynamics &mdash; Subsystem that specifies the dynamic model for
% the ego vehicle.
% * Metrics Assessment &mdash; Subsystem that specifies metrics to assess
% system level behavior.
% 
% The Vehicle Dynamics subsystem models the ego vehicle using a
% <docid:driving_ref#mw_9569b0a8-23c5-4aaf-ba41-e3ebe9ef7971 Bicycle Model>
% and updates its state using commands received from the Lane Change
% Controller. For more details on Vehicle Dynamics subsystem, see
% <docid:driving_ug#mw_574c4bc8-4369-4da6-a4d3-aa48e7e31f92 Highway Lane
% Following> example.
%
% The Scenario and Environment subsystem uses the
% <docid:driving_ref#mw_0abe0f52-f25a-4829-babb-d9bafe8fdbf3 Scenario
% Reader> block to provide road network and vehicle ground truth
% positions. This block also outputs map data required for the highway lane
% change planner algorithm. Open the Scenario and Environment subsystem.
%%
open_system('HLCPlannerAndControllerTestBench/Scenario and Environment')

%% 
% The <docid:driving_ref#mw_0abe0f52-f25a-4829-babb-d9bafe8fdbf3 Scenario
% Reader> block is configured to read the <docid:driving_ref#bvm8jbf
% drivingScenario> object from the base workspace. It uses this object to
% read the actor data. It takes in ego vehicle information to perform a
% closed-loop simulation. This block outputs ground truth information of
% the lanes and actors in ego vehicle coordinates. The
% <docid:driving_ref#mw_27640406-e319-4cd8-8d2c-6231f1a6a233 Vehicle To
% World> block is used to convert target vehicle positions from the vehicle
% coordinates to world coordinates. This subsystem reads map data from the
% base workspace and outputs information about the lanes and reference
% path.
%
% The Planner Configuration Parameters subsystem reads base workspace
% variables using constant blocks and constructs a bus structure using the
% Bus Creator block. The bus created by this subsystem is used by the lane
% change planner.
%
% The Highway Lane Change Planner reference model uses ground truth actor
% positions in world coordinates, map data, and planner configuration
% parameters to perform trajectory planning for the automated lane change
% maneuver.
%
%% Model Highway Lane Change Planner
% The Highway Lane Change Planner reference model implements the main
% algorithm for the highway lane change system. The model finds the MIOs
% surrounding the ego vehicle using the Fernet coordinate system.
% Subsequently, the model samples terminal states for different behaviors,
% predicts the motion of target actors, and generates multiple
% trajectories. Finally, the model evaluates the costs of generated
% trajectories and checks for the possibility of collision and kinematic
% feasibility to estimate the optimal trajectory. Open the Highway Lane
% Change Planner reference model.
%%
open_system('HighwayLaneChangePlanner')

%% 
% The Highway Lane Change Planner model contains the following blocks:
%
% * The Frenet State Converter block converts the pose of the ego vehicle
% and other vehicles in the scenario into the Frenet coordinate system from
% world coordinates.
% * The Find MIOs block identifies the MIOs surrounding the ego vehicle.
% * The Terminal State Sampler block samples terminal states for cruise
% control, lead car following, and lane change behaviors. The Motion
% Prediction module predicts the motion of MIOs.
% * The Motion Planner reference model generates an optimal trajectory from
% the sampled trajectories. This model checks the sampled trajectories for
% cost, feasibility, and the possibility of collision to identify the
% optimal trajectory. This block also computes the appropriate point on the
% trajectory for the ego vehicle to follow. For more information on the
% Highway Lane Change Planner, see
% <docid:driving_ug#mw_9b6a86d6-11cc-4390-bc96-a96db64f099e Generate Code
% For Highway Lane Change Planner>.

%% Model Lane Change Controller
% The Lane Change Controller reference model simulates a path following
% control mechanism that keeps the ego vehicle traveling along the
% generated trajectory while tracking a set velocity. To do so, the
% controller adjusts both the longitudinal acceleration and front steering
% angle of the ego vehicle. The controller computes optimal control actions
% while satisfying velocity, acceleration, and steering angle constraints
% using adaptive model predictive control (MPC). Open the Lane Change
% Controller reference model.
%%
open_system('LaneChangeController')

%%
% * The Virtual Lane Center subsystem creates a virtual lane from the
% path point. The virtual lane matches the format required by the Path
% Following Controller block.
% * The Preview Curvature subsystem converts trajectory to curvature
% input required by Path Following Controller block.
% * The Path Following Controller block uses the
% <docid:mpc_ref#mw_78ac0ef3-0622-4d5f-907b-52f6c7bebff6 Path Following
% Control System> block from the Model Predictive Control Toolbox&trade;.
%% 
% The Path Following Controller block keeps the vehicle traveling within
% a marked lane of a highway while maintaining a user-set velocity. This
% controller includes combined longitudinal and lateral control of the ego
% vehicle:
%% 
% * Longitudinal control maintains a user-set velocity of the ego vehicle.
% * Lateral control keeps the ego vehicle traveling along the center line
% of its lane by adjusting the steering of the ego vehicle.
%
%% Explore Metrics Assessment
% The Metrics Assessment subsystem assesses system level behavior of the
% LCM system using the metrics mentioned below. Open the Metrics
% Assessment subsystem.
open_system('HLCPlannerAndControllerTestBench/Metrics Assessment')

%% 
% * The *DetectCollision* block detects the collision of the ego vehicle
% with other vehicles and halts the simulation if a collision is detected.
% * The *DetectLeadVehicle* block computes the headway between the ego and
% lead vehicles, which is used for computing the *TimeGap* value.
% * The *TimeGap* value is calculated using the distance to the lead vehicle
% (headway) and the longitudinal velocity of the ego vehicle, and it is
% evaluated against prescribed limits.
% * The *LongitudinalJerk* value is calculated using the longitudinal 
% velocity and evaluated against prescribed limits.
% * The *LateralJerk* value is calculated using the lateral velocity
% evaluated against prescribed limits.
%
%% Simulate and Visualize System Behavior
% Set up and run the *HLCPlannerAndControllerTestBench* simulation model to
% visualize the behavior of the system during a lane change. The
% Visualization block in the model creates a MATLAB figure that shows the
% chase view and top view of the scenario and plots the ego vehicle,
% sampled trajectories, capsule list, and other vehicles in the scenario.
% Configure the *HLCPlannerAndControllerTestBench* model to use the
% |scenario_LC_15_StopnGo_Curved| scenario.
helperSLHLCPlannerAndControllerSetup('scenarioFcnName','scenario_LC_15_StopnGo_Curved')

%%
% Simulate the model for 5 seconds. The highway lane change planner
% reference model generates a trajectory to navigate the vehicle in the
% scenario. To reduce command-window output, first turn off the MPC update
% messages.
mpcverbosity('off');
sim('HLCPlannerAndControllerTestBench','StopTime','5');

%%
% Close the figure.

hLCPlot = findobj( 'Type', 'Figure', 'Name', 'Lane Change Status Plot');
if ~isempty(hLCPlot)
    close(hLCPlot);
end
%% 
% Run the simulation for 8 seconds. The highway lane change planner
% reference model generates a trajectory to navigate around a slower lead
% vehicle.
sim('HLCPlannerAndControllerTestBench','StopTime','8');

%%
% Close the figure.

hLCPlot = findobj( 'Type', 'Figure', 'Name', 'Lane Change Status Plot');
if ~isempty(hLCPlot)
    close(hLCPlot);
end
%% 
% Run the simulation for 18 seconds. The highway lane change planner
% reference model generates a trajectory to navigate the vehicle to the
% left lane and then to the right lane to avoid collision with the slow
% moving lead vehicle. Observe that the ego vehicle performs a lane change
% twice to avoid collision while maintaining a set velocity.
simout = sim('HLCPlannerAndControllerTestBench','StopTime','18');
%%
% Close the figure.

hLCPlot = findobj( 'Type', 'Figure', 'Name', 'Lane Change Status Plot');
if ~isempty(hLCPlot)
    close(hLCPlot);
end
%%
% During the simulation, the model logs signals to base workspace as
% |logsout|. You can analyze the simulation results and debug any failures
% in the system behavior using the 
% |HelperAnalyzeLCPlannerSimulationResults| object. The 
% |visualizeSimulationData| function of the object creates a MATLAB figure 
% and plots chase view of the scenario.For more details on this
% figure, see the <docid:driving_ug#mw_9b6a86d6-11cc-4390-bc96-a96db64f099e
% Generate Code for Highway Lane Change Planner> example. Run the function
% and explore the plot.
%%
visualizatonObj = HelperAnalyzeLCPlannerSimulationResults(simout.logsout);
visualizatonObj.visualizeSimulationData

%% Explore Other Scenarios
% In the previous section, you explored the system behavior for the
% |scenario_LC_15_StopnGo_Curved| scenario. Below is a list of scenarios
% that are compatible with the
% |HLCPlannerAndControllerTestBench| model.
%
%   scenario_LC_01_SlowMoving
%   scenario_LC_02_SlowMovingWithPassingCar
%   scenario_LC_03_DisabledCar
%   scenario_LC_04_CutInWithBrake
%   scenario_LC_05_SingleLaneChange
%   scenario_LC_06_DoubleLaneChange
%   scenario_LC_07_RightLaneChange
%   scenario_LC_08_SlowmovingCar_Curved
%   scenario_LC_09_CutInWithBrake_Curved
%   scenario_LC_10_SingleLaneChange_Curved
%   scenario_LC_11_MergingCar_HighwayEntry
%   scenario_LC_12_CutInCar_HighwayEntry
%   scenario_LC_13_DisabledCar_Ushape
%   scenario_LC_14_DoubleLaneChange_Ushape
%   scenario_LC_15_StopnGo_Curved [Default]
%
% These scenarios are created using the
% <docid:driving_ref#mw_07e6310f-b9c9-4f4c-b2f9-51e31d407766 Driving
% Scenario Designer> and are exported to a scenario file. Examine the
% comments in each file for more details on the road and vehicles in each
% scenario. You can configure the
% |HLCPlannerAndControllerTestBench| and workspace to simulate
% these scenarios using the |helperSLHLCPlannerAndControllerSetup| function.
% For example, you can configure the simulation for a curved road scenario.
helperSLHLCPlannerAndControllerSetup('scenarioFcnName','scenario_LC_10_SingleLaneChange_Curved');

%% Conclusion
% This example shows how to simulate a highway lane change maneuver using
% ground truth vehicle positions.

%%
% Enable the MPC update messages again.
mpcverbosity('on');

%%
% Copyright 2021-2022 The MathWorks, Inc.