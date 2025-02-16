using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NicoAgentTomas : Agent
{

    // -------------------------------------
    // 1. Fields & References
    // -------------------------------------
    // a) Articulation references
    // b) Finger constraint settings
    // c) Stored initial states (targets, positions, velocities)
    // d) Lists or arrays to hold incremental changes
    // e) Variables for limiting joint angle updates
    // f) Distance metrics, etc.

    public ArticulationBody nico;

    private List<float> initial_targets = new List<float>();
    private List<float> initial_positions = new List<float>();
    private List<float> initial_velocities = new List<float>();
    private List<float> targets = new List<float>();
    private List<float> initial_changes = new List<float>();
    private List<float> changes = new List<float>();

    public GameObject target;
    public GameObject effector;
    
    private List<float> upperLimits = new List<float>();
    private List<float> lowerLimits = new List<float>();
    private float prevDistance;

    // -------------------------------------
    // 2. Initialize()
    // -------------------------------------
    // - Called by ML-Agents once per agent (at scene load)
    // - Good place to:
    //   1. Fetch all articulation bodies and their indices
    //   2. Retrieve and store joint limits (lowerLimit, upperLimit)
    //   3. Capture initial drive targets, positions, velocities
    //   4. Identify finger bones (tag-based or name-based)
    //   5. Set up dof_ind (if you’re automatically mapping DOFs)
    // - This is a one-time setup that persists across episodes.

    // -------------------------------------
    // 3. OnEpisodeBegin()
    // -------------------------------------
    // - Called before each episode
    // - Resets environment and agent to known start states
    //   1. Optionally randomize the target’s position/rotation
    //   2. Reset the agent’s joint positions, velocities, and drive targets to initial values
    //   3. Reset incremental changes (if using incremental updates)
    //   4. Reinitialize any distance-based variables (like last_dist = initial distance)

    // -------------------------------------
    // 4. CollectObservations(VectorSensor sensor)
    // -------------------------------------
    // - Feeds observation data to the neural network
    //   1. Current joint targets or angles
    //   2. Relative positions (e.g., distance from effector to target)
    //   3. Possibly velocities or orientation info
    //   4. (Optional) If controlling fingers individually, add finger joint angles
    // - If you’re constraining fingers to 4 DOFs, remove or merge certain indices

    // -------------------------------------
    // 5. OnActionReceived(ActionBuffers actions)
    // -------------------------------------
    // - Receives the continuous actions from the policy each step
    //   1. Parse the float actions and apply them to the incremental “changes” array/lists
    //   2. Sum changes into new drive targets (clamp to joint limits)
    //   3. Set the drive targets via nico.SetDriveTargets()
    // - Reward logic:
    //   1. Compare old vs. new distances to target (incremental reward)
    //   2. Add or subtract reward for movement cost, orientation, or other criteria
    //   3. Possibly end episode if the agent is “close enough” to the target
    // - Update tracking variables, e.g., last_dist.

    // -------------------------------------
    // 6. Heuristic(in ActionBuffers actionsOut)
    // -------------------------------------
    // - (Optional) For manual testing / debugging
    // - Maps user inputs to the actions array so you can see if joints move as expected in the Editor

    // -------------------------------------
    // 7. Utility Functions (optional)
    // -------------------------------------
    // - e.g., private void GetLimits(ArticulationBody root, List<float> llimits, List<float> hlimits)
    // - e.g., private void SetJointTargets(List<float> newTargets)
    // - e.g., private void ApplyFingerConstraints(...) for 4-DOF hand
}

