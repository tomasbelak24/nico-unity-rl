using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;

public class GazeAgent : Agent
{
    // Articulation body and joint variables
    ArticulationBody nico;
    
    private List<float> initial_targets = new List<float>();
    private List<float> initial_positions = new List<float>();
    private List<float> initial_velocities = new List<float>();
    private List<float> targets = new List<float>();
    private List<float> initial_changes = new List<float>();
    private List<float> changes = new List<float>();

    private List<int> dof_ind = new List<int>();
    private List<float> low_limits;
    private List<float> high_limits;

    private int dofs;
    private int abs;

    public bool constrain_fingers = true;

    [Tooltip("The target object")]
    public GameObject target;

    [Tooltip("head position where the raycast will originate from")]
    public Transform head;

    private float last_dist;

    public override void Initialize()
    {
        base.Initialize();

        nico = GetComponent<ArticulationBody>();
        nico.GetDofStartIndices(dof_ind);
        abs = nico.GetDriveTargets(initial_targets);
        nico.GetJointPositions(initial_positions);
        nico.GetJointVelocities(initial_velocities);

        low_limits = new List<float>(new float[abs]);
        high_limits = new List<float>(new float[abs]);

        targets = new List<float>(initial_targets);
        changes = new List<float>(initial_targets);

        last_dist = float.MaxValue; // Initialize with a large value
    }

    public override void OnEpisodeBegin()
    {
        // Randomize target cube position
        target.transform.position = new Vector3(Random.Range(0.2f, 1.25f), Random.Range(1.35f, 1.8f), Random.Range(-0.2f, 0.5f));
        target.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);

        // Reset joint positions, velocities, targets
        nico.SetDriveTargets(initial_targets);
        nico.SetJointPositions(initial_positions);
        nico.SetJointVelocities(initial_velocities);

        changes = new List<float>(initial_targets);
        targets = new List<float>(initial_targets);

        last_dist = float.MaxValue;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Observe the robot's joint targets
        List<float> observation = new List<float>();
        nico.GetDriveTargets(observation);

        sensor.AddObservation(observation);

        // Observe direction to target
        sensor.AddObservation((target.transform.position - head.position).normalized);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float max_range = Mathf.Deg2Rad * 0.1f;
        float change_magnitude = Mathf.Deg2Rad * 0.05f;

        // Apply changes to articulation joints
        for (int i = 0; i < dofs; ++i)
        {
            changes[i] = Mathf.Clamp(changes[i] + actions.ContinuousActions[i] * change_magnitude, -max_range, max_range);
        }

        for (int i = 0; i < abs; ++i)
        {
            targets[i] = Mathf.Clamp(targets[i] + changes[i], Mathf.Deg2Rad * low_limits[i], Mathf.Deg2Rad * high_limits[i]);
        }

        // Apply new targets
        nico.SetDriveTargets(targets);

        RaycastHit hit;
        bool hitSomething = Physics.Raycast(head.position, head.forward, out hit, 10f);

        Debug.DrawRay(head.position, head.forward * 10f, Color.red); // Debugging Ray

        if (hitSomething)
        {
            // Distance from hit point to target
            float distanceToTarget = Vector3.Distance(hit.point, target.transform.position);

            // Reward for getting closer to the target
            if (distanceToTarget < last_dist)
            {
                AddReward(0.05f); // Small reward
            }
            else
            {
                AddReward(-0.05f); // Small penalty
            }
            last_dist = distanceToTarget;

            // Check if looking exactly at the red cube
            if (hit.transform.gameObject == target.gameObject)
            {
                SetReward(1.0f);  // Big reward
                EndEpisode();
            }
            else
            {
                // Small penalty if looking at a different object
                AddReward(-0.05f);
            }
        }
        else
        {
            // Penalize if not hitting anything (e.g., looking at the sky)
            AddReward(-0.1f);
        }

    }

    public override void Heuristic(in ActionBuffers actionsOut)
{
    var continuousActionsOut = actionsOut.ContinuousActions;

    // Control the neck rotation (left-right) using A/D keys
    continuousActionsOut[0] = Input.GetKey(KeyCode.A) ? -1f : (Input.GetKey(KeyCode.D) ? 1f : 0f);

    // Control the head rotation (up-down) using W/S keys
    continuousActionsOut[1] = Input.GetKey(KeyCode.W) ? 1f : (Input.GetKey(KeyCode.S) ? -1f : 0f);
}

}
