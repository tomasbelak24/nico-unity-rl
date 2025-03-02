using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Threading;

public class TomasAgent : Agent
{
    
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

    private int dofs; // pocet dof
    private int abs; // pocet articulation bodies

    private Dictionary<string, float> desiredAngles = new Dictionary<string, float>
    {
        {"r_shoulder",   0f},
        {"r_collarbone", 0f},
        {"r_upper_arm",   -65f},
        {"r_lower_arm",   20f},
        {"r_forearm",    100f},
        {"r_palm",       0f},
    };



    [Tooltip("The target object")]
    public GameObject target;
    Vector3 defaultTargetPosition;

    [Tooltip("End effector")]
    public GameObject effector;

    private float last_dist;

    
    private void GetLimits(ArticulationBody root, List<float> llimits, List<float> hlimits)
    {
        GameObject curr_obj = root.gameObject;
        int num_ch = curr_obj.transform.childCount;
        for (int i = 0; i < num_ch; ++i)
        {
            // get articulation body component from child with index i, get its drive limits, write them to lists, call recursively on children
            GameObject child = curr_obj.transform.GetChild(i).gameObject;
            ArticulationBody child_ab = child.GetComponent<ArticulationBody>();
            if (child_ab != null)
            {
                int j = child_ab.index;
                llimits[j - 1] = child_ab.xDrive.lowerLimit;
                hlimits[j - 1] = child_ab.xDrive.upperLimit;
                GetLimits(child_ab, llimits, hlimits);
            }
        }
    }


    private void setNaturalPosition(ArticulationBody root, Dictionary<string, float> desiredAngles)
{
    if (root == null)
    {
        Debug.LogError("Root ArticulationBody is null.");
        return;
    }

    foreach (var kvp in desiredAngles)
    {
        string targetName = kvp.Key;
        float targetAngle = kvp.Value;

        ArticulationBody targetBody = FindArticulationBodyByName(root, targetName);
        if (targetBody != null)
        {   
            targetBody.SetDriveTarget(ArticulationDriveAxis.X, targetAngle);
            //var drive = targetBody.xDrive;
            //drive.target = targetAngle;
            //targetBody.xDrive = drive;
        }
        else
        {
            Debug.LogWarning($"ArticulationBody '{targetName}' not found in hierarchy.");
        }
    }
    Debug.Log("Set natural position");
    //Thread.Sleep(15000);
}

    private ArticulationBody FindArticulationBodyByName(ArticulationBody root, string name)
    {
        if (root.name == name)
            return root;

        foreach (Transform child in root.transform)
        {
            ArticulationBody childBody = child.GetComponent<ArticulationBody>();
            if (childBody != null)
            {
                ArticulationBody found = FindArticulationBodyByName(childBody, name);
                if (found != null)
                    return found;
            }
        }
        return null;
    }

    /*void Start()
    {
        //Debug.Log("Started");
        ArticulationBody root = GetComponent<ArticulationBody>();
        setNaturalPosition(root, desiredAngles);
    }*/

    public override void Initialize()
    {
        base.Initialize();

        // remember initial joint positions and velocities so we can reset them at next episode start

        nico = GetComponent<ArticulationBody>();
        //setNaturalPosition(nico, desiredAngles);
        nico.GetDofStartIndices(dof_ind);
        abs = nico.GetDriveTargets(initial_targets);
        dofs = abs;
        nico.GetJointPositions(initial_positions);
        nico.GetJointVelocities(initial_velocities);

        low_limits = new List<float>(new float[abs]);
        high_limits = new List<float>(new float[abs]);

        GetLimits(nico, low_limits, high_limits);
        //Debug.Log("Low limits: " + string.Join(", ", low_limits));
        //Debug.Log("High limits: " + string.Join(", ", high_limits));

        defaultTargetPosition = target.transform.position; // default position of target (red cube)

        for (int i = 0; i < dofs; ++i)
        {
            initial_changes.Add(0f);
        }

        changes = new List<float>(initial_changes);
        targets = new List<float>(initial_targets);


        last_dist = (target.transform.position - effector.transform.position).magnitude;
        //Debug.Log("Initialized");
    
    }
    
    
    
    public override void OnEpisodeBegin()
    {   
        //Debug.Log("Episode begin");
        target.transform.position = defaultTargetPosition + new Vector3(Random.Range(-0.1f, 0.95f), Random.Range(0.0f, 0.45f), Random.Range(-0.6f, 0.1f));
        
        target.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);

        nico.SetDriveTargets(initial_targets);
        nico.SetJointPositions(initial_positions);
        nico.SetJointVelocities(initial_velocities);

        changes = new List<float>(initial_changes);
        targets = new List<float>(initial_targets);
    }


    public override void CollectObservations(VectorSensor sensor)
    {
        // observe current targets
        List<float> observation = new List<float>();
        nico.GetDriveTargets(observation);
        sensor.AddObservation(observation);
        sensor.AddObservation(target.transform.position - effector.transform.position);
    }


    public override void OnActionReceived(ActionBuffers actions)
    {
        float max_range = Mathf.Deg2Rad * 0.1f;
        float change_magnitude = Mathf.Deg2Rad * 0.05f;

        for (int i = 0; i < dofs; ++i)
        {
            changes[i] = Mathf.Clamp(changes[i] + actions.ContinuousActions[i] * change_magnitude, -max_range, max_range);
        }

        //Debug.Log($"abs: {abs}");
        //Debug.Log($"changes: {string.Join(", ", changes)}");
        //Debug.Log($"dofs: {dofs}");
        //Debug.Log($"targets: {string.Join(", ", targets)}");
        for (int i = 0; i < abs; ++i)
        {       
                //Debug.Log($"i: {i}");
                targets[i] = Mathf.Clamp(targets[i] + changes[i], Mathf.Deg2Rad * low_limits[i], Mathf.Deg2Rad * high_limits[i]);
        }

        // set joint targets to modified values

        nico.SetDriveTargets(targets);

        // calculate reward

        float new_dist = (target.transform.position - effector.transform.position).magnitude;

        // penalize for movements so the arm does not shake

        float movement_reward = 0f;
        for (int i = 0; i < dofs; ++i)
        {
            movement_reward += -0.5f * Mathf.Abs(changes[i]);
        }

        // give reward for moving closer to target

        float got_closer_reward = (last_dist - new_dist) * 0.1f;

        // penalize from moving away from target

        if (got_closer_reward <= 0)
        {
            got_closer_reward = -3f;
        }
        last_dist = new_dist;

        // penalize for distance to target

        float proximity_reward = -1f * new_dist;

        // add all reward components together and provide reward to agent

        AddReward(got_closer_reward + proximity_reward + movement_reward);
        //Debug.Log($"Reward: {got_closer_reward + proximity_reward + movement_reward}");
    }


    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
    }
        
}