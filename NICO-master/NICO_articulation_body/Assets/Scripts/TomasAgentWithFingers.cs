using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;
using Grpc.Core;
using System.Runtime.CompilerServices;

public class TomasAgentWithFingers : Agent
{
    // initialize used variables

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

    [Tooltip("End effector")]
    public GameObject effector;

    [Tooltip("Eye position")]
    public GameObject eye_position;

    private int thumb_root;
    private List<int> thumb_parts = new List<int>();

    private int index_root;
    private List<int> index_parts = new List<int>();

    private int fingers_root;
    private List<int> finger_parts = new List<int>();

    private float last_dist;
    
    private bool pointing = true;

    public void togglePointing(){
        // toggle between pointing and not pointing
        float thumb_rot = 0f;
        float index_rot = 0f;
        float fingers_rot = 0f;


        if (constrain_fingers)
        {
            // first three actions control finger roots, others are mapped to the rest of body
            if(pointing){
                thumb_rot = Mathf.Deg2Rad * -50f;
                index_rot = Mathf.Deg2Rad * 0;
                fingers_rot = Mathf.Deg2Rad * 60;
            } else {
                thumb_rot = Mathf.Deg2Rad * 70 - 0.1f;
                index_rot = Mathf.Deg2Rad * 0;
                fingers_rot = Mathf.Deg2Rad * 0;
            }
            

            int j = 3;
            for (int i = 0; i < abs; ++i)
            {
                switch (i)
                {
                    case int k when k == thumb_root:
                        initial_targets[i] = thumb_rot;
                        break;
                    case int k when k == index_root:
                        initial_targets[i] = index_rot;
                        break;
                    case int k when k == fingers_root:
                        initial_targets[i] = fingers_rot;
                        break;
                    case int k when thumb_parts.Contains(k):
                        initial_targets[i] = thumb_rot / 4;
                        break;
                    case int k when index_parts.Contains(k):
                        initial_targets[i] = index_rot;
                        break;
                    case int k when finger_parts.Contains(k):
                        initial_targets[i] = fingers_rot;
                        break;
                    default:
                        initial_targets[i] = initial_targets[i];
                        j++;
                        break;
                }
            }
            //Debug.Log("Targets: ");
            //foreach (var i in targets){
                //Debug.Log(i);
            //}
        }
        else
        {
            Debug.LogError("Fingers are not constrained, cannot toggle pointing");
        }
    }

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

                //Debug.Log("Child: " + child.name + " index: " + j);
                // getting indices of individual finger parts for constrained (4 DoF) version of control
                switch (child.tag)
                {
                    case "ThumbBase":
                        thumb_root = dof_ind[j];
                        break;
                    case "ThumbContinuation":
                        thumb_parts.Add(dof_ind[j]);
                        break;
                    case "IndexBase":
                        index_root = dof_ind[j];
                        break;
                    case "IndexContinuation":
                        index_parts.Add(dof_ind[j]);
                        break;
                    case "FingerBase":
                        fingers_root = dof_ind[j];
                        break;
                    case "FingerContinuation":
                        finger_parts.Add(dof_ind[j]);
                        break;
                }
                // getting limits
                llimits[j - 1] = child_ab.xDrive.lowerLimit;
                hlimits[j - 1] = child_ab.xDrive.upperLimit;
                GetLimits(child_ab, llimits, hlimits);
            }
        }
    }

    public override void Initialize()
    {
        base.Initialize();

        // remember initial joint positions and velocities so we can reset them at next episode start

        nico = GetComponent<ArticulationBody>();
        nico.GetDofStartIndices(dof_ind);
        abs = nico.GetDriveTargets(initial_targets);
        nico.GetJointPositions(initial_positions);
        nico.GetJointVelocities(initial_velocities);

        //Debug.Log("Initial targets: ");
        //for(int i = 0; i < abs; i++){
            //Debug.Log("Initial target " + i + ": " + initial_targets[i] * Mathf.Rad2Deg);
        //}
        

        low_limits = new List<float>(new float[abs]);
        high_limits = new List<float>(new float[abs]);

        GetLimits(nico, low_limits, high_limits);

        /*Debug.Log("dof_ind: ");
        foreach (var i in dof_ind)
        {
            Debug.Log(i);
        }*/

        // if fingers are constrained, number of DoFs is different than number of art. bodies

        if (constrain_fingers)
        {
            dofs = abs - thumb_parts.Count - index_parts.Count - finger_parts.Count;
        }
        else
        {
            dofs = abs;
        }

        for (int i = 0; i < dofs; ++i)
        {
            initial_changes.Add(0f);
        }

        changes = new List<float>(initial_changes);
        targets = new List<float>(initial_targets);

        //Debug.Log("Roots: " + thumb_root + " " + index_root + " " + fingers_root);
        //Debug.Log("Parts: " + thumb_parts.Count + " " + index_parts.Count + " " + finger_parts.Count);

        //togglePointing();
        
        //nico.SetDriveTargets(targets);

        // get distance from target to end effector

        last_dist = (target.transform.position - effector.transform.position).magnitude;
    }

    public override void OnEpisodeBegin()
    {
        // move target cube to a random position

        target.transform.position = new Vector3(Random.Range(0.2f, 1.25f), Random.Range(1.35f, 1.8f), Random.Range(-0.2f, 0.5f));
        target.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);

        // reset joint positions, velocities, targets

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

        // remove redundant info from observation if fingers are constrained

        var to_remove = thumb_parts.Concat(index_parts).Concat(finger_parts).ToList();

        if (constrain_fingers)
        {
            List<float> new_obs = new List<float>();
            for (int i = 0; i < abs; i++)
            {
                if (!to_remove.Contains(i))
                {
                    new_obs.Add(observation[i]);
                }
            }
            sensor.AddObservation(new_obs);
        }
        else
        {
            sensor.AddObservation(observation);
        }

        // get vector from end effector to target

        sensor.AddObservation(target.transform.position - effector.transform.position);
        Vector3 relativeTargetPosition = target.transform.position - eye_position.transform.position;
        sensor.AddObservation(relativeTargetPosition.normalized); // 3d vector from head to target, normalized because magnitude is not important
    }


    //TODO: upravit reward funkciu tak, aby sa pouzival akoby zrotovany vektor z hlavy smerom k targetu tym padom
    // nebude zaporny kosinus uhlu medzi vektormi ked pojdu opacnym smerom, ale ked kocka bude vpravo a nico bude pozerat dolava
    public override void OnActionReceived(ActionBuffers actions)
    {
        float max_range = Mathf.Deg2Rad * 0.1f;
        float change_magnitude = Mathf.Deg2Rad * 0.05f;

        // modify incremental changes according to policy outputs

        for (int i = 0; i < dofs; ++i)
        {
            changes[i] = Mathf.Clamp(changes[i] + actions.ContinuousActions[i] * change_magnitude, -max_range, max_range);
        }


        // add changes to targets, then clamp to joint limits
        if (constrain_fingers)
        {
            // first three actions control finger roots, others are mapped to the rest of body
            float thumb_rot = Mathf.Clamp(targets[thumb_root] + changes[0], Mathf.Deg2Rad * low_limits[thumb_root], Mathf.Deg2Rad * high_limits[thumb_root]);
            float index_rot = Mathf.Clamp(targets[index_root] + changes[1], Mathf.Deg2Rad * low_limits[index_root], Mathf.Deg2Rad * high_limits[index_root]);
            float fingers_rot = Mathf.Clamp(targets[fingers_root] + changes[2], Mathf.Deg2Rad * low_limits[fingers_root], Mathf.Deg2Rad * high_limits[fingers_root]);

            int j = 3;
            for (int i = 0; i < abs; ++i)
            {
                switch (i)
                {
                    case int k when k == thumb_root:
                        targets[i] = thumb_rot;
                        break;
                    case int k when k == index_root:
                        targets[i] = index_rot;
                        break;
                    case int k when k == fingers_root:
                        targets[i] = fingers_rot;
                        break;
                    case int k when thumb_parts.Contains(k):
                        targets[i] = thumb_rot / 4;
                        break;
                    case int k when index_parts.Contains(k):
                        targets[i] = index_rot;
                        break;
                    case int k when finger_parts.Contains(k):
                        targets[i] = fingers_rot;
                        break;
                    case int k when k != 1 && k != 3: // ked to nie je hlava ani krk
                        targets[i] = 0;
                        j++;
                        break;
                    
                    default:
                        targets[i] = Mathf.Clamp(targets[i] + changes[j], Mathf.Deg2Rad * low_limits[i], Mathf.Deg2Rad * high_limits[i]);
                        j++;
                        break;
                }
            }
        }
        else
        {
            for (int i = 0; i < abs; ++i)
            {
                targets[i] = Mathf.Clamp(targets[i] + changes[i], Mathf.Deg2Rad * low_limits[i], Mathf.Deg2Rad * high_limits[i]);
            }
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
        AddReward(movement_reward);

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

        //AddReward(got_closer_reward + proximity_reward + movement_reward);

        float pointing_reward = 0f;

        // Reward index finger for being extended
        float index_diff = Mathf.Abs(targets[index_root] - Mathf.Deg2Rad * low_limits[index_root]);
        pointing_reward += -index_diff * 5f;

        // Reward other fingers for being tucked
        float fingers_diff = Mathf.Abs(targets[fingers_root] - Mathf.Deg2Rad * high_limits[fingers_root]);
        pointing_reward += -fingers_diff * 5f;

        // Reward other fingers for being tucked
        float thumb_diff = Mathf.Abs(targets[thumb_root] - Mathf.Deg2Rad * low_limits[thumb_root]);
        pointing_reward += -thumb_diff * 5f;

        if (index_diff < 0.05f && fingers_diff < 0.05f && thumb_diff < 0.05f)
        {
            pointing_reward += 1f;
        }

        // Total reward
        //AddReward(pointing_reward);
        //Debug.Log("Pointing reward: " + pointing_reward);

        // rewarding nico for looking at the target
        Vector3 headForward = eye_position.transform.rotation * Vector3.right;; // head forward vector
        Debug.Log("Head forward vector: " + headForward);
        Vector3 relativeTargetPosition = (target.transform.position - eye_position.transform.position).normalized;
        Debug.Log("Relative target position: " + relativeTargetPosition);

        float dotProduct = Vector3.Dot(headForward, relativeTargetPosition);
        
        float alignmentReward = (dotProduct + 1) / 2; // Normalize (-1 to 1) → (0 to 1)

        AddReward(alignmentReward);
        Debug.Log("Alignment reward: " + alignmentReward);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[4] = Input.GetAxis("Horizontal"); // krk
        continuousActionsOut[6] = Input.GetAxis("Vertical"); // hlava
        //continuousActionsOut[7] = Input.GetAxis("Jump"); // 
    }

}

