using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class ShoulderAgent : Agent
{
    public ArticulationBody NICO;
    private ArticulationBody shoulder;
    //public int maxStepsPerEpisode;
    
    [Tooltip("The target object")]
    public GameObject target;
    Vector3 defaultTargetPosition;
    
    [Tooltip("End effector")]
    public GameObject indexFingertip;

    private float lowerLimit;
    private float upperLimit;

    private float prevDistance;

    private float initial_target_angle;
    private float target_angle;
    private float change;

    //private int stepCounter = 0;


    public override void Initialize()
    {
        NICO = GetComponent<ArticulationBody>();
        shoulder = NICO.transform.Find("r_shoulder").GetComponent<ArticulationBody>();

        if (shoulder == null)
        {
            Debug.LogError("Shoulder ArticulationBody is not assigned!");
            return;
        }
        
        if (shoulder.jointType != ArticulationJointType.RevoluteJoint)
        {   
            Debug.Log($"- Shoulder join type: {shoulder.jointType}");
            Debug.LogError("Shoulder joint type should be Revolute!");
            
            return;
        }

        lowerLimit = -90f;//shoulder.xDrive.lowerLimit;
        upperLimit = 90f;//shoulder.xDrive.upperLimit;
        defaultTargetPosition = target.transform.position;

        initial_target_angle = shoulder.xDrive.target * Mathf.Deg2Rad;
        target_angle = initial_target_angle;
        change = 0f;

        // Get the initial distance between the fingertip and the target
        prevDistance = (indexFingertip.transform.position - target.transform.position).magnitude;
        // Debug logs
        //var drive = shoulder.xDrive;
        //Debug.Log($"Initialized with:");
        //Debug.Log($"- Joint Position: {shoulder.jointPosition} degrees");
        //Debug.Log($"- Joint Limits: {lowerLimit} to {upperLimit} degrees");
        //Debug.Log($"- Drive Settings:");
        //Debug.Log($"  - Stiffness: {drive.stiffness}");
        //Debug.Log($"  - Damping: {drive.damping}");
        //Debug.Log($"  - Force Limit: {drive.forceLimit}");
        //Debug.Log($"  - Target: {drive.target} degrees");
    }

    public override void OnEpisodeBegin()
    {
        //target.transform.position = new Vector3(Random.Range(-4f, 4f), Random.Range(0f, 4f), Random.Range(-4f, 4f));
        //target.transform.position = defaultTargetPosition;
        //stepCounter = 0;
        //target.transform.position = new Vector3(Random.Range(0.2f, 1.25f), Random.Range(1.35f, 1.8f), Random.Range(-0.2f, 0.5f));
        target.transform.position = defaultTargetPosition + new Vector3(
        Random.Range(-0.1f, 1.25f), 
        Random.Range(0.0f, 0.05f), 
        Random.Range(-0.2f, 0.2f)
    );
        target.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);

        // Reset the shoulder joint to its initial position
        var drive = shoulder.xDrive;
        drive.target = initial_target_angle;
        shoulder.xDrive = drive;

        change = 0f;
        target_angle = initial_target_angle;


        // Reset the distance between the fingertip and the target
        prevDistance = (indexFingertip.transform.position - target.transform.position).magnitude;
    }


    public override void CollectObservations(VectorSensor sensor)
    {
        // Add the current shoulder joint target
        sensor.AddObservation(shoulder.xDrive.target);
        // Add the vector from the index fingertip to the target
        sensor.AddObservation(target.transform.position - indexFingertip.transform.position);
    }


    public override void OnActionReceived(ActionBuffers actions)
    {
        float max_range = Mathf.Deg2Rad * 0.1f;
        float change_magnitude = Mathf.Deg2Rad * 0.05f;

        // Get the action values
        float action = actions.ContinuousActions[0];
        //Debug.Log($"Action Received: {action}");

        change = Mathf.Clamp(change + action * change_magnitude, -max_range, max_range);
        //Debug.Log($"Change: {change}");
        //Debug.Log($"Lower Limit: {lowerLimit} degrees");
        //Debug.Log($"Upper Limit: {upperLimit} degrees");
        target_angle = Mathf.Clamp(target_angle + change, 
                              Mathf.Deg2Rad * lowerLimit, 
                              Mathf.Deg2Rad * upperLimit);
        var drive = shoulder.xDrive;
        drive.target = target_angle * Mathf.Rad2Deg;
        shoulder.xDrive = drive;

        //Debug.Log($"Target Angle: {drive.target} degrees");
        //Debug.Log($"Current Joint Position: {shoulder.jointPosition[0]} degrees");

        // Calculate the new distance between the fingertip and the target
        float distance = (indexFingertip.transform.position - target.transform.position).magnitude;
        
        if (distance < prevDistance)
        {
            AddReward(1f);
        }
        else
        {
            AddReward(-1.2f);
        }
        
        //stepCounter++;
        //if (stepCounter >= maxStepsPerEpisode)
        //{
        //    EndEpisode();
        //}

    }


    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        float horizontalInput = Input.GetAxis("Horizontal");
        //Debug.Log($"Horizontal Input: {horizontalInput}");
        continuousActions[0] = horizontalInput;
    }
        
}