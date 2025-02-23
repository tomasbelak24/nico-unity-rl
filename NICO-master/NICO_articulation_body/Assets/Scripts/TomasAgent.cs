using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class TomasAgent : Agent
{
    public ArticulationBody NICO;
    private ArticulationBody shoulder;
    private ArticulationBody collarbone;
    private ArticulationBody upperarm;
    private ArticulationBody lowerarm;
    private ArticulationBody forearm;

    [Tooltip("The target object")]
    public GameObject target;
    Vector3 defaultTargetPosition;
    
    [Tooltip("End effector")]
    public GameObject indexFingertip;

    private List<float> lowerLimits;
    private List<float> upperLimits;
    private List<ArticulationBody> joints;

    private void get_limits()
    {
    joints = new List<ArticulationBody>();
    lowerLimits = new List<float>();
    upperLimits = new List<float>();

    joints.Add(shoulder);
    joints.Add(collarbone);
    joints.Add(upperarm);
    joints.Add(lowerarm);
    joints.Add(forearm);

    foreach (var joint in joints)
    {
        if (joint != null)
        {
            var drive = joint.xDrive;
            lowerLimits.Add(drive.lowerLimit * Mathf.Deg2Rad);
            upperLimits.Add(drive.upperLimit * Mathf.Deg2Rad);
        }
    }
}


    public override void Initialize()
    {
        NICO = GetComponent<ArticulationBody>();
        shoulder = NICO.transform.Find("r_shoulder").GetComponent<ArticulationBody>();
        collarbone = NICO.transform.Find("r_collarbone").GetComponent<ArticulationBody>();
        upperarm = NICO.transform.Find("r_upper_arm").GetComponent<ArticulationBody>();
        lowerarm = NICO.transform.Find("r_lower_arm").GetComponent<ArticulationBody>();
        forearm = NICO.transform.Find("r_forearm").GetComponent<ArticulationBody>();
        get_limits();

    
    }
    
    
    
    public override void OnEpisodeBegin()
    {
        target.transform.position = defaultTargetPosition + new Vector3(
            Random.Range(-0.1f, 1.25f), 
            Random.Range(0.0f, 0.05f), 
            Random.Range(-0.2f, 0.2f)
        );
        
        target.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);

        //prevDistance = (indexFingertip.transform.position - target.transform.position).magnitude;
    }


    public override void CollectObservations(VectorSensor sensor)
    {
        
    }


    public override void OnActionReceived(ActionBuffers actions)
    {
        float max_range = Mathf.Deg2Rad * 0.1f;
        float change_magnitude = Mathf.Deg2Rad * 0.05f;

        

    }


    public override void Heuristic(in ActionBuffers actionsOut)
    {
    
    }
        
}