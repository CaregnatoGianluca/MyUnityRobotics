using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "niryo_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_NiryoOne;
    public GameObject NiryoOne { get => m_NiryoOne; set => m_NiryoOne = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;

    [Header("Randomization Settings")]
    [SerializeField] float m_MinRadius = 0.2f;
    [SerializeField] float m_MaxRadius = 0.366f;
    [SerializeField] float m_SpawnY = 0.645f;
    [SerializeField] float m_RestartDelay = 1.0f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

    bool m_WaitingForInput = false;
    Vector3 m_CurrentPickPosition;

    void Update()
    {
        if (m_WaitingForInput && Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            Plane tablePlane = new Plane(Vector3.up, new Vector3(0, m_SpawnY, 0));

            if (tablePlane.Raycast(ray, out float distance))
            {
                m_CurrentPickPosition = ray.GetPoint(distance);
                m_WaitingForInput = false;
                PublishJoints();
            }
        }
    }

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // Find left and right fingers
        var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";

        m_RightGripper = m_NiryoOne.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_NiryoOne.transform.Find(leftGripper).GetComponent<ArticulationBody>();

        var targetPlacementComponent = m_TargetPlacement.GetComponent<Unity.Robotics.PickAndPlace.TargetPlacement>();
        if (targetPlacementComponent != null)
        {
            targetPlacementComponent.OnTargetPlaced += OnTargetPlaced;
        }

        m_CurrentPickPosition = m_Target.transform.position;
        m_WaitingForInput = true;
    }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = 0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 0.01f;
        rightDrive.target = -0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>NiryoMoveitJoints</returns>
    NiryoMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new NiryoMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (m_CurrentPickPosition + m_PickPoseOffset).To<FLU>(),

            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogWarning("No trajectory returned from MoverService. The pose might possess impossible rotational constraints! Randomizing both Cube and Placement zone for a fresh start.");
            
            // Randomize Drop-Off
            m_TargetPlacement.transform.position = GetRandomPosition();
            
            // Randomize Cube Target
            Vector3 newTargetPos = GetRandomPosition();
            m_Target.transform.position = newTargetPos;
            var targetRb = m_Target.GetComponent<Rigidbody>();
            if (targetRb != null)
            {
                targetRb.velocity = Vector3.zero;
                targetRb.angularVelocity = Vector3.zero;
            }

            m_WaitingForInput = true;
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from niryo_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();

            // Wait briefly for the object physics to settle
            yield return new WaitForSeconds(1.0f);

            var targetPlacementComponent = m_TargetPlacement.GetComponent<Unity.Robotics.PickAndPlace.TargetPlacement>();
            if (targetPlacementComponent != null && targetPlacementComponent.CurrentState != Unity.Robotics.PickAndPlace.TargetPlacement.PlacementState.InsidePlaced)
            {
                Debug.LogWarning("Pick and place sequence finished, but the cube isn't in the zone. Click again to retry!");
                m_WaitingForInput = true;
            }
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }

    void OnTargetPlaced()
    {
        StartCoroutine(RestartRoutine());
    }a

    IEnumerator RestartRoutine()
    {
        yield return new WaitForSeconds(m_RestartDelay);

        // Randomize the drop-off position
        m_TargetPlacement.transform.position = GetRandomPosition();

        // Randomize the cube position
        Vector3 newTargetPos = GetRandomPosition();
        m_Target.transform.position = newTargetPos;
        var targetRb = m_Target.GetComponent<Rigidbody>();
        if (targetRb != null)
        {
            targetRb.velocity = Vector3.zero;
            targetRb.angularVelocity = Vector3.zero;
        }

        m_WaitingForInput = true;
    }

    Vector3 GetRandomPosition()
    {
        float randomAngle = UnityEngine.Random.Range(0f, Mathf.PI * 2f);
        float randomRadius = Mathf.Sqrt(UnityEngine.Random.Range(m_MinRadius * m_MinRadius, m_MaxRadius * m_MaxRadius));
        return new Vector3(randomRadius * Mathf.Cos(randomAngle), m_SpawnY, randomRadius * Mathf.Sin(randomAngle));
    }
}
