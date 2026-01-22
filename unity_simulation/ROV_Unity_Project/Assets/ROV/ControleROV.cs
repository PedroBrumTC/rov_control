using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;

public class ControleROV : MonoBehaviour
{
    ROSConnection ros;

    [Header("ROS Topics")]
    public string thrusterTopic = "/motor_cmd";
    public string imuTopic = "/imu_raw";
    public string pressureTopic = "/pressure_raw";

    [Header("ROV Physics")]
    Rigidbody rovRb;
    Vector3[] thrusterLocalPositions =
    {
        new Vector3( -0.2f, 0f, 0f),  // motor direito
        new Vector3(0.2f, 0f, 0f)   // motor esquerdo
    };
    public float maxThrusterForce = 2f;

    Vector3 lastVelocity = Vector3.zero;

    Float32MultiArrayMsg lastThrusterCmd;


    const float rho = 1000f;
    const float deltaV = 2e-5f; // 20 ml (duas seringas)

    float maxSyringeForce; // ≈ 0.098 N


    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<Float32MultiArrayMsg>(thrusterTopic, ThrusterCallback);

        ros.RegisterPublisher<ImuMsg>(imuTopic);
        ros.RegisterPublisher<FluidPressureMsg>(pressureTopic);

        rovRb = GetComponent<Rigidbody>();

        maxSyringeForce =
            rho * Physics.gravity.magnitude * deltaV;
    }

    void FixedUpdate()
    {
        ApplyThrusters();
        PublishIMU();
        PublishPressure();
        ApplyBuoyancy();

    }
    void ThrusterCallback(Float32MultiArrayMsg msg)
    {
        lastThrusterCmd = msg;
    }

    void ApplyThrusters()
    {
        
        if (lastThrusterCmd == null) return;

        for (int i = 0; i < thrusterLocalPositions.Length; i++)
        {
            float cmd = Mathf.Clamp(
                (float)lastThrusterCmd.data[i],
                -1f, 1f
            );

            Vector3 worldPos =
                transform.TransformPoint(thrusterLocalPositions[i]);

            Vector3 worldDir =
                transform.TransformDirection(Vector3.forward);

            Vector3 force = worldDir * cmd * maxThrusterForce;

            rovRb.AddForceAtPosition(force, worldPos);

            // DEBUG visual
            Debug.DrawRay(worldPos, worldDir * 1f, Color.red);
        }
    }

    void ApplyBuoyancy()
    {
        if (lastThrusterCmd == null) return;
        if (lastThrusterCmd.data.Length < 3) return;

        float s1 = Mathf.Clamp(
            (float)lastThrusterCmd.data[2],
            -1f, 1f
        );
        
        // Cálculo da submersão (Z é altura!)
        float depth = 0.0f - transform.position.y;
        float submergedFactor = Mathf.Clamp01(depth / 0.15f);

        float buoyancyForce = - s1 * maxSyringeForce * submergedFactor;

        Debug.Log($"Comando : {s1} ||||  Empuxo aplicado: {buoyancyForce} N");
        // +Z é cima no seu modelo
        Vector3 upDir = transform.TransformDirection(Vector3.up);

        rovRb.AddForce(upDir * buoyancyForce);

        Debug.DrawRay(
            transform.position,
            upDir * 1 * buoyancyForce,
            Color.blue
        );
    }
    void PublishIMU()
{
    ImuMsg imu = new ImuMsg();

    // === ORIENTAÇÃO ===
    Quaternion q = rovRb.rotation;

    imu.orientation.x = q.z;
    imu.orientation.y = -q.x;
    imu.orientation.z = q.y;
    imu.orientation.w = q.w;

    // === VELOCIDADE ANGULAR ===
    Vector3 w = rovRb.angularVelocity;

    imu.angular_velocity.x = -w.y;
    imu.angular_velocity.y = -w.x;
    imu.angular_velocity.z =  w.z;

    ros.Publish(imuTopic, imu);


}
    void PublishPressure()
    {
        float rho = 1000f;   // água (kg/m³)

        float depth = -rovRb.position.y; // y < 0 → submerso
        // depth = Mathf.Max(0f, depth);

        float pressure = rho * Physics.gravity.magnitude * depth + 101325.0f;

        FluidPressureMsg p = new FluidPressureMsg();
        p.fluid_pressure = pressure;
        p.variance = 0.01;

        ros.Publish(pressureTopic, p);
    }
}