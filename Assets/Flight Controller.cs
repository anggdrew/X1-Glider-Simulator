using UnityEngine;

public class FlightController : MonoBehaviour
{
    public Vector2 targetLatLon; // Set in degrees
    public Vector2 currentLatLon; // Simulated GPS position
    public float targetPitch = -6f;
    public float maxPitch = -10f;
    public float maxDeflection = 45f;
    public float servoNeutral = 0f;

    public Rigidbody rb;

    private PID rollPID;
    private PID pitchPID;

    void Start()
    {
        rollPID = new PID(1.5, 0.01, 0.5);
        pitchPID = new PID(2, 0.02, 0.6);
    }

    void FixedUpdate()
    {
        Vector3 currentForward = transform.forward;
        float currentBearing = transform.eulerAngles.y;

        float targetBearing = CalculateBearing(currentLatLon, targetLatLon);
        float rollError = NormalizeAngle(targetBearing - currentBearing);

        float currentPitch = NormalizeAngle(transform.eulerAngles.x);
        float pitchError = Mathf.Clamp(targetPitch - currentPitch, maxPitch - currentPitch, 0);

        float rollOutput = rollPID.Compute(rollError, Time.fixedDeltaTime);
        float pitchOutput = pitchPID.Compute(pitchError, Time.fixedDeltaTime);

        ApplyElevonForces(rollOutput, pitchOutput);
    }

    float CalculateBearing(Vector2 from, Vector2 to)
    {
        float dLon = Mathf.Deg2Rad * (to.y - from.y);
        float lat1 = Mathf.Deg2Rad * from.x;
        float lat2 = Mathf.Deg2Rad * to.x;

        float y = Mathf.Sin(dLon) * Mathf.Cos(lat2);
        float x = Mathf.Cos(lat1) * Mathf.Sin(lat2) -
                  Mathf.Sin(lat1) * Mathf.Cos(lat2) * Mathf.Cos(dLon);
        float bearing = Mathf.Atan2(y, x);
        return (Mathf.Rad2Deg * bearing + 360) % 360;
    }

    float NormalizeAngle(float angle)
    {
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    void ApplyElevonForces(float roll, float pitch)
    {
        float leftElevon = servoNeutral + Mathf.Clamp(roll + pitch, -maxDeflection, maxDeflection);
        float rightElevon = servoNeutral + Mathf.Clamp(-roll + pitch, -maxDeflection, maxDeflection);

        // Simulate elevon effects by applying torque
        float rollTorque = roll * Mathf.Deg2Rad;
        float pitchTorque = pitch * Mathf.Deg2Rad;

        rb.AddRelativeTorque(Vector3.right * pitchTorque, ForceMode.Force);
        rb.AddRelativeTorque(Vector3.back * rollTorque, ForceMode.Force);
    }
}

public class PID
{
    private double kp, ki, kd;
    private double integral, previousError;

    public PID(double kp, double ki, double kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public float Compute(double error, float deltaTime)
    {
        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;
        previousError = error;

        return (float)(kp * error + ki * integral + kd * derivative);
    }
}