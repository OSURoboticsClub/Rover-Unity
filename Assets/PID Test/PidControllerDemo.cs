using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PidControllerDemo : MonoBehaviour
{
    public Transform target; // The target position
    public float kp = 1.0f;  // Proportional gain
    public float ki = 0.0f;  // Integral gain
    public float kd = 0.0f;  // Derivative gain

    private Rigidbody2D rb;
    float previousError;
    [SerializeField] float error;
    [SerializeField] float integral;
    [SerializeField] float integralTerm;
    [SerializeField] float proportional;
    [SerializeField] float derivative;
    [SerializeField] float derivativeTerm;
    [SerializeField] float output;

    void Start()
    {
        rb = GetComponent<Rigidbody2D>();
    }

    void FixedUpdate()
    {
        // Calculate error
        error = target.position.x - transform.position.x;

        // Proportional term
        proportional = kp * error;

        // Integral term
        integral += error * Time.fixedDeltaTime;
        integralTerm = ki * integral;

        // Derivative term
        derivative = (error - previousError) / Time.fixedDeltaTime;
        derivativeTerm = kd * derivative;

        // PID output
        output = proportional + integralTerm + derivativeTerm;

        // Apply force to the Rigidbody
        rb.AddForce(Vector2.right * output, ForceMode2D.Force);

        // Update previous error
        previousError = error;
    }
}
