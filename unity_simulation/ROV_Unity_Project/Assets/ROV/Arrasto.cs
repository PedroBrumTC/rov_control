using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Arrasto : MonoBehaviour
{
     public float dragCoefficient = 1.2f;
    public float referenceArea = 0.0442f;
    private float waterDensity = 1000f;

    Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        Vector3 v = rb.velocity;

        Vector3 dragForce =
            -0.5f * waterDensity * dragCoefficient * referenceArea
            * v.magnitude * v;

        rb.AddForce(dragForce);

        Vector3 angularDrag = -0.5f * rb.angularVelocity;
        rb.AddTorque(angularDrag);

    }
}
