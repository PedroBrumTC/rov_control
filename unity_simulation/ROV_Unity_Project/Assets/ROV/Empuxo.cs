using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Empuxo : MonoBehaviour
{
     private float waterDensity = 1000f;

    public float volumeFactor = 1.01f; // m³ (ajuste!)
    public float waterLevel = 0f;
    public float height = 0.3f; // altura do ROV

    Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        
        float volume = volumeFactor * rb.mass / waterDensity;
        float depth = waterLevel - transform.position.y;

        float submergedFactor = Mathf.Clamp01(depth / height);

        float buoyancy =
            waterDensity * volume * Physics.gravity.magnitude
            * submergedFactor;

        rb.AddForce(Vector3.up * buoyancy);
    }

}