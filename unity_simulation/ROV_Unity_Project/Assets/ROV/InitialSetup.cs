using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InitialSetup : MonoBehaviour
{
       Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        rb.centerOfMass = new Vector3(0f, -0.0f, 0f);
    }
}
