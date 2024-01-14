using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotControl : MonoBehaviour
{
    Transform[] checkPoints;
    public float speed = 0.1f;
    public float turnSpeed = 0.1f;
    // Update is called once per frame
    void Update()
    {
        Vector3 rotationToAdd = new Vector3(0, -1, 0) * turnSpeed;
        transform.Rotate(rotationToAdd);

        //float x = Input.GetAxis("Horizontal");
        //float z = Input.GetAxis("Vertical");
        //Vector3 movement = new Vector3(x, 0, z);
        //Vector3 movement = new Vector3(0.0f, 0.0f, 1.0f);
        //movement = Vector3.ClampMagnitude(movement, 1);
        //transform.Translate(movement * speed * Time.deltaTime);
        //if (Input.GetButton("TurnLeft"))
        //{
        //    Vector3 rotationToAdd = new Vector3(0, -1, 0) * turnSpeed;
        //    transform.Rotate(rotationToAdd);
        //}
        //if (Input.GetButton("TurnRight"))
        //{
        //    Vector3 rotationToAdd = new Vector3(0, 1, 0) * turnSpeed;
        //    transform.Rotate(rotationToAdd);
        //}
    }
}
