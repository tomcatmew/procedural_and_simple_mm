using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IKTarget : MonoBehaviour
{
    [SerializeField] Transform initialPosition;
    private Vector3 oldPosition;
    private Vector3 newPosition;
    [Tooltip("How height to lift the leg.")]
    public float stepHeight;
    public float speed;
    private float lerp = 1;
    [SerializeField] private bool isLeftLeg;
    // Start is called before the first frame update
    void Start()
    {
        transform.position = initialPosition.position;
        oldPosition = initialPosition.position;
        newPosition = initialPosition.position;
    }

    // Update is called once per frame
    void Update()
    {
        //if (lerp < 1)
        //{
        //    Vector3 tempPosition = Vector3.Lerp(oldPosition, newPosition, lerp);
        //    tempPosition.y += Mathf.Sin(lerp * Mathf.PI) * stepHeight;

        //    transform.position = tempPosition;
        //    lerp += Time.deltaTime * speed;
        //}
        //else
        //{
        //    oldPosition = newPosition;
        //    LegManager.Instance.FrontLeftMove = false;
        //}

        if (isLeftLeg)
        {
            if ((lerp < 1) && (LegManager.Instance.FrontLeftMove))
            {
                Vector3 tempPosition = Vector3.Lerp(oldPosition, newPosition, lerp);
                tempPosition.y += Mathf.Sin(lerp * Mathf.PI) * stepHeight;

                transform.position = tempPosition;
                lerp += Time.deltaTime * speed;
            }
            else if ((lerp < 1) && (!LegManager.Instance.FrontLeftMove))
            {
                LegManager.Instance.FrontLeftMove = false;
            }
            else
            {
                oldPosition = newPosition;
                LegManager.Instance.FrontLeftMove = false;
            }
        }
        else
        {
            if ((lerp < 1) && (!LegManager.Instance.FrontLeftMove))
            {
                Vector3 tempPosition = Vector3.Lerp(oldPosition, newPosition, lerp);
                tempPosition.y += Mathf.Sin(lerp * Mathf.PI) * stepHeight;

                transform.position = tempPosition;
                lerp += Time.deltaTime * speed;
            }
            else if ((lerp < 1) && (LegManager.Instance.FrontLeftMove))
            {
                LegManager.Instance.FrontLeftMove = true;
            }
            else
            {
                oldPosition = newPosition;
                LegManager.Instance.FrontLeftMove = true;
            }
        }
    }

    public void UpdateTarget(Vector3 NewTarget)
    {
        lerp = 0;
        newPosition = NewTarget;
    }

    private void OnDrawGizmos()
    {

        Gizmos.color = Color.red;
        Gizmos.DrawSphere(transform.position, 0.05f);
    }
}
