using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IKUpdateTargetBack : MonoBehaviour
{
    [SerializeField] Transform initialPosition;
    [SerializeField] LayerMask floorLayer = default;
    [SerializeField] IKTargetBack ikTarget;
    public float stepDistance;
    public float forwardOffset;
    private Vector3 oldPosition;
    private Vector3 debugPos;
    // Start is called before the first frame update
    void Start()
    {
        transform.position = initialPosition.position;
        oldPosition = initialPosition.position;
        transform.localPosition = transform.localPosition + new Vector3(0.0f, 2.5f, 0.0f);
    }

    // Update is called once per frame
    void Update()
    {
        //transform.position = RobotCenter.position + OffSet;
        Ray FloorRay = new Ray(transform.position, Vector3.down);
        if (Physics.Raycast(FloorRay, out RaycastHit hit, 10, floorLayer.value))
        {
            if (Vector3.Distance(oldPosition, new Vector3(hit.point.x, oldPosition.y, hit.point.z)) > stepDistance)
            {
                Vector3 movingDir = Vector3.Normalize(hit.point - oldPosition);
                Vector3 newRayOrigin = hit.point + movingDir * forwardOffset + new Vector3(0.0f, 2.5f, 0.0f);
                Ray PredictFloorRay = new Ray(newRayOrigin, Vector3.down);
                if (Physics.Raycast(PredictFloorRay, out RaycastHit hitPredict, 10, floorLayer.value))
                {
                    oldPosition = hitPredict.point;
                }
                ikTarget.UpdateTarget(oldPosition);
            }
            debugPos = hit.point;
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(debugPos, 0.1f);
    }
}
