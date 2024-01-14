using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IKUpdateTargetMain : MonoBehaviour
{
    [SerializeField] Transform initialPosition;
    [SerializeField] LayerMask floorLayer = default;
    [SerializeField] IKTargetMain ikTarget;
    public float stepDistance;
    public float forwardOffset;
    private Vector3 oldPosition;
    private Vector3 debugPos;
    // Start is called before the first frame update
    void Start()
    {
        oldPosition = initialPosition.position + Vector3.forward * forwardOffset;
        ikTarget.Instantiation(oldPosition);
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
                oldPosition = hit.point + new Vector3(0,0.025f,0);
                ikTarget.UpdateTarget(oldPosition);
            }
            debugPos = hit.point;
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(debugPos, 0.03f);
    }
}
