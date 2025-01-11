using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InverseKinematics : MonoBehaviour
{
    [SerializeField] float L1 = 6;
    [SerializeField] float L2 = 6;
    [SerializeField] Transform sphere;
    [SerializeField] float theta1 = 0;
    [SerializeField] float theta2 = 0;
    [SerializeField] Transform forearm;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    public void SetPositions()
    {
        float x = sphere.position.x;
        float y = sphere.position.y;
        float cos = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        float sin = -Mathf.Sqrt(1 - cos * cos);
        theta2 = Mathf.Atan2(sin, cos) * Mathf.Rad2Deg;
        float k1 = L1 + L2 * cos;
        float k2 = L2 * sin;
        theta1 = (Mathf.Atan2(y, x) - Mathf.Atan2(k2, k1)) * Mathf.Rad2Deg;

        gameObject.transform.localEulerAngles = new Vector3(0, 0, -180f + theta1);
        forearm.localEulerAngles = new Vector3(0, 0, theta2);
    }
}
