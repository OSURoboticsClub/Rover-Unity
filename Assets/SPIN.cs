using UnityEngine;
using UnityEngine.UI;

[RequireComponent(typeof(RawImage))]
public class SPIN : MonoBehaviour
{
    [SerializeField] private float degreesPerSecond = 90f;

    void Update()
    {
        transform.Rotate(0f, 0f, degreesPerSecond * Time.deltaTime);
    }
}
