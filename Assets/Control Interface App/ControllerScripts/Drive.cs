using UnityEngine;
using UnityEngine.InputSystem;

public class PlayerController : MonoBehaviour
{
    private GameController controls;  // Use GameController, not GameControls
    
    void Awake()
    {
        controls = new GameController();  // Correct class name
    }
    
    void OnEnable()
    {
        controls.Enable();
    }
    
    void OnDisable()
    {
        controls.Disable();
    }
    
    void Update()
    {
        // Access with capital J - Joy, not joy
        Vector2 joyValue = controls.DriveControl.Joy.ReadValue<Vector2>();
        
        //Debug.Log($"Joystick: X={joyValue.x}, Y={joyValue.y}");
    }
}
