using System.Collections;
using System.Collections.Generic;
using UnityEngine.InputSystem;
using UnityEngine;

public class MidiBoard : MonoBehaviour
{
    private GameController controls;
    
    
    void Awake()
    {
    controls = new GameController();
    controls.Enable();
    controls.BoardControl.Enable();
    }
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
	  

	
    }
}
