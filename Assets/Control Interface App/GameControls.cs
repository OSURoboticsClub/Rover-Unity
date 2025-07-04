//------------------------------------------------------------------------------
// <auto-generated>
//     This code was auto-generated by com.unity.inputsystem:InputActionCodeGenerator
//     version 1.7.0
//     from Assets/Control Interface App/GameControls.inputactions
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Utilities;

public partial class @GameControls: IInputActionCollection2, IDisposable
{
    public InputActionAsset asset { get; }
    public @GameControls()
    {
        asset = InputActionAsset.FromJson(@"{
    ""name"": ""GameControls"",
    ""maps"": [
        {
            ""name"": ""DriveControl"",
            ""id"": ""9ce3293c-2e41-471b-ad67-00e4216bca08"",
            ""actions"": [
                {
                    ""name"": ""DriveLeft"",
                    ""type"": ""Value"",
                    ""id"": ""781b5061-8bff-4fb6-824b-8326b1c8c2cb"",
                    ""expectedControlType"": ""Vector2"",
                    ""processors"": """",
                    ""interactions"": """",
                    ""initialStateCheck"": true
                },
                {
                    ""name"": ""DriveRight"",
                    ""type"": ""Value"",
                    ""id"": ""eea65a2f-e411-4c9d-825d-a3cfe24d4729"",
                    ""expectedControlType"": ""Vector2"",
                    ""processors"": """",
                    ""interactions"": """",
                    ""initialStateCheck"": true
                }
            ],
            ""bindings"": [
                {
                    ""name"": """",
                    ""id"": ""4d3b9edb-72f9-471a-9190-82a4b1513d3b"",
                    ""path"": ""<Gamepad>/leftStick"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""DriveLeft"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": """",
                    ""id"": ""b5d0f74f-9c46-44d4-a6ea-98ae73641b00"",
                    ""path"": ""<Gamepad>/rightStick"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""DriveRight"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                }
            ]
        }
    ],
    ""controlSchemes"": [
        {
            ""name"": ""test"",
            ""bindingGroup"": ""test"",
            ""devices"": []
        }
    ]
}");
        // DriveControl
        m_DriveControl = asset.FindActionMap("DriveControl", throwIfNotFound: true);
        m_DriveControl_DriveLeft = m_DriveControl.FindAction("DriveLeft", throwIfNotFound: true);
        m_DriveControl_DriveRight = m_DriveControl.FindAction("DriveRight", throwIfNotFound: true);
    }

    public void Dispose()
    {
        UnityEngine.Object.Destroy(asset);
    }

    public InputBinding? bindingMask
    {
        get => asset.bindingMask;
        set => asset.bindingMask = value;
    }

    public ReadOnlyArray<InputDevice>? devices
    {
        get => asset.devices;
        set => asset.devices = value;
    }

    public ReadOnlyArray<InputControlScheme> controlSchemes => asset.controlSchemes;

    public bool Contains(InputAction action)
    {
        return asset.Contains(action);
    }

    public IEnumerator<InputAction> GetEnumerator()
    {
        return asset.GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }

    public void Enable()
    {
        asset.Enable();
    }

    public void Disable()
    {
        asset.Disable();
    }

    public IEnumerable<InputBinding> bindings => asset.bindings;

    public InputAction FindAction(string actionNameOrId, bool throwIfNotFound = false)
    {
        return asset.FindAction(actionNameOrId, throwIfNotFound);
    }

    public int FindBinding(InputBinding bindingMask, out InputAction action)
    {
        return asset.FindBinding(bindingMask, out action);
    }

    // DriveControl
    private readonly InputActionMap m_DriveControl;
    private List<IDriveControlActions> m_DriveControlActionsCallbackInterfaces = new List<IDriveControlActions>();
    private readonly InputAction m_DriveControl_DriveLeft;
    private readonly InputAction m_DriveControl_DriveRight;
    public struct DriveControlActions
    {
        private @GameControls m_Wrapper;
        public DriveControlActions(@GameControls wrapper) { m_Wrapper = wrapper; }
        public InputAction @DriveLeft => m_Wrapper.m_DriveControl_DriveLeft;
        public InputAction @DriveRight => m_Wrapper.m_DriveControl_DriveRight;
        public InputActionMap Get() { return m_Wrapper.m_DriveControl; }
        public void Enable() { Get().Enable(); }
        public void Disable() { Get().Disable(); }
        public bool enabled => Get().enabled;
        public static implicit operator InputActionMap(DriveControlActions set) { return set.Get(); }
        public void AddCallbacks(IDriveControlActions instance)
        {
            if (instance == null || m_Wrapper.m_DriveControlActionsCallbackInterfaces.Contains(instance)) return;
            m_Wrapper.m_DriveControlActionsCallbackInterfaces.Add(instance);
            @DriveLeft.started += instance.OnDriveLeft;
            @DriveLeft.performed += instance.OnDriveLeft;
            @DriveLeft.canceled += instance.OnDriveLeft;
            @DriveRight.started += instance.OnDriveRight;
            @DriveRight.performed += instance.OnDriveRight;
            @DriveRight.canceled += instance.OnDriveRight;
        }

        private void UnregisterCallbacks(IDriveControlActions instance)
        {
            @DriveLeft.started -= instance.OnDriveLeft;
            @DriveLeft.performed -= instance.OnDriveLeft;
            @DriveLeft.canceled -= instance.OnDriveLeft;
            @DriveRight.started -= instance.OnDriveRight;
            @DriveRight.performed -= instance.OnDriveRight;
            @DriveRight.canceled -= instance.OnDriveRight;
        }

        public void RemoveCallbacks(IDriveControlActions instance)
        {
            if (m_Wrapper.m_DriveControlActionsCallbackInterfaces.Remove(instance))
                UnregisterCallbacks(instance);
        }

        public void SetCallbacks(IDriveControlActions instance)
        {
            foreach (var item in m_Wrapper.m_DriveControlActionsCallbackInterfaces)
                UnregisterCallbacks(item);
            m_Wrapper.m_DriveControlActionsCallbackInterfaces.Clear();
            AddCallbacks(instance);
        }
    }
    public DriveControlActions @DriveControl => new DriveControlActions(this);
    private int m_testSchemeIndex = -1;
    public InputControlScheme testScheme
    {
        get
        {
            if (m_testSchemeIndex == -1) m_testSchemeIndex = asset.FindControlSchemeIndex("test");
            return asset.controlSchemes[m_testSchemeIndex];
        }
    }
    public interface IDriveControlActions
    {
        void OnDriveLeft(InputAction.CallbackContext context);
        void OnDriveRight(InputAction.CallbackContext context);
    }
}
