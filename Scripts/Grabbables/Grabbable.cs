using Godot;
using Godot.Collections;
using System;

public abstract partial class Grabbable : Node3D
{
    [Signal] public delegate void OnGrabEnterEventHandler(PhysbodyHand hand);
    [Signal] public delegate void OnGrabStayEventHandler(PhysbodyHand hand, float gripStrength);
    [Signal] public delegate void OnGrabEndEventHandler(PhysbodyHand hand);

    [ExportCategory("Grabbable Settings")]
    [Export] public RigidBody3D ParentRigidBody { get; protected set; }
    [Export] public Array<Tool> Tools { get; protected set; }
    [Export] protected bool _DebugMode;
    [Export] protected PackedScene _AxisHelper;


    private Node3D _AxisHelperInstance;
    private PhysbodyHand _LeftHand = null;
    private PhysbodyHand _RightHand = null;


    public override void _Ready()
    {
        if (_DebugMode)
        {
            _AxisHelperInstance = (Node3D)_AxisHelper.Instantiate();
            _AxisHelperInstance.Scale = Vector3.One * 0.1f;
            AddChild(_AxisHelperInstance);
        }
    }

    public override void _Process(double delta)
    {
        if (_LeftHand != null)
        {
            OnHandStay(_LeftHand, _LeftHand.GetGripStrength());
        }
        if (_RightHand != null)
        {
            OnHandStay(_RightHand, _RightHand.GetGripStrength());
        }
    }


    public abstract Transform3D CalculateGrabPose(PhysbodyHand Hand);
    public virtual bool Grab(PhysbodyHand Hand) { return true; }
    public virtual bool Release(PhysbodyHand Hand) { return true; }

    public virtual void OnHandEntered(PhysbodyHand Hand) 
    {
        if (Hand.IsLeftHanded)
        {
            _LeftHand = Hand;
        }
        else
        {
            _RightHand = Hand;
        }
        Hand.IntroduceGrabbable(this);
        EmitSignal(SignalName.OnGrabEnter, Hand);
    }
    public virtual void OnHandExit(PhysbodyHand Hand) 
    {
        if (Hand.IsLeftHanded)
        {
            _LeftHand = null;
        }
        else
        {
            _RightHand = null;
        }
        Hand.RemoveGrabbable(this);
        EmitSignal(SignalName.OnGrabEnd, Hand);
    }
    public virtual void OnHandStay(PhysbodyHand Hand, float GripStrength) 
    {
        if (_DebugMode)
        {
            _AxisHelperInstance.GlobalTransform = ParentRigidBody.GlobalTransform * CalculateGrabPose(Hand);
            _AxisHelperInstance.GlobalBasis *= Basis.Identity.Scaled(Vector3.One * 0.1f);
        }
        EmitSignal(SignalName.OnGrabStay, Hand, GripStrength);
    }

    public void SetToolPrimary(float state)
    {
        foreach(Tool T in Tools)
        {
            T.SetPrimary(state);
        }
    }
    public void SetToolSecondary(float state)
    {
        foreach (Tool T in Tools)
        {
            T.SetSecondary(state);
        }
    }


    public void OnBodyEntered(Node3D body)
    {
        PhysbodyHand hand;
        if (IsHand(body, out hand))
        {
            OnHandEntered(hand);
            return;
        }
    }
    public void OnBodyExited(Node3D body)
    {
        PhysbodyHand hand;
        if (IsHand(body, out hand))
        {
            OnHandExit(hand);
            return;
        }
    }


    private bool IsHand(Node3D body, out PhysbodyHand hand)
    {
        if (body is PhysbodyHand)
        {
            hand = (PhysbodyHand)body;
            return true;
        }

        hand = null;
        return false;
    }
}
