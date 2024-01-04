using Godot;
using System;

public partial class GrabbableJoint : Node3D
{
	public PhysbodyHand _HandRB { get; private set; }
	public RigidBody3D _GrabbableRB { get; private set; }
    public Vector3 TargetPosition;
	public Basis TargetRotation;

    public GrabbableJoint(PhysbodyHand hand, RigidBody3D grabbable)
	{
		_HandRB = hand;
		_GrabbableRB = grabbable;
		_HandRB.AddCollisionExceptionWith(_GrabbableRB);
	}
	public void HandleDestruction()
	{
		_HandRB.RemoveCollisionExceptionWith(_GrabbableRB);
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
		if (_GrabbableRB.Freeze)
        {
            HandleFrozenGrab(delta);
        }
		else
		{
            HandleNonFrozenGrab(delta);
        }
	}
    private void HandleFrozenGrab(double delta)
	{
		//the hand is essentially stuck to the frozen body right now. as such, move the hand to the required
		//position and rotation
		_HandRB.GlobalBasis = _GrabbableRB.GlobalBasis * TargetRotation.Inverse();
		_HandRB.GlobalPosition = _GrabbableRB.GlobalPosition - (_HandRB.GlobalBasis * TargetPosition);
		_HandRB.LinearVelocity = Vector3.Zero; //if we don't do this, the hand doesn't know it's being held back
		_HandRB.AngularVelocity = Vector3.Zero;
    }


    private void HandleNonFrozenGrab(double delta)
	{
        //TODO angular momentum stuff
        _GrabbableRB.GlobalBasis = _HandRB.GlobalBasis * TargetRotation;


        //the bodies are effectively fused right now. as such, position them together, moving them
        //inversely proportional to their masses
        Vector3 grabbableDesiredPos = _HandRB.GlobalPosition + (_HandRB.GlobalBasis * TargetPosition);
        float weighting = _HandRB.Mass / (_HandRB.Mass + _GrabbableRB.Mass);
        Vector3 targetPos = _GrabbableRB.GlobalPosition.Lerp(grabbableDesiredPos, weighting);
        _GrabbableRB.MoveAndCollide(targetPos - _GrabbableRB.GlobalPosition);
        _HandRB.GlobalPosition = _GrabbableRB.GlobalPosition - (_HandRB.GlobalBasis * TargetPosition);

        //their velocities must also be equal. average them based on total momentum
        Vector3 totalMomentum = (_HandRB.LinearVelocity * _HandRB.Mass) + (_GrabbableRB.LinearVelocity * _GrabbableRB.Mass);
        Vector3 targetVel = totalMomentum / (_HandRB.Mass + _GrabbableRB.Mass);
        _HandRB.LinearVelocity = targetVel;
        _GrabbableRB.LinearVelocity = targetVel;
    }
}
