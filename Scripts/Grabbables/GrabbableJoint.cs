using Godot;
using System;

public partial class GrabbableJoint : Node3D
{
	[Export] private PhysbodyHand _HandRB;
	[Export] private RigidBody3D _GrabbableRB;
	[Export] public Vector3 TargetPosition;
	[Export] public Basis TargetRotation;

	private bool _IgnoreCollision;


    public GrabbableJoint(PhysbodyHand hand, RigidBody3D grabbable, bool ignoreCollision = true)
	{
		_HandRB = hand;
		_GrabbableRB = grabbable;
		_IgnoreCollision = ignoreCollision;

		if (ignoreCollision)
		{
			_HandRB.AddCollisionExceptionWith(_GrabbableRB);
        }
	}
	public void HandleDestruction()
	{
		if (_IgnoreCollision)
		{
			_HandRB.RemoveCollisionExceptionWith(_GrabbableRB);
		}
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


        //TODO angular momentum stuff
        _GrabbableRB.GlobalBasis = _HandRB.GlobalBasis * TargetRotation;
    }
}
