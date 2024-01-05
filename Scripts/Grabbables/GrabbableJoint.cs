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
        //the bodies are effectively fused right now. as such, position them together, moving them
        //inversely proportional to their masses
        Vector3 grabbableDesiredPos = _HandRB.GlobalPosition + (_HandRB.GlobalBasis * TargetPosition);
        float weighting = _HandRB.Mass / (_HandRB.Mass + _GrabbableRB.Mass);
        Vector3 targetPos = _GrabbableRB.GlobalPosition.Lerp(grabbableDesiredPos, weighting);
        _GrabbableRB.MoveAndCollide(targetPos - _GrabbableRB.GlobalPosition);
        _HandRB.GlobalPosition = _GrabbableRB.GlobalPosition - (_HandRB.GlobalBasis * TargetPosition);
        //TODO basis should be set proportional to the inertia of the objects in question
        _GrabbableRB.GlobalBasis = _HandRB.GlobalBasis * TargetRotation;



        //we'll need these for the upcoming calculations
        Basis totalInertia = AddBases(_HandRB.GetInverseInertiaTensor().Inverse(), _GrabbableRB.GetInverseInertiaTensor().Inverse());
        Vector3 handCOM = _HandRB.GlobalBasis * _HandRB.CenterOfMass;
        Vector3 grabbableCOM = _GrabbableRB.GlobalBasis * _GrabbableRB.CenterOfMass;
        Vector3 COM = ((handCOM * _HandRB.Mass) + (grabbableCOM * _GrabbableRB.Mass)) / (_HandRB.Mass + _GrabbableRB.Mass);

        //the momentum of the COM will be the combined linear momentum of the two bodies. from there, velocity
        Vector3 COMMomentum = (_HandRB.LinearVelocity * _HandRB.Mass) + (_GrabbableRB.LinearVelocity * _GrabbableRB.Mass);
        Vector3 COMVel = COMMomentum / (_HandRB.Mass + _GrabbableRB.Mass);


        //https://www.physicsforums.com/threads/total-angular-momentum-of-2-connected-falling-bodies.566219/
        Vector3 angularMomentum = (_HandRB.GetInverseInertiaTensor().Inverse() * _HandRB.AngularVelocity)
            + (_GrabbableRB.GetInverseInertiaTensor().Inverse() * _GrabbableRB.AngularVelocity)
            + (_HandRB.Mass * (handCOM - COM).Cross(COMVel - _HandRB.LinearVelocity))
            + (_GrabbableRB.Mass * (grabbableCOM - COM).Cross(COMVel - _GrabbableRB.LinearVelocity));
        Vector3 angularVelocity = totalInertia.Inverse() * angularMomentum;

        //the bodies will have the same angular velocity as they are joined together perfectly
        _GrabbableRB.AngularVelocity = angularVelocity;
        _HandRB.AngularVelocity = angularVelocity;


        //from COM velocity and angular velocity, we can calculate the linear velocity
        _HandRB.LinearVelocity = COMVel + (handCOM - COM).Cross(angularVelocity);
        _GrabbableRB.LinearVelocity = COMVel + (grabbableCOM - COM).Cross(angularVelocity);
    }

    public static Basis AddBases(Basis left, Basis right)
    {
        return new Basis(left.Column0 + right.Column0, left.Column1 + right.Column1, left.Column2 + right.Column2);
    }
}