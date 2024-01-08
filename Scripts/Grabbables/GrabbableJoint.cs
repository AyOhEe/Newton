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
        Vector3 handCOM = _HandRB.GlobalTransform * _HandRB.CenterOfMass;
        Vector3 grabbableCOM = _GrabbableRB.GlobalTransform * _GrabbableRB.CenterOfMass;
        Vector3 COM = PhysicsHelpers.CalculateCentreOfMass(_HandRB, _GrabbableRB);
        Basis totalInertia = PhysicsHelpers.CombineInertiaTensors(COM, _HandRB, _GrabbableRB);

        //the momentum of the COM will be the combined linear momentum of the two bodies. from there, velocity
        Vector3 COMMomentum = (_HandRB.LinearVelocity * _HandRB.Mass) + (_GrabbableRB.LinearVelocity * _GrabbableRB.Mass);
        Vector3 COMVel = COMMomentum / (_HandRB.Mass + _GrabbableRB.Mass);


        //https://www.physicsforums.com/threads/total-angular-momentum-of-2-connected-falling-bodies.566219/
        Vector3 angularMomentum = (_HandRB.GetInverseInertiaTensor().Inverse() * _HandRB.AngularVelocity)
            + (_GrabbableRB.GetInverseInertiaTensor().Inverse() * _GrabbableRB.AngularVelocity)
            + (_HandRB.Mass * (handCOM - COM).Cross(_HandRB.LinearVelocity - COMVel))
            + (_GrabbableRB.Mass * (grabbableCOM - COM).Cross(_GrabbableRB.LinearVelocity - COMVel));
        Vector3 angularVelocity = totalInertia.Inverse() * angularMomentum;

        //the bodies will have the same angular velocity as they are joined together perfectly
        _GrabbableRB.AngularVelocity = angularVelocity;
        _HandRB.AngularVelocity = angularVelocity;


        //from COM velocity and angular velocity, we can calculate the linear velocity
        _HandRB.LinearVelocity = COMVel + (handCOM - COM).Cross(angularVelocity);
        _GrabbableRB.LinearVelocity = COMVel + (grabbableCOM - COM).Cross(angularVelocity);
    }
}

public static class PhysicsHelpers
{
    public static Vector3 CalculateCentreOfMass(params RigidBody3D[] bodies)
    {
        Vector3 weightedTotal = Vector3.Zero;
        float totalMass = 0;

        foreach (RigidBody3D body in bodies)
        {
            totalMass += body.Mass;
            weightedTotal += body.GlobalTransform * body.CenterOfMass * body.Mass;
        }

        return weightedTotal / totalMass;
    }

    public static Basis CombineInertiaTensors(Vector3 COM, params RigidBody3D[] bodies)
    {
        Basis inertia = new Basis(); //all zeros

        foreach (RigidBody3D body in bodies)
        {
            //https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor_of_translation
            inertia = PhysicsHelpers.AddBases(inertia, body.GetInverseInertiaTensor().Inverse());

            Vector3 relativePosition = (body.GlobalTransform * body.CenterOfMass) - COM;
            Basis outerProduct = PhysicsHelpers.OuterProduct(relativePosition, relativePosition);
            Basis scaledIdentity = Basis.Identity.Scaled(Vector3.One * relativePosition.LengthSquared());
            Basis difference = PhysicsHelpers.SubBases(scaledIdentity, outerProduct);
            inertia = PhysicsHelpers.AddBases(inertia, difference.Scaled(Vector3.One * body.Mass));
        }

        return inertia;
    }

    public static Basis OuterProduct(Vector3 l, Vector3 r)
    {
        return new Basis(l.X * r.X, l.X * r.Y, l.X * r.Z,
                         l.Y * r.X, l.Y * r.Y, l.Y * r.Z,
                         l.Z * r.X, l.Z * r.Y, l.Z * r.Z);
    }

    public static Basis AddBases(Basis left, Basis right)
    {
        return new Basis(left.Column0 + right.Column0, left.Column1 + right.Column1, left.Column2 + right.Column2);
    }
    public static Basis SubBases(Basis left, Basis right)
    {
        return new Basis(left.Column0 - right.Column0, left.Column1 - right.Column1, left.Column2 - right.Column2);
    }
}