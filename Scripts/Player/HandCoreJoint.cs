using Godot;
using System;

public partial class HandCoreJoint : Node
{
	[Export] private bool _IsLeftHanded;

	[ExportCategory("References")]
	[Export] private GrabCoordinator _GrabCoordinator;
    [Export] private RigidBody3D _HandRB;
	[Export] private RigidBody3D _CoreRB;


    [ExportCategory("Linear motion settings")]
	[Export] private float _LinearApproachTime = 0.02f;
    [Export] private float _MaxImpulsePerSecond = 10;
	[Export] private float _MomentumDumpImpulsePerSecond = 10;


	[ExportCategory("Angular motion settings")]
	[Export] private float _AngularApproachTime = 0.05f;
	[Export] private float _MaxTorque = 1;


	private bool _HandIsHolding = false;
	private RigidBody3D _HandHeldObject;

	// TODO When holding an object, desired angular velocity should impact desired linear velocity.
	// TODO the problem for two handed grabbing jitters seems to be that the HandCoreJoints get
	//      too comfortable when they're near the desired wrist position, stop outputting enough force
	//      on one end, and then get stuck in a loop of overcorrection. i think this is, in part, because
	//      the grabbable joints don't properly apply the linear velocity coming from angular forces
	//      (e.g. why objects rotate around your wrist IRL, and not around the shared COM) 


    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _PhysicsProcess(double delta)
	{
		Transform3D wristTransform = _GrabCoordinator.GetWristTransform(_IsLeftHanded);

        HandleLinearMotion(wristTransform, delta);
        HandleAngularMotion(wristTransform, delta);

		DumpCoreMomentum(delta);
    }

    private void DumpCoreMomentum(double delta)
    {
		Vector3 deltaI = (_CoreRB.LinearVelocity * _CoreRB.Mass) - (_HandRB.LinearVelocity * _HandRB.Mass);
		deltaI = deltaI.LimitLength(_MomentumDumpImpulsePerSecond * (float)delta);

		_HandRB.LinearVelocity += deltaI / _HandRB.Mass;
		_CoreRB.LinearVelocity -= deltaI / _CoreRB.Mass;
    }

    private void HandleLinearMotion(Transform3D wristTransform, double delta)
	{
        //calculate hand momentum and desired hand momentum
        GD.Print((_IsLeftHanded ? "Left" : "Right") + ": LV " + _HandRB.LinearVelocity.ToString());
        Vector3 momentum = _HandRB.LinearVelocity * _HandRB.Mass 
	   + (_HandIsHolding ? _HandHeldObject.LinearVelocity * additionalMass() : Vector3.Zero);

		float totalMass = _HandRB.Mass + additionalMass();
        Vector3 desiredMomentum = CalculateDesiredHandVel(wristTransform.Origin) * totalMass;

        //calculate the required impulse to achieve this velocity, clamped to the maximum
        //we are allowed to apply per second
        Vector3 requiredImpulse = desiredMomentum - momentum;
		//ensure the hands cannot apply too much force at once by clamping the impulse
		requiredImpulse = requiredImpulse.LimitLength(_MaxImpulsePerSecond * (float)delta);

		//apply this to the hand, and apply the opposite to the core (consv. momentum)
		if (requiredImpulse.IsFinite())
        {
            _HandRB.LinearVelocity += requiredImpulse / _HandRB.Mass;
            _CoreRB.LinearVelocity -= requiredImpulse / _CoreRB.Mass;
        }
		else
		{
			GD.PrintErr("HandCoreJoint calculated infinite deltaV");
		}
    }

	private float additionalMass()
	{
		return (_HandIsHolding && (!_HandHeldObject.Freeze)) ? _HandHeldObject.Mass : 0;
    }

	private Vector3 CalculateDesiredHandVel(Vector3 wristPos)
	{
		Vector3 clampedWristPos = ClampWristToElbow(wristPos);
		Vector3 wristApproachVel = (clampedWristPos - _HandRB.GlobalPosition) / _LinearApproachTime;
		Vector3 remappedApproachVel = wristApproachVel.Normalized() * (0.8f * MathF.Log2(wristApproachVel.Length() + 1));

		return remappedApproachVel + _CoreRB.LinearVelocity;
	}

	private Vector3 ClampWristToElbow(Vector3 wristPos)
	{
		Vector3 elbowPos = _GrabCoordinator.GetElbowTransform(_IsLeftHanded).Origin;

		return (wristPos - elbowPos).LimitLength(VRUserMeasurements.Forearm) + elbowPos;
	}


	private void HandleAngularMotion(Transform3D wristTransform, double delta)
	{
		float fdelta = (float)delta;

		//convert the current and desired rotations into quaternsions, then get the difference
		Quaternion currentRot = _HandRB.GlobalBasis.GetRotationQuaternion();
		Quaternion desiredRot = wristTransform.Basis.GetRotationQuaternion();
		Quaternion deltaQ = desiredRot * currentRot.Inverse();

		//use the smallest rotation possible (one quaternion can represent two possible rotations)
		if (deltaQ.GetAngle() > Mathf.Pi)
		{
			deltaQ = -deltaQ;
		}

		//use this to calculate the axis and angle of rotation
		Vector3 desiredAngVel = (deltaQ.GetAxis() * deltaQ.GetAngle()) / _AngularApproachTime;
		Vector3 desiredAngMomentum = _GrabCoordinator.CalculateDesiredAngularMomentum(_HandRB, desiredAngVel, _IsLeftHanded);
		Vector3 currentAngMomentum = _HandRB.GetInverseInertiaTensor().Inverse() * _HandRB.AngularVelocity;
		desiredAngMomentum = (desiredAngMomentum - currentAngMomentum).LimitLength(fdelta * _MaxTorque) + currentAngMomentum;
		Vector3 angVel = _HandRB.GetInverseInertiaTensor() * desiredAngMomentum;

		if (angVel.IsFinite())
        {
            _HandRB.AngularVelocity = angVel;
        }
        else
        {
            GD.PrintErr("HandCoreJoint calculated infinite angVel");
        }
    }


	public void BeginGrab(RigidBody3D _, Grabbable target)
	{
		if (_HandIsHolding)
		{
			return;
		}

		_HandIsHolding = true;
		_HandHeldObject = target.ParentRigidBody;
	}

	public void ExitGrab(RigidBody3D _, Grabbable target)
	{
		_HandIsHolding = false;
		_HandHeldObject = null;
	}
}
